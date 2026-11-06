/*
 * ps3recomp - PPU <-> SPU lock-line coherence
 *
 * See spu_coherency.h for what this is for. The state here is deliberately
 * plain: a bit per 128-byte line, a small table of the SPU contexts that have
 * ever reserved one, and the lock-line spinlock. Everything that WRITES it runs
 * with the lock-line lock held (the SPU's GETLLAR, the PPU's coherent store),
 * so the read-modify-write on a shared bitmap byte is serialized. Only
 * spu_coh_is_reserved reads without the lock, which is the entire point: it is
 * on every PPU store. The one other unlocked path is a GETLLAR that re-reserves
 * a marked line (spu_coh_reserve_fast); its ordering argument is there.
 */

#include <stdlib.h>
#include "spu_coherency.h"
#include "spu_coh_bitmap.h"

#include <stdint.h>
#include <stdio.h>
#include "ps3emu/log_first.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * The lock-line lock
 *
 * Moved here from spu_channels.c, where it was a file-static and so could
 * serialize SPU against SPU but not SPU against PPU -- which left the PPU free
 * to write a line in the middle of a PUTLLC's compare-and-commit. Same
 * exclusion semantics, now shared. MSVC uses a bounded polite spin loop;
 * POSIX spins briefly and then parks on a mutex (see below).
 * ===========================================================================*/
#if defined(_WIN32)
#include <windows.h>   /* SwitchToThread, WaitOnAddress */
static volatile long g_lockline = 0;
static volatile long g_lockline_waiters = 0;
typedef BOOL (WINAPI* woa_fn)(volatile VOID*, PVOID, SIZE_T, DWORD);
typedef VOID (WINAPI* wba_fn)(PVOID);
static woa_fn s_woa; static wba_fn s_wba; static volatile int s_ll_init;
static int s_ll_spin = 4096;
/* Test-and-test-and-set: spin on a plain read (no cache-line ping-pong) with a
 * pause, yield after a while, and then really wait. The bare exchange loop let
 * GH3's four idle job-policy pollers saturate the line and delay every other SPU
 * atomic -- FMOD's mixer task among them, 25-40 ms at a time.
 *
 * This was under _MSC_VER until v99, and the Windows build is zig's clang: it got
 * the bare exchange loop after all. With more threads than CPUs a holder that loses
 * its CPU kept all the others hammering the line until it got one back (Windows'
 * time slice: 15+ ms): 48 fps on 6 threads and 1-5 on 4, game audio at 6-14 blocks a
 * second of its 187.5, with every CPU flat out. Waiting frees the CPU the holder needs. */
void spu_lockline_lock(void)
{
    if (!_InterlockedExchange(&g_lockline, 1)) return;
    if (!s_ll_init) {
        extern int racacit_few_cpus(void);
        HMODULE m = LoadLibraryA("api-ms-win-core-synch-l1-2-0.dll");
        if (m) { s_woa = (woa_fn)(void*)GetProcAddress(m, "WaitOnAddress"); s_wba = (wba_fn)(void*)GetProcAddress(m, "WakeByAddressSingle"); }
        if (racacit_few_cpus()) s_ll_spin = 128;
        s_ll_init = 1;
    }
    unsigned spins = 0, rounds = 0;
    do {
        while (g_lockline) {
            YieldProcessor();
            if (++spins < (unsigned)s_ll_spin) continue;
            spins = 0;
            if (++rounds < 3 || !s_woa) { SwitchToThread(); continue; }
            long held = 1;
            _InterlockedIncrement(&g_lockline_waiters);
            if (g_lockline) s_woa(&g_lockline, &held, sizeof held, 1);
            _InterlockedDecrement(&g_lockline_waiters);
        }
    } while (_InterlockedExchange(&g_lockline, 1));
}
void spu_lockline_unlock(void)
{
    _InterlockedExchange(&g_lockline, 0);
    if (g_lockline_waiters && s_wba) s_wba((PVOID)&g_lockline);
}
/* The short locks' wait: on a plain read, with a pause; a holder off its CPU gets a yield. */
void spu_spin_wait(volatile long* lock)
{
    unsigned spins = 0;
    while (*lock) {
        YieldProcessor();
        if (++spins >= 1024u) { spins = 0; SwitchToThread(); }
    }
}
#elif 0   /* racacit: the _WIN32 branch above covers clang/MinGW too (upstream: plain spin) */
#include <stdatomic.h>
static atomic_flag g_lockline = ATOMIC_FLAG_INIT;
void spu_lockline_lock(void)   { while (atomic_flag_test_and_set_explicit(&g_lockline, memory_order_acquire)) { } }
void spu_lockline_unlock(void) { atomic_flag_clear_explicit(&g_lockline, memory_order_release); }
#else
#include <pthread.h>
#include <time.h>
/* Every SPU reservation and every PPU coherent store takes this one lock, so
 * it is acquired very often and held very briefly (a line copy and a short
 * list walk). But its owner is an ordinary host thread: with more guest
 * threads than cores it can be preempted while holding the lock, and a bare
 * test-and-set loop then keeps every waiter spinning on a full core -- the
 * cores the owner needs to run again -- for the rest of its time slice.
 *
 * So: retry for up to 20 us, with exponentially spaced attempts so the
 * waiters do not keep pulling the lock's cache line away from its owner, and
 * then park on the mutex. A running owner releases well inside the spin; a
 * preempted one no longer costs the waiters their cores. The mutex keeps the
 * same non-recursive exclusion and acquire/release publication. */
static pthread_mutex_t g_lockline = PTHREAD_MUTEX_INITIALIZER;
static inline uint64_t spu_lockline_now_ns(void)
{
#if defined(__APPLE__)
    return clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
#endif
}
static inline void spu_lockline_relax(void)
{
#if defined(__aarch64__)
    __asm__ __volatile__("yield");
#elif defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#endif
}
void spu_lockline_lock(void)
{
    if (!pthread_mutex_trylock(&g_lockline)) return;
    const uint64_t give_up = spu_lockline_now_ns() + 20000u;
    for (unsigned backoff = 16;; backoff = backoff < 1024u ? backoff * 2u : backoff) {
        for (unsigned i = 0; i < backoff; ++i) spu_lockline_relax();
        if (!pthread_mutex_trylock(&g_lockline)) return;
        if (spu_lockline_now_ns() >= give_up) break;
    }
    if (pthread_mutex_lock(&g_lockline)) abort();
}
void spu_lockline_unlock(void) { if (pthread_mutex_unlock(&g_lockline)) abort(); }
#endif

/* ===========================================================================
 * The reserved-line bitmap
 *
 * One bit per 128-byte line over the whole 32-bit guest address space: 4 MiB of
 * zero-initialized BSS, of which only the pages around addresses an SPU has
 * actually reserved are ever touched. The reference implementation scopes its
 * bitmap to the one window a single title's SPURS structures live in; this
 * runtime hands out guest memory from several disjoint regions (the ELF image,
 * the sys_memory window at 0x40000000, the raw-SPU local-store windows), and a
 * line outside a hardcoded window would be silently non-coherent, which is the
 * failure this whole file exists to remove.
 * ===========================================================================*/
#define SPU_COH_LINES       (1u << (32 - SPU_COH_LINE_SHIFT))   /* 2^25 lines */
#define SPU_COH_BITMAP_SZ   (SPU_COH_LINES / 8)                 /* 4 MiB */

unsigned char spu_coh_bitmap_g[SPU_COH_BITMAP_SZ];   /* read inline by spu_coh_is_reserved (header) */
#define s_coh_bitmap spu_coh_bitmap_g

/* g_spu_coh_bits (spu_coh_bitmap.h) stays NULL until the first GETLLAR
 * anywhere. That keeps the PPU fast path off the bitmap entirely for a title
 * that runs no SPU code, and keeps every existing test that stores through
 * vm_write* paying one compare against a hot global. Written under the lock;
 * read unlocked, where a stale NULL costs at most one missed event on the very
 * first reservation of the run. */
/* racacit: the same moment as a flag, for the lifted PPU store fast path
 * (ppu_lifter.py, ppu_loader.cpp: spu_coh_is_reserved_fast). */
volatile int spu_coh_armed_g;

/* Every live virtual context may retain a reservation between scheduling
 * quanta; physical SPU count is not a bound on this set. All access is under
 * the lock-line lock. Nodes are removed before their context's lifetime ends. */
typedef struct spu_coh_entry {
    spu_context* ctx;
    struct spu_coh_entry* next;
} spu_coh_entry;
static spu_coh_entry* s_coh_ctxs;
static spu_coh_watch* s_coh_watches;

unsigned long g_spu_lr_raise = 0;

/* For the [mem] log line (game/main.cpp): lines marked in the bitmap now, and
 * writes spu_coh_notify_write was told of. A line stays marked once any SPU
 * reserved it, and every PPU store into a marked line takes the locked path:
 * if this count keeps climbing through a long session, so does that cost. */
unsigned long g_spu_coh_lines = 0;
unsigned long g_spu_coh_writes = 0;
unsigned long g_ppu_coh_stores = 0;   /* of those, PPU stores (ppu_loader.cpp; the rest are SPU atomics) */

void spu_coh_reserve(spu_context* ctx, uint32_t ea)
{
    /* Read before writing. The kernel reserves the same few lines over and
     * over, and a store of the same value still takes the cache line away
     * from every other core: each PPU store then missed on the pointer and on
     * the bitmap byte. */
    uint32_t line = ea >> SPU_COH_LINE_SHIFT;
    const unsigned char bit = (unsigned char)(1u << (line & 7));
    if (!(s_coh_bitmap[line >> 3] & bit)) {
        s_coh_bitmap[line >> 3] |= bit;
        g_spu_coh_lines++;
    }
    if (!g_spu_coh_bits) {
        g_spu_coh_bits = s_coh_bitmap;
        spu_coh_armed_g = 1;
    }

    if (!ctx) return;
    for (spu_coh_entry* entry = s_coh_ctxs; entry; entry = entry->next)
        if (entry->ctx == ctx) { __atomic_store_n(&ctx->coh_listed, 1, __ATOMIC_RELAXED); return; }
    spu_coh_entry* entry = (spu_coh_entry*)malloc(sizeof(*entry));
    if (!entry) {
        fprintf(stderr, "[spu-coh] cannot register reservation holder %p\n", (void*)ctx);
        fflush(stderr);
        abort(); /* A missed holder loses invalidation events, including ABA. */
    }
    entry->ctx = ctx;
    entry->next = s_coh_ctxs;
    s_coh_ctxs = entry;
    __atomic_store_n(&ctx->coh_listed, 1, __ATOMIC_RELAXED);
}

void spu_coh_watch_add(spu_coh_watch* watch, uint32_t ea,
                       void (*wake)(void*), void* opaque)
{
    /* Arming the existing bitmap makes PPU stores/CAS and MFC writes use
     * the same serialized commit/notify path as SPU reservations. */
    spu_coh_reserve(NULL, ea);
    watch->line = ea & ~127u;
    watch->wake = wake;
    watch->opaque = opaque;
    watch->next = s_coh_watches;
    s_coh_watches = watch;
}

void spu_coh_watch_remove(spu_coh_watch* watch)
{
    for (spu_coh_watch** link = &s_coh_watches; *link; link = &(*link)->next) {
        if (*link != watch) continue;
        *link = watch->next;
        watch->next = NULL;
        return;
    }
}

static void spu_coh_wake_watches(uint32_t line)
{
    for (spu_coh_watch* w = s_coh_watches; w; w = w->next)
        if (w->line == line) w->wake(w->opaque);
}

/* Drop a context from the reserving set.
 *
 * There was no way to do this, and the contexts that most need it are
 * TRANSIENT: spu_run_lifted_job_abi runs each job on a STACK-LOCAL
 * spu_context, which registered itself here on its first reservation and then
 * returned. The entry outlived the object, so spu_coh_notify_write walked a
 * dangling pointer -- reading resv_valid, writing event_status and calling
 * spu_ch_wake on reclaimed stack. Undefined behaviour, and observable: Guitar
 * Hero III showed THREE live-looking contexts for one dispatch, stealing each
 * other`s reservations, with every PUTLLC failing "no reservation". */
void spu_coh_unregister(spu_context* ctx)
{
    if (!ctx) return;
    /* Under the lock-line lock, which every walker of s_coh_ctxs holds: a
     * notifier that loaded this pointer just before removal would
     * otherwise go on dereferencing it after the job returned and its stack
     * (or its whole async thread) was gone. GH3 at -O2 crashed that way in
     * spu_coh_notify_write, reading a freed thread stack. */
    spu_lockline_lock();
    for (spu_coh_entry** link = &s_coh_ctxs; *link; link = &(*link)->next) {
        if ((*link)->ctx != ctx) continue;
        spu_coh_entry* entry = *link;
        *link = entry->next;
        free(entry);
        break;
    }
    __atomic_store_n(&ctx->coh_listed, 0, __ATOMIC_RELAXED);
    spu_lockline_unlock();
}

/* Drop every registered context that lies in [lo, hi) -- a host thread's
 * stack, called as that thread exits. A stack-local context still registered
 * then has leaked past its unregister; the next walker would read an unmapped
 * stack (GH3 at the end of a Havok task: spu_coh_notify_write faulting).
 * ponytail: safety net that also names the leak; find the path that skips
 * spu_coh_unregister if it keeps firing. */
void spu_coh_forget_range(uintptr_t lo, uintptr_t hi)
{
    spu_lockline_lock();
    for (spu_coh_entry** link = &s_coh_ctxs; *link;) {
        spu_coh_entry* entry = *link;
        uintptr_t c = (uintptr_t)entry->ctx;
        if (c >= lo && c < hi) {
            static int _n = 0;
            if (PS3_LOG_FIRST(_n, 16))
                fprintf(stderr, "[spu-coh] leaked ctx %p (img=%d) dropped at thread exit\n",
                        (void*)c, entry->ctx->image_id);
            *link = entry->next;
            free(entry);
        } else {
            link = &entry->next;
        }
    }
    spu_lockline_unlock();
}

/* racacit: hand a line a title marked with spu_coh_reserve(NULL, ...) back to
 * the PPU's lock-free store path (game/stubs.cpp, the collision records) --
 * unless an SPU holds a reservation on it right now; its next GETLLAR marks it
 * again anyway. Lock-line lock held. */
void spu_coh_unreserve_line(uint32_t ea)
{
    uint32_t line = ea & ~127u;
    for (spu_coh_entry* e = s_coh_ctxs; e; e = e->next) {
        spu_context* c = e->ctx;
        if (c && c->resv_valid && (c->resv_ea & ~127u) == line) return;
    }
    line >>= SPU_COH_LINE_SHIFT;
    if (s_coh_bitmap[line >> 3] & (1u << (line & 7))) g_spu_coh_lines--;
    s_coh_bitmap[line >> 3] &= (unsigned char)~(1u << (line & 7));
}

/* Hot paths use the inline spu_coh_is_reserved_fast (spu_coherency.h). */
int spu_coh_is_reserved(uint32_t addr) { return spu_coh_is_reserved_fast(addr); }

/* A DMA PUT is issued BY an SPU, and hardware does not take that SPU's own
 * reservation away for its own MFC write -- the MFC is part of the same SPE.
 * Our notify had no way to say "everyone but me", so an SPU that reserved a
 * line and then DMA'd a buffer overlapping it killed its own reservation and
 * its next PUTLLC could never succeed. Guitar Hero III's job policy module
 * livelocks exactly there: 16 million atomic ops on one line, every PUTLLC
 * failing for "no reservation", 600k of them self-inflicted.
 *
 * The PUTLLC path already guards this case by dropping its own reservation
 * before notifying; this gives the DMA path the same ability. */
/* ppu_loader.cpp, when linked. The loader is compiled per game, so the host,
 * the fixtures and any build without it have no PPU reservations to break.
 * POSIX: a weak no-op definition the loader's overrides (a weak undefined
 * reference is not enough: ld64 only lets those stay unresolved for dylib
 * symbols). link.exe/lld-link: the linker's own default (see ppu_gcm_pump in
 * ppu_loader.cpp). */
#if defined(_MSC_VER)
void ppu_resv_break_line(uint32_t ea);
void ppu_resv_break_line_absent(uint32_t ea) { (void)ea; }
#pragma comment(linker, "/alternatename:ppu_resv_break_line=ppu_resv_break_line_absent")
#else
__attribute__((weak)) void ppu_resv_break_line(uint32_t ea) { (void)ea; }
#endif

/* ===========================================================================
 * GETLLAR without the lock-line lock
 *
 * Every SPURS policy module polls its workload lines with GETLLAR, so taking
 * the one global lock for each poll serialized every SPU on a 128-byte copy.
 * A poll is almost always a context that already holds a node in s_coh_ctxs
 * re-reading a line that is already marked, and for that case the lock buys
 * nothing the reservation itself does not:
 *
 *   reader (spu_coh_reserve_fast + caller)    writer (any notifier caller)
 *     resv_valid = 0, resv_ea = ea,             store the new line data
 *     resv_valid = 1                            spu_coh_writer_fence()
 *     seq_cst fence                             load resv_valid / resv_ea
 *     copy the line                             if they cover the line:
 *                                                 LR, resv_valid = 0
 *
 * This is the store-buffering (Dekker) shape: with a seq_cst fence between the
 * store and the load on both sides, the two fences are ordered, so at least
 * one side sees the other. Either the reader's copy already holds the write
 * (an ordinary GETLLAR after the store), or the writer sees the reservation
 * and ends it -- the reader may then hold a stale or torn copy, but its
 * PUTLLC fails and it has SPU_EVENT_LR, exactly as for a write landing just
 * after a locked GETLLAR. A writer that loads stale resv_valid/resv_ea is one
 * whose fence came first, so the copy already holds its data; at worst it
 * ends the new reservation spuriously. What the reader gives up is a copy
 * atomic against a writer's multi-store critical section, but such a copy is
 * always followed by that writer's notify, so the reservation is ended.
 *
 * Two preconditions keep every writer of the line on that right-hand side:
 *  - the line is already marked. Marks are never cleared, so every store path
 *    that checks spu_coh_is_reserved takes the notifier for this line.
 *  - ctx is already in s_coh_ctxs (coh_listed). The node was linked under the
 *    lock by an earlier GETLLAR of this same thread, so any writer whose
 *    critical section could overlap this read walks past it.
 * Anything else takes the lock and spu_coh_reserve as before.
 *
 * Writers still hold the lock-line lock, so they stay serialized with each
 * other and with PUTLLC's compare-and-commit; only the reader stopped
 * waiting. resv_valid and resv_ea are accessed atomically on both sides of
 * the race; everything else about a reservation is still only touched by the
 * owning thread or under the lock.
 * ===========================================================================*/
static inline void spu_coh_writer_fence(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

static inline int spu_coh_holds(const spu_context* c, uint32_t line)
{
    return __atomic_load_n(&c->resv_valid, __ATOMIC_RELAXED) &&
           (__atomic_load_n(&c->resv_ea, __ATOMIC_RELAXED) & ~127u) == line;
}

int spu_coh_reserve_fast(spu_context* ctx, uint32_t ea)
{
    if (!ctx || !__atomic_load_n(&ctx->coh_listed, __ATOMIC_RELAXED) ||
        !spu_coh_is_reserved(ea))
        return 0;
    /* Retire any older reservation before retargeting it. The fence is what
     * the argument above rests on. */
    __atomic_store_n(&ctx->resv_valid, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&ctx->resv_ea, ea, __ATOMIC_RELAXED);
    __atomic_store_n(&ctx->resv_valid, 1, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 1;
}

void spu_coh_notify_write_except(uint32_t ea, const void* self)
{
    uint32_t line = ea & ~127u;
    ppu_resv_break_line(ea);
    spu_coh_writer_fence();
    spu_coh_wake_watches(line);
    for (spu_coh_entry* entry = s_coh_ctxs; entry; entry = entry->next) {
        spu_context* c = entry->ctx;
        if (!c || (const void*)c == self) continue;
        if (spu_coh_holds(c, line)) {
            SPU_EVENT_SET(c, SPU_EVENT_LR);
            __atomic_store_n(&c->resv_valid, 0, __ATOMIC_RELAXED);
            g_spu_lr_raise++;
            spu_ch_wake(c);
        }
    }
}

/* racacit: title hooks on SPU DMA of one guest structure (game/stubs.cpp, the
 * A Crack in Time collision command record). NULL for every other title.
 *   g_coh_write_hook     -- every write spu_coh_notify_write is told of: a PPU
 *                           store / stwcx. / vector store, or an SPU atomic, into
 *                           a reserved line (DMA PUTs use the _except variant
 *                           and are not reported). Lock-line lock held.
 *   g_spu_get_copy_hook  -- a DMA GET; returns 1 if it made the copy itself.
 *   g_spu_put_skip_hook  -- a DMA PUT into a reserved line, lock held; returns 1
 *                           to drop the copy (the transfer still completes). */
void (*g_coh_write_hook)(uint32_t ea) = 0;
int  (*g_spu_get_copy_hook)(void* spu, const uint8_t* ls, uint32_t lsa, uint32_t ea, uint32_t size,
                            uint8_t* dst, const uint8_t* src) = 0;
int  (*g_spu_put_skip_hook)(void* spu, const uint8_t* ls, uint32_t lsa, uint32_t ea, uint32_t size) = 0;

void spu_coh_notify_write(uint32_t ea)
{
    g_spu_coh_writes++;
    if (g_coh_write_hook) g_coh_write_hook(ea);
    uint32_t line = ea & ~127u;
    ppu_resv_break_line(ea);
    spu_coh_writer_fence();
    spu_coh_wake_watches(line);
    for (spu_coh_entry* entry = s_coh_ctxs; entry; entry = entry->next) {
        spu_context* c = entry->ctx;
        if (!c) continue;
        if (spu_coh_holds(c, line)) {
            SPU_EVENT_SET(c, SPU_EVENT_LR);
            __atomic_store_n(&c->resv_valid, 0, __ATOMIC_RELAXED);   /* reservation lost, PUTLLC must fail */
            /* SPU_PUTLLC_WHY=1: name the agent that killed it. A PUTLLC that
             * always fails for "no reservation" is useless without knowing who
             * took it away -- a peer SPU, or the PPU committing to the line. */
            { static int s_w = -1;
              if (s_w < 0) s_w = getenv("SPU_PUTLLC_WHY") ? 1 : 0;
              if (s_w) { static unsigned long long n;
                  if ((++n % 200000) == 1)
                      fprintf(stderr, "[resv-lost] %llu: line 0x%08X taken from img=%d ctx=%p\n",
                              n, line, c->image_id, (void*)c); } }
            g_spu_lr_raise++;
            /* The event_status store has to be visible before the wake. The
             * waiter re-polls its predicate on a timeout as well, so a
             * straggling store costs latency and not the wakeup itself. */
            spu_ch_wake(c);
        }
    }
}

/* racacit: wake every parked SPU now. A Crack in Time posts its SPU jobs
 * with plain stores (no lock-line notify reaches the idle SPURS kernels),
 * then the main thread waits on them -- about nine submit/wait round trips
 * per view per frame (func_004DFDF0). An idle lane only noticed the new work
 * when its spu_lr_wait bound ran out (SPU_LR_WAIT_US), so every round trip
 * paid that bound. Called at the entry of the game's job/batch waits
 * (patch_lift step "spu kick"): each parked lane gets a reservation-lost
 * event, re-reads its workload state at once and finds the job. A spurious
 * LR event is harmless -- the kernel re-polls and parks again.
 * MEASURED 2026-10-18 (Galacton, standing still, 2x2 runs, order swapped):
 * no effect -- frame-waits stayed ~135 ms/s and frame times moved only with
 * run order. The waits are the jobs' own run time (igMobyPreRender ~1.4 ms a
 * frame), not lane pickup. So it is OFF by default; SPU_KICK_ON_WAIT=1 turns
 * it on. */
unsigned long g_spu_kicks = 0;
void racacit_spu_kick(void)
{
    static int s_on = -1;
    if (s_on < 0) { const char* e = getenv("SPU_KICK_ON_WAIT"); s_on = e ? atoi(e) != 0 : 0; }
    if (!s_on) return;
    spu_lockline_lock();
    for (spu_coh_entry* e = s_coh_ctxs; e; e = e->next) {
        spu_context* c = e->ctx;
        if (!c || !c->resv_valid) continue;
        c->event_status |= SPU_EVENT_LR;
        c->resv_valid = 0;
        g_spu_lr_raise++;
        spu_ch_wake(c);
    }
    spu_lockline_unlock();
    g_spu_kicks++;
}

#ifdef __cplusplus
}
#endif
