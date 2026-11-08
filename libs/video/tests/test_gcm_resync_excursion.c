/* A resync from outside the ring sweeps the block put is in, not the IO
 * space between the two (cellGcmSys.c, gcm_fifo_resync_why).
 *
 * Ratchet & Clank: A Crack in Time double-buffers its frames in two 5 MB
 * blocks, A and B. Each starts with a header (semaphore releases and a flip),
 * the frame follows, and the frame ends in a JUMP to the other block's header;
 * put stops just past that header. Reading text in the museum or the Battleplex
 * sometimes left the walker taking a JUMP from the start of B into low IO that
 * held a data table, not commands. It hit a bad word there and resynced to put
 * (A + 0x24). The fence sweep then ran from that low address up to put, across
 * 22 to 27 MB of IO: textures, vertex data and the stale frames in A. It
 * queued 781 to 5724 old SET_REFERENCE values, more the longer the session.
 * The label walk stopped on the first bad word, so the releases in A's header
 * were never replayed. The title spun on that frame's release until its own
 * "GPU sit and spin" timeout, about 40 seconds with the screen frozen.
 *
 * The walker took a JUMP into A's header two frames earlier, so that target
 * is where put's block starts. The resync now sweeps from there.
 *
 * Layout (IO offsets):
 *   0x3F0000  previous frame's tail: JUMP A
 *   0x400000  A: header (release 0x400), a method, JUMP B
 *   0x480000  B: header (release 0x400), then a JUMP into low IO
 *   0x001000  low IO: one method, then a word that branches to unmapped IO
 *   0x002000  stale data: 64 old SET_REFERENCE commands
 *
 * zig cc -target x86_64-windows-gnu -std=gnu17 -I include -I libs/video
 *   libs/video/tests/test_gcm_resync_excursion.c -ffunction-sections
 *   -fdata-sections -Wl,--gc-sections -o test_gcm_resync_excursion.exe
 */
#include "../cellGcmSys.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
uint8_t* vm_base;
uint32_t ppu_vm_size;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
int spu_coh_is_reserved(uint32_t a) { (void)a; return 0; }
void spu_coh_notify_write(uint32_t a) { (void)a; }
void spu_lockline_lock(void) {}
void spu_lockline_unlock(void) {}
ps3_guest_caller_fn g_ps3_guest_caller;
PPU_THREAD_LOCAL ppu_context* g_active_ctx;
u32 g_rsx_last_reference;
void (*g_spu_put_hook)(uint32_t ea, uint32_t size);
void ppu_dump_bctrl_ring(uint32_t n, const char* why) { (void)n; (void)why; }
void ppu_dump_guest_stack(ppu_context* c, const char* why) { (void)c; (void)why; }
void rsx_state_init(rsx_state* state) { (void)state; }
int  rsx_process_method(rsx_state* state, u32 method, u32 data) { (void)state; (void)method; (void)data; return 0; }
int  rsx_draw_engine_enabled(void) { return 0; }
int  rsx_process_vp_run(rsx_state* state, u32 method, const u8* src, u32 count)
{ (void)state; (void)method; (void)src; (void)count; return 0; }
rsx_backend* rsx_get_backend(void) { return NULL; }
void rsx_draw_engine_method(u32 method, u32 arg) { (void)method; (void)arg; }
void rsx_draw_engine_note_flip(u32 buffer_id) { (void)buffer_id; }
void rsx_draw_engine_set_display_buffer(u32 id, u32 location, u32 offset, u32 pitch, u32 width, u32 height)
{ (void)id; (void)location; (void)offset; (void)pitch; (void)width; (void)height; }
void rsx_draw_engine_guest_read(u32 l, u32 o, u32 p, u32 w, u32 h) { (void)l; (void)o; (void)p; (void)w; (void)h; }
void rsx_draw_engine_guest_wrote(u32 l, u32 o, u32 p, u32 w, u32 h) { (void)l; (void)o; (void)p; (void)w; (void)h; }
int  rsx_live_draw_enabled(void) { return 0; }
void rsx_live_draw_method(u32 method, u32 arg) { (void)method; (void)arg; }
int  rsx_live_draw_blit(u32 a, u32 b, u32 c, u32 d, u32 e, u32 f, u32 w, u32 h)
{ (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)w; (void)h; return 0; }
void rsx_live_draw_note_resolve(u32 a, u32 b, u32 c, u32 d) { (void)a; (void)b; (void)c; (void)d; }
int  rsx_live_draw_resolve_blit(u32 a, u32 b, u32 c, u32 d, u32 e, u32 f, u32 g, u32 h,
                                u32 i, u32 j, u32 k, u32 l)
{ (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; (void)h; (void)i; (void)j; (void)k; (void)l; return 0; }
void rsx_raise_user_cmd(uint32_t arg) { (void)arg; }
/* RACiT's runtime hooks the walker also calls. */
void edgezlib_check_range(u32 ea, u32 len) { (void)ea; (void)len; }
uint64_t ps3_guest_clock_epoch_qpc(void) { return 0; }
void ps3_tl(char ev, u32 a, u32 b) { (void)ev; (void)a; (void)b; }
u32  rsx_draw_engine_park_id(void) { return 0; }
void rsx_stats_add(u32 a, u32 b, u32 c) { (void)a; (void)b; (void)c; }
void rsx_ww_poll(void) {}
void rsx_ww_poll_heap(void) {}
int  vm_plain_copy(u32 dst, u32 src, u32 n) { memmove(vm_base + dst, vm_base + src, n); return 1; }
u8*  vm_plain_span(u32 ea, u32 n, int write) { (void)n; (void)write; return vm_base + ea; }

#define IO_ADDR   0x00800000u
#define IO_SIZE   0x00800000u
#define CMD_SIZE  0x00010000u

#define BLK_A     0x00400000u
#define BLK_B     0x00480000u
#define PREV_TAIL 0x003F0000u
#define LOW       0x00001000u
#define STALE     0x00002000u
#define N_STALE   64u

static uint32_t g_heap = 0x00400000u;
static u32 host_alloc(u32 size, u32 align)
{
    if (!align) align = 4;
    g_heap = (g_heap + align - 1u) & ~(align - 1u);
    u32 r = g_heap;
    g_heap += size;
    return r;
}
static void gw32(uint32_t ea, uint32_t v)
{
    uint8_t* p = vm_base + ea;
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static void host_w32(u32 ea, u32 v) { gw32(ea, v); }

static int fails;
static void check(int ok, const char* what)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    fails += !ok;
}

/* One method, count 1, subchannel 0. */
static u32 put_method(u32 io, u32 method, u32 arg)
{
    gw32(IO_ADDR + io, (1u << 18) | method);
    gw32(IO_ADDR + io + 4, arg);
    return io + 8;
}
static u32 put_jump(u32 io, u32 target)
{
    gw32(IO_ADDR + io, 0x20000000u | target);
    return io + 4;
}
/* A block header: NV406E semaphore offset 0x400, release `v`. */
static u32 put_header(u32 io, u32 v)
{
    io = put_method(io, 0x0064u, 0x0400u);
    return put_method(io, 0x006Cu, v);
}

static void run(const uint32_t ctrl, u32 put)
{
    vm_write32(ctrl + 0, put);
    cellGcm_rsx_process_fifo();
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    const uint32_t vm_size = ppu_hle_inject_base + 0x10000u;
    vm_base = (uint8_t*)calloc(1, vm_size);
    if (!vm_base) { puts("FAIL: guest VM alloc"); return 1; }
    ppu_vm_size = vm_size;

    u32 ctx_out = host_alloc(4, 4);
    (void)cellGcmSetupContext(ctx_out, CMD_SIZE, IO_SIZE, IO_ADDR, host_alloc, host_w32);
    const uint32_t ctrl = (uint32_t)(uintptr_t)cellGcmGetControlRegister();

    /* Stale data well below the ring: old fences a linear sweep would find. */
    for (u32 i = 0; i < N_STALE; i++)
        (void)put_method(STALE + i * 16u, 0x0050u, 0xBAD00000u + i);

    /* Frame 1: the previous frame's tail jumps to A; A's frame ends in a
     * JUMP to B; put stops past B's header. */
    (void)put_jump(PREV_TAIL, BLK_A);
    u32 io = put_header(BLK_A, 0x11u);
    io = put_method(io, 0x1FCCu, 0x6Fu);
    (void)put_jump(io, BLK_B);
    const u32 b_put = put_header(BLK_B, 0x22u);

    s_fifo_getoff = PREV_TAIL;
    vm_write32(ctrl + 4, PREV_TAIL);
    run(ctrl, b_put);
    check(s_fifo_getoff == b_put, "frame 1 walks A and stops past B's header");
    check(vm_read32(GCM_LABEL_GUEST_BASE + 0x400u) == 0x22u, "B's header release reached the label");

    /* Frame 2: B's frame starts with a JUMP into low IO, where a method is
     * followed by a word that branches to unmapped IO. A gets its next header
     * and put stops past it. */
    (void)put_jump(b_put, LOW);
    io = put_method(LOW, 0x1FCCu, 0x6Fu);
    gw32(IO_ADDR + io, 0x3F800000u);                    /* float 1.0: JUMP 0x1F800000 */
    const u32 a_put = put_header(BLK_A, 0x33u);

    const u32 pushed_before = s_ref_qtail;
    run(ctrl, a_put);
    check(s_fifo_getoff == a_put, "the bad branch resyncs get to put");
    check(vm_read32(GCM_LABEL_GUEST_BASE + 0x400u) == 0x33u,
          "the release in A's header, skipped by the resync, was replayed");
    check(s_ref_qtail == pushed_before,
          "no stale SET_REFERENCE from below the ring was queued");

    printf("[gcm_resync_excursion] %s (%d failed)\n", fails ? "FAIL" : "PASS", fails);
    free(vm_base);
    return fails ? 1 : 0;
}
