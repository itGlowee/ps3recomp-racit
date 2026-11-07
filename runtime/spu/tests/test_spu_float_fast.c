/* racacit SPU_FLOAT_FAST (RACiT's float model, spu_helpers.h): spu_fa/fs/fm/fma/fms/fnms and
 * spu_fcgt/fceq, run under the SPU threads' MXCSR (DAZ + FTZ + round toward zero), against the
 * exact references (spu_*_ref, run under the default MXCSR they assume) on every kind of input:
 * ordinary, zeros and denormals, the top of the range and exponent 255. Reports how many lanes
 * are identical, how many one ulp apart (the model's double rounding in fa/fs/fma), and any that
 * differ by more -- those fail. Prints a speed comparison too. Build on x86:
 *   cc -O2 -mavx2 -mfma -DSPU_FLOAT_FAST=1 -I.. -I../.. -I../../.. -I../../../include \
 *      test_spu_float_fast.c -o t && ./t */
#include "../spu_helpers.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <xmmintrin.h>
#if defined(SPU_FLOAT_FAST)
volatile long g_spu_xf_exact[8];   /* spu_drain.c in the runtime */
int g_spu_xf_stats_on;
#endif

static uint64_t s_rng = 0x9E3779B97F4A7C15ull;
static uint32_t rnd32(void)
{
    s_rng ^= s_rng << 13; s_rng ^= s_rng >> 7; s_rng ^= s_rng << 17;
    return (uint32_t)(s_rng >> 16);
}
static uint32_t rnd_float(void)
{
    uint32_t sign = (rnd32() & 1u) << 31, man = rnd32() & 0x7FFFFFu, e;
    switch (rnd32() % 10) {
    case 0: e = 0; break;                              /* zero / denormal */
    case 1: e = 1 + rnd32() % 4; break;                /* bottom of normal range */
    case 2: e = 248 + rnd32() % 8; break;              /* top, into extended range */
    case 3: return rnd32();                            /* anything */
    case 4: return sign;                               /* +-0 */
    default: e = 110 + rnd32() % 36; break;            /* ordinary */
    }
    return sign | (e << 23) | man;
}

static long s_n, s_same, s_ulp1, s_bad;
static int ok_pair(uint32_t got, uint32_t want)
{
    if (got == want) { s_same++; return 1; }
    if (!((got | want) & 0x7FFFFFFFu)) { s_same++; return 1; }   /* +0 / -0: equal to the SPU */
    if ((got ^ want) & 0x80000000u) return 0;
    const uint32_t g = got & 0x7FFFFFFFu, w = want & 0x7FFFFFFFu;
    if ((g > w ? g - w : w - g) == 1) { s_ulp1++; return 1; }
    return 0;
}
static void check(const char* op, u128 got, u128 want, u128 a, u128 b, u128 c)
{
    for (int i = 0; i < 4; i++) {
        s_n++;
        if (ok_pair(got._u32[i], want._u32[i])) continue;
        if (s_bad++ < 20)
            printf("%s lane %d: %08X %08X %08X -> got %08X want %08X\n", op, i,
                   a._u32[i], b._u32[i], c._u32[i], got._u32[i], want._u32[i]);
    }
}
static void check_cmp(const char* op, u128 got, u128 want, u128 a, u128 b)
{
    for (int i = 0; i < 4; i++) {
        s_n++;
        if (got._u32[i] == want._u32[i]) { s_same++; continue; }
        if (s_bad++ < 20) printf("%s lane %d: %08X %08X -> got %08X want %08X\n", op, i, a._u32[i], b._u32[i], got._u32[i], want._u32[i]);
    }
}

int main(void)
{
#if !defined(SPU_FLOAT_FAST)
    puts("test_spu_float_fast: SPU_FLOAT_FAST not defined, nothing to compare");
    return 0;
#else
    const unsigned csr0 = _mm_getcsr(), csr_spu = csr0 | 0xE040u;
    const long N = 1000000;
    for (long it = 0; it < N; it++) {
        u128 a, b, c, w[8], g[8];
        for (int i = 0; i < 4; i++) { a._u32[i] = rnd_float(); b._u32[i] = rnd_float(); c._u32[i] = rnd_float(); }
        _mm_setcsr(csr0);
        w[0] = spu_fa_ref(a, b); w[1] = spu_fs_ref(a, b); w[2] = spu_fm_ref(a, b);
        w[3] = spu_fma_ref(a, b, c); w[4] = spu_fms_ref(a, b, c); w[5] = spu_fnms_ref(a, b, c);
        w[6] = spu__exact_fcgt(a, b); w[7] = spu__exact_fceq(a, b);
        _mm_setcsr(csr_spu);
        g[0] = spu_fa(a, b); g[1] = spu_fs(a, b); g[2] = spu_fm(a, b);
        g[3] = spu_fma(a, b, c); g[4] = spu_fms(a, b, c); g[5] = spu_fnms(a, b, c);
        g[6] = spu_fcgt(a, b); g[7] = spu_fceq(a, b);
        _mm_setcsr(csr0);
        static const char* nm[6] = { "fa", "fs", "fm", "fma", "fms", "fnms" };
        for (int k = 0; k < 6; k++) check(nm[k], g[k], w[k], a, b, c);
        check_cmp("fcgt", g[6], w[6], a, b); check_cmp("fceq", g[7], w[7], a, b);
    }
    printf("lanes %ld: identical %ld (%.3f%%), 1 ulp %ld, WRONG %ld\n",
           s_n, s_same, 100.0 * s_same / s_n, s_ulp1, s_bad);
    u128 x, y, z; for (int i = 0; i < 4; i++) { x._f32[i] = 1.0f + i; y._f32[i] = 0.999f; z._f32[i] = 0.5f; }
    _mm_setcsr(csr_spu);
    clock_t t0 = clock(); u128 r = x;
    for (long i = 0; i < 50000000; i++) r = spu_fma(r, y, z);
    _mm_setcsr(csr0);
    clock_t t1 = clock(); u128 q = x;
    for (long i = 0; i < 50000000; i++) q = spu__exact_fma(q, y, z);
    clock_t t2 = clock();
    printf("fma x50M: RACiT model %.0f ms, upstream exact %.0f ms (%08X %08X)\n",
           (t1 - t0) * 1000.0 / CLOCKS_PER_SEC, (t2 - t1) * 1000.0 / CLOCKS_PER_SEC, r._u32[0], q._u32[0]);
    return s_bad ? 1 : 0;
#endif
}
