/*
 * ps3recomp - HLSL -> SPIR-V through glslang, for the Vulkan backend
 *
 * See rsx_shader_spirv.h for the binding contract. The front end is driven
 * exactly as rsx_shader_msl.cpp drives it (C++ API for setEntryPoint, Vulkan
 * rules, auto-mapped bindings and locations, then mapIO, legalized SPIR-V),
 * with three differences, all because the output goes to a Vulkan driver
 * rather than to spirv-cross:
 *
 *   - Vulkan 1.0 client, SPIR-V 1.0: the backend creates a 1.0 instance and
 *     promises to run on 1.0-level drivers. (The MSL path targets 1.3 because
 *     that is spirv-cross's floor, not a driver's.)
 *   - Texture and sampler registers are shifted apart (setShiftBinding),
 *     because Vulkan has one binding namespace per set.
 *   - The vertex stage flips Y (setInvertY): D3D clip space is +y up.
 *
 * Translations are cached. glslang takes 20-150 ms per shader, on the thread
 * that walks the FIFO, and the engine asks for the same HLSL again for every
 * pipeline that shares a program (Twisted Metal: 236 translations in its
 * first minute, 74 distinct). The cache key is the stage plus the whole HLSL
 * text, so a hit returns exactly the words that text translated to. With
 * PS3RECOMP_SPIRV_CACHE=<existing directory> the cache is also kept on disk,
 * one file per shader, and the next run starts warm. A file whose key,
 * glslang version or translator build differs from the request is a miss,
 * and is rewritten.
 */

#include "rsx_shader_spirv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <string>

#if defined(PS3RECOMP_HAVE_SPIRV_TRANSLATION)

#include <glslang/Public/ShaderLang.h>
#include <glslang/Public/ResourceLimits.h>
#if __has_include(<glslang/SPIRV/GlslangToSpv.h>)
#include <glslang/SPIRV/GlslangToSpv.h>     /* installed package layout */
#else
#include <SPIRV/GlslangToSpv.h>             /* glslang source tree (Android/Windows builds) */
#endif

#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <stdlib.h>

namespace {

void put_log(char* log, u32 log_size, const char* what, const char* detail)
{
    if (!log || log_size == 0) return;
    snprintf(log, log_size, "%s%s%s", what ? what : "",
             (what && detail && *detail) ? ": " : "", detail ? detail : "");
}

bool glslang_ready()
{
    static bool inited = false;
    if (!inited) {
        if (!glslang::InitializeProcess()) return false;
        inited = true;
    }
    return true;
}

int translate(const char* hlsl, int stage, std::vector<unsigned int>& spirv,
              char* log, u32 log_size)
{
    if (!glslang_ready()) {
        put_log(log, log_size, "glslang::InitializeProcess failed", nullptr);
        return -1;
    }

    const EShLanguage lang = (stage == RSX_SHADER_STAGE_VERTEX) ? EShLangVertex
                                                                : EShLangFragment;
    const EShMessages msgs =
        (EShMessages)(EShMsgSpvRules | EShMsgVulkanRules | EShMsgReadHlsl);

    glslang::TShader shader(lang);
    const char* strings[1] = { hlsl };
    shader.setStrings(strings, 1);
    shader.setEnvInput(glslang::EShSourceHlsl, lang, glslang::EShClientVulkan, 100);
    shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_0);
    shader.setEnvTarget(glslang::EShTargetSpv, glslang::EShTargetSpv_1_0);
    /* SPIR-V FragCoord.w is 1/w (FXC's SV_POSITION.w is w): tells the FP
     * decompiler's _rsx_inv_w helper which one it gets. */
    shader.setPreamble("#define RSX_FRAGCOORD_W_IS_RCP 1\n");
    shader.setEntryPoint("main");
    shader.setAutoMapBindings(true);
    shader.setAutoMapLocations(true);
    shader.setShiftBinding(glslang::EResTexture, RSX_SPIRV_TEXTURE_BINDING);
    shader.setShiftBinding(glslang::EResSampler, RSX_SPIRV_SAMPLER_BINDING);
    if (lang == EShLangVertex) shader.setInvertY(true);

    if (!shader.parse(GetDefaultResources(), 100, false, msgs)) {
        put_log(log, log_size, "HLSL parse", shader.getInfoLog());
        /* RSX_HLSL_FAIL_DUMP=<dir>: keep the source that did not parse
         * (hlsl_fail_<n>_<vs|fs>.hlsl, first 16), so a translator bug can be
         * read rather than inferred from a line number. */
        static int s_n = 0;
        const char* dir = getenv("RSX_HLSL_FAIL_DUMP");
        if (dir && *dir && s_n < 16) {
            char path[1024];
            snprintf(path, sizeof path, "%s/hlsl_fail_%02d_%s.hlsl", dir, s_n++,
                     stage == RSX_SHADER_STAGE_VERTEX ? "vs" : "fs");
            if (FILE* f = fopen(path, "wb")) { fwrite(hlsl, 1, strlen(hlsl), f); fclose(f); }
        }
        return -1;
    }
    glslang::TProgram program;
    program.addShader(&shader);
    if (!program.link(msgs)) {
        put_log(log, log_size, "HLSL link", program.getInfoLog());
        return -1;
    }
    if (!program.mapIO()) {
        put_log(log, log_size, "HLSL io map", program.getInfoLog());
        return -1;
    }

    spv::SpvBuildLogger logger;
    glslang::SpvOptions opts;
    opts.disableOptimizer = false;     /* run the HLSL legalization passes */
    glslang::GlslangToSpv(*program.getIntermediate(lang), spirv, &logger, &opts);
    if (spirv.empty()) {
        put_log(log, log_size, "SPIR-V generation", logger.getAllMessages().c_str());
        return -1;
    }
    return 0;
}

/* ---- translation cache ------------------------------------------------- */

const u32 kCacheFormat = 1;
const u32 kCacheMagic = 0x56505352u;   /* "RSPV" */
/* This file's compile time stands for translate()'s options and the binding
 * contract in rsx_shader_spirv.h: when either changes, this file is rebuilt,
 * and the files an older build wrote become misses. */
/* racacit: a fixed tag, not __DATE__ __TIME__ (the port builds with
 * -Werror=date-time); bump it when translate() or the contract changes. */
const char kTranslatorBuild[24] = "racacit-v0.18-spv-1";

struct Cache {
    std::mutex lock;
    std::unordered_map<std::string, std::vector<unsigned int>> map;
    std::string dir;
    bool dir_read = false;
    RsxSpirvCacheStats stats = {};
};

Cache& cache()
{
    static Cache c;
    return c;
}

const std::string& cache_dir(Cache& c)
{
    if (!c.dir_read) {
        const char* e = getenv("PS3RECOMP_SPIRV_CACHE");
        c.dir = (e && *e) ? e : "";
        c.dir_read = true;
    }
    return c.dir;
}

/* The key is the stage, then the HLSL text. */
std::string cache_key(const char* hlsl, int stage)
{
    std::string key(1, (char)('0' + stage));
    key += hlsl;
    return key;
}

std::string cache_path(const std::string& dir, const std::string& key)
{
    unsigned long long h = 0xcbf29ce484222325ull;       /* FNV-1a 64 */
    for (unsigned char ch : key) { h ^= ch; h *= 0x100000001b3ull; }
    char name[40];
    snprintf(name, sizeof name, "/%016llx.spv", h);
    return dir + name;
}

/* Header: magic, format, glslang major/minor/patch, translator build (six
 * words), key bytes, word count; then the key and the words. */
const int kHeaderWords = 13;

void cache_header(u32 hdr[kHeaderWords], const std::string& key, size_t nwords)
{
    const glslang::Version v = glslang::GetVersion();
    hdr[0] = kCacheMagic; hdr[1] = kCacheFormat;
    hdr[2] = (u32)v.major; hdr[3] = (u32)v.minor; hdr[4] = (u32)v.patch;
    memcpy(&hdr[5], kTranslatorBuild, sizeof kTranslatorBuild);
    hdr[11] = (u32)key.size(); hdr[12] = (u32)nwords;
}

bool cache_load(const std::string& path, const std::string& key,
                std::vector<unsigned int>& spirv)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    u32 want[kHeaderWords], hdr[kHeaderWords];
    cache_header(want, key, 0);
    bool ok = fread(hdr, sizeof hdr, 1, f) == 1 &&
              memcmp(hdr, want, (kHeaderWords - 1) * sizeof(u32)) == 0 &&
              hdr[kHeaderWords - 1] != 0 && hdr[kHeaderWords - 1] <= (1u << 24);
    if (ok) {
        std::string stored(key.size(), '\0');
        ok = fread(&stored[0], 1, stored.size(), f) == stored.size() && stored == key;
    }
    if (ok) {
        spirv.resize(hdr[kHeaderWords - 1]);
        ok = fread(spirv.data(), sizeof(u32), spirv.size(), f) == spirv.size();
    }
    fclose(f);
    if (!ok) spirv.clear();
    return ok;
}

/* Written under a temporary name and renamed, so a reader never sees half a file. */
void cache_store(const std::string& path, const std::string& key,
                 const std::vector<unsigned int>& spirv)
{
    char suffix[48];
    snprintf(suffix, sizeof suffix, ".%zx.tmp", std::hash<std::thread::id>{}(std::this_thread::get_id()));
    const std::string tmp = path + suffix;
    FILE* f = fopen(tmp.c_str(), "wb");
    if (!f) return;
    u32 hdr[kHeaderWords];
    cache_header(hdr, key, spirv.size());
    bool ok = fwrite(hdr, sizeof hdr, 1, f) == 1 &&
              fwrite(key.data(), 1, key.size(), f) == key.size() &&
              fwrite(spirv.data(), sizeof(u32), spirv.size(), f) == spirv.size();
    ok = (fclose(f) == 0) && ok;
#ifdef _WIN32
    if (ok) remove(path.c_str());      /* rename does not replace on Windows */
#endif
    if (!ok || rename(tmp.c_str(), path.c_str()) != 0) remove(tmp.c_str());
}

} // namespace

extern "C" int rsx_hlsl_to_spirv_available(void) { return 1; }

static int hlsl_to_spirv_upstream(const char* hlsl, int stage,
                                  u32* words, u32 max_words, u32* out_words,
                                  char* log, u32 log_size)
{
    if (out_words) *out_words = 0;
    if (!hlsl || !words || !out_words || max_words == 0) {
        put_log(log, log_size, "bad arguments", nullptr);
        return -1;
    }
    if (stage != RSX_SHADER_STAGE_VERTEX && stage != RSX_SHADER_STAGE_FRAGMENT) {
        put_log(log, log_size, "unknown shader stage", nullptr);
        return -1;
    }

    /* One lock around the whole lookup and translation: glslang's process
     * state is not documented as thread-safe either. */
    Cache& c = cache();
    std::lock_guard<std::mutex> guard(c.lock);
    const std::string key = cache_key(hlsl, stage);
    auto it = c.map.find(key);
    if (it != c.map.end()) {
        c.stats.memory_hits++;
    } else {
        std::vector<unsigned int> spirv;
        const std::string& dir = cache_dir(c);
        const std::string path = dir.empty() ? std::string() : cache_path(dir, key);
        if (!dir.empty() && cache_load(path, key, spirv)) {
            c.stats.disk_hits++;
        } else {
            if (translate(hlsl, stage, spirv, log, log_size)) return -1;
            c.stats.translations++;
            if (!dir.empty()) cache_store(path, key, spirv);
        }
        it = c.map.emplace(key, std::move(spirv)).first;
    }
    const std::vector<unsigned int>& spirv = it->second;
    if (spirv.size() > max_words) {
        char n[64];
        snprintf(n, sizeof n, "%zu words, room for %u", spirv.size(), max_words);
        put_log(log, log_size, "SPIR-V output too large", n);
        return -1;
    }
    memcpy(words, spirv.data(), spirv.size() * sizeof(u32));
    *out_words = (u32)spirv.size();
    return 0;
}

/* SPIR-V cache. A translation costs several milliseconds of the render
 * thread, and a level brings hundreds of programs the moment new scenery
 * comes into view: a stall of seconds. Results are kept in memory per run
 * and, with RSX_SHADER_CACHE=<file>, on disk across runs, keyed by the HLSL
 * text and stage (two 64-bit hashes) plus the options above (the salt: bump
 * it whenever hlsl_to_spirv_compile drives glslang differently).
 *
 * File: "RSXSPV01", then records { u64 k1, u64 k2, u32 words, u32 check,
 * u32 spirv[words] }. A torn tail (the process killed mid-append) is cut off
 * by rewriting the file from the records before it.
 *
 * racacit: only an INDEX (key -> file offset, ~24 bytes a program) stays in
 * memory; a hit reads its words from the file (microseconds from the OS
 * cache, against milliseconds for a translation). The whole cache used to be
 * loaded into RAM and kept -- 98 MB in a dev run/ folder -- and the file only
 * ever grew. Over RSX_SHADER_CACHE_MB (default 192: a full playthrough must
 * fit, or every start would drop and re-translate a level's programs) at load, the file is
 * rewritten with its newest records up to half that; anything dropped is
 * translated again once if a level still needs it. Without a cache file the
 * words are kept in memory as before. */
namespace {

/* v2: ps3recomp v0.18 translate() and binding contract */
const char kSalt[] = "hlsl->spv glslang vk1.0 spv1.0 invertY shifted legalized v2";
const char kMagic[8] = { 'R', 'S', 'X', 'S', 'P', 'V', '0', '1' };

struct Key { u64 a, b; bool operator==(const Key& o) const { return a == o.a && b == o.b; } };
struct KeyHash { size_t operator()(const Key& k) const { return (size_t)(k.a ^ (k.b >> 7)); } };

struct Rec { long long off; u32 n; };   /* words at file offset off (+24 header) */
std::unordered_map<Key, std::vector<u32>, KeyHash> s_cache;   /* no file: words in memory */
std::unordered_map<Key, Rec, KeyHash> s_index;                /* file: where they are */
FILE* s_file;                 /* "a+b": appends at the end, reads anywhere */
bool s_loaded;

u64 fnv(const void* p, size_t n, u64 h)
{
    const unsigned char* b = (const unsigned char*)p;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ull; }
    return h;
}

Key make_key(const char* hlsl, int stage)
{
    const size_t n = strlen(hlsl);
    Key k;
    k.a = fnv(kSalt, sizeof kSalt, 1469598103934665603ull);
    k.a = fnv(&stage, sizeof stage, k.a);
    k.a = fnv(hlsl, n, k.a);
    /* Second hash: other basis, and the text walked with its length mixed
     * in, so a collision needs both to agree. */
    k.b = fnv(&n, sizeof n, 0x84222325CBF29CE4ull ^ (u64)stage);
    k.b = fnv(hlsl, n, k.b);
    k.b = fnv(kSalt, sizeof kSalt, k.b);
    return k;
}

void put_record(FILE* f, const Key& k, const u32* w, u32 n)
{
    const u32 check = n ^ 0x5350565Au;
    fwrite(&k.a, 8, 1, f); fwrite(&k.b, 8, 1, f);
    fwrite(&n, 4, 1, f); fwrite(&check, 4, 1, f);
    fwrite(w, 4, n, f);
}

void cache_load()
{
    s_loaded = true;
    const char* path = getenv("RSX_SHADER_CACHE");
    if (!path || !*path) return;
    const char* mb = getenv("RSX_SHADER_CACHE_MB");
    const long long cap = (long long)(mb && atoi(mb) > 0 ? atoi(mb) : 192) << 20;
    /* Pass 1: index the valid records (a torn tail ends the file). */
    bool torn = false, fresh = true;
    long long end = 8;
    std::vector<std::pair<Key, Rec>> recs;
    if (FILE* f = fopen(path, "rb")) {
        char m[8];
        if (fread(m, 1, 8, f) == 8 && !memcmp(m, kMagic, 8)) {
            fresh = false;
            for (;;) {
                Key k; u32 n, check;
                size_t got = fread(&k.a, 8, 1, f) + fread(&k.b, 8, 1, f) +
                             fread(&n, 4, 1, f) + fread(&check, 4, 1, f);
                if (got == 0 && feof(f)) break;
                if (got != 4 || check != (n ^ 0x5350565Au) || n == 0 || n > (64u << 20) / 4u) { torn = true; break; }
                if (fseek(f, (long)n * 4L, SEEK_CUR) != 0) { torn = true; break; }
                recs.push_back({ k, Rec{ end, n } });
                end += 24 + (long long)n * 4;
            }
            if (!torn && ftell(f) != (long)end) torn = true;
        }
        fclose(f);
    }
    const bool over = end > cap;
    if (!fresh && (torn || over)) {
        /* Rewrite: the valid records, or only the newest ones (the file is in
         * append order) up to half the cap, through a temporary file. */
        size_t first = 0;
        if (over) {
            long long keep = 0;
            first = recs.size();
            while (first > 0 && keep + 24 + (long long)recs[first - 1].second.n * 4 <= cap / 2) {
                keep += 24 + (long long)recs[first - 1].second.n * 4; first--;
            }
        }
        std::string tmp = std::string(path) + ".tmp";
        FILE* in = fopen(path, "rb");
        FILE* out = in ? fopen(tmp.c_str(), "wb") : nullptr;
        bool ok = out != nullptr;
        if (ok) {
            fwrite(kMagic, 1, 8, out);
            std::vector<u32> w;
            for (size_t i = first; ok && i < recs.size(); i++) {
                w.resize(recs[i].second.n);
                ok = fseek(in, (long)(recs[i].second.off + 24), SEEK_SET) == 0 &&
                     fread(w.data(), 4, w.size(), in) == w.size();
                if (ok) put_record(out, recs[i].first, w.data(), (u32)w.size());
            }
            ok = fclose(out) == 0 && ok;
        }
        if (in) fclose(in);
        if (ok && (remove(path) == 0 || errno == ENOENT) && rename(tmp.c_str(), path) == 0) {
            fprintf(stderr, "[spirv-cache] %s: rewrote %s, kept %zu of %zu programs%c", path,
                    torn ? "(torn tail)" : "(over the cap)", recs.size() - first, recs.size(), 10);
            recs.erase(recs.begin(), recs.begin() + (long)first);
            long long off = 8;
            for (auto& r : recs) { r.second.off = off; off += 24 + (long long)r.second.n * 4; }
        } else {
            remove(tmp.c_str());
            recs.clear(); fresh = true;   /* could not rewrite: start a new file */
        }
    }
    if (fresh) {
        s_file = fopen(path, "w+b");
        if (s_file) { fwrite(kMagic, 1, 8, s_file); fflush(s_file); }
        recs.clear();
    } else {
        s_file = fopen(path, "a+b");
    }
    if (s_file) for (const auto& r : recs) s_index.emplace(r.first, r.second);
    fprintf(stderr, "[spirv-cache] %s: %zu programs indexed%s%c", path, s_index.size(),
            s_file ? "" : " (cannot open: in-memory only)", 10);
}

/* The words of an indexed program; false when the file read fails. */
bool cache_read(const Rec& r, u32* words)
{
    if (!s_file) return false;
    fflush(s_file);   /* switching from appending to reading */
    if (fseek(s_file, (long)(r.off + 24), SEEK_SET) != 0) return false;
    return fread(words, 4, r.n, s_file) == r.n;
}

bool file_cache_on()
{
    static int on = -1;
    if (on < 0) { const char* p = getenv("RSX_SHADER_CACHE"); on = (p && *p) ? 1 : 0; }
    return on != 0;
}

} // namespace

/* racacit: with RSX_SHADER_CACHE=<file> (the launcher's shadercache.bin) the
 * file cache above answers, and upstream's per-shader memory map and
 * PS3RECOMP_SPIRV_CACHE directory are not used: they would keep every
 * program's words in RAM again. Without it, upstream's cache runs as is.
 * Hits count as disk_hits, translations as translations. */
extern "C" int rsx_hlsl_to_spirv(const char* hlsl, int stage,
                                 u32* words, u32 max_words, u32* out_words,
                                 char* log, u32 log_size)
{
    if (!file_cache_on())
        return hlsl_to_spirv_upstream(hlsl, stage, words, max_words, out_words, log, log_size);
    if (out_words) *out_words = 0;
    if (!hlsl || !words || !out_words || max_words == 0) {
        put_log(log, log_size, "bad arguments", nullptr);
        return -1;
    }
    if (stage != RSX_SHADER_STAGE_VERTEX && stage != RSX_SHADER_STAGE_FRAGMENT) {
        put_log(log, log_size, "unknown shader stage", nullptr);
        return -1;
    }
    const Key k = make_key(hlsl, stage);
    /* One lock around lookup and translation, as upstream: glslang's process
     * state is not documented as thread-safe. */
    Cache& c = cache();
    std::lock_guard<std::mutex> g(c.lock);
    if (!s_loaded) cache_load();
    auto ix = s_index.find(k);
    if (ix != s_index.end() && ix->second.n <= max_words && cache_read(ix->second, words)) {
        *out_words = ix->second.n;
        c.stats.disk_hits++;
        return 0;
    }
    auto it = s_cache.find(k);
    if (it != s_cache.end() && it->second.size() <= max_words) {
        memcpy(words, it->second.data(), it->second.size() * 4u);
        *out_words = (u32)it->second.size();
        c.stats.memory_hits++;
        return 0;
    }
    std::vector<unsigned int> spirv;
    if (translate(hlsl, stage, spirv, log, log_size)) return -1;
    c.stats.translations++;
    if (spirv.size() > max_words) {
        char n[64];
        snprintf(n, sizeof n, "%zu words, room for %u", spirv.size(), max_words);
        put_log(log, log_size, "SPIR-V output too large", n);
        return -1;
    }
    memcpy(words, spirv.data(), spirv.size() * sizeof(u32));
    *out_words = (u32)spirv.size();
    if (s_file) {
        if (s_index.find(k) == s_index.end()) {
            fflush(s_file);
            fseek(s_file, 0, SEEK_END);           /* "a+" appends anyway; this gives the offset */
            const long long off = (long long)ftell(s_file);
            put_record(s_file, k, words, *out_words); fflush(s_file);
            if (off >= 8) s_index.emplace(k, Rec{ off, *out_words });
        }
    } else {
        s_cache.emplace(k, std::vector<u32>(words, words + *out_words));
    }
    return 0;
}

extern "C" void rsx_hlsl_to_spirv_cache_stats(RsxSpirvCacheStats* out)
{
    Cache& c = cache();
    std::lock_guard<std::mutex> guard(c.lock);
    if (out) *out = c.stats;
}

extern "C" void rsx_hlsl_to_spirv_cache_reset(void)
{
    Cache& c = cache();
    std::lock_guard<std::mutex> guard(c.lock);
    c.map.clear();
    c.dir_read = false;
    c.stats = {};
}

#else /* !PS3RECOMP_HAVE_SPIRV_TRANSLATION */

extern "C" int rsx_hlsl_to_spirv_available(void) { return 0; }

extern "C" int rsx_hlsl_to_spirv(const char* hlsl, int stage,
                                 u32* words, u32 max_words, u32* out_words,
                                 char* log, u32 log_size)
{
    (void)hlsl; (void)stage; (void)words; (void)max_words;
    if (out_words) *out_words = 0;
    if (log && log_size)
        snprintf(log, log_size, "HLSL -> SPIR-V translation not built (glslang not found)");
    return -1;
}

extern "C" void rsx_hlsl_to_spirv_cache_stats(RsxSpirvCacheStats* out)
{
    if (out) memset(out, 0, sizeof *out);
}

extern "C" void rsx_hlsl_to_spirv_cache_reset(void) {}

#endif

extern "C" void rsx_hlsl_to_spirv_cache_counts(unsigned long* hits, unsigned long* misses)
{
    RsxSpirvCacheStats s;
    rsx_hlsl_to_spirv_cache_stats(&s);
    if (hits) *hits = (unsigned long)s.memory_hits + (unsigned long)s.disk_hits;
    if (misses) *misses = (unsigned long)s.translations;
}

