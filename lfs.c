/*
 * The little filesystem
 *
 * Copyright (c) 2022, The littlefs authors.
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "lfs.h"
#include "lfs_util.h"


// some constants used throughout the code
#define LFS_BLOCK_NULL ((lfs_block_t)-1)
#define LFS_BLOCK_INLINE ((lfs_block_t)-2)

enum {
    LFS_OK_RELOCATED = 1,
    LFS_OK_DROPPED   = 2,
    LFS_OK_ORPHANED  = 3,
};

enum {
    LFS_CMP_EQ = 0,
    LFS_CMP_LT = 1,
    LFS_CMP_GT = 2,
};


/// Caching block device operations ///

static inline void lfs_cache_drop(lfs_t *lfs, lfs_cache_t *rcache) {
    // do not zero, cheaper if cache is readonly or only going to be
    // written with identical data (during relocates)
    (void)lfs;
    rcache->block = LFS_BLOCK_NULL;
}

static inline void lfs_cache_zero(lfs_t *lfs, lfs_cache_t *pcache) {
    // zero to avoid information leak
    memset(pcache->buffer, 0xff, lfs->cfg->cache_size);
    pcache->block = LFS_BLOCK_NULL;
}

static int lfs_bd_read(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache, lfs_size_t hint,
        lfs_block_t block, lfs_off_t off,
        void *buffer, lfs_size_t size) {
    uint8_t *data = buffer;
    if (block >= lfs->cfg->block_count ||
            off+size > lfs->cfg->block_size) {
        return LFS_ERR_CORRUPT;
    }

    while (size > 0) {
        lfs_size_t diff = size;

        if (pcache && block == pcache->block &&
                off < pcache->off + pcache->size) {
            if (off >= pcache->off) {
                // is already in pcache?
                diff = lfs_min(diff, pcache->size - (off-pcache->off));
                memcpy(data, &pcache->buffer[off-pcache->off], diff);

                data += diff;
                off += diff;
                size -= diff;
                continue;
            }

            // pcache takes priority
            diff = lfs_min(diff, pcache->off-off);
        }

        if (block == rcache->block &&
                off < rcache->off + rcache->size) {
            if (off >= rcache->off) {
                // is already in rcache?
                diff = lfs_min(diff, rcache->size - (off-rcache->off));
                memcpy(data, &rcache->buffer[off-rcache->off], diff);

                data += diff;
                off += diff;
                size -= diff;
                continue;
            }

            // rcache takes priority
            diff = lfs_min(diff, rcache->off-off);
        }

        if (size >= hint && off % lfs->cfg->read_size == 0 &&
                size >= lfs->cfg->read_size) {
            // bypass cache?
            diff = lfs_aligndown(diff, lfs->cfg->read_size);
            int err = lfs->cfg->read(lfs->cfg, block, off, data, diff);
            if (err) {
                return err;
            }

            data += diff;
            off += diff;
            size -= diff;
            continue;
        }

        // load to cache, first condition can no longer fail
        LFS_ASSERT(block < lfs->cfg->block_count);
        rcache->block = block;
        rcache->off = lfs_aligndown(off, lfs->cfg->read_size);
        rcache->size = lfs_min(
                lfs_min(
                    lfs_alignup(off+hint, lfs->cfg->read_size),
                    lfs->cfg->block_size)
                - rcache->off,
                lfs->cfg->cache_size);
        int err = lfs->cfg->read(lfs->cfg, rcache->block,
                rcache->off, rcache->buffer, rcache->size);
        LFS_ASSERT(err <= 0);
        if (err) {
            return err;
        }
    }

    return 0;
}

static int lfs_bd_cmp(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache, lfs_size_t hint,
        lfs_block_t block, lfs_off_t off,
        const void *buffer, lfs_size_t size) {
    const uint8_t *data = buffer;
    lfs_size_t diff = 0;

    for (lfs_off_t i = 0; i < size; i += diff) {
        uint8_t dat[8];

        diff = lfs_min(size-i, sizeof(dat));
        int err = lfs_bd_read(lfs,
                pcache, rcache, hint-i,
                block, off+i, &dat, diff);
        if (err) {
            return err;
        }

        int res = memcmp(dat, data + i, diff);
        if (res) {
            return res < 0 ? LFS_CMP_LT : LFS_CMP_GT;
        }
    }

    return LFS_CMP_EQ;
}

static int lfs_bd_crc(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache, lfs_size_t hint,
        lfs_block_t block, lfs_off_t off, lfs_size_t size, uint32_t *crc) {
    lfs_size_t diff = 0;

    for (lfs_off_t i = 0; i < size; i += diff) {
        uint8_t dat[8];
        diff = lfs_min(size-i, sizeof(dat));
        int err = lfs_bd_read(lfs,
                pcache, rcache, hint-i,
                block, off+i, &dat, diff);
        if (err) {
            return err;
        }

        *crc = lfs_crc(*crc, &dat, diff);
    }

    return 0;
}

static int lfs_bd_crc32c(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache, lfs_size_t hint,
        lfs_block_t block, lfs_off_t off, lfs_size_t size, uint32_t *crc) {
    lfs_size_t diff = 0;

    for (lfs_off_t i = 0; i < size; i += diff) {
        uint8_t dat[8];
        diff = lfs_min(size-i, sizeof(dat));
        int err = lfs_bd_read(lfs,
                pcache, rcache, hint-i,
                block, off+i, &dat, diff);
        if (err) {
            return err;
        }

        *crc = lfs_crc32c(*crc, &dat, diff);
    }

    return 0;
}

#ifndef LFS_READONLY
static int lfs_bd_flush(lfs_t *lfs,
        lfs_cache_t *pcache, lfs_cache_t *rcache, bool validate) {
    if (pcache->block != LFS_BLOCK_NULL && pcache->block != LFS_BLOCK_INLINE) {
        LFS_ASSERT(pcache->block < lfs->cfg->block_count);
        lfs_size_t diff = lfs_alignup(pcache->size, lfs->cfg->prog_size);
        int err = lfs->cfg->prog(lfs->cfg, pcache->block,
                pcache->off, pcache->buffer, diff);
        LFS_ASSERT(err <= 0);
        if (err) {
            return err;
        }

        if (validate) {
            // check data on disk
            lfs_cache_drop(lfs, rcache);
            int res = lfs_bd_cmp(lfs,
                    NULL, rcache, diff,
                    pcache->block, pcache->off, pcache->buffer, diff);
            if (res < 0) {
                return res;
            }

            if (res != LFS_CMP_EQ) {
                return LFS_ERR_CORRUPT;
            }
        }

        lfs_cache_zero(lfs, pcache);
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_bd_sync(lfs_t *lfs,
        lfs_cache_t *pcache, lfs_cache_t *rcache, bool validate) {
    lfs_cache_drop(lfs, rcache);

    int err = lfs_bd_flush(lfs, pcache, rcache, validate);
    if (err) {
        return err;
    }

    err = lfs->cfg->sync(lfs->cfg);
    LFS_ASSERT(err <= 0);
    return err;
}
#endif

#ifndef LFS_READONLY
static int lfs_bd_prog(lfs_t *lfs,
        lfs_cache_t *pcache, lfs_cache_t *rcache, bool validate,
        lfs_block_t block, lfs_off_t off,
        const void *buffer, lfs_size_t size) {
    const uint8_t *data = buffer;
    LFS_ASSERT(block == LFS_BLOCK_INLINE || block < lfs->cfg->block_count);
    LFS_ASSERT(off + size <= lfs->cfg->block_size);

    // update rcache if we overlap
    if (rcache
            && block == rcache->block
            && off < rcache->off + rcache->size
            && off + size > rcache->off) {
        lfs_off_t off_ = lfs_max(off, rcache->off);
        lfs_size_t size_ = lfs_min(size, rcache->size - (off_-rcache->off));
        memcpy(&rcache->buffer[off_-rcache->off], data+(off_-off), size_);
    }

    while (size > 0) {
        if (block == pcache->block &&
                off >= pcache->off &&
                off < pcache->off + lfs->cfg->cache_size) {
            // already fits in pcache?
            lfs_size_t diff = lfs_min(size,
                    lfs->cfg->cache_size - (off-pcache->off));
            memcpy(&pcache->buffer[off-pcache->off], data, diff);

            data += diff;
            off += diff;
            size -= diff;

            pcache->size = lfs_max(pcache->size, off - pcache->off);
            if (pcache->size == lfs->cfg->cache_size) {
                // eagerly flush out pcache if we fill up
                int err = lfs_bd_flush(lfs, pcache, rcache, validate);
                if (err) {
                    return err;
                }
            }

            continue;
        }

        // pcache must have been flushed, either by programming and
        // entire block or manually flushing the pcache
        LFS_ASSERT(pcache->block == LFS_BLOCK_NULL);

        // prepare pcache, first condition can no longer fail
        pcache->block = block;
        pcache->off = lfs_aligndown(off, lfs->cfg->prog_size);
        pcache->size = 0;
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_bd_erase(lfs_t *lfs, lfs_block_t block) {
    LFS_ASSERT(block < lfs->cfg->block_count);
    int err = lfs->cfg->erase(lfs->cfg, block);
    LFS_ASSERT(err <= 0);
    return err;
}
#endif


/// Small type-level utilities ///

// operations on block pairs
static inline void lfs_pair_swap(lfs_block_t pair[2]) {
    lfs_block_t t = pair[0];
    pair[0] = pair[1];
    pair[1] = t;
}

static inline bool lfs_pair_isnull(const lfs_block_t pair[2]) {
    return pair[0] == LFS_BLOCK_NULL || pair[1] == LFS_BLOCK_NULL;
}

static inline int lfs_pair_cmp(
        const lfs_block_t paira[2],
        const lfs_block_t pairb[2]) {
    return !(paira[0] == pairb[0] || paira[1] == pairb[1] ||
             paira[0] == pairb[1] || paira[1] == pairb[0]);
}

static inline bool lfs_pair_issync(
        const lfs_block_t paira[2],
        const lfs_block_t pairb[2]) {
    return (paira[0] == pairb[0] && paira[1] == pairb[1]) ||
           (paira[0] == pairb[1] && paira[1] == pairb[0]);
}

static inline void lfs_pair_fromle32(lfs_block_t pair[2]) {
    pair[0] = lfs_fromle32(pair[0]);
    pair[1] = lfs_fromle32(pair[1]);
}

#ifndef LFS_READONLY
static inline void lfs_pair_tole32(lfs_block_t pair[2]) {
    pair[0] = lfs_tole32(pair[0]);
    pair[1] = lfs_tole32(pair[1]);
}
#endif

// operations on 32-bit entry tags
typedef uint32_t lfs_tag_t;
typedef int32_t lfs_stag_t;

#define LFS_MKTAG(type, id, size) \
    (((lfs_tag_t)(type) << 20) | ((lfs_tag_t)(id) << 10) | (lfs_tag_t)(size))

#define LFS_MKTAG_IF(cond, type, id, size) \
    ((cond) ? LFS_MKTAG(type, id, size) : LFS_MKTAG(LFS_FROM_NOOP, 0, 0))

#define LFS_MKTAG_IF_ELSE(cond, type1, id1, size1, type2, id2, size2) \
    ((cond) ? LFS_MKTAG(type1, id1, size1) : LFS_MKTAG(type2, id2, size2))

static inline bool lfs_tag_isvalid(lfs_tag_t tag) {
    return !(tag & 0x80000000);
}

static inline bool lfs_tag_isdelete(lfs_tag_t tag) {
    return ((int32_t)(tag << 22) >> 22) == -1;
}

static inline uint16_t lfs_tag_type1(lfs_tag_t tag) {
    return (tag & 0x70000000) >> 20;
}

static inline uint16_t lfs_tag_type2(lfs_tag_t tag) {
    return (tag & 0x78000000) >> 20;
}

static inline uint16_t lfs_tag_type3(lfs_tag_t tag) {
    return (tag & 0x7ff00000) >> 20;
}

static inline uint8_t lfs_tag_chunk(lfs_tag_t tag) {
    return (tag & 0x0ff00000) >> 20;
}

static inline int8_t lfs_tag_splice(lfs_tag_t tag) {
    return (int8_t)lfs_tag_chunk(tag);
}

static inline uint16_t lfs_tag_id(lfs_tag_t tag) {
    return (tag & 0x000ffc00) >> 10;
}

static inline lfs_size_t lfs_tag_size(lfs_tag_t tag) {
    return tag & 0x000003ff;
}

static inline lfs_size_t lfs_tag_dsize(lfs_tag_t tag) {
    return sizeof(tag) + lfs_tag_size(tag + lfs_tag_isdelete(tag));
}

// 32-bit metadata tags
//
// in-device, these are effectively 31-bit unsigned integers
// on-disk, these are encoded as leb128, so smaller constants are prefered
typedef uint32_t lfs_rtag_t;
typedef int32_t lfs_srtag_t;

enum lfs_rtag_type1 {
    LFS_TYPE1_CREATE    = 0x0040,
    LFS_TYPE1_CREATEREG = 0x00c0,
    LFS_TYPE1_CREATEDIR = 0x0140,
    LFS_TYPE1_DELETE    = 0x0048,
    LFS_TYPE1_STRUCT    = 0x0050,
    LFS_TYPE1_UATTR     = 0x0060,

    LFS_TYPE1_TAIL      = 0x0008,
    LFS_TYPE1_GSTATE    = 0x0010,

    LFS_TYPE1_CRC       = 0x0002,
    LFS_TYPE1_FCRC      = 0x000a,

    LFS_TYPE1_RM        = 0x0001,
    LFS_TYPE1_ALT       = 0x0004,
};

enum lfs_rtag_pat {
    LFS_PAT_GET         = 0x0000,
    LFS_PAT_FIND        = 0x0001,
};

#define LFS_ALT_B false
#define LFS_ALT_R true

#define LFS_ALT_LT false
#define LFS_ALT_GT true

#define LFS_MKRTAG_(type1, type2, id) \
    (((0x7fff & (lfs_rtag_t)(type1)) << 0) \
        | ((0xff & (lfs_rtag_t)(type2)) << 7) \
        | ((0xffff & (lfs_rtag_t)(id)) << 15))

#define LFS_MKRTAG(type1, type2, id) \
    LFS_MKRTAG_(LFS_TYPE1_##type1, type2, id)

#define LFS_MKRRMTAG_(type1, type2, id) \
    LFS_MKRTAG_(LFS_TYPE1_RM | (type1), type2, id)

#define LFS_MKRRMTAG(type1, type2, id) \
    LFS_MKRRMTAG_(LFS_TYPE1_##type1, type2, id)

#define LFS_MKRPATTAG_(pat, type1, type2, id) \
    LFS_MKRTAG_((pat) | (type1), type2, id)

#define LFS_MKRPATTAG(pat, type1, type2, id) \
    LFS_MKRPATTAG_(LFS_PAT_##pat, type1, type2, id)

#define LFS_MKRALT_(color, dir, weight) \
    (LFS_TYPE1_ALT \
        | ((0x1 & (lfs_rtag_t)(color)) << 0) \
        | ((0x1 & (lfs_rtag_t)(dir)) << 1) \
        | ((0xfffffff & (lfs_rtag_t)(weight)) << 3))

#define LFS_MKRALT(color, dir, weight) \
    LFS_MKRALT_(LFS_ALT_##color, LFS_ALT_##dir, weight)

// tag operations
static inline bool lfs_rtag_isvalid(lfs_rtag_t tag) {
    return !(tag & 0x80000000);
}

static inline lfs_rtag_t lfs_rtag_valid(lfs_rtag_t tag) {
    return tag & ~0x80000000;
}

static inline lfs_rtag_t lfs_rtag_invalid(lfs_rtag_t tag) {
    return tag | 0x80000000;
}

static inline bool lfs_rtag_isalt(lfs_rtag_t tag) {
    return tag & 0x4;
}

static inline bool lfs_rtag_intree(lfs_rtag_t tag) {
    return !(tag & 0x2);
}

static inline bool lfs_rtag_isrm(lfs_rtag_t tag) {
    return tag & 0x1;
}

static inline uint8_t lfs_rtag_type1(lfs_rtag_t tag) {
    return tag & 0x7f;
}

static inline uint8_t lfs_rtag_type2(lfs_rtag_t tag) {
    return (tag >> 7) & 0xff;
}

static inline uint16_t lfs_rtag_type3(lfs_rtag_t tag) {
    return tag & 0x7fff;
}

static inline uint16_t lfs_rtag_id(lfs_rtag_t tag) {
    return tag >> 15;
}

static inline lfs_rtag_t lfs_rtag_setid(lfs_rtag_t tag, uint16_t id) {
    return (tag & 0x7fff) | ((lfs_rtag_t)id << 15);
}

static inline lfs_rtag_t lfs_rtag_incid(lfs_rtag_t tag) {
    return tag + (1 << 15);
}

static inline lfs_rtag_t lfs_rtag_decid(lfs_rtag_t tag) {
    return tag - (1 << 15);
}

static inline lfs_rtag_t lfs_rtag_inc(lfs_rtag_t tag) {
    return tag + 0x8;
}

static inline uint8_t lfs_rtag_pat(lfs_rtag_t tag) {
    return tag & 0x7;
}

// alt operations
static inline bool lfs_rtag_isblack(lfs_rtag_t tag) {
    return !(tag & 0x1);
}

static inline bool lfs_rtag_isred(lfs_rtag_t tag) {
    return tag & 0x1;
}

static inline lfs_rtag_t lfs_rtag_red(lfs_rtag_t tag) {
    return tag | 0x1;
}

static inline lfs_rtag_t lfs_rtag_black(lfs_rtag_t tag) {
    return tag & ~0x1;
}

static inline bool lfs_rtag_islt(lfs_rtag_t tag) {
    return !(tag & 0x2);
}

static inline bool lfs_rtag_isgt(lfs_rtag_t tag) {
    return tag & 0x2;
}

static inline lfs_rtag_t lfs_rtag_isparallel(lfs_rtag_t a, lfs_rtag_t b) {
    return (a & 0x2) == (b & 0x2);
}

static inline lfs_srtag_t lfs_rtag_weight(lfs_rtag_t tag) {
    return tag >> 3;
}

static inline lfs_rtag_t lfs_rtag_merge(lfs_rtag_t a, lfs_rtag_t b) {
    return a + (b & ~0x7);
}

static inline lfs_srtag_t lfs_rtag_weight_lt(lfs_rtag_t tag, uint16_t count) {
    (void)count;
    return lfs_rtag_weight(tag);
}

static inline lfs_srtag_t lfs_rtag_weight_gt(lfs_rtag_t tag, uint16_t count) {
    return (((lfs_srtag_t)count) << 12)-1 - lfs_rtag_weight(tag);
}

static inline bool lfs_rtag_follow(lfs_rtag_t alt,
        lfs_srtag_t lt, lfs_srtag_t gt) {
    if (lfs_rtag_islt(alt)) {
        return lfs_rtag_weight(alt) > lt;
    } else {
        return lfs_rtag_weight(alt) > gt;
    }
}

static inline lfs_rtag_t lfs_rtag_flip(
        lfs_rtag_t alt, lfs_rtag_t lt, lfs_rtag_t gt) {
    return LFS_MKRALT_(
            lfs_rtag_isred(alt),
            !lfs_rtag_isgt(alt),
            (lt+gt+1) - lfs_rtag_weight(alt));
}

static inline void lfs_rtag_trim(lfs_rtag_t alt,
        lfs_srtag_t *lt, lfs_srtag_t *gt) {
    if (lfs_rtag_islt(alt)) {
        *lt = *lt - lfs_rtag_weight(alt);
    } else {
        *gt = *gt - lfs_rtag_weight(alt);
    }
}

static inline void lfs_rtag_untrim(lfs_rtag_t alt,
        lfs_srtag_t *lt, lfs_srtag_t *gt) {
    if (lfs_rtag_islt(alt)) {
        *lt = *lt + lfs_rtag_weight(alt);
    } else {
        *gt = *gt + lfs_rtag_weight(alt);
    }
}

// operations on attribute lists
struct lfs_mattr {
    lfs_tag_t tag;
    const void *buffer;
};

struct lfs_diskoff {
    lfs_block_t block;
    lfs_off_t off;
};

#define LFS_MKATTRS(...) \
    (struct lfs_mattr[]){__VA_ARGS__}, \
    sizeof((struct lfs_mattr[]){__VA_ARGS__}) / sizeof(struct lfs_mattr)

struct lfs_rattr {
    lfs_rtag_t tag;
    const void *buffer;
    lfs_size_t size;
    const struct lfs_rattr *next;
};

#define LFS_MKRATTR_(...) \
    (&(const struct lfs_rattr){__VA_ARGS__})

#define LFS_MKRATTR(type1, type2, id, buffer, size, next) \
    (&(const struct lfs_rattr){ \
        LFS_MKRTAG(type1, type2, id), \
        buffer, size, next})

#define LFS_MKRRMATTR(type1, type2, id, next) \
    (&(const struct lfs_rattr){ \
        LFS_MKRRMTAG(type1, type2, id), \
        NULL, 0, next})


// operations on pattern lists
struct lfs_rpat {
    lfs_rtag_t tag;
    union {
        struct {
            void *buffer;
            lfs_size_t size;
        } get;
        struct {
            const void *buffer;
            lfs_size_t size;
        } find;
    } u;
    struct lfs_rpat *next;
};

#define LFS_MKRGETPAT_(tag, buffer, size, next) \
    (&(struct lfs_rpat){ \
        .tag = LFS_PAT_GET | (tag), \
        .u.get.buffer = buffer, \
        .u.get.size = size, \
        .next = next})

#define LFS_MKRGETPAT(type1, type2, buffer, size, next) \
    LFS_MKRGET_(LFS_MKRTAG(type1, type2, 0), buffer, size, next)

#define LFS_MKRFINDPAT_(tag, buffer, size, next) \
    (&(struct lfs_rpat){ \
        .tag = LFS_PAT_FIND | (tag), \
        .u.find.buffer = buffer, \
        .u.find.size = size, \
        .next = next})

#define LFS_MKRFINDPAT(type1, type2, buffer, size, next) \
    LFS_MKRFIND_(LFS_MKRTAG(type1, type2, 0), buffer, size, next)


// operations on global state
static inline void lfs_gstate_xor(lfs_gstate_t *a, const lfs_gstate_t *b) {
    for (int i = 0; i < 3; i++) {
        ((uint32_t*)a)[i] ^= ((const uint32_t*)b)[i];
    }
}

static inline bool lfs_gstate_iszero(const lfs_gstate_t *a) {
    for (int i = 0; i < 3; i++) {
        if (((uint32_t*)a)[i] != 0) {
            return false;
        }
    }
    return true;
}

#ifndef LFS_READONLY
static inline bool lfs_gstate_hasorphans(const lfs_gstate_t *a) {
    return lfs_tag_size(a->tag);
}

static inline uint8_t lfs_gstate_getorphans(const lfs_gstate_t *a) {
    return lfs_tag_size(a->tag);
}

static inline bool lfs_gstate_hasmove(const lfs_gstate_t *a) {
    return lfs_tag_type1(a->tag);
}
#endif

static inline bool lfs_gstate_hasmovehere(const lfs_gstate_t *a,
        const lfs_block_t *pair) {
    return lfs_tag_type1(a->tag) && lfs_pair_cmp(a->pair, pair) == 0;
}

static inline void lfs_gstate_fromle32(lfs_gstate_t *a) {
    a->tag     = lfs_fromle32(a->tag);
    a->pair[0] = lfs_fromle32(a->pair[0]);
    a->pair[1] = lfs_fromle32(a->pair[1]);
}

#ifndef LFS_READONLY
static inline void lfs_gstate_tole32(lfs_gstate_t *a) {
    a->tag     = lfs_tole32(a->tag);
    a->pair[0] = lfs_tole32(a->pair[0]);
    a->pair[1] = lfs_tole32(a->pair[1]);
}
#endif

// operations on forward-CRCs used to track erased state
struct lfs_fcrc {
    lfs_size_t size;
    uint32_t crc;
};

static void lfs_fcrc_fromle32(struct lfs_fcrc *fcrc) {
    fcrc->size = lfs_fromle32(fcrc->size);
    fcrc->crc = lfs_fromle32(fcrc->crc);
}

#ifndef LFS_READONLY
static void lfs_fcrc_tole32(struct lfs_fcrc *fcrc) {
    fcrc->size = lfs_tole32(fcrc->size);
    fcrc->crc = lfs_tole32(fcrc->crc);
}
#endif

struct lfs_rfcrc {
    uint32_t crc;
    lfs_size_t size;
};

static lfs_ssize_t lfs_rfcrc_todisk(struct lfs_rfcrc *fcrc) {
    lfs_tole32_(fcrc->crc, &fcrc->crc);

    lfs_ssize_t delta = lfs_toleb128(fcrc->size, (uint8_t*)&fcrc->size, 4);
    if (delta < 0) {
        return delta;
    }

    return sizeof(uint32_t) + delta;
}

static lfs_ssize_t lfs_rfcrc_fromdisk(struct lfs_rfcrc *fcrc) {
    fcrc->crc = lfs_fromle32_(&fcrc->crc);

    lfs_ssize_t delta = lfs_fromleb128(&fcrc->size, (uint8_t*)&fcrc->size, 4);
    if (delta < 0) {
        return delta;
    }

    return sizeof(uint32_t) + delta;
}

// other endianness operations
static void lfs_ctz_fromle32(struct lfs_ctz *ctz) {
    ctz->head = lfs_fromle32(ctz->head);
    ctz->size = lfs_fromle32(ctz->size);
}

#ifndef LFS_READONLY
static void lfs_ctz_tole32(struct lfs_ctz *ctz) {
    ctz->head = lfs_tole32(ctz->head);
    ctz->size = lfs_tole32(ctz->size);
}
#endif

static inline void lfs_superblock_fromle32(lfs_superblock_t *superblock) {
    superblock->version     = lfs_fromle32(superblock->version);
    superblock->block_size  = lfs_fromle32(superblock->block_size);
    superblock->block_count = lfs_fromle32(superblock->block_count);
    superblock->name_max    = lfs_fromle32(superblock->name_max);
    superblock->file_max    = lfs_fromle32(superblock->file_max);
    superblock->attr_max    = lfs_fromle32(superblock->attr_max);
}

#ifndef LFS_READONLY
static inline void lfs_superblock_tole32(lfs_superblock_t *superblock) {
    superblock->version     = lfs_tole32(superblock->version);
    superblock->block_size  = lfs_tole32(superblock->block_size);
    superblock->block_count = lfs_tole32(superblock->block_count);
    superblock->name_max    = lfs_tole32(superblock->name_max);
    superblock->file_max    = lfs_tole32(superblock->file_max);
    superblock->attr_max    = lfs_tole32(superblock->attr_max);
}
#endif

#ifndef LFS_NO_ASSERT
static bool lfs_mlist_isopen(struct lfs_mlist *head,
        struct lfs_mlist *node) {
    for (struct lfs_mlist **p = &head; *p; p = &(*p)->next) {
        if (*p == (struct lfs_mlist*)node) {
            return true;
        }
    }

    return false;
}
#endif

static void lfs_mlist_remove(lfs_t *lfs, struct lfs_mlist *mlist) {
    for (struct lfs_mlist **p = &lfs->mlist; *p; p = &(*p)->next) {
        if (*p == mlist) {
            *p = (*p)->next;
            break;
        }
    }
}

static void lfs_mlist_append(lfs_t *lfs, struct lfs_mlist *mlist) {
    mlist->next = lfs->mlist;
    lfs->mlist = mlist;
}


/// Internal operations predeclared here ///
#ifndef LFS_READONLY
static int lfs_dir_commit(lfs_t *lfs, lfs_mdir_t *dir,
        const struct lfs_mattr *attrs, int attrcount);
static int lfs_dir_compact(lfs_t *lfs,
        lfs_mdir_t *dir, const struct lfs_mattr *attrs, int attrcount,
        lfs_mdir_t *source, uint16_t begin, uint16_t end);
static lfs_ssize_t lfs_file_flushedwrite(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size);
static lfs_ssize_t lfs_file_rawwrite(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size);
static int lfs_file_rawsync(lfs_t *lfs, lfs_file_t *file);
static int lfs_file_outline(lfs_t *lfs, lfs_file_t *file);
static int lfs_file_flush(lfs_t *lfs, lfs_file_t *file);

static int lfs_fs_deorphan(lfs_t *lfs, bool powerloss);
static int lfs_fs_preporphans(lfs_t *lfs, int8_t orphans);
static void lfs_fs_prepmove(lfs_t *lfs,
        uint16_t id, const lfs_block_t pair[2]);
static int lfs_fs_pred(lfs_t *lfs, const lfs_block_t dir[2],
        lfs_mdir_t *pdir);
static lfs_stag_t lfs_fs_parent(lfs_t *lfs, const lfs_block_t dir[2],
        lfs_mdir_t *parent);
static int lfs_fs_forceconsistency(lfs_t *lfs);
#endif

#ifdef LFS_MIGRATE
static int lfs1_traverse(lfs_t *lfs,
        int (*cb)(void*, lfs_block_t), void *data);
#endif

static int lfs_dir_rawrewind(lfs_t *lfs, lfs_dir_t *dir);

static lfs_ssize_t lfs_file_flushedread(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size);
static lfs_ssize_t lfs_file_rawread(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size);
static int lfs_file_rawclose(lfs_t *lfs, lfs_file_t *file);
static lfs_soff_t lfs_file_rawsize(lfs_t *lfs, lfs_file_t *file);

static lfs_ssize_t lfs_fs_rawsize(lfs_t *lfs);
static int lfs_fs_rawtraverse(lfs_t *lfs,
        int (*cb)(void *data, lfs_block_t block), void *data,
        bool includeorphans);

static int lfs_deinit(lfs_t *lfs);
static int lfs_rawunmount(lfs_t *lfs);


/// Block allocator ///
#ifndef LFS_READONLY
static int lfs_alloc_lookahead(void *p, lfs_block_t block) {
    lfs_t *lfs = (lfs_t*)p;
    lfs_block_t off = ((block - lfs->free.off)
            + lfs->cfg->block_count) % lfs->cfg->block_count;

    if (off < lfs->free.size) {
        lfs->free.buffer[off / 32] |= 1U << (off % 32);
    }

    return 0;
}
#endif

// indicate allocated blocks have been committed into the filesystem, this
// is to prevent blocks from being garbage collected in the middle of a
// commit operation
static void lfs_alloc_ack(lfs_t *lfs) {
    lfs->free.ack = lfs->cfg->block_count;
}

// drop the lookahead buffer, this is done during mounting and failed
// traversals in order to avoid invalid lookahead state
static void lfs_alloc_drop(lfs_t *lfs) {
    lfs->free.size = 0;
    lfs->free.i = 0;
    lfs_alloc_ack(lfs);
}

#ifndef LFS_READONLY
static int lfs_alloc(lfs_t *lfs, lfs_block_t *block) {
    while (true) {
        while (lfs->free.i != lfs->free.size) {
            lfs_block_t off = lfs->free.i;
            lfs->free.i += 1;
            lfs->free.ack -= 1;

            if (!(lfs->free.buffer[off / 32] & (1U << (off % 32)))) {
                // found a free block
                *block = (lfs->free.off + off) % lfs->cfg->block_count;

                // eagerly find next off so an alloc ack can
                // discredit old lookahead blocks
                while (lfs->free.i != lfs->free.size &&
                        (lfs->free.buffer[lfs->free.i / 32]
                            & (1U << (lfs->free.i % 32)))) {
                    lfs->free.i += 1;
                    lfs->free.ack -= 1;
                }

                return 0;
            }
        }

        // check if we have looked at all blocks since last ack
        if (lfs->free.ack == 0) {
            LFS_ERROR("No more free space %"PRIu32,
                    lfs->free.i + lfs->free.off);
            return LFS_ERR_NOSPC;
        }

        lfs->free.off = (lfs->free.off + lfs->free.size)
                % lfs->cfg->block_count;
        lfs->free.size = lfs_min(8*lfs->cfg->lookahead_size, lfs->free.ack);
        lfs->free.i = 0;

        // find mask of free blocks from tree
        memset(lfs->free.buffer, 0, lfs->cfg->lookahead_size);
        int err = lfs_fs_rawtraverse(lfs, lfs_alloc_lookahead, lfs, true);
        if (err) {
            lfs_alloc_drop(lfs);
            return err;
        }
    }
}
#endif


/// Red-black-yellow Dhara tree operations ///

// TODO actually should this be an lfs_bd_ operation?
static lfs_ssize_t lfs_rbyd_readtag(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache, lfs_size_t hint,
        lfs_block_t block, lfs_off_t off,
        lfs_rtag_t *tag, lfs_size_t *size, uint32_t *crc) {
    // needed to quiet an uninitialized warning, zeroing tag on error is
    // probably a good idea anyways
    *tag = 0;

    // read a pair of leb128s
    uint8_t buffer[2*4];
    lfs_size_t i = 0;

    // force leb decoding to overflow when truncated
    memset(buffer, 0xff, 2*4);

    // TODO allow different hint for lookup? bench this? does our hint work backwards?
    // TODO should lfs_bd_read allow a range for reads?
    int err = lfs_bd_read(lfs,
            pcache, rcache, hint,
            block, off, &buffer,
            lfs_min(sizeof(buffer), lfs->cfg->block_size-off));
    if (err) {
        return err;
    }

    lfs_rtag_t tag_;
    ssize_t delta = lfs_fromleb128(&tag_, &buffer[i], 4);
    if (delta < 0) {
        return delta;
    }
    i += delta;

    lfs_size_t size_;
    delta = lfs_fromleb128(&size_, &buffer[i], 4);
    if (delta < 0) {
        return delta;
    }
    i += delta;

    // optionally crc
    if (crc) {
        uint32_t crc_ = *crc;

        // on-disk, the tags valid bit must reflect the parity of the
        // preceding data, fortunately for crc32c this is the same as the
        // parity of the crc
        if ((tag_ & 1) != (lfs_popc(crc_) & 1)) {
            return LFS_ERR_INVAL;
        }

        *crc = lfs_crc32c(crc_, buffer, i);
    }

    // convert to in-device tag repr, we want the valid bit in the sign bit
    tag_ >>= 1;
    *tag = tag_;
    *size = size_;

    return i;
}

static int lfs_rbyd_fetch(lfs_t *lfs,
        lfs_rbyd_t *rbyd, lfs_block_t block,
        struct lfs_rpat *patterns) {
    // TODO this
    // first mark patterns as invalid until we find matches
    for (struct lfs_rpat *pattern = patterns;
            pattern;
            pattern = pattern->next) {
        pattern->tag = lfs_rtag_invalid(pattern->tag);
    }

    // read the revision count and get the crc started
    uint32_t rev;
    int err = lfs_bd_read(lfs,
            NULL, &lfs->rcache, lfs->cfg->block_size,
            block, 0, &rev, sizeof(uint32_t));
    if (err) {
        return err;
    }

    // calculate crc before endian conversion
    uint32_t crc = lfs_crc32c(0, &rev, sizeof(uint32_t));
    rev = lfs_fromle32_(&rev);

    rbyd->block = block;
    rbyd->off = 0;
    rbyd->rev = rev;

    // temporary state until we validate a crc
    lfs_off_t off = sizeof(uint32_t);
    lfs_off_t trunk = 0;
    bool wastrunk = false;
    uint16_t count = 0;

    // assume unerased until proven otherwise
    bool maybeerased = false;
    bool hasfcrc = false;
    struct lfs_rfcrc fcrc;

    // scan tags, checking valid bits, crcs, etc
    while (true) {
        lfs_rtag_t tag;
        lfs_size_t size;
        lfs_ssize_t delta = lfs_rbyd_readtag(lfs,
                NULL, &lfs->rcache, lfs->cfg->block_size,
                block, off, &tag, &size, &crc);
        if (delta < 0) {
            if (delta == LFS_ERR_INVAL
                    || delta == LFS_ERR_CORRUPT
                    || delta == LFS_ERR_OVERFLOW) {
                maybeerased = (delta == LFS_ERR_INVAL);
                break;
            }
            return delta;
        }

        // found trunk of tree?
        if (!wastrunk && (lfs_rtag_isalt(tag) || lfs_rtag_intree(tag))) {
            trunk = off;
            wastrunk = true;
        }

        off += delta;

        // we mostly just skip alt pointers here
        if (lfs_rtag_isalt(tag)) {
            continue;
        }

        // trunk ends at non-alt tag
        wastrunk = false;

        // tag goes out of range?
        if (off + size > lfs->cfg->block_size) {
            break;
        }

        // not an end-of-commit crc
        if ((lfs_rtag_type1(tag) & ~0x1) != LFS_TYPE1_CRC) {
            // fcrc is only valid if the last tag was a crc
            hasfcrc = false;

            // crc the entry first, hopefully leaving it in the cache
            err = lfs_bd_crc32c(lfs,
                    NULL, &lfs->rcache, lfs->cfg->block_size,
                    block, off, size, &crc);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    break;
                }
                return err;
            }

            // found a create? increase count of ids
            if (lfs_rtag_type1(tag) == LFS_TYPE1_CREATE) {
                count += 1;
            // found an fcrc? save for later
            } else if (lfs_rtag_type1(tag) == LFS_TYPE1_FCRC) {
                err = lfs_bd_read(lfs,
                        NULL, &lfs->rcache, lfs->cfg->block_size,
                        block, off, &fcrc,
                        lfs_min(size, sizeof(struct lfs_rfcrc)));
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        break;
                    }
                    return err;
                }

                lfs_ssize_t delta = lfs_rfcrc_fromdisk(&fcrc);
                if (delta < 0) {
                    return delta;
                }
                hasfcrc = true;
            }
        // is an end-of-commit crc
        } else {
            uint32_t crc_ = 0;
            err = lfs_bd_read(lfs,
                    NULL, &lfs->rcache, lfs->cfg->block_size,
                    block, off, &crc_, sizeof(uint32_t));
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    break;
                }
                return err;
            }
            crc_ = lfs_fromle32_(&crc_);

            if (crc != crc_) {
                // uh oh, crcs don't match
                break;
            }

            // toss our crc into the filesystem seed for
            // pseudorandom numbers, note we use another crc here
            // as a collection function because it is sufficiently
            // random and convenient
            lfs->seed = lfs_crc32c(lfs->seed, &crc, sizeof(uint32_t));

            // save what we've found so far
            printf("trunk => %x\n", trunk);
            rbyd->trunk = trunk;
            rbyd->off = off;
            rbyd->crc = crc;
            rbyd->count = count;
        }

        off += size;
    }

    // no valid commits at all?
    if (rbyd->off == 0) {
        return LFS_ERR_CORRUPT;
    }

    // did we end on a valid commit? we may have an erased block
    rbyd->erased = false;
    if (maybeerased && hasfcrc && rbyd->off % lfs->cfg->prog_size == 0) {
        // check for an fcrc matching the next prog's erased state, if
        // this failed most likely a previous prog was interrupted, we
        // need a new erase
        uint32_t fcrc_ = 0xffffffff;
        int err = lfs_bd_crc32c(lfs,
                NULL, &lfs->rcache, lfs->cfg->block_size,
                rbyd->block, rbyd->off, fcrc.size, &fcrc_);
        if (err && err != LFS_ERR_CORRUPT) {
            return err;
        }

        // found beginning of erased part?
        rbyd->erased = (fcrc_ == fcrc.crc);
    }

    return 0;
}

static lfs_srtag_t lfs_rbyd_lookup(lfs_t *lfs, const lfs_rbyd_t *rbyd,
        lfs_rtag_t tag, lfs_off_t *off, lfs_size_t *size) {
    printf("lookup(%08x)\n", tag);
tryagain:;
    // no trunk yet?
    lfs_off_t branch = rbyd->trunk;
    if (!branch) {
        return LFS_ERR_NOENT;
    }

    // weights for pruning
    lfs_srtag_t lt = lfs_rtag_weight_lt(tag, rbyd->count+1);
    lfs_srtag_t gt = lfs_rtag_weight_gt(tag, rbyd->count+1);
    printf("lt, gt = (%x, %x)\n", lt, gt);

    // descend down tree
    while (true) {
        lfs_rtag_t alt;
        lfs_off_t jump;
        lfs_ssize_t delta = lfs_rbyd_readtag(lfs,
                &lfs->pcache, &lfs->rcache, lfs->cfg->block_size,
                rbyd->block, branch, &alt, &jump, NULL);
        if (delta < 0) {
            return delta;
        }

        // found an alt?
        if (lfs_rtag_isalt(alt)) {
            printf("follow %c%x (%x, %x)? => %d\n", lfs_rtag_isgt(alt) ? '>' : '<', lfs_rtag_weight(alt), lt, gt, lfs_rtag_follow(alt, lt, gt));
            if (lfs_rtag_follow(alt, lt, gt)) {
                lfs_rtag_trim(lfs_rtag_flip(alt, lt, gt), &lt, &gt);
                branch = branch - jump;
            } else {
                lfs_rtag_trim(alt, &lt, &gt);
                branch = branch + delta;
            }

        // found end of tree?
        } else {
            // update the tag id
            lfs_rtag_t tag_ = lfs_rtag_setid(alt, lfs_rtag_id(tag));

            // was removed? go on to the next tag
            if (lfs_rtag_isrm(tag_)) {
                tag = lfs_rtag_inc(tag_);
                goto tryagain;
            }

            printf("lt, gt = (%x, %x)\n", lt, gt);
            printf("lookup %08x => %08x (raw %08x)\n", tag, tag_, alt);

            // not what we're looking for?
            if (tag_ < tag) {
                return LFS_ERR_NOENT;
            }

            // save what we found
            *off = branch + delta;
            *size = jump;
            return tag_;
        }
    }
}

static lfs_ssize_t lfs_rbyd_get(lfs_t *lfs, const lfs_rbyd_t *rbyd,
        lfs_rtag_t tag, void *buffer, lfs_size_t size) {
    lfs_off_t found_off;
    lfs_size_t found_size;
    lfs_srtag_t found_tag = lfs_rbyd_lookup(lfs, rbyd, tag,
            &found_off, &found_size);
    if (found_tag < 0) {
        return found_tag;
    }

    // lookup finds the next-smallest tag, here we fail if it's not
    // an exact match
    if ((lfs_tag_t)found_tag != tag) {
        return LFS_ERR_NOENT;
    }

    // note that it's highly likely the data is in our cache now
    lfs_size_t delta = lfs_min(size, found_size);
    int err = lfs_bd_read(lfs,
            &lfs->pcache, &lfs->rcache, delta,
            rbyd->block, found_off, buffer, delta);
    if (err) {
        return err;
    }

    return found_size;
}

static int lfs_rbyd_prog(lfs_t *lfs, lfs_rbyd_t *rbyd_,
        const void *buffer, lfs_size_t size, uint32_t *crc) {
    // check for out-of-bounds here
    // TODO should we just move this to lfs_bd_prog?
    // TODO actually should we just build crc into lfs_bd_prog as well?
    if (rbyd_->off+size > lfs->cfg->block_size) {
        return LFS_ERR_RANGE;
    }

    int err = lfs_bd_prog(lfs,
            &lfs->pcache, &lfs->rcache, false,
            rbyd_->block, rbyd_->off, buffer, size);
    if (err) {
        return err;
    }

    // update off
    rbyd_->off += size;
    // TODO should this not be optional? should we move the range check
    // into bd_prog? so we can get rid of the one use of this in
    // lfs_rbyd_commit?
    // optionally crc
    if (crc) {
        *crc = lfs_crc32c(*crc, buffer, size);
    }

    return 0;
}

static int lfs_rbyd_progtag(lfs_t *lfs, lfs_rbyd_t *rbyd_,
        lfs_rtag_t tag, lfs_size_t size, uint32_t *crc) {
    // convert to on-disk repr
    tag <<= 1;

    // make sure to include the parity of the current crc
    tag |= lfs_popc(rbyd_->crc) & 1;

    // compress into pair of leb128s
    uint8_t buffer[2*4];
    lfs_size_t delta = 0;

    ssize_t delta_ = lfs_toleb128(tag, &buffer[delta], 4);
    if (delta_ < 0) {
        return delta_;
    }
    delta += delta_;

    delta_ = lfs_toleb128(size, &buffer[delta], 4);
    if (delta_ < 0) {
        return delta_;
    }
    delta += delta_;

    int err = lfs_rbyd_prog(lfs, rbyd_, &buffer, delta, crc);
    if (err) {
        return err;
    }

    return 0;
}

static int lfs_rbyd_p_flush(lfs_t *lfs, lfs_rbyd_t *rbyd_,
        lfs_rtag_t p_alts[static 3],
        lfs_rtag_t p_jumps[static 3],
        unsigned count) {
    // write out some number of alt pointers in our queue
    for (unsigned i = 0; i < count; i++) {
        if (p_alts[3-1-i]) {
            // change to a relative jump at the last minute
            lfs_rtag_t alt = p_alts[3-1-i];
            lfs_off_t jump = rbyd_->off - p_jumps[3-1-i];
            printf("%x: writing %08x %08x\n", rbyd_->off, alt, jump);

            int err = lfs_rbyd_progtag(lfs, rbyd_, alt, jump, &rbyd_->crc);
            if (err) {
                return err;
            }
        }
    }

    return 0;
}

static inline int lfs_rbyd_p_push(lfs_t *lfs, lfs_rbyd_t *rbyd_,
        lfs_rtag_t p_alts[static 3],
        lfs_off_t p_jumps[static 3],
        lfs_rtag_t alt, lfs_off_t jump) {
    int err = lfs_rbyd_p_flush(lfs, rbyd_, p_alts, p_jumps, 1);
    if (err) {
        return err;
    }

    p_alts[2] = p_alts[1];
    p_jumps[2] = p_jumps[1];
    p_alts[1] = p_alts[0];
    p_jumps[1] = p_jumps[0];
    p_alts[0] = alt;
    p_jumps[0] = jump;

    return 0;
}

static inline void lfs_rbyd_p_pop(
        lfs_rtag_t p_alts[static 3],
        lfs_off_t p_jumps[static 3]) {
    p_alts[0] = p_alts[1];
    p_jumps[0] = p_jumps[1];
    p_alts[1] = p_alts[2];
    p_jumps[1] = p_jumps[2];
    p_alts[2] = 0;
    p_jumps[2] = 0;
}

static void lfs_rbyd_p_red(
        lfs_rtag_t p_alts[static 3],
        lfs_off_t p_jumps[static 3]) {
    LFS_ASSERT(lfs_rtag_isblack(p_alts[0]));

    // recolor with red edge
    if (p_alts[1]) {
        p_alts[1] = lfs_rtag_red(p_alts[1]);

        // reorder so that top two edges always go in the same direction
        if (p_alts[2] && lfs_rtag_isred(p_alts[2])) {
            if (lfs_rtag_isparallel(p_alts[1], p_alts[2])) {
                // no reorder needed
            } else if (lfs_rtag_isparallel(p_alts[0], p_alts[2])) {
                lfs_rtag_t alt_ = p_alts[1];
                lfs_off_t jump_ = p_jumps[1];
                p_alts[1] = lfs_rtag_red(p_alts[0]);
                p_jumps[1] = p_jumps[0];
                p_alts[0] = lfs_rtag_black(alt_);
                p_jumps[0] = jump_;
            } else if (lfs_rtag_isparallel(p_alts[0], p_alts[1])) {
                lfs_rtag_t alt_ = p_alts[2];
                lfs_off_t jump_ = p_jumps[2];
                p_alts[2] = lfs_rtag_red(p_alts[1]);
                p_jumps[2] = p_jumps[1];
                p_alts[1] = lfs_rtag_red(p_alts[0]);
                p_jumps[1] = p_jumps[0];
                p_alts[0] = lfs_rtag_black(alt_);
                p_jumps[0] = jump_;
            } else {
                LFS_ASSERT(false);
            }
        }
    }
}

static int lfs_rbyd_append(lfs_t *lfs, lfs_rbyd_t *rbyd_,
        lfs_rtag_t tag, const void *buffer, lfs_size_t size) {
    printf("append()\n");
    LFS_ASSERT(lfs_rtag_id(tag) <= rbyd_->count+1);

    // assume we'll update our trunk
    lfs_off_t branch = rbyd_->trunk;
    rbyd_->trunk = rbyd_->off;

    // no trunk yet?
    if (!branch) {
        goto leaf;
    }

    // weights for pruning
    lfs_srtag_t lt;
    lfs_srtag_t gt;
    if (lfs_rtag_type1(tag) == LFS_TYPE1_CREATE) {
        // inserting a new id? align down
        // TODO special function for this?
//            if (lfs_rtag_id(attr->tag) == count+1) {
//                lt = (count+1) << 12;
//                gt = 0;
//            } else {
//                lt = lfs_rtag_weight_lt(attr->tag & ~0x7fff, count+1);
//                gt = lfs_rtag_weight_gt(attr->tag & ~0x7fff, count+1);
//            }
        lt = lfs_rtag_weight_lt(tag & ~0x7fff, rbyd_->count+1);
        gt = lfs_rtag_weight_gt(tag & ~0x7fff, rbyd_->count+1) + 1;
    } else {
        lt = lfs_rtag_weight_lt(tag, rbyd_->count+1);
        gt = lfs_rtag_weight_gt(tag, rbyd_->count+1);
    }

    printf("lt, gt = (%x, %x)\n", lt, gt);

    // queue of pending alts we can emulate rotations with
    lfs_rtag_t p_alts[3] = {0, 0, 0};
    lfs_off_t p_jumps[3] = {0, 0, 0};
    lfs_off_t graft = 0;

    // descend down tree, building alt pointers
    while (true) {
        lfs_rtag_t alt;
        lfs_off_t jump;
        lfs_ssize_t delta = lfs_rbyd_readtag(lfs,
                &lfs->pcache, &lfs->rcache, lfs->cfg->block_size,
                rbyd_->block, branch, &alt, &jump, NULL);
        if (delta < 0) {
            return delta;
        }

        // found an alt?
        if (lfs_rtag_isalt(alt)) {
            // make jump absolute
            jump = branch - jump;
            lfs_rtag_t branch_ = branch + delta;

            // prune?
            if (lfs_rtag_weight(alt) >= lt+gt+1) {
                printf("prune!\n");
                LFS_ASSERT(p_alts[0]);

                alt = lfs_rtag_black(p_alts[0]);
                branch_ = jump;
                jump = p_jumps[0];
                lfs_rbyd_p_pop(p_alts, p_jumps);

                lfs_rtag_untrim(alt, &lt, &gt);
            }

            // two reds makes a yellow, split?
            if (lfs_rtag_isred(alt)
                    && p_alts[0]
                    && lfs_rtag_isred(p_alts[0])) {
                LFS_ASSERT(lfs_rtag_isparallel(alt, p_alts[0]));

                // if we take the red or yellow alt we can just point
                // to the black alt, otherwise we need to point to the
                // yellow alt and prune later
                if (lfs_rtag_follow(alt, lt, gt)) {
                    printf("ysplit follow\n");
                    lfs_rtag_t alt_ = p_alts[0];
                    lfs_off_t jump_ = p_jumps[0];
                    p_alts[0] = lfs_rtag_black(
                            lfs_rtag_flip(alt, lt, gt));
                    p_jumps[0] = branch_;
                    alt = lfs_rtag_black(alt_);
                    branch_ = jump;
                    jump = jump_;

                    lfs_rtag_untrim(alt, &lt, &gt);
                    lfs_rtag_trim(p_alts[0], &lt, &gt);
                    lfs_rbyd_p_red(p_alts, p_jumps);

                } else {
                    printf("ysplit nofollow\n");
                    p_alts[0] = lfs_rtag_black(
                            lfs_rtag_merge(alt, p_alts[0]));
                    p_jumps[0] = graft;

                    lfs_rtag_trim(alt, &lt, &gt);
                    lfs_rbyd_p_red(p_alts, p_jumps);

                    graft = branch;
                    branch = branch_;
                    continue;
                }
            }

            // should've taken red alt? needs a flip
            if (lt < 0 || gt < 0) {
                LFS_ASSERT(lfs_rtag_isblack(alt));
                LFS_ASSERT(p_alts[0]);
                LFS_ASSERT(lfs_rtag_isred(p_alts[0]));

                printf("rflip %s (%x,%x)\n",
                        lfs_rtag_isparallel(alt, p_alts[0]) ? "parallel" : "perpendicular",
                        lt, gt);

                // if black alt would've been taken, it also needs a flip
                if (lfs_rtag_isparallel(alt, p_alts[0])) {
                    alt = lfs_rtag_flip(alt, lt, gt);
                    lfs_off_t jump_ = jump;
                    jump = branch_;
                    branch_ = jump_;
                }

                lfs_rtag_t alt_ = p_alts[0];
                lfs_off_t jump_ = p_jumps[0];
                p_alts[0] = lfs_rtag_red(alt);
                p_jumps[0] = jump;
                alt = lfs_rtag_black(alt_);
                jump = jump_;

                lfs_rtag_untrim(alt, &lt, &gt);
                lfs_rtag_trim(p_alts[0], &lt, &gt);
            }

            // take black alt? needs a flip
            if (lfs_rtag_isblack(alt) && lfs_rtag_follow(alt, lt, gt)) {
                printf("bflip\n");
                alt = lfs_rtag_flip(alt, lt, gt);
                lfs_off_t jump_ = jump;
                jump = branch_;
                branch_ = jump_;
            }

            // push alt onto queue
            int err = lfs_rbyd_p_push(lfs, rbyd_,
                    p_alts, p_jumps,
                    alt, jump);
            if (err) {
                return err;
            }

            // continue to next alt
            lfs_rtag_trim(alt, &lt, &gt);
            graft = branch;
            branch = branch_;

        // found end of tree?
        } else {
            // update the tag id
            // TODO should this be a function? maybe see what create/delete need
            // TODO alternatively can we do this a bit more directly here
//                lfs_rtag_t tag_ = lfs_rtag_setid(alt,
//                        lfs_min(lfs_rtag_id(attr->tag), count));
            lfs_rtag_t tag_ = lfs_rtag_setid(alt, lfs_rtag_id(tag));
            // TODO I don't really know why this works
            if (lfs_rtag_type1(tag) == LFS_TYPE1_CREATE && gt == 0) {
                tag_ = lfs_rtag_decid(tag_);
            }
                    //gt == 0 ? lfs_rtag_id(attr->tag)-1 : lfs_rtag_id(attr->tag));
            printf("found %x (%d, %d)\n", tag_, lt >> 12, gt >> 12);

            // split leaf?
            // TODO we might be able to rearrange this a bit better
            if (lfs_rtag_type1(tag) == LFS_TYPE1_CREATE
                    || tag_ != tag) {
                // inserting a new id?
                if (lfs_rtag_type1(tag) == LFS_TYPE1_CREATE) {
                    // bias the weights so that lookups always find the
                    // next biggest tag
                    if (lfs_rtag_weight(tag_)
                            < lfs_rtag_weight(tag & ~0x7fff)) {
                        printf("lt insert\n");
                        //
                        //           lt            gt
                        //      .----'---.       .-'.
                        //        alt
                        //      .--'--.
                        // <---+------+---|--+--|---+------+--->
                        //     a     old    new     d      e
                        //
                        alt = LFS_MKRALT(B, LT, lt
                                - (lfs_rtag_weight(tag & ~0x7fff)-1
                                    - lfs_rtag_weight(tag_))
                                + 1);
                    } else {
                        printf("gt insert\n");
                        printf("hmmm? %08x %08x\n", lfs_rtag_weight(tag_), lfs_rtag_weight(tag & ~0x7fff));
                        //
                        //            lt           gt
                        //      .-----'--.       .-'.
                        //                      alt
                        //                    .--'--.
                        // <---+------+---|--+--|---+------+--->
                        //     a      b     new    old     e
                        //
                        alt = LFS_MKRALT(B, GT,
                                // TODO hm, can this be done differently?
                                lfs_rtag_weight(lfs_rtag_incid(tag_))
                                    - lfs_rtag_weight(tag));
                    }
                } else {
                    // bias the weights so that lookups always find the
                    // next biggest tag
                    if (lfs_rtag_weight(tag_)
                            < lfs_rtag_weight(tag)) {
                        //
                        //            lt         gt
                        //      .-----'-----. .--'--.
                        //        alt
                        //      .--'--.
                        // <---+------+------+------+------+--->
                        //     a     old    new     d      e
                        //
                        alt = LFS_MKRALT(B, LT, lt
                                + 1
                                - (lfs_rtag_weight(tag)
                                    - lfs_rtag_weight(tag_)));
                    } else {
                        //
                        //            lt         gt
                        //      .-----'-----. .--'--.
                        //                      alt
                        //                    .--'--.
                        // <---+------+------+------+------+--->
                        //     a      b     new    old     e
                        //
                        alt = LFS_MKRALT(B, GT, gt);
                    }
                }

                int err = lfs_rbyd_p_push(lfs, rbyd_,
                        p_alts, p_jumps,
                        alt, branch);
                if (err) {
                    return err;
                }

                lfs_rbyd_p_red(p_alts, p_jumps);
            }

            // flush any pending alts
            int err = lfs_rbyd_p_flush(lfs, rbyd_,
                    p_alts, p_jumps, 3);
            if (err) {
                return err;
            }

            // done! lets get out of here
            goto leaf;
        }
    }

leaf:;
    // write the tag
    int err = lfs_rbyd_progtag(lfs, rbyd_, tag, size, &rbyd_->crc);
    if (err) {
        return err;
    }

    // don't forget the actual data!
    err = lfs_rbyd_prog(lfs, rbyd_, buffer, size, &rbyd_->crc);
    if (err) {
        return err;
    }

    // if we're inserting, increase the id count, indirectly shifting
    // all ids >= over one
    //
    // note we do this here since it is possible to insert into an
    // empty tree
    if (lfs_rtag_type1(tag) == LFS_TYPE1_CREATE) {
        rbyd_->count += 1;
    }

    return 0;
}

static int lfs_rbyd_commit(lfs_t *lfs, lfs_rbyd_t *rbyd,
        const struct lfs_rattr *attrs) {
    printf("commit()\n");
    LFS_ASSERT(rbyd->erased);

    // mark as unerased in case we fail
    rbyd->erased = false;

    // setup commit state
    lfs_rbyd_t rbyd_ = *rbyd;

    // include revision count?
    if (!rbyd_.off) {
        uint32_t rev;
        lfs_tole32_(rbyd_.rev, &rev);
        int err = lfs_rbyd_prog(lfs, &rbyd_,
                &rev, sizeof(uint32_t), &rbyd_.crc);
        if (err) {
            return err;
        }
    }

    // append each tag to the tree
    for (const struct lfs_rattr *attr = attrs; attr; attr = attr->next) {
        int err = lfs_rbyd_append(lfs, &rbyd_,
                attr->tag, attr->buffer, attr->size);
        if (err) {
            return err;
        }
    }

    // align to the next prog unit
    //
    // this gets a bit complicated as we have two types of crcs:
    // - 7-word crc with fcrc to check following prog (middle of block)
    //     1-byte fcrc tag
    //   + 1-byte fcrc len (worst case)
    //   + 4-byte fcrc crc
    //   + 4-byte fcrc crc-len (worst case)
    //   + 1-byte crc tag
    //   + 4-byte crc len (worst case)
    //   + 4-byte crc crc
    //   = 19 bytes
    // - 3-word crc with no following prog (end of block)
    //     1-byte crc tag
    //   + 4-byte crc len (worst case)
    //   + 4-byte crc crc
    //   = 9 bytes
    // 
    const lfs_off_t aligned = lfs_alignup(
            rbyd_.off + 1+1+4+4 + 1+4+4,
            lfs->cfg->prog_size);

    // space for fcrc?
    uint8_t perturb = 0;
    if (aligned <= lfs->cfg->block_size - lfs->cfg->prog_size) {
        // read the leading byte in case we need to change the expected
        // value of the next tag's valid bit
        int err = lfs_bd_read(lfs,
                &lfs->pcache, &lfs->rcache, lfs->cfg->prog_size,
                rbyd_.block, aligned, &perturb, 1);
        if (err && err != LFS_ERR_CORRUPT) {
            return err;
        }

        // find the expected fcrc, don't bother avoiding a reread of the
        // perturb byte, as it should still be in our cache
        struct lfs_rfcrc fcrc = {.crc=0, .size=lfs->cfg->prog_size};
        err = lfs_bd_crc32c(lfs,
                &lfs->pcache, &lfs->rcache, lfs->cfg->prog_size,
                rbyd_.block, aligned, fcrc.size, &fcrc.crc);
        if (err && err != LFS_ERR_CORRUPT) {
            return err;
        }

        lfs_size_t fcrc_delta = lfs_rfcrc_todisk(&fcrc);
        err = lfs_rbyd_progtag(lfs, &rbyd_,
                LFS_MKRTAG(FCRC, 0, 0), fcrc_delta, &rbyd_.crc);
        if (err) {
            return err;
        }

        err = lfs_rbyd_prog(lfs, &rbyd_,
                &fcrc, fcrc_delta, &rbyd_.crc);
        if (err) {
            return err;
        }

        rbyd_.erased = true;
    }

    // build end-of-commit crc
    //
    // note padding-size depends on leb-encoding depends on padding-size, to
    // get around this catch-22 we just always write a fully-expanded leb128
    // encoding
    uint8_t buffer[1+4+4];
    buffer[0] = (LFS_MKRTAG(CRC, 0, 0) << 1) | (lfs_popc(rbyd_.crc) & 1);

    lfs_off_t padding = aligned - (rbyd_.off + 1+4);
    buffer[1] = 0x80 | (0x7f & (padding >>  0));
    buffer[2] = 0x80 | (0x7f & (padding >>  7));
    buffer[3] = 0x80 | (0x7f & (padding >> 14));
    buffer[4] = 0x00 | (0x7f & (padding >> 21));

    rbyd_.crc = lfs_crc32c(rbyd_.crc, buffer, 1+4);
    // we can't let the next tag appear as valid, so intentionally perturb the
    // commit if this happens, note parity(crc(m)) == parity(m) with crc32c,
    // so we can really change any bit to make this happen, we've reserved a bit
    // in crc tags just for this purpose
    if ((lfs_popc(rbyd_.crc) & 1) == (perturb & 1)) {
        buffer[0] ^= 0x2;
        rbyd_.crc ^= 0x7022df58; // note crc(a ^ b) == crc(a) ^ crc(b)
    }
    lfs_tole32_(rbyd_.crc, &buffer[1+4]);

    int err = lfs_rbyd_prog(lfs, &rbyd_, buffer, 1+4+4, NULL);
    if (err) {
        return err;
    }

    // flush our caches, finalizing the commit on-disk
    err = lfs_bd_sync(lfs, &lfs->pcache, &lfs->rcache, false);
    if (err) {
        return err;
    }

    // succesful commit, check checksum to make sure
    uint32_t crc_ = rbyd->crc;
    err = lfs_bd_crc32c(lfs,
            NULL, &lfs->rcache, rbyd_.off-4,
            rbyd_.block, rbyd->off, rbyd_.off-4 - rbyd->off, &crc_);
    if (err) {
        return err;
    }

    if (crc_ != rbyd_.crc) {
        // oh no, something went wrong
        printf("oh no %08x != %08x\n", crc_, rbyd_.crc);
        return LFS_ERR_CORRUPT;
    }

    // ok, everything is good, save what we've committed
    rbyd_.off = aligned;
    *rbyd = rbyd_;

    return 0;
}


/// Metadata pair and directory operations ///
static lfs_stag_t lfs_dir_getslice(lfs_t *lfs, const lfs_mdir_t *dir,
        lfs_tag_t gmask, lfs_tag_t gtag,
        lfs_off_t goff, void *gbuffer, lfs_size_t gsize) {
    lfs_off_t off = dir->off;
    lfs_tag_t ntag = dir->etag;
    lfs_stag_t gdiff = 0;

    if (lfs_gstate_hasmovehere(&lfs->gdisk, dir->pair) &&
            lfs_tag_id(gmask) != 0 &&
            lfs_tag_id(lfs->gdisk.tag) <= lfs_tag_id(gtag)) {
        // synthetic moves
        gdiff -= LFS_MKTAG(0, 1, 0);
    }

    // iterate over dir block backwards (for faster lookups)
    while (off >= sizeof(lfs_tag_t) + lfs_tag_dsize(ntag)) {
        off -= lfs_tag_dsize(ntag);
        lfs_tag_t tag = ntag;
        int err = lfs_bd_read(lfs,
                NULL, &lfs->rcache, sizeof(ntag),
                dir->pair[0], off, &ntag, sizeof(ntag));
        if (err) {
            return err;
        }

        ntag = (lfs_frombe32(ntag) ^ tag) & 0x7fffffff;

        if (lfs_tag_id(gmask) != 0 &&
                lfs_tag_type1(tag) == LFS_TYPE_SPLICE &&
                lfs_tag_id(tag) <= lfs_tag_id(gtag - gdiff)) {
            if (tag == (LFS_MKTAG(LFS_TYPE_CREATE, 0, 0) |
                    (LFS_MKTAG(0, 0x3ff, 0) & (gtag - gdiff)))) {
                // found where we were created
                return LFS_ERR_NOENT;
            }

            // move around splices
            gdiff += LFS_MKTAG(0, lfs_tag_splice(tag), 0);
        }

        if ((gmask & tag) == (gmask & (gtag - gdiff))) {
            if (lfs_tag_isdelete(tag)) {
                return LFS_ERR_NOENT;
            }

            lfs_size_t diff = lfs_min(lfs_tag_size(tag), gsize);
            err = lfs_bd_read(lfs,
                    NULL, &lfs->rcache, diff,
                    dir->pair[0], off+sizeof(tag)+goff, gbuffer, diff);
            if (err) {
                return err;
            }

            memset((uint8_t*)gbuffer + diff, 0, gsize - diff);

            return tag + gdiff;
        }
    }

    return LFS_ERR_NOENT;
}

static lfs_stag_t lfs_dir_get(lfs_t *lfs, const lfs_mdir_t *dir,
        lfs_tag_t gmask, lfs_tag_t gtag, void *buffer) {
    return lfs_dir_getslice(lfs, dir,
            gmask, gtag,
            0, buffer, lfs_tag_size(gtag));
}

static int lfs_dir_getread(lfs_t *lfs, const lfs_mdir_t *dir,
        const lfs_cache_t *pcache, lfs_cache_t *rcache, lfs_size_t hint,
        lfs_tag_t gmask, lfs_tag_t gtag,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    uint8_t *data = buffer;
    if (off+size > lfs->cfg->block_size) {
        return LFS_ERR_CORRUPT;
    }

    while (size > 0) {
        lfs_size_t diff = size;

        if (pcache && pcache->block == LFS_BLOCK_INLINE &&
                off < pcache->off + pcache->size) {
            if (off >= pcache->off) {
                // is already in pcache?
                diff = lfs_min(diff, pcache->size - (off-pcache->off));
                memcpy(data, &pcache->buffer[off-pcache->off], diff);

                data += diff;
                off += diff;
                size -= diff;
                continue;
            }

            // pcache takes priority
            diff = lfs_min(diff, pcache->off-off);
        }

        if (rcache->block == LFS_BLOCK_INLINE &&
                off < rcache->off + rcache->size) {
            if (off >= rcache->off) {
                // is already in rcache?
                diff = lfs_min(diff, rcache->size - (off-rcache->off));
                memcpy(data, &rcache->buffer[off-rcache->off], diff);

                data += diff;
                off += diff;
                size -= diff;
                continue;
            }

            // rcache takes priority
            diff = lfs_min(diff, rcache->off-off);
        }

        // load to cache, first condition can no longer fail
        rcache->block = LFS_BLOCK_INLINE;
        rcache->off = lfs_aligndown(off, lfs->cfg->read_size);
        rcache->size = lfs_min(lfs_alignup(off+hint, lfs->cfg->read_size),
                lfs->cfg->cache_size);
        int err = lfs_dir_getslice(lfs, dir, gmask, gtag,
                rcache->off, rcache->buffer, rcache->size);
        if (err < 0) {
            return err;
        }
    }

    return 0;
}

#ifndef LFS_READONLY
static int lfs_dir_traverse_filter(void *p,
        lfs_tag_t tag, const void *buffer) {
    lfs_tag_t *filtertag = p;
    (void)buffer;

    // which mask depends on unique bit in tag structure
    uint32_t mask = (tag & LFS_MKTAG(0x100, 0, 0))
            ? LFS_MKTAG(0x7ff, 0x3ff, 0)
            : LFS_MKTAG(0x700, 0x3ff, 0);

    // check for redundancy
    if ((mask & tag) == (mask & *filtertag) ||
            lfs_tag_isdelete(*filtertag) ||
            (LFS_MKTAG(0x7ff, 0x3ff, 0) & tag) == (
                LFS_MKTAG(LFS_TYPE_DELETE, 0, 0) |
                    (LFS_MKTAG(0, 0x3ff, 0) & *filtertag))) {
        *filtertag = LFS_MKTAG(LFS_FROM_NOOP, 0, 0);
        return true;
    }

    // check if we need to adjust for created/deleted tags
    if (lfs_tag_type1(tag) == LFS_TYPE_SPLICE &&
            lfs_tag_id(tag) <= lfs_tag_id(*filtertag)) {
        *filtertag += LFS_MKTAG(0, lfs_tag_splice(tag), 0);
    }

    return false;
}
#endif

#ifndef LFS_READONLY
// maximum recursive depth of lfs_dir_traverse, the deepest call:
//
// traverse with commit
// '-> traverse with move
//     '-> traverse with filter
//
#define LFS_DIR_TRAVERSE_DEPTH 3

struct lfs_dir_traverse {
    const lfs_mdir_t *dir;
    lfs_off_t off;
    lfs_tag_t ptag;
    const struct lfs_mattr *attrs;
    int attrcount;

    lfs_tag_t tmask;
    lfs_tag_t ttag;
    uint16_t begin;
    uint16_t end;
    int16_t diff;

    int (*cb)(void *data, lfs_tag_t tag, const void *buffer);
    void *data;

    lfs_tag_t tag;
    const void *buffer;
    struct lfs_diskoff disk;
};

static int lfs_dir_traverse(lfs_t *lfs,
        const lfs_mdir_t *dir, lfs_off_t off, lfs_tag_t ptag,
        const struct lfs_mattr *attrs, int attrcount,
        lfs_tag_t tmask, lfs_tag_t ttag,
        uint16_t begin, uint16_t end, int16_t diff,
        int (*cb)(void *data, lfs_tag_t tag, const void *buffer), void *data) {
    // This function in inherently recursive, but bounded. To allow tool-based
    // analysis without unnecessary code-cost we use an explicit stack
    struct lfs_dir_traverse stack[LFS_DIR_TRAVERSE_DEPTH-1];
    unsigned sp = 0;
    int res;

    // iterate over directory and attrs
    lfs_tag_t tag;
    const void *buffer;
    struct lfs_diskoff disk;
    while (true) {
        {
            if (off+lfs_tag_dsize(ptag) < dir->off) {
                off += lfs_tag_dsize(ptag);
                int err = lfs_bd_read(lfs,
                        NULL, &lfs->rcache, sizeof(tag),
                        dir->pair[0], off, &tag, sizeof(tag));
                if (err) {
                    return err;
                }

                tag = (lfs_frombe32(tag) ^ ptag) | 0x80000000;
                disk.block = dir->pair[0];
                disk.off = off+sizeof(lfs_tag_t);
                buffer = &disk;
                ptag = tag;
            } else if (attrcount > 0) {
                tag = attrs[0].tag;
                buffer = attrs[0].buffer;
                attrs += 1;
                attrcount -= 1;
            } else {
                // finished traversal, pop from stack?
                res = 0;
                break;
            }

            // do we need to filter?
            lfs_tag_t mask = LFS_MKTAG(0x7ff, 0, 0);
            if ((mask & tmask & tag) != (mask & tmask & ttag)) {
                continue;
            }

            if (lfs_tag_id(tmask) != 0) {
                LFS_ASSERT(sp < LFS_DIR_TRAVERSE_DEPTH);
                // recurse, scan for duplicates, and update tag based on
                // creates/deletes
                stack[sp] = (struct lfs_dir_traverse){
                    .dir        = dir,
                    .off        = off,
                    .ptag       = ptag,
                    .attrs      = attrs,
                    .attrcount  = attrcount,
                    .tmask      = tmask,
                    .ttag       = ttag,
                    .begin      = begin,
                    .end        = end,
                    .diff       = diff,
                    .cb         = cb,
                    .data       = data,
                    .tag        = tag,
                    .buffer     = buffer,
                    .disk       = disk,
                };
                sp += 1;

                tmask = 0;
                ttag = 0;
                begin = 0;
                end = 0;
                diff = 0;
                cb = lfs_dir_traverse_filter;
                data = &stack[sp-1].tag;
                continue;
            }
        }

popped:
        // in filter range?
        if (lfs_tag_id(tmask) != 0 &&
                !(lfs_tag_id(tag) >= begin && lfs_tag_id(tag) < end)) {
            continue;
        }

        // handle special cases for mcu-side operations
        if (lfs_tag_type3(tag) == LFS_FROM_NOOP) {
            // do nothing
        } else if (lfs_tag_type3(tag) == LFS_FROM_MOVE) {
            // Without this condition, lfs_dir_traverse can exhibit an
            // extremely expensive O(n^3) of nested loops when renaming.
            // This happens because lfs_dir_traverse tries to filter tags by
            // the tags in the source directory, triggering a second
            // lfs_dir_traverse with its own filter operation.
            //
            // traverse with commit
            // '-> traverse with filter
            //     '-> traverse with move
            //         '-> traverse with filter
            //
            // However we don't actually care about filtering the second set of
            // tags, since duplicate tags have no effect when filtering.
            //
            // This check skips this unnecessary recursive filtering explicitly,
            // reducing this runtime from O(n^3) to O(n^2).
            if (cb == lfs_dir_traverse_filter) {
                continue;
            }

            // recurse into move
            stack[sp] = (struct lfs_dir_traverse){
                .dir        = dir,
                .off        = off,
                .ptag       = ptag,
                .attrs      = attrs,
                .attrcount  = attrcount,
                .tmask      = tmask,
                .ttag       = ttag,
                .begin      = begin,
                .end        = end,
                .diff       = diff,
                .cb         = cb,
                .data       = data,
                .tag        = LFS_MKTAG(LFS_FROM_NOOP, 0, 0),
            };
            sp += 1;

            uint16_t fromid = lfs_tag_size(tag);
            uint16_t toid = lfs_tag_id(tag);
            dir = buffer;
            off = 0;
            ptag = 0xffffffff;
            attrs = NULL;
            attrcount = 0;
            tmask = LFS_MKTAG(0x600, 0x3ff, 0);
            ttag = LFS_MKTAG(LFS_TYPE_STRUCT, 0, 0);
            begin = fromid;
            end = fromid+1;
            diff = toid-fromid+diff;
        } else if (lfs_tag_type3(tag) == LFS_FROM_USERATTRS) {
            for (unsigned i = 0; i < lfs_tag_size(tag); i++) {
                const struct lfs_attr *a = buffer;
                res = cb(data, LFS_MKTAG(LFS_TYPE_USERATTR + a[i].type,
                        lfs_tag_id(tag) + diff, a[i].size), a[i].buffer);
                if (res < 0) {
                    return res;
                }

                if (res) {
                    break;
                }
            }
        } else {
            res = cb(data, tag + LFS_MKTAG(0, diff, 0), buffer);
            if (res < 0) {
                return res;
            }

            if (res) {
                break;
            }
        }
    }

    if (sp > 0) {
        // pop from the stack and return, fortunately all pops share
        // a destination
        dir         = stack[sp-1].dir;
        off         = stack[sp-1].off;
        ptag        = stack[sp-1].ptag;
        attrs       = stack[sp-1].attrs;
        attrcount   = stack[sp-1].attrcount;
        tmask       = stack[sp-1].tmask;
        ttag        = stack[sp-1].ttag;
        begin       = stack[sp-1].begin;
        end         = stack[sp-1].end;
        diff        = stack[sp-1].diff;
        cb          = stack[sp-1].cb;
        data        = stack[sp-1].data;
        tag         = stack[sp-1].tag;
        buffer      = stack[sp-1].buffer;
        disk        = stack[sp-1].disk;
        sp -= 1;
        goto popped;
    } else {
        return res;
    }
}
#endif

static lfs_stag_t lfs_dir_fetchmatch(lfs_t *lfs,
        lfs_mdir_t *dir, const lfs_block_t pair[2],
        lfs_tag_t fmask, lfs_tag_t ftag, uint16_t *id,
        int (*cb)(void *data, lfs_tag_t tag, const void *buffer), void *data) {
    // we can find tag very efficiently during a fetch, since we're already
    // scanning the entire directory
    lfs_stag_t besttag = -1;

    // if either block address is invalid we return LFS_ERR_CORRUPT here,
    // otherwise later writes to the pair could fail
    if (pair[0] >= lfs->cfg->block_count || pair[1] >= lfs->cfg->block_count) {
        return LFS_ERR_CORRUPT;
    }

    // find the block with the most recent revision
    uint32_t revs[2] = {0, 0};
    int r = 0;
    for (int i = 0; i < 2; i++) {
        int err = lfs_bd_read(lfs,
                NULL, &lfs->rcache, sizeof(revs[i]),
                pair[i], 0, &revs[i], sizeof(revs[i]));
        revs[i] = lfs_fromle32(revs[i]);
        if (err && err != LFS_ERR_CORRUPT) {
            return err;
        }

        if (err != LFS_ERR_CORRUPT &&
                lfs_scmp(revs[i], revs[(i+1)%2]) > 0) {
            r = i;
        }
    }

    dir->pair[0] = pair[(r+0)%2];
    dir->pair[1] = pair[(r+1)%2];
    dir->rev = revs[(r+0)%2];
    dir->off = 0; // nonzero = found some commits

    // now scan tags to fetch the actual dir and find possible match
    for (int i = 0; i < 2; i++) {
        lfs_off_t off = 0;
        lfs_tag_t ptag = 0xffffffff;

        uint16_t tempcount = 0;
        lfs_block_t temptail[2] = {LFS_BLOCK_NULL, LFS_BLOCK_NULL};
        bool tempsplit = false;
        lfs_stag_t tempbesttag = besttag;

        // assume not erased until proven otherwise
        bool maybeerased = false;
        bool hasfcrc = false;
        struct lfs_fcrc fcrc;

        dir->rev = lfs_tole32(dir->rev);
        uint32_t crc = lfs_crc(0xffffffff, &dir->rev, sizeof(dir->rev));
        dir->rev = lfs_fromle32(dir->rev);

        while (true) {
            // extract next tag
            lfs_tag_t tag;
            off += lfs_tag_dsize(ptag);
            int err = lfs_bd_read(lfs,
                    NULL, &lfs->rcache, lfs->cfg->block_size,
                    dir->pair[0], off, &tag, sizeof(tag));
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    // can't continue?
                    break;
                }
                return err;
            }

            crc = lfs_crc(crc, &tag, sizeof(tag));
            tag = lfs_frombe32(tag) ^ ptag;

            // next commit not yet programmed?
            if (!lfs_tag_isvalid(tag)) {
                maybeerased = true;
                break;
            // out of range?
            } else if (off + lfs_tag_dsize(tag) > lfs->cfg->block_size) {
                break;
            }

            ptag = tag;

            if (lfs_tag_type2(tag) == LFS_TYPE_CCRC) {
                // check the crc attr
                uint32_t dcrc;
                err = lfs_bd_read(lfs,
                        NULL, &lfs->rcache, lfs->cfg->block_size,
                        dir->pair[0], off+sizeof(tag), &dcrc, sizeof(dcrc));
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        break;
                    }
                    return err;
                }
                dcrc = lfs_fromle32(dcrc);

                if (crc != dcrc) {
                    break;
                }

                // reset the next bit if we need to
                ptag ^= (lfs_tag_t)(lfs_tag_chunk(tag) & 1U) << 31;

                // toss our crc into the filesystem seed for
                // pseudorandom numbers, note we use another crc here
                // as a collection function because it is sufficiently
                // random and convenient
                lfs->seed = lfs_crc(lfs->seed, &crc, sizeof(crc));

                // update with what's found so far
                besttag = tempbesttag;
                dir->off = off + lfs_tag_dsize(tag);
                dir->etag = ptag;
                dir->count = tempcount;
                dir->tail[0] = temptail[0];
                dir->tail[1] = temptail[1];
                dir->split = tempsplit;

                // reset crc
                crc = 0xffffffff;
                continue;
            }

            // fcrc is only valid when last tag was a crc
            hasfcrc = false;

            // crc the entry first, hopefully leaving it in the cache
            err = lfs_bd_crc(lfs,
                    NULL, &lfs->rcache, lfs->cfg->block_size,
                    dir->pair[0], off+sizeof(tag),
                    lfs_tag_dsize(tag)-sizeof(tag), &crc);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    break;
                }
                return err;
            }

            // directory modification tags?
            if (lfs_tag_type1(tag) == LFS_TYPE_NAME) {
                // increase count of files if necessary
                if (lfs_tag_id(tag) >= tempcount) {
                    tempcount = lfs_tag_id(tag) + 1;
                }
            } else if (lfs_tag_type1(tag) == LFS_TYPE_SPLICE) {
                tempcount += lfs_tag_splice(tag);

                if (tag == (LFS_MKTAG(LFS_TYPE_DELETE, 0, 0) |
                        (LFS_MKTAG(0, 0x3ff, 0) & tempbesttag))) {
                    tempbesttag |= 0x80000000;
                } else if (tempbesttag != -1 &&
                        lfs_tag_id(tag) <= lfs_tag_id(tempbesttag)) {
                    tempbesttag += LFS_MKTAG(0, lfs_tag_splice(tag), 0);
                }
            } else if (lfs_tag_type1(tag) == LFS_TYPE_TAIL) {
                tempsplit = (lfs_tag_chunk(tag) & 1);

                err = lfs_bd_read(lfs,
                        NULL, &lfs->rcache, lfs->cfg->block_size,
                        dir->pair[0], off+sizeof(tag), &temptail, 8);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        break;
                    }
                    return err;
                }
                lfs_pair_fromle32(temptail);
            } else if (lfs_tag_type3(tag) == LFS_TYPE_FCRC) {
                err = lfs_bd_read(lfs,
                        NULL, &lfs->rcache, lfs->cfg->block_size,
                        dir->pair[0], off+sizeof(tag),
                        &fcrc, sizeof(fcrc));
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        break;
                    }
                }

                lfs_fcrc_fromle32(&fcrc);
                hasfcrc = true;
            }

            // found a match for our fetcher?
            if ((fmask & tag) == (fmask & ftag)) {
                int res = cb(data, tag, &(struct lfs_diskoff){
                        dir->pair[0], off+sizeof(tag)});
                if (res < 0) {
                    if (res == LFS_ERR_CORRUPT) {
                        break;
                    }
                    return res;
                }

                if (res == LFS_CMP_EQ) {
                    // found a match
                    tempbesttag = tag;
                } else if ((LFS_MKTAG(0x7ff, 0x3ff, 0) & tag) ==
                        (LFS_MKTAG(0x7ff, 0x3ff, 0) & tempbesttag)) {
                    // found an identical tag, but contents didn't match
                    // this must mean that our besttag has been overwritten
                    tempbesttag = -1;
                } else if (res == LFS_CMP_GT &&
                        lfs_tag_id(tag) <= lfs_tag_id(tempbesttag)) {
                    // found a greater match, keep track to keep things sorted
                    tempbesttag = tag | 0x80000000;
                }
            }
        }

        // found no valid commits?
        if (dir->off == 0) {
            // try the other block?
            lfs_pair_swap(dir->pair);
            dir->rev = revs[(r+1)%2];
            continue;
        }

        // did we end on a valid commit? we may have an erased block
        dir->erased = false;
        if (maybeerased && hasfcrc && dir->off % lfs->cfg->prog_size == 0) {
            // check for an fcrc matching the next prog's erased state, if
            // this failed most likely a previous prog was interrupted, we
            // need a new erase
            uint32_t fcrc_ = 0xffffffff;
            int err = lfs_bd_crc(lfs,
                    NULL, &lfs->rcache, lfs->cfg->block_size,
                    dir->pair[0], dir->off, fcrc.size, &fcrc_);
            if (err && err != LFS_ERR_CORRUPT) {
                return err;
            }

            // found beginning of erased part?
            dir->erased = (fcrc_ == fcrc.crc);
        }

        // synthetic move
        if (lfs_gstate_hasmovehere(&lfs->gdisk, dir->pair)) {
            if (lfs_tag_id(lfs->gdisk.tag) == lfs_tag_id(besttag)) {
                besttag |= 0x80000000;
            } else if (besttag != -1 &&
                    lfs_tag_id(lfs->gdisk.tag) < lfs_tag_id(besttag)) {
                besttag -= LFS_MKTAG(0, 1, 0);
            }
        }

        // found tag? or found best id?
        if (id) {
            *id = lfs_min(lfs_tag_id(besttag), dir->count);
        }

        if (lfs_tag_isvalid(besttag)) {
            return besttag;
        } else if (lfs_tag_id(besttag) < dir->count) {
            return LFS_ERR_NOENT;
        } else {
            return 0;
        }
    }

    LFS_ERROR("Corrupted dir pair at {0x%"PRIx32", 0x%"PRIx32"}",
            dir->pair[0], dir->pair[1]);
    return LFS_ERR_CORRUPT;
}

static int lfs_dir_fetch(lfs_t *lfs,
        lfs_mdir_t *dir, const lfs_block_t pair[2]) {
    // note, mask=-1, tag=-1 can never match a tag since this
    // pattern has the invalid bit set
    return (int)lfs_dir_fetchmatch(lfs, dir, pair,
            (lfs_tag_t)-1, (lfs_tag_t)-1, NULL, NULL, NULL);
}

static int lfs_dir_getgstate(lfs_t *lfs, const lfs_mdir_t *dir,
        lfs_gstate_t *gstate) {
    lfs_gstate_t temp;
    lfs_stag_t res = lfs_dir_get(lfs, dir, LFS_MKTAG(0x7ff, 0, 0),
            LFS_MKTAG(LFS_TYPE_MOVESTATE, 0, sizeof(temp)), &temp);
    if (res < 0 && res != LFS_ERR_NOENT) {
        return res;
    }

    if (res != LFS_ERR_NOENT) {
        // xor together to find resulting gstate
        lfs_gstate_fromle32(&temp);
        lfs_gstate_xor(gstate, &temp);
    }

    return 0;
}

static int lfs_dir_getinfo(lfs_t *lfs, lfs_mdir_t *dir,
        uint16_t id, struct lfs_info *info) {
    if (id == 0x3ff) {
        // special case for root
        strcpy(info->name, "/");
        info->type = LFS_TYPE_DIR;
        return 0;
    }

    lfs_stag_t tag = lfs_dir_get(lfs, dir, LFS_MKTAG(0x780, 0x3ff, 0),
            LFS_MKTAG(LFS_TYPE_NAME, id, lfs->name_max+1), info->name);
    if (tag < 0) {
        return (int)tag;
    }

    info->type = lfs_tag_type3(tag);

    struct lfs_ctz ctz;
    tag = lfs_dir_get(lfs, dir, LFS_MKTAG(0x700, 0x3ff, 0),
            LFS_MKTAG(LFS_TYPE_STRUCT, id, sizeof(ctz)), &ctz);
    if (tag < 0) {
        return (int)tag;
    }
    lfs_ctz_fromle32(&ctz);

    if (lfs_tag_type3(tag) == LFS_TYPE_CTZSTRUCT) {
        info->size = ctz.size;
    } else if (lfs_tag_type3(tag) == LFS_TYPE_INLINESTRUCT) {
        info->size = lfs_tag_size(tag);
    }

    return 0;
}

struct lfs_dir_find_match {
    lfs_t *lfs;
    const void *name;
    lfs_size_t size;
};

static int lfs_dir_find_match(void *data,
        lfs_tag_t tag, const void *buffer) {
    struct lfs_dir_find_match *name = data;
    lfs_t *lfs = name->lfs;
    const struct lfs_diskoff *disk = buffer;

    // compare with disk
    lfs_size_t diff = lfs_min(name->size, lfs_tag_size(tag));
    int res = lfs_bd_cmp(lfs,
            NULL, &lfs->rcache, diff,
            disk->block, disk->off, name->name, diff);
    if (res != LFS_CMP_EQ) {
        return res;
    }

    // only equal if our size is still the same
    if (name->size != lfs_tag_size(tag)) {
        return (name->size < lfs_tag_size(tag)) ? LFS_CMP_LT : LFS_CMP_GT;
    }

    // found a match!
    return LFS_CMP_EQ;
}

static lfs_stag_t lfs_dir_find(lfs_t *lfs, lfs_mdir_t *dir,
        const char **path, uint16_t *id) {
    // we reduce path to a single name if we can find it
    const char *name = *path;
    if (id) {
        *id = 0x3ff;
    }

    // default to root dir
    lfs_stag_t tag = LFS_MKTAG(LFS_TYPE_DIR, 0x3ff, 0);
    dir->tail[0] = lfs->root[0];
    dir->tail[1] = lfs->root[1];

    while (true) {
nextname:
        // skip slashes
        name += strspn(name, "/");
        lfs_size_t namelen = strcspn(name, "/");

        // skip '.' and root '..'
        if ((namelen == 1 && memcmp(name, ".", 1) == 0) ||
            (namelen == 2 && memcmp(name, "..", 2) == 0)) {
            name += namelen;
            goto nextname;
        }

        // skip if matched by '..' in name
        const char *suffix = name + namelen;
        lfs_size_t sufflen;
        int depth = 1;
        while (true) {
            suffix += strspn(suffix, "/");
            sufflen = strcspn(suffix, "/");
            if (sufflen == 0) {
                break;
            }

            if (sufflen == 2 && memcmp(suffix, "..", 2) == 0) {
                depth -= 1;
                if (depth == 0) {
                    name = suffix + sufflen;
                    goto nextname;
                }
            } else {
                depth += 1;
            }

            suffix += sufflen;
        }

        // found path
        if (name[0] == '\0') {
            return tag;
        }

        // update what we've found so far
        *path = name;

        // only continue if we hit a directory
        if (lfs_tag_type3(tag) != LFS_TYPE_DIR) {
            return LFS_ERR_NOTDIR;
        }

        // grab the entry data
        if (lfs_tag_id(tag) != 0x3ff) {
            lfs_stag_t res = lfs_dir_get(lfs, dir, LFS_MKTAG(0x700, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_STRUCT, lfs_tag_id(tag), 8), dir->tail);
            if (res < 0) {
                return res;
            }
            lfs_pair_fromle32(dir->tail);
        }

        // find entry matching name
        while (true) {
            tag = lfs_dir_fetchmatch(lfs, dir, dir->tail,
                    LFS_MKTAG(0x780, 0, 0),
                    LFS_MKTAG(LFS_TYPE_NAME, 0, namelen),
                     // are we last name?
                    (strchr(name, '/') == NULL) ? id : NULL,
                    lfs_dir_find_match, &(struct lfs_dir_find_match){
                        lfs, name, namelen});
            if (tag < 0) {
                return tag;
            }

            if (tag) {
                break;
            }

            if (!dir->split) {
                return LFS_ERR_NOENT;
            }
        }

        // to next name
        name += namelen;
    }
}

// commit logic
struct lfs_commit {
    lfs_block_t block;
    lfs_off_t off;
    lfs_tag_t ptag;
    uint32_t crc;

    lfs_off_t begin;
    lfs_off_t end;
};

#ifndef LFS_READONLY
static int lfs_dir_commitprog(lfs_t *lfs, struct lfs_commit *commit,
        const void *buffer, lfs_size_t size) {
    int err = lfs_bd_prog(lfs,
            &lfs->pcache, &lfs->rcache, false,
            commit->block, commit->off ,
            (const uint8_t*)buffer, size);
    if (err) {
        return err;
    }

    commit->crc = lfs_crc(commit->crc, buffer, size);
    commit->off += size;
    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_commitattr(lfs_t *lfs, struct lfs_commit *commit,
        lfs_tag_t tag, const void *buffer) {
    // check if we fit
    lfs_size_t dsize = lfs_tag_dsize(tag);
    if (commit->off + dsize > commit->end) {
        return LFS_ERR_NOSPC;
    }

    // write out tag
    lfs_tag_t ntag = lfs_tobe32((tag & 0x7fffffff) ^ commit->ptag);
    int err = lfs_dir_commitprog(lfs, commit, &ntag, sizeof(ntag));
    if (err) {
        return err;
    }

    if (!(tag & 0x80000000)) {
        // from memory
        err = lfs_dir_commitprog(lfs, commit, buffer, dsize-sizeof(tag));
        if (err) {
            return err;
        }
    } else {
        // from disk
        const struct lfs_diskoff *disk = buffer;
        for (lfs_off_t i = 0; i < dsize-sizeof(tag); i++) {
            // rely on caching to make this efficient
            uint8_t dat;
            err = lfs_bd_read(lfs,
                    NULL, &lfs->rcache, dsize-sizeof(tag)-i,
                    disk->block, disk->off+i, &dat, 1);
            if (err) {
                return err;
            }

            err = lfs_dir_commitprog(lfs, commit, &dat, 1);
            if (err) {
                return err;
            }
        }
    }

    commit->ptag = tag & 0x7fffffff;
    return 0;
}
#endif

#ifndef LFS_READONLY

static int lfs_dir_commitcrc(lfs_t *lfs, struct lfs_commit *commit) {
    // align to program units
    //
    // this gets a bit complex as we have two types of crcs:
    // - 5-word crc with fcrc to check following prog (middle of block)
    // - 2-word crc with no following prog (end of block)
    const lfs_off_t end = lfs_alignup(
            lfs_min(commit->off + 5*sizeof(uint32_t), lfs->cfg->block_size),
            lfs->cfg->prog_size);

    lfs_off_t off1 = 0;
    uint32_t crc1 = 0;

    // create crc tags to fill up remainder of commit, note that
    // padding is not crced, which lets fetches skip padding but
    // makes committing a bit more complicated
    while (commit->off < end) {
        lfs_off_t noff = (
                lfs_min(end - (commit->off+sizeof(lfs_tag_t)), 0x3fe)
                + (commit->off+sizeof(lfs_tag_t)));
        // too large for crc tag? need padding commits
        if (noff < end) {
            noff = lfs_min(noff, end - 5*sizeof(uint32_t));
        }

        // space for fcrc?
        uint8_t eperturb = -1;
        if (noff >= end && noff <= lfs->cfg->block_size - lfs->cfg->prog_size) {
            // first read the leading byte, this always contains a bit
            // we can perturb to avoid writes that don't change the fcrc
            int err = lfs_bd_read(lfs,
                    NULL, &lfs->rcache, lfs->cfg->prog_size,
                    commit->block, noff, &eperturb, 1);
            if (err && err != LFS_ERR_CORRUPT) {
                return err;
            }

            // find the expected fcrc, don't bother avoiding a reread
            // of the eperturb, it should still be in our cache
            struct lfs_fcrc fcrc = {.size=lfs->cfg->prog_size, .crc=0xffffffff};
            err = lfs_bd_crc(lfs,
                    NULL, &lfs->rcache, lfs->cfg->prog_size,
                    commit->block, noff, fcrc.size, &fcrc.crc);
            if (err && err != LFS_ERR_CORRUPT) {
                return err;
            }

            lfs_fcrc_tole32(&fcrc);
            err = lfs_dir_commitattr(lfs, commit,
                    LFS_MKTAG(LFS_TYPE_FCRC, 0x3ff, sizeof(struct lfs_fcrc)),
                    &fcrc);
            if (err) {
                return err;
            }
        }

        // build commit crc
        struct {
            lfs_tag_t tag;
            uint32_t crc;
        } ccrc;
        lfs_tag_t ntag = LFS_MKTAG(
                LFS_TYPE_CCRC + (((uint8_t)~eperturb) >> 7), 0x3ff,
                noff - (commit->off+sizeof(lfs_tag_t)));
        ccrc.tag = lfs_tobe32(ntag ^ commit->ptag);
        commit->crc = lfs_crc(commit->crc, &ccrc.tag, sizeof(lfs_tag_t));
        ccrc.crc = lfs_tole32(commit->crc);

        int err = lfs_bd_prog(lfs,
                &lfs->pcache, &lfs->rcache, false,
                commit->block, commit->off, &ccrc, sizeof(ccrc));
        if (err) {
            return err;
        }

        // keep track of non-padding checksum to verify
        if (off1 == 0) {
            off1 = commit->off + sizeof(lfs_tag_t);
            crc1 = commit->crc;
        }

        commit->off = noff;
        // perturb valid bit?
        commit->ptag = ntag ^ ((0x80 & ~eperturb) << 24);
        // reset crc for next commit
        commit->crc = 0xffffffff;

        // manually flush here since we don't prog the padding, this confuses
        // the caching layer
        if (noff >= end || noff >= lfs->pcache.off + lfs->cfg->cache_size) {
            // flush buffers
            int err = lfs_bd_sync(lfs, &lfs->pcache, &lfs->rcache, false);
            if (err) {
                return err;
            }
        }
    }

    // successful commit, check checksums to make sure
    //
    // note that we don't need to check padding commits, worst
    // case if they are corrupted we would have had to compact anyways
    lfs_off_t off = commit->begin;
    uint32_t crc = 0xffffffff;
    int err = lfs_bd_crc(lfs,
            NULL, &lfs->rcache, off1+sizeof(uint32_t),
            commit->block, off, off1-off, &crc);
    if (err) {
        return err;
    }

    // check non-padding commits against known crc
    if (crc != crc1) {
        return LFS_ERR_CORRUPT;
    }

    // make sure to check crc in case we happen to pick
    // up an unrelated crc (frozen block?)
    err = lfs_bd_crc(lfs,
            NULL, &lfs->rcache, sizeof(uint32_t),
            commit->block, off1, sizeof(uint32_t), &crc);
    if (err) {
        return err;
    }

    if (crc != 0) {
        return LFS_ERR_CORRUPT;
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_alloc(lfs_t *lfs, lfs_mdir_t *dir) {
    // allocate pair of dir blocks (backwards, so we write block 1 first)
    for (int i = 0; i < 2; i++) {
        int err = lfs_alloc(lfs, &dir->pair[(i+1)%2]);
        if (err) {
            return err;
        }
    }

    // zero for reproducibility in case initial block is unreadable
    dir->rev = 0;

    // rather than clobbering one of the blocks we just pretend
    // the revision may be valid
    int err = lfs_bd_read(lfs,
            NULL, &lfs->rcache, sizeof(dir->rev),
            dir->pair[0], 0, &dir->rev, sizeof(dir->rev));
    dir->rev = lfs_fromle32(dir->rev);
    if (err && err != LFS_ERR_CORRUPT) {
        return err;
    }

    // to make sure we don't immediately evict, align the new revision count
    // to our block_cycles modulus, see lfs_dir_compact for why our modulus
    // is tweaked this way
    if (lfs->cfg->block_cycles > 0) {
        dir->rev = lfs_alignup(dir->rev, ((lfs->cfg->block_cycles+1)|1));
    }

    // set defaults
    dir->off = sizeof(dir->rev);
    dir->etag = 0xffffffff;
    dir->count = 0;
    dir->tail[0] = LFS_BLOCK_NULL;
    dir->tail[1] = LFS_BLOCK_NULL;
    dir->erased = false;
    dir->split = false;

    // don't write out yet, let caller take care of that
    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_drop(lfs_t *lfs, lfs_mdir_t *dir, lfs_mdir_t *tail) {
    // steal state
    int err = lfs_dir_getgstate(lfs, tail, &lfs->gdelta);
    if (err) {
        return err;
    }

    // steal tail
    lfs_pair_tole32(tail->tail);
    err = lfs_dir_commit(lfs, dir, LFS_MKATTRS(
            {LFS_MKTAG(LFS_TYPE_TAIL + tail->split, 0x3ff, 8), tail->tail}));
    lfs_pair_fromle32(tail->tail);
    if (err) {
        return err;
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_split(lfs_t *lfs,
        lfs_mdir_t *dir, const struct lfs_mattr *attrs, int attrcount,
        lfs_mdir_t *source, uint16_t split, uint16_t end) {
    // create tail metadata pair
    lfs_mdir_t tail;
    int err = lfs_dir_alloc(lfs, &tail);
    if (err) {
        return err;
    }

    tail.split = dir->split;
    tail.tail[0] = dir->tail[0];
    tail.tail[1] = dir->tail[1];

    // note we don't care about LFS_OK_RELOCATED
    int res = lfs_dir_compact(lfs, &tail, attrs, attrcount, source, split, end);
    if (res < 0) {
        return res;
    }

    dir->tail[0] = tail.pair[0];
    dir->tail[1] = tail.pair[1];
    dir->split = true;

    // update root if needed
    if (lfs_pair_cmp(dir->pair, lfs->root) == 0 && split == 0) {
        lfs->root[0] = tail.pair[0];
        lfs->root[1] = tail.pair[1];
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_commit_size(void *p, lfs_tag_t tag, const void *buffer) {
    lfs_size_t *size = p;
    (void)buffer;

    *size += lfs_tag_dsize(tag);
    return 0;
}
#endif

#ifndef LFS_READONLY
struct lfs_dir_commit_commit {
    lfs_t *lfs;
    struct lfs_commit *commit;
};
#endif

#ifndef LFS_READONLY
static int lfs_dir_commit_commit(void *p, lfs_tag_t tag, const void *buffer) {
    struct lfs_dir_commit_commit *commit = p;
    return lfs_dir_commitattr(commit->lfs, commit->commit, tag, buffer);
}
#endif

#ifndef LFS_READONLY
static bool lfs_dir_needsrelocation(lfs_t *lfs, lfs_mdir_t *dir) {
    // If our revision count == n * block_cycles, we should force a relocation,
    // this is how littlefs wear-levels at the metadata-pair level. Note that we
    // actually use (block_cycles+1)|1, this is to avoid two corner cases:
    // 1. block_cycles = 1, which would prevent relocations from terminating
    // 2. block_cycles = 2n, which, due to aliasing, would only ever relocate
    //    one metadata block in the pair, effectively making this useless
    return (lfs->cfg->block_cycles > 0
            && ((dir->rev + 1) % ((lfs->cfg->block_cycles+1)|1) == 0));
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_compact(lfs_t *lfs,
        lfs_mdir_t *dir, const struct lfs_mattr *attrs, int attrcount,
        lfs_mdir_t *source, uint16_t begin, uint16_t end) {
    // save some state in case block is bad
    bool relocated = false;
    bool tired = lfs_dir_needsrelocation(lfs, dir);

    // increment revision count
    dir->rev += 1;

    // do not proactively relocate blocks during migrations, this
    // can cause a number of failure states such: clobbering the
    // v1 superblock if we relocate root, and invalidating directory
    // pointers if we relocate the head of a directory. On top of
    // this, relocations increase the overall complexity of
    // lfs_migration, which is already a delicate operation.
#ifdef LFS_MIGRATE
    if (lfs->lfs1) {
        tired = false;
    }
#endif

    if (tired && lfs_pair_cmp(dir->pair, (const lfs_block_t[2]){0, 1}) != 0) {
        // we're writing too much, time to relocate
        goto relocate;
    }

    // begin loop to commit compaction to blocks until a compact sticks
    while (true) {
        {
            // setup commit state
            struct lfs_commit commit = {
                .block = dir->pair[1],
                .off = 0,
                .ptag = 0xffffffff,
                .crc = 0xffffffff,

                .begin = 0,
                .end = (lfs->cfg->metadata_max ?
                    lfs->cfg->metadata_max : lfs->cfg->block_size) - 8,
            };

            // erase block to write to
            int err = lfs_bd_erase(lfs, dir->pair[1]);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // write out header
            dir->rev = lfs_tole32(dir->rev);
            err = lfs_dir_commitprog(lfs, &commit,
                    &dir->rev, sizeof(dir->rev));
            dir->rev = lfs_fromle32(dir->rev);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // traverse the directory, this time writing out all unique tags
            err = lfs_dir_traverse(lfs,
                    source, 0, 0xffffffff, attrs, attrcount,
                    LFS_MKTAG(0x400, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_NAME, 0, 0),
                    begin, end, -begin,
                    lfs_dir_commit_commit, &(struct lfs_dir_commit_commit){
                        lfs, &commit});
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // commit tail, which may be new after last size check
            if (!lfs_pair_isnull(dir->tail)) {
                lfs_pair_tole32(dir->tail);
                err = lfs_dir_commitattr(lfs, &commit,
                        LFS_MKTAG(LFS_TYPE_TAIL + dir->split, 0x3ff, 8),
                        dir->tail);
                lfs_pair_fromle32(dir->tail);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        goto relocate;
                    }
                    return err;
                }
            }

            // bring over gstate?
            lfs_gstate_t delta = {0};
            if (!relocated) {
                lfs_gstate_xor(&delta, &lfs->gdisk);
                lfs_gstate_xor(&delta, &lfs->gstate);
            }
            lfs_gstate_xor(&delta, &lfs->gdelta);
            delta.tag &= ~LFS_MKTAG(0, 0, 0x3ff);

            err = lfs_dir_getgstate(lfs, dir, &delta);
            if (err) {
                return err;
            }

            if (!lfs_gstate_iszero(&delta)) {
                lfs_gstate_tole32(&delta);
                err = lfs_dir_commitattr(lfs, &commit,
                        LFS_MKTAG(LFS_TYPE_MOVESTATE, 0x3ff,
                            sizeof(delta)), &delta);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        goto relocate;
                    }
                    return err;
                }
            }

            // complete commit with crc
            err = lfs_dir_commitcrc(lfs, &commit);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // successful compaction, swap dir pair to indicate most recent
            LFS_ASSERT(commit.off % lfs->cfg->prog_size == 0);
            lfs_pair_swap(dir->pair);
            dir->count = end - begin;
            dir->off = commit.off;
            dir->etag = commit.ptag;
            // update gstate
            lfs->gdelta = (lfs_gstate_t){0};
            if (!relocated) {
                lfs->gdisk = lfs->gstate;
            }
        }
        break;

relocate:
        // commit was corrupted, drop caches and prepare to relocate block
        relocated = true;
        lfs_cache_drop(lfs, &lfs->pcache);
        if (!tired) {
            LFS_DEBUG("Bad block at 0x%"PRIx32, dir->pair[1]);
        }

        // can't relocate superblock, filesystem is now frozen
        if (lfs_pair_cmp(dir->pair, (const lfs_block_t[2]){0, 1}) == 0) {
            LFS_WARN("Superblock 0x%"PRIx32" has become unwritable",
                    dir->pair[1]);
            return LFS_ERR_NOSPC;
        }

        // relocate half of pair
        int err = lfs_alloc(lfs, &dir->pair[1]);
        if (err && (err != LFS_ERR_NOSPC || !tired)) {
            return err;
        }

        tired = false;
        continue;
    }

    return relocated ? LFS_OK_RELOCATED : 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_splittingcompact(lfs_t *lfs, lfs_mdir_t *dir,
        const struct lfs_mattr *attrs, int attrcount,
        lfs_mdir_t *source, uint16_t begin, uint16_t end) {
    while (true) {
        // find size of first split, we do this by halving the split until
        // the metadata is guaranteed to fit
        //
        // Note that this isn't a true binary search, we never increase the
        // split size. This may result in poorly distributed metadata but isn't
        // worth the extra code size or performance hit to fix.
        lfs_size_t split = begin;
        while (end - split > 1) {
            lfs_size_t size = 0;
            int err = lfs_dir_traverse(lfs,
                    source, 0, 0xffffffff, attrs, attrcount,
                    LFS_MKTAG(0x400, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_NAME, 0, 0),
                    split, end, -split,
                    lfs_dir_commit_size, &size);
            if (err) {
                return err;
            }

            // space is complicated, we need room for:
            //
            // - tail:         4+2*4 = 12 bytes
            // - gstate:       4+3*4 = 16 bytes
            // - move delete:  4     = 4 bytes
            // - crc:          4+4   = 8 bytes
            //                 total = 40 bytes
            //
            // And we cap at half a block to avoid degenerate cases with
            // nearly-full metadata blocks.
            //
            if (end - split < 0xff
                    && size <= lfs_min(
                        lfs->cfg->block_size - 40,
                        lfs_alignup(
                            (lfs->cfg->metadata_max
                                ? lfs->cfg->metadata_max
                                : lfs->cfg->block_size)/2,
                            lfs->cfg->prog_size))) {
                break;
            }

            split = split + ((end - split) / 2);
        }

        if (split == begin) {
            // no split needed
            break;
        }

        // split into two metadata pairs and continue
        int err = lfs_dir_split(lfs, dir, attrs, attrcount,
                source, split, end);
        if (err && err != LFS_ERR_NOSPC) {
            return err;
        }

        if (err) {
            // we can't allocate a new block, try to compact with degraded
            // performance
            LFS_WARN("Unable to split {0x%"PRIx32", 0x%"PRIx32"}",
                    dir->pair[0], dir->pair[1]);
            break;
        } else {
            end = split;
        }
    }

    if (lfs_dir_needsrelocation(lfs, dir)
            && lfs_pair_cmp(dir->pair, (const lfs_block_t[2]){0, 1}) == 0) {
        // oh no! we're writing too much to the superblock,
        // should we expand?
        lfs_ssize_t size = lfs_fs_rawsize(lfs);
        if (size < 0) {
            return size;
        }

        // do we have extra space? littlefs can't reclaim this space
        // by itself, so expand cautiously
        if ((lfs_size_t)size < lfs->cfg->block_count/2) {
            LFS_DEBUG("Expanding superblock at rev %"PRIu32, dir->rev);
            int err = lfs_dir_split(lfs, dir, attrs, attrcount,
                    source, begin, end);
            if (err && err != LFS_ERR_NOSPC) {
                return err;
            }

            if (err) {
                // welp, we tried, if we ran out of space there's not much
                // we can do, we'll error later if we've become frozen
                LFS_WARN("Unable to expand superblock");
            } else {
                end = begin;
            }
        }
    }

    return lfs_dir_compact(lfs, dir, attrs, attrcount, source, begin, end);
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_relocatingcommit(lfs_t *lfs, lfs_mdir_t *dir,
        const lfs_block_t pair[2],
        const struct lfs_mattr *attrs, int attrcount,
        lfs_mdir_t *pdir) {
    int state = 0;

    // calculate changes to the directory
    bool hasdelete = false;
    for (int i = 0; i < attrcount; i++) {
        if (lfs_tag_type3(attrs[i].tag) == LFS_TYPE_CREATE) {
            dir->count += 1;
        } else if (lfs_tag_type3(attrs[i].tag) == LFS_TYPE_DELETE) {
            LFS_ASSERT(dir->count > 0);
            dir->count -= 1;
            hasdelete = true;
        } else if (lfs_tag_type1(attrs[i].tag) == LFS_TYPE_TAIL) {
            dir->tail[0] = ((lfs_block_t*)attrs[i].buffer)[0];
            dir->tail[1] = ((lfs_block_t*)attrs[i].buffer)[1];
            dir->split = (lfs_tag_chunk(attrs[i].tag) & 1);
            lfs_pair_fromle32(dir->tail);
        }
    }

    // should we actually drop the directory block?
    if (hasdelete && dir->count == 0) {
        LFS_ASSERT(pdir);
        int err = lfs_fs_pred(lfs, dir->pair, pdir);
        if (err && err != LFS_ERR_NOENT) {
            return err;
        }

        if (err != LFS_ERR_NOENT && pdir->split) {
            state = LFS_OK_DROPPED;
            goto fixmlist;
        }
    }

    if (dir->erased) {
        // try to commit
        struct lfs_commit commit = {
            .block = dir->pair[0],
            .off = dir->off,
            .ptag = dir->etag,
            .crc = 0xffffffff,

            .begin = dir->off,
            .end = (lfs->cfg->metadata_max ?
                lfs->cfg->metadata_max : lfs->cfg->block_size) - 8,
        };

        // traverse attrs that need to be written out
        lfs_pair_tole32(dir->tail);
        int err = lfs_dir_traverse(lfs,
                dir, dir->off, dir->etag, attrs, attrcount,
                0, 0, 0, 0, 0,
                lfs_dir_commit_commit, &(struct lfs_dir_commit_commit){
                    lfs, &commit});
        lfs_pair_fromle32(dir->tail);
        if (err) {
            if (err == LFS_ERR_NOSPC || err == LFS_ERR_CORRUPT) {
                goto compact;
            }
            return err;
        }

        // commit any global diffs if we have any
        lfs_gstate_t delta = {0};
        lfs_gstate_xor(&delta, &lfs->gstate);
        lfs_gstate_xor(&delta, &lfs->gdisk);
        lfs_gstate_xor(&delta, &lfs->gdelta);
        delta.tag &= ~LFS_MKTAG(0, 0, 0x3ff);
        if (!lfs_gstate_iszero(&delta)) {
            err = lfs_dir_getgstate(lfs, dir, &delta);
            if (err) {
                return err;
            }

            lfs_gstate_tole32(&delta);
            err = lfs_dir_commitattr(lfs, &commit,
                    LFS_MKTAG(LFS_TYPE_MOVESTATE, 0x3ff,
                        sizeof(delta)), &delta);
            if (err) {
                if (err == LFS_ERR_NOSPC || err == LFS_ERR_CORRUPT) {
                    goto compact;
                }
                return err;
            }
        }

        // finalize commit with the crc
        err = lfs_dir_commitcrc(lfs, &commit);
        if (err) {
            if (err == LFS_ERR_NOSPC || err == LFS_ERR_CORRUPT) {
                goto compact;
            }
            return err;
        }

        // successful commit, update dir
        LFS_ASSERT(commit.off % lfs->cfg->prog_size == 0);
        dir->off = commit.off;
        dir->etag = commit.ptag;
        // and update gstate
        lfs->gdisk = lfs->gstate;
        lfs->gdelta = (lfs_gstate_t){0};

        goto fixmlist;
    }

compact:
    // fall back to compaction
    lfs_cache_drop(lfs, &lfs->pcache);

    state = lfs_dir_splittingcompact(lfs, dir, attrs, attrcount,
            dir, 0, dir->count);
    if (state < 0) {
        return state;
    }

    goto fixmlist;

fixmlist:;
    // this complicated bit of logic is for fixing up any active
    // metadata-pairs that we may have affected
    //
    // note we have to make two passes since the mdir passed to
    // lfs_dir_commit could also be in this list, and even then
    // we need to copy the pair so they don't get clobbered if we refetch
    // our mdir.
    lfs_block_t oldpair[2] = {pair[0], pair[1]};
    for (struct lfs_mlist *d = lfs->mlist; d; d = d->next) {
        if (lfs_pair_cmp(d->m.pair, oldpair) == 0) {
            d->m = *dir;
            if (d->m.pair != pair) {
                for (int i = 0; i < attrcount; i++) {
                    if (lfs_tag_type3(attrs[i].tag) == LFS_TYPE_DELETE &&
                            d->id == lfs_tag_id(attrs[i].tag)) {
                        d->m.pair[0] = LFS_BLOCK_NULL;
                        d->m.pair[1] = LFS_BLOCK_NULL;
                    } else if (lfs_tag_type3(attrs[i].tag) == LFS_TYPE_DELETE &&
                            d->id > lfs_tag_id(attrs[i].tag)) {
                        d->id -= 1;
                        if (d->type == LFS_TYPE_DIR) {
                            ((lfs_dir_t*)d)->pos -= 1;
                        }
                    } else if (lfs_tag_type3(attrs[i].tag) == LFS_TYPE_CREATE &&
                            d->id >= lfs_tag_id(attrs[i].tag)) {
                        d->id += 1;
                        if (d->type == LFS_TYPE_DIR) {
                            ((lfs_dir_t*)d)->pos += 1;
                        }
                    }
                }
            }

            while (d->id >= d->m.count && d->m.split) {
                // we split and id is on tail now
                d->id -= d->m.count;
                int err = lfs_dir_fetch(lfs, &d->m, d->m.tail);
                if (err) {
                    return err;
                }
            }
        }
    }

    return state;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_orphaningcommit(lfs_t *lfs, lfs_mdir_t *dir,
        const struct lfs_mattr *attrs, int attrcount) {
    // check for any inline files that aren't RAM backed and
    // forcefully evict them, needed for filesystem consistency
    for (lfs_file_t *f = (lfs_file_t*)lfs->mlist; f; f = f->next) {
        if (dir != &f->m && lfs_pair_cmp(f->m.pair, dir->pair) == 0 &&
                f->type == LFS_TYPE_REG && (f->flags & LFS_F_INLINE) &&
                f->ctz.size > lfs->cfg->cache_size) {
            int err = lfs_file_outline(lfs, f);
            if (err) {
                return err;
            }

            err = lfs_file_flush(lfs, f);
            if (err) {
                return err;
            }
        }
    }

    lfs_block_t lpair[2] = {dir->pair[0], dir->pair[1]};
    lfs_mdir_t ldir = *dir;
    lfs_mdir_t pdir;
    int state = lfs_dir_relocatingcommit(lfs, &ldir, dir->pair,
            attrs, attrcount, &pdir);
    if (state < 0) {
        return state;
    }

    // update if we're not in mlist, note we may have already been
    // updated if we are in mlist
    if (lfs_pair_cmp(dir->pair, lpair) == 0) {
        *dir = ldir;
    }

    // commit was successful, but may require other changes in the
    // filesystem, these would normally be tail recursive, but we have
    // flattened them here avoid unbounded stack usage

    // need to drop?
    if (state == LFS_OK_DROPPED) {
        // steal state
        int err = lfs_dir_getgstate(lfs, dir, &lfs->gdelta);
        if (err) {
            return err;
        }

        // steal tail, note that this can't create a recursive drop
        lpair[0] = pdir.pair[0];
        lpair[1] = pdir.pair[1];
        lfs_pair_tole32(dir->tail);
        state = lfs_dir_relocatingcommit(lfs, &pdir, lpair, LFS_MKATTRS(
                    {LFS_MKTAG(LFS_TYPE_TAIL + dir->split, 0x3ff, 8),
                        dir->tail}),
                NULL);
        lfs_pair_fromle32(dir->tail);
        if (state < 0) {
            return state;
        }

        ldir = pdir;
    }

    // need to relocate?
    bool orphans = false;
    while (state == LFS_OK_RELOCATED) {
        LFS_DEBUG("Relocating {0x%"PRIx32", 0x%"PRIx32"} "
                    "-> {0x%"PRIx32", 0x%"PRIx32"}",
                lpair[0], lpair[1], ldir.pair[0], ldir.pair[1]);
        state = 0;

        // update internal root
        if (lfs_pair_cmp(lpair, lfs->root) == 0) {
            lfs->root[0] = ldir.pair[0];
            lfs->root[1] = ldir.pair[1];
        }

        // update internally tracked dirs
        for (struct lfs_mlist *d = lfs->mlist; d; d = d->next) {
            if (lfs_pair_cmp(lpair, d->m.pair) == 0) {
                d->m.pair[0] = ldir.pair[0];
                d->m.pair[1] = ldir.pair[1];
            }

            if (d->type == LFS_TYPE_DIR &&
                    lfs_pair_cmp(lpair, ((lfs_dir_t*)d)->head) == 0) {
                ((lfs_dir_t*)d)->head[0] = ldir.pair[0];
                ((lfs_dir_t*)d)->head[1] = ldir.pair[1];
            }
        }

        // find parent
        lfs_stag_t tag = lfs_fs_parent(lfs, lpair, &pdir);
        if (tag < 0 && tag != LFS_ERR_NOENT) {
            return tag;
        }

        bool hasparent = (tag != LFS_ERR_NOENT);
        if (tag != LFS_ERR_NOENT) {
            // note that if we have a parent, we must have a pred, so this will
            // always create an orphan
            int err = lfs_fs_preporphans(lfs, +1);
            if (err) {
                return err;
            }

            // fix pending move in this pair? this looks like an optimization but
            // is in fact _required_ since relocating may outdate the move.
            uint16_t moveid = 0x3ff;
            if (lfs_gstate_hasmovehere(&lfs->gstate, pdir.pair)) {
                moveid = lfs_tag_id(lfs->gstate.tag);
                LFS_DEBUG("Fixing move while relocating "
                        "{0x%"PRIx32", 0x%"PRIx32"} 0x%"PRIx16"\n",
                        pdir.pair[0], pdir.pair[1], moveid);
                lfs_fs_prepmove(lfs, 0x3ff, NULL);
                if (moveid < lfs_tag_id(tag)) {
                    tag -= LFS_MKTAG(0, 1, 0);
                }
            }

            lfs_block_t ppair[2] = {pdir.pair[0], pdir.pair[1]};
            lfs_pair_tole32(ldir.pair);
            state = lfs_dir_relocatingcommit(lfs, &pdir, ppair, LFS_MKATTRS(
                        {LFS_MKTAG_IF(moveid != 0x3ff,
                            LFS_TYPE_DELETE, moveid, 0), NULL},
                        {tag, ldir.pair}),
                    NULL);
            lfs_pair_fromle32(ldir.pair);
            if (state < 0) {
                return state;
            }

            if (state == LFS_OK_RELOCATED) {
                lpair[0] = ppair[0];
                lpair[1] = ppair[1];
                ldir = pdir;
                orphans = true;
                continue;
            }
        }

        // find pred
        int err = lfs_fs_pred(lfs, lpair, &pdir);
        if (err && err != LFS_ERR_NOENT) {
            return err;
        }
        LFS_ASSERT(!(hasparent && err == LFS_ERR_NOENT));

        // if we can't find dir, it must be new
        if (err != LFS_ERR_NOENT) {
            if (lfs_gstate_hasorphans(&lfs->gstate)) {
                // next step, clean up orphans
                err = lfs_fs_preporphans(lfs, -hasparent);
                if (err) {
                    return err;
                }
            }

            // fix pending move in this pair? this looks like an optimization
            // but is in fact _required_ since relocating may outdate the move.
            uint16_t moveid = 0x3ff;
            if (lfs_gstate_hasmovehere(&lfs->gstate, pdir.pair)) {
                moveid = lfs_tag_id(lfs->gstate.tag);
                LFS_DEBUG("Fixing move while relocating "
                        "{0x%"PRIx32", 0x%"PRIx32"} 0x%"PRIx16"\n",
                        pdir.pair[0], pdir.pair[1], moveid);
                lfs_fs_prepmove(lfs, 0x3ff, NULL);
            }

            // replace bad pair, either we clean up desync, or no desync occured
            lpair[0] = pdir.pair[0];
            lpair[1] = pdir.pair[1];
            lfs_pair_tole32(ldir.pair);
            state = lfs_dir_relocatingcommit(lfs, &pdir, lpair, LFS_MKATTRS(
                        {LFS_MKTAG_IF(moveid != 0x3ff,
                            LFS_TYPE_DELETE, moveid, 0), NULL},
                        {LFS_MKTAG(LFS_TYPE_TAIL + pdir.split, 0x3ff, 8),
                            ldir.pair}),
                    NULL);
            lfs_pair_fromle32(ldir.pair);
            if (state < 0) {
                return state;
            }

            ldir = pdir;
        }
    }

    return orphans ? LFS_OK_ORPHANED : 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_dir_commit(lfs_t *lfs, lfs_mdir_t *dir,
        const struct lfs_mattr *attrs, int attrcount) {
    int orphans = lfs_dir_orphaningcommit(lfs, dir, attrs, attrcount);
    if (orphans < 0) {
        return orphans;
    }

    if (orphans) {
        // make sure we've removed all orphans, this is a noop if there
        // are none, but if we had nested blocks failures we may have
        // created some
        int err = lfs_fs_deorphan(lfs, false);
        if (err) {
            return err;
        }
    }

    return 0;
}
#endif


/// Top level directory operations ///
#ifndef LFS_READONLY
static int lfs_rawmkdir(lfs_t *lfs, const char *path) {
    // deorphan if we haven't yet, needed at most once after poweron
    int err = lfs_fs_forceconsistency(lfs);
    if (err) {
        return err;
    }

    struct lfs_mlist cwd;
    cwd.next = lfs->mlist;
    uint16_t id;
    err = lfs_dir_find(lfs, &cwd.m, &path, &id);
    if (!(err == LFS_ERR_NOENT && id != 0x3ff)) {
        return (err < 0) ? err : LFS_ERR_EXIST;
    }

    // check that name fits
    lfs_size_t nlen = strlen(path);
    if (nlen > lfs->name_max) {
        return LFS_ERR_NAMETOOLONG;
    }

    // build up new directory
    lfs_alloc_ack(lfs);
    lfs_mdir_t dir;
    err = lfs_dir_alloc(lfs, &dir);
    if (err) {
        return err;
    }

    // find end of list
    lfs_mdir_t pred = cwd.m;
    while (pred.split) {
        err = lfs_dir_fetch(lfs, &pred, pred.tail);
        if (err) {
            return err;
        }
    }

    // setup dir
    lfs_pair_tole32(pred.tail);
    err = lfs_dir_commit(lfs, &dir, LFS_MKATTRS(
            {LFS_MKTAG(LFS_TYPE_SOFTTAIL, 0x3ff, 8), pred.tail}));
    lfs_pair_fromle32(pred.tail);
    if (err) {
        return err;
    }

    // current block not end of list?
    if (cwd.m.split) {
        // update tails, this creates a desync
        err = lfs_fs_preporphans(lfs, +1);
        if (err) {
            return err;
        }

        // it's possible our predecessor has to be relocated, and if
        // our parent is our predecessor's predecessor, this could have
        // caused our parent to go out of date, fortunately we can hook
        // ourselves into littlefs to catch this
        cwd.type = 0;
        cwd.id = 0;
        lfs->mlist = &cwd;

        lfs_pair_tole32(dir.pair);
        err = lfs_dir_commit(lfs, &pred, LFS_MKATTRS(
                {LFS_MKTAG(LFS_TYPE_SOFTTAIL, 0x3ff, 8), dir.pair}));
        lfs_pair_fromle32(dir.pair);
        if (err) {
            lfs->mlist = cwd.next;
            return err;
        }

        lfs->mlist = cwd.next;
        err = lfs_fs_preporphans(lfs, -1);
        if (err) {
            return err;
        }
    }

    // now insert into our parent block
    lfs_pair_tole32(dir.pair);
    err = lfs_dir_commit(lfs, &cwd.m, LFS_MKATTRS(
            {LFS_MKTAG(LFS_TYPE_CREATE, id, 0), NULL},
            {LFS_MKTAG(LFS_TYPE_DIR, id, nlen), path},
            {LFS_MKTAG(LFS_TYPE_DIRSTRUCT, id, 8), dir.pair},
            {LFS_MKTAG_IF(!cwd.m.split,
                LFS_TYPE_SOFTTAIL, 0x3ff, 8), dir.pair}));
    lfs_pair_fromle32(dir.pair);
    if (err) {
        return err;
    }

    return 0;
}
#endif

static int lfs_dir_rawopen(lfs_t *lfs, lfs_dir_t *dir, const char *path) {
    lfs_stag_t tag = lfs_dir_find(lfs, &dir->m, &path, NULL);
    if (tag < 0) {
        return tag;
    }

    if (lfs_tag_type3(tag) != LFS_TYPE_DIR) {
        return LFS_ERR_NOTDIR;
    }

    lfs_block_t pair[2];
    if (lfs_tag_id(tag) == 0x3ff) {
        // handle root dir separately
        pair[0] = lfs->root[0];
        pair[1] = lfs->root[1];
    } else {
        // get dir pair from parent
        lfs_stag_t res = lfs_dir_get(lfs, &dir->m, LFS_MKTAG(0x700, 0x3ff, 0),
                LFS_MKTAG(LFS_TYPE_STRUCT, lfs_tag_id(tag), 8), pair);
        if (res < 0) {
            return res;
        }
        lfs_pair_fromle32(pair);
    }

    // fetch first pair
    int err = lfs_dir_fetch(lfs, &dir->m, pair);
    if (err) {
        return err;
    }

    // setup entry
    dir->head[0] = dir->m.pair[0];
    dir->head[1] = dir->m.pair[1];
    dir->id = 0;
    dir->pos = 0;

    // add to list of mdirs
    dir->type = LFS_TYPE_DIR;
    lfs_mlist_append(lfs, (struct lfs_mlist *)dir);

    return 0;
}

static int lfs_dir_rawclose(lfs_t *lfs, lfs_dir_t *dir) {
    // remove from list of mdirs
    lfs_mlist_remove(lfs, (struct lfs_mlist *)dir);

    return 0;
}

static int lfs_dir_rawread(lfs_t *lfs, lfs_dir_t *dir, struct lfs_info *info) {
    memset(info, 0, sizeof(*info));

    // special offset for '.' and '..'
    if (dir->pos == 0) {
        info->type = LFS_TYPE_DIR;
        strcpy(info->name, ".");
        dir->pos += 1;
        return true;
    } else if (dir->pos == 1) {
        info->type = LFS_TYPE_DIR;
        strcpy(info->name, "..");
        dir->pos += 1;
        return true;
    }

    while (true) {
        if (dir->id == dir->m.count) {
            if (!dir->m.split) {
                return false;
            }

            int err = lfs_dir_fetch(lfs, &dir->m, dir->m.tail);
            if (err) {
                return err;
            }

            dir->id = 0;
        }

        int err = lfs_dir_getinfo(lfs, &dir->m, dir->id, info);
        if (err && err != LFS_ERR_NOENT) {
            return err;
        }

        dir->id += 1;
        if (err != LFS_ERR_NOENT) {
            break;
        }
    }

    dir->pos += 1;
    return true;
}

static int lfs_dir_rawseek(lfs_t *lfs, lfs_dir_t *dir, lfs_off_t off) {
    // simply walk from head dir
    int err = lfs_dir_rawrewind(lfs, dir);
    if (err) {
        return err;
    }

    // first two for ./..
    dir->pos = lfs_min(2, off);
    off -= dir->pos;

    // skip superblock entry
    dir->id = (off > 0 && lfs_pair_cmp(dir->head, lfs->root) == 0);

    while (off > 0) {
        int diff = lfs_min(dir->m.count - dir->id, off);
        dir->id += diff;
        dir->pos += diff;
        off -= diff;

        if (dir->id == dir->m.count) {
            if (!dir->m.split) {
                return LFS_ERR_INVAL;
            }

            err = lfs_dir_fetch(lfs, &dir->m, dir->m.tail);
            if (err) {
                return err;
            }

            dir->id = 0;
        }
    }

    return 0;
}

static lfs_soff_t lfs_dir_rawtell(lfs_t *lfs, lfs_dir_t *dir) {
    (void)lfs;
    return dir->pos;
}

static int lfs_dir_rawrewind(lfs_t *lfs, lfs_dir_t *dir) {
    // reload the head dir
    int err = lfs_dir_fetch(lfs, &dir->m, dir->head);
    if (err) {
        return err;
    }

    dir->id = 0;
    dir->pos = 0;
    return 0;
}


/// File index list operations ///
static int lfs_ctz_index(lfs_t *lfs, lfs_off_t *off) {
    lfs_off_t size = *off;
    lfs_off_t b = lfs->cfg->block_size - 2*4;
    lfs_off_t i = size / b;
    if (i == 0) {
        return 0;
    }

    i = (size - 4*(lfs_popc(i-1)+2)) / b;
    *off = size - b*i - 4*lfs_popc(i);
    return i;
}

static int lfs_ctz_find(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache,
        lfs_block_t head, lfs_size_t size,
        lfs_size_t pos, lfs_block_t *block, lfs_off_t *off) {
    if (size == 0) {
        *block = LFS_BLOCK_NULL;
        *off = 0;
        return 0;
    }

    lfs_off_t current = lfs_ctz_index(lfs, &(lfs_off_t){size-1});
    lfs_off_t target = lfs_ctz_index(lfs, &pos);

    while (current > target) {
        lfs_size_t skip = lfs_min(
                lfs_npw2(current-target+1) - 1,
                lfs_ctz(current));

        int err = lfs_bd_read(lfs,
                pcache, rcache, sizeof(head),
                head, 4*skip, &head, sizeof(head));
        head = lfs_fromle32(head);
        if (err) {
            return err;
        }

        current -= 1 << skip;
    }

    *block = head;
    *off = pos;
    return 0;
}

#ifndef LFS_READONLY
static int lfs_ctz_extend(lfs_t *lfs,
        lfs_cache_t *pcache, lfs_cache_t *rcache,
        lfs_block_t head, lfs_size_t size,
        lfs_block_t *block, lfs_off_t *off) {
    while (true) {
        // go ahead and grab a block
        lfs_block_t nblock;
        int err = lfs_alloc(lfs, &nblock);
        if (err) {
            return err;
        }

        {
            err = lfs_bd_erase(lfs, nblock);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            if (size == 0) {
                *block = nblock;
                *off = 0;
                return 0;
            }

            lfs_size_t noff = size - 1;
            lfs_off_t index = lfs_ctz_index(lfs, &noff);
            noff = noff + 1;

            // just copy out the last block if it is incomplete
            if (noff != lfs->cfg->block_size) {
                for (lfs_off_t i = 0; i < noff; i++) {
                    uint8_t data;
                    err = lfs_bd_read(lfs,
                            NULL, rcache, noff-i,
                            head, i, &data, 1);
                    if (err) {
                        return err;
                    }

                    err = lfs_bd_prog(lfs,
                            pcache, rcache, true,
                            nblock, i, &data, 1);
                    if (err) {
                        if (err == LFS_ERR_CORRUPT) {
                            goto relocate;
                        }
                        return err;
                    }
                }

                *block = nblock;
                *off = noff;
                return 0;
            }

            // append block
            index += 1;
            lfs_size_t skips = lfs_ctz(index) + 1;
            lfs_block_t nhead = head;
            for (lfs_off_t i = 0; i < skips; i++) {
                nhead = lfs_tole32(nhead);
                err = lfs_bd_prog(lfs, pcache, rcache, true,
                        nblock, 4*i, &nhead, 4);
                nhead = lfs_fromle32(nhead);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        goto relocate;
                    }
                    return err;
                }

                if (i != skips-1) {
                    err = lfs_bd_read(lfs,
                            NULL, rcache, sizeof(nhead),
                            nhead, 4*i, &nhead, sizeof(nhead));
                    nhead = lfs_fromle32(nhead);
                    if (err) {
                        return err;
                    }
                }
            }

            *block = nblock;
            *off = 4*skips;
            return 0;
        }

relocate:
        LFS_DEBUG("Bad block at 0x%"PRIx32, nblock);

        // just clear cache and try a new block
        lfs_cache_drop(lfs, pcache);
    }
}
#endif

static int lfs_ctz_traverse(lfs_t *lfs,
        const lfs_cache_t *pcache, lfs_cache_t *rcache,
        lfs_block_t head, lfs_size_t size,
        int (*cb)(void*, lfs_block_t), void *data) {
    if (size == 0) {
        return 0;
    }

    lfs_off_t index = lfs_ctz_index(lfs, &(lfs_off_t){size-1});

    while (true) {
        int err = cb(data, head);
        if (err) {
            return err;
        }

        if (index == 0) {
            return 0;
        }

        lfs_block_t heads[2];
        int count = 2 - (index & 1);
        err = lfs_bd_read(lfs,
                pcache, rcache, count*sizeof(head),
                head, 0, &heads, count*sizeof(head));
        heads[0] = lfs_fromle32(heads[0]);
        heads[1] = lfs_fromle32(heads[1]);
        if (err) {
            return err;
        }

        for (int i = 0; i < count-1; i++) {
            err = cb(data, heads[i]);
            if (err) {
                return err;
            }
        }

        head = heads[count-1];
        index -= count;
    }
}


/// Top level file operations ///
static int lfs_file_rawopencfg(lfs_t *lfs, lfs_file_t *file,
        const char *path, int flags,
        const struct lfs_file_config *cfg) {
#ifndef LFS_READONLY
    // deorphan if we haven't yet, needed at most once after poweron
    if ((flags & LFS_O_WRONLY) == LFS_O_WRONLY) {
        int err = lfs_fs_forceconsistency(lfs);
        if (err) {
            return err;
        }
    }
#else
    LFS_ASSERT((flags & LFS_O_RDONLY) == LFS_O_RDONLY);
#endif

    // setup simple file details
    int err;
    file->cfg = cfg;
    file->flags = flags;
    file->pos = 0;
    file->off = 0;
    file->cache.buffer = NULL;

    // allocate entry for file if it doesn't exist
    lfs_stag_t tag = lfs_dir_find(lfs, &file->m, &path, &file->id);
    if (tag < 0 && !(tag == LFS_ERR_NOENT && file->id != 0x3ff)) {
        err = tag;
        goto cleanup;
    }

    // get id, add to list of mdirs to catch update changes
    file->type = LFS_TYPE_REG;
    lfs_mlist_append(lfs, (struct lfs_mlist *)file);

#ifdef LFS_READONLY
    if (tag == LFS_ERR_NOENT) {
        err = LFS_ERR_NOENT;
        goto cleanup;
#else
    if (tag == LFS_ERR_NOENT) {
        if (!(flags & LFS_O_CREAT)) {
            err = LFS_ERR_NOENT;
            goto cleanup;
        }

        // check that name fits
        lfs_size_t nlen = strlen(path);
        if (nlen > lfs->name_max) {
            err = LFS_ERR_NAMETOOLONG;
            goto cleanup;
        }

        // get next slot and create entry to remember name
        err = lfs_dir_commit(lfs, &file->m, LFS_MKATTRS(
                {LFS_MKTAG(LFS_TYPE_CREATE, file->id, 0), NULL},
                {LFS_MKTAG(LFS_TYPE_REG, file->id, nlen), path},
                {LFS_MKTAG(LFS_TYPE_INLINESTRUCT, file->id, 0), NULL}));

        // it may happen that the file name doesn't fit in the metadata blocks, e.g., a 256 byte file name will
        // not fit in a 128 byte block.
        err = (err == LFS_ERR_NOSPC) ? LFS_ERR_NAMETOOLONG : err;
        if (err) {
            goto cleanup;
        }

        tag = LFS_MKTAG(LFS_TYPE_INLINESTRUCT, 0, 0);
    } else if (flags & LFS_O_EXCL) {
        err = LFS_ERR_EXIST;
        goto cleanup;
#endif
    } else if (lfs_tag_type3(tag) != LFS_TYPE_REG) {
        err = LFS_ERR_ISDIR;
        goto cleanup;
#ifndef LFS_READONLY
    } else if (flags & LFS_O_TRUNC) {
        // truncate if requested
        tag = LFS_MKTAG(LFS_TYPE_INLINESTRUCT, file->id, 0);
        file->flags |= LFS_F_DIRTY;
#endif
    } else {
        // try to load what's on disk, if it's inlined we'll fix it later
        tag = lfs_dir_get(lfs, &file->m, LFS_MKTAG(0x700, 0x3ff, 0),
                LFS_MKTAG(LFS_TYPE_STRUCT, file->id, 8), &file->ctz);
        if (tag < 0) {
            err = tag;
            goto cleanup;
        }
        lfs_ctz_fromle32(&file->ctz);
    }

    // fetch attrs
    for (unsigned i = 0; i < file->cfg->attr_count; i++) {
        // if opened for read / read-write operations
        if ((file->flags & LFS_O_RDONLY) == LFS_O_RDONLY) {
            lfs_stag_t res = lfs_dir_get(lfs, &file->m,
                    LFS_MKTAG(0x7ff, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_USERATTR + file->cfg->attrs[i].type,
                        file->id, file->cfg->attrs[i].size),
                        file->cfg->attrs[i].buffer);
            if (res < 0 && res != LFS_ERR_NOENT) {
                err = res;
                goto cleanup;
            }
        }

#ifndef LFS_READONLY
        // if opened for write / read-write operations
        if ((file->flags & LFS_O_WRONLY) == LFS_O_WRONLY) {
            if (file->cfg->attrs[i].size > lfs->attr_max) {
                err = LFS_ERR_NOSPC;
                goto cleanup;
            }

            file->flags |= LFS_F_DIRTY;
        }
#endif
    }

    // allocate buffer if needed
    if (file->cfg->buffer) {
        file->cache.buffer = file->cfg->buffer;
    } else {
        file->cache.buffer = lfs_malloc(lfs->cfg->cache_size);
        if (!file->cache.buffer) {
            err = LFS_ERR_NOMEM;
            goto cleanup;
        }
    }

    // zero to avoid information leak
    lfs_cache_zero(lfs, &file->cache);

    if (lfs_tag_type3(tag) == LFS_TYPE_INLINESTRUCT) {
        // load inline files
        file->ctz.head = LFS_BLOCK_INLINE;
        file->ctz.size = lfs_tag_size(tag);
        file->flags |= LFS_F_INLINE;
        file->cache.block = file->ctz.head;
        file->cache.off = 0;
        file->cache.size = lfs->cfg->cache_size;

        // don't always read (may be new/trunc file)
        if (file->ctz.size > 0) {
            lfs_stag_t res = lfs_dir_get(lfs, &file->m,
                    LFS_MKTAG(0x700, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_STRUCT, file->id,
                        lfs_min(file->cache.size, 0x3fe)),
                    file->cache.buffer);
            if (res < 0) {
                err = res;
                goto cleanup;
            }
        }
    }

    return 0;

cleanup:
    // clean up lingering resources
#ifndef LFS_READONLY
    file->flags |= LFS_F_ERRED;
#endif
    lfs_file_rawclose(lfs, file);
    return err;
}

#ifndef LFS_NO_MALLOC
static int lfs_file_rawopen(lfs_t *lfs, lfs_file_t *file,
        const char *path, int flags) {
    static const struct lfs_file_config defaults = {0};
    int err = lfs_file_rawopencfg(lfs, file, path, flags, &defaults);
    return err;
}
#endif

static int lfs_file_rawclose(lfs_t *lfs, lfs_file_t *file) {
#ifndef LFS_READONLY
    int err = lfs_file_rawsync(lfs, file);
#else
    int err = 0;
#endif

    // remove from list of mdirs
    lfs_mlist_remove(lfs, (struct lfs_mlist*)file);

    // clean up memory
    if (!file->cfg->buffer) {
        lfs_free(file->cache.buffer);
    }

    return err;
}


#ifndef LFS_READONLY
static int lfs_file_relocate(lfs_t *lfs, lfs_file_t *file) {
    while (true) {
        // just relocate what exists into new block
        lfs_block_t nblock;
        int err = lfs_alloc(lfs, &nblock);
        if (err) {
            return err;
        }

        err = lfs_bd_erase(lfs, nblock);
        if (err) {
            if (err == LFS_ERR_CORRUPT) {
                goto relocate;
            }
            return err;
        }

        // either read from dirty cache or disk
        for (lfs_off_t i = 0; i < file->off; i++) {
            uint8_t data;
            if (file->flags & LFS_F_INLINE) {
                err = lfs_dir_getread(lfs, &file->m,
                        // note we evict inline files before they can be dirty
                        NULL, &file->cache, file->off-i,
                        LFS_MKTAG(0xfff, 0x1ff, 0),
                        LFS_MKTAG(LFS_TYPE_INLINESTRUCT, file->id, 0),
                        i, &data, 1);
                if (err) {
                    return err;
                }
            } else {
                err = lfs_bd_read(lfs,
                        &file->cache, &lfs->rcache, file->off-i,
                        file->block, i, &data, 1);
                if (err) {
                    return err;
                }
            }

            err = lfs_bd_prog(lfs,
                    &lfs->pcache, &lfs->rcache, true,
                    nblock, i, &data, 1);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }
        }

        // copy over new state of file
        memcpy(file->cache.buffer, lfs->pcache.buffer, lfs->cfg->cache_size);
        file->cache.block = lfs->pcache.block;
        file->cache.off = lfs->pcache.off;
        file->cache.size = lfs->pcache.size;
        lfs_cache_zero(lfs, &lfs->pcache);

        file->block = nblock;
        file->flags |= LFS_F_WRITING;
        return 0;

relocate:
        LFS_DEBUG("Bad block at 0x%"PRIx32, nblock);

        // just clear cache and try a new block
        lfs_cache_drop(lfs, &lfs->pcache);
    }
}
#endif

#ifndef LFS_READONLY
static int lfs_file_outline(lfs_t *lfs, lfs_file_t *file) {
    file->off = file->pos;
    lfs_alloc_ack(lfs);
    int err = lfs_file_relocate(lfs, file);
    if (err) {
        return err;
    }

    file->flags &= ~LFS_F_INLINE;
    return 0;
}
#endif

static int lfs_file_flush(lfs_t *lfs, lfs_file_t *file) {
    if (file->flags & LFS_F_READING) {
        if (!(file->flags & LFS_F_INLINE)) {
            lfs_cache_drop(lfs, &file->cache);
        }
        file->flags &= ~LFS_F_READING;
    }

#ifndef LFS_READONLY
    if (file->flags & LFS_F_WRITING) {
        lfs_off_t pos = file->pos;

        if (!(file->flags & LFS_F_INLINE)) {
            // copy over anything after current branch
            lfs_file_t orig = {
                .ctz.head = file->ctz.head,
                .ctz.size = file->ctz.size,
                .flags = LFS_O_RDONLY,
                .pos = file->pos,
                .cache = lfs->rcache,
            };
            lfs_cache_drop(lfs, &lfs->rcache);

            while (file->pos < file->ctz.size) {
                // copy over a byte at a time, leave it up to caching
                // to make this efficient
                uint8_t data;
                lfs_ssize_t res = lfs_file_flushedread(lfs, &orig, &data, 1);
                if (res < 0) {
                    return res;
                }

                res = lfs_file_flushedwrite(lfs, file, &data, 1);
                if (res < 0) {
                    return res;
                }

                // keep our reference to the rcache in sync
                if (lfs->rcache.block != LFS_BLOCK_NULL) {
                    lfs_cache_drop(lfs, &orig.cache);
                    lfs_cache_drop(lfs, &lfs->rcache);
                }
            }

            // write out what we have
            while (true) {
                int err = lfs_bd_flush(lfs, &file->cache, &lfs->rcache, true);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        goto relocate;
                    }
                    return err;
                }

                break;

relocate:
                LFS_DEBUG("Bad block at 0x%"PRIx32, file->block);
                err = lfs_file_relocate(lfs, file);
                if (err) {
                    return err;
                }
            }
        } else {
            file->pos = lfs_max(file->pos, file->ctz.size);
        }

        // actual file updates
        file->ctz.head = file->block;
        file->ctz.size = file->pos;
        file->flags &= ~LFS_F_WRITING;
        file->flags |= LFS_F_DIRTY;

        file->pos = pos;
    }
#endif

    return 0;
}

#ifndef LFS_READONLY
static int lfs_file_rawsync(lfs_t *lfs, lfs_file_t *file) {
    if (file->flags & LFS_F_ERRED) {
        // it's not safe to do anything if our file errored
        return 0;
    }

    int err = lfs_file_flush(lfs, file);
    if (err) {
        file->flags |= LFS_F_ERRED;
        return err;
    }


    if ((file->flags & LFS_F_DIRTY) &&
            !lfs_pair_isnull(file->m.pair)) {
        // update dir entry
        uint16_t type;
        const void *buffer;
        lfs_size_t size;
        struct lfs_ctz ctz;
        if (file->flags & LFS_F_INLINE) {
            // inline the whole file
            type = LFS_TYPE_INLINESTRUCT;
            buffer = file->cache.buffer;
            size = file->ctz.size;
        } else {
            // update the ctz reference
            type = LFS_TYPE_CTZSTRUCT;
            // copy ctz so alloc will work during a relocate
            ctz = file->ctz;
            lfs_ctz_tole32(&ctz);
            buffer = &ctz;
            size = sizeof(ctz);
        }

        // commit file data and attributes
        err = lfs_dir_commit(lfs, &file->m, LFS_MKATTRS(
                {LFS_MKTAG(type, file->id, size), buffer},
                {LFS_MKTAG(LFS_FROM_USERATTRS, file->id,
                    file->cfg->attr_count), file->cfg->attrs}));
        if (err) {
            file->flags |= LFS_F_ERRED;
            return err;
        }

        file->flags &= ~LFS_F_DIRTY;
    }

    return 0;
}
#endif

static lfs_ssize_t lfs_file_flushedread(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size) {
    uint8_t *data = buffer;
    lfs_size_t nsize = size;

    if (file->pos >= file->ctz.size) {
        // eof if past end
        return 0;
    }

    size = lfs_min(size, file->ctz.size - file->pos);
    nsize = size;

    while (nsize > 0) {
        // check if we need a new block
        if (!(file->flags & LFS_F_READING) ||
                file->off == lfs->cfg->block_size) {
            if (!(file->flags & LFS_F_INLINE)) {
                int err = lfs_ctz_find(lfs, NULL, &file->cache,
                        file->ctz.head, file->ctz.size,
                        file->pos, &file->block, &file->off);
                if (err) {
                    return err;
                }
            } else {
                file->block = LFS_BLOCK_INLINE;
                file->off = file->pos;
            }

            file->flags |= LFS_F_READING;
        }

        // read as much as we can in current block
        lfs_size_t diff = lfs_min(nsize, lfs->cfg->block_size - file->off);
        if (file->flags & LFS_F_INLINE) {
            int err = lfs_dir_getread(lfs, &file->m,
                    NULL, &file->cache, lfs->cfg->block_size,
                    LFS_MKTAG(0xfff, 0x1ff, 0),
                    LFS_MKTAG(LFS_TYPE_INLINESTRUCT, file->id, 0),
                    file->off, data, diff);
            if (err) {
                return err;
            }
        } else {
            int err = lfs_bd_read(lfs,
                    NULL, &file->cache, lfs->cfg->block_size,
                    file->block, file->off, data, diff);
            if (err) {
                return err;
            }
        }

        file->pos += diff;
        file->off += diff;
        data += diff;
        nsize -= diff;
    }

    return size;
}

static lfs_ssize_t lfs_file_rawread(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size) {
    LFS_ASSERT((file->flags & LFS_O_RDONLY) == LFS_O_RDONLY);

#ifndef LFS_READONLY
    if (file->flags & LFS_F_WRITING) {
        // flush out any writes
        int err = lfs_file_flush(lfs, file);
        if (err) {
            return err;
        }
    }
#endif

    return lfs_file_flushedread(lfs, file, buffer, size);
}


#ifndef LFS_READONLY
static lfs_ssize_t lfs_file_flushedwrite(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size) {
    const uint8_t *data = buffer;
    lfs_size_t nsize = size;

    if ((file->flags & LFS_F_INLINE) &&
            lfs_max(file->pos+nsize, file->ctz.size) >
            lfs_min(0x3fe, lfs_min(
                lfs->cfg->cache_size,
                (lfs->cfg->metadata_max ?
                    lfs->cfg->metadata_max : lfs->cfg->block_size) / 8))) {
        // inline file doesn't fit anymore
        int err = lfs_file_outline(lfs, file);
        if (err) {
            file->flags |= LFS_F_ERRED;
            return err;
        }
    }

    while (nsize > 0) {
        // check if we need a new block
        if (!(file->flags & LFS_F_WRITING) ||
                file->off == lfs->cfg->block_size) {
            if (!(file->flags & LFS_F_INLINE)) {
                if (!(file->flags & LFS_F_WRITING) && file->pos > 0) {
                    // find out which block we're extending from
                    int err = lfs_ctz_find(lfs, NULL, &file->cache,
                            file->ctz.head, file->ctz.size,
                            file->pos-1, &file->block, &file->off);
                    if (err) {
                        file->flags |= LFS_F_ERRED;
                        return err;
                    }

                    // mark cache as dirty since we may have read data into it
                    lfs_cache_zero(lfs, &file->cache);
                }

                // extend file with new blocks
                lfs_alloc_ack(lfs);
                int err = lfs_ctz_extend(lfs, &file->cache, &lfs->rcache,
                        file->block, file->pos,
                        &file->block, &file->off);
                if (err) {
                    file->flags |= LFS_F_ERRED;
                    return err;
                }
            } else {
                file->block = LFS_BLOCK_INLINE;
                file->off = file->pos;
            }

            file->flags |= LFS_F_WRITING;
        }

        // program as much as we can in current block
        lfs_size_t diff = lfs_min(nsize, lfs->cfg->block_size - file->off);
        while (true) {
            int err = lfs_bd_prog(lfs, &file->cache, &lfs->rcache, true,
                    file->block, file->off, data, diff);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                file->flags |= LFS_F_ERRED;
                return err;
            }

            break;
relocate:
            err = lfs_file_relocate(lfs, file);
            if (err) {
                file->flags |= LFS_F_ERRED;
                return err;
            }
        }

        file->pos += diff;
        file->off += diff;
        data += diff;
        nsize -= diff;

        lfs_alloc_ack(lfs);
    }

    return size;
}

static lfs_ssize_t lfs_file_rawwrite(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size) {
    LFS_ASSERT((file->flags & LFS_O_WRONLY) == LFS_O_WRONLY);

    if (file->flags & LFS_F_READING) {
        // drop any reads
        int err = lfs_file_flush(lfs, file);
        if (err) {
            return err;
        }
    }

    if ((file->flags & LFS_O_APPEND) && file->pos < file->ctz.size) {
        file->pos = file->ctz.size;
    }

    if (file->pos + size > lfs->file_max) {
        // Larger than file limit?
        return LFS_ERR_FBIG;
    }

    if (!(file->flags & LFS_F_WRITING) && file->pos > file->ctz.size) {
        // fill with zeros
        lfs_off_t pos = file->pos;
        file->pos = file->ctz.size;

        while (file->pos < pos) {
            lfs_ssize_t res = lfs_file_flushedwrite(lfs, file, &(uint8_t){0}, 1);
            if (res < 0) {
                return res;
            }
        }
    }

    lfs_ssize_t nsize = lfs_file_flushedwrite(lfs, file, buffer, size);
    if (nsize < 0) {
        return nsize;
    }

    file->flags &= ~LFS_F_ERRED;
    return nsize;
}
#endif

static lfs_soff_t lfs_file_rawseek(lfs_t *lfs, lfs_file_t *file,
        lfs_soff_t off, int whence) {
    // find new pos
    lfs_off_t npos = file->pos;
    if (whence == LFS_SEEK_SET) {
        npos = off;
    } else if (whence == LFS_SEEK_CUR) {
        if ((lfs_soff_t)file->pos + off < 0) {
            return LFS_ERR_INVAL;
        } else {
            npos = file->pos + off;
        }
    } else if (whence == LFS_SEEK_END) {
        lfs_soff_t res = lfs_file_rawsize(lfs, file) + off;
        if (res < 0) {
            return LFS_ERR_INVAL;
        } else {
            npos = res;
        }
    }

    if (npos > lfs->file_max) {
        // file position out of range
        return LFS_ERR_INVAL;
    }

    if (file->pos == npos) {
        // noop - position has not changed
        return npos;
    }

    // if we're only reading and our new offset is still in the file's cache
    // we can avoid flushing and needing to reread the data
    if (
#ifndef LFS_READONLY
        !(file->flags & LFS_F_WRITING)
#else
        true
#endif
            ) {
        int oindex = lfs_ctz_index(lfs, &(lfs_off_t){file->pos});
        lfs_off_t noff = npos;
        int nindex = lfs_ctz_index(lfs, &noff);
        if (oindex == nindex
                && noff >= file->cache.off
                && noff < file->cache.off + file->cache.size) {
            file->pos = npos;
            file->off = noff;
            return npos;
        }
    }

    // write out everything beforehand, may be noop if rdonly
    int err = lfs_file_flush(lfs, file);
    if (err) {
        return err;
    }

    // update pos
    file->pos = npos;
    return npos;
}

#ifndef LFS_READONLY
static int lfs_file_rawtruncate(lfs_t *lfs, lfs_file_t *file, lfs_off_t size) {
    LFS_ASSERT((file->flags & LFS_O_WRONLY) == LFS_O_WRONLY);

    if (size > LFS_FILE_MAX) {
        return LFS_ERR_INVAL;
    }

    lfs_off_t pos = file->pos;
    lfs_off_t oldsize = lfs_file_rawsize(lfs, file);
    if (size < oldsize) {
        // need to flush since directly changing metadata
        int err = lfs_file_flush(lfs, file);
        if (err) {
            return err;
        }

        // lookup new head in ctz skip list
        err = lfs_ctz_find(lfs, NULL, &file->cache,
                file->ctz.head, file->ctz.size,
                size, &file->block, &file->off);
        if (err) {
            return err;
        }

        // need to set pos/block/off consistently so seeking back to
        // the old position does not get confused
        file->pos = size;
        file->ctz.head = file->block;
        file->ctz.size = size;
        file->flags |= LFS_F_DIRTY | LFS_F_READING;
    } else if (size > oldsize) {
        // flush+seek if not already at end
        lfs_soff_t res = lfs_file_rawseek(lfs, file, 0, LFS_SEEK_END);
        if (res < 0) {
            return (int)res;
        }

        // fill with zeros
        while (file->pos < size) {
            res = lfs_file_rawwrite(lfs, file, &(uint8_t){0}, 1);
            if (res < 0) {
                return (int)res;
            }
        }
    }

    // restore pos
    lfs_soff_t res = lfs_file_rawseek(lfs, file, pos, LFS_SEEK_SET);
    if (res < 0) {
      return (int)res;
    }

    return 0;
}
#endif

static lfs_soff_t lfs_file_rawtell(lfs_t *lfs, lfs_file_t *file) {
    (void)lfs;
    return file->pos;
}

static int lfs_file_rawrewind(lfs_t *lfs, lfs_file_t *file) {
    lfs_soff_t res = lfs_file_rawseek(lfs, file, 0, LFS_SEEK_SET);
    if (res < 0) {
        return (int)res;
    }

    return 0;
}

static lfs_soff_t lfs_file_rawsize(lfs_t *lfs, lfs_file_t *file) {
    (void)lfs;

#ifndef LFS_READONLY
    if (file->flags & LFS_F_WRITING) {
        return lfs_max(file->pos, file->ctz.size);
    }
#endif

    return file->ctz.size;
}


/// General fs operations ///
static int lfs_rawstat(lfs_t *lfs, const char *path, struct lfs_info *info) {
    lfs_mdir_t cwd;
    lfs_stag_t tag = lfs_dir_find(lfs, &cwd, &path, NULL);
    if (tag < 0) {
        return (int)tag;
    }

    return lfs_dir_getinfo(lfs, &cwd, lfs_tag_id(tag), info);
}

#ifndef LFS_READONLY
static int lfs_rawremove(lfs_t *lfs, const char *path) {
    // deorphan if we haven't yet, needed at most once after poweron
    int err = lfs_fs_forceconsistency(lfs);
    if (err) {
        return err;
    }

    lfs_mdir_t cwd;
    lfs_stag_t tag = lfs_dir_find(lfs, &cwd, &path, NULL);
    if (tag < 0 || lfs_tag_id(tag) == 0x3ff) {
        return (tag < 0) ? (int)tag : LFS_ERR_INVAL;
    }

    struct lfs_mlist dir;
    dir.next = lfs->mlist;
    if (lfs_tag_type3(tag) == LFS_TYPE_DIR) {
        // must be empty before removal
        lfs_block_t pair[2];
        lfs_stag_t res = lfs_dir_get(lfs, &cwd, LFS_MKTAG(0x700, 0x3ff, 0),
                LFS_MKTAG(LFS_TYPE_STRUCT, lfs_tag_id(tag), 8), pair);
        if (res < 0) {
            return (int)res;
        }
        lfs_pair_fromle32(pair);

        err = lfs_dir_fetch(lfs, &dir.m, pair);
        if (err) {
            return err;
        }

        if (dir.m.count > 0 || dir.m.split) {
            return LFS_ERR_NOTEMPTY;
        }

        // mark fs as orphaned
        err = lfs_fs_preporphans(lfs, +1);
        if (err) {
            return err;
        }

        // I know it's crazy but yes, dir can be changed by our parent's
        // commit (if predecessor is child)
        dir.type = 0;
        dir.id = 0;
        lfs->mlist = &dir;
    }

    // delete the entry
    err = lfs_dir_commit(lfs, &cwd, LFS_MKATTRS(
            {LFS_MKTAG(LFS_TYPE_DELETE, lfs_tag_id(tag), 0), NULL}));
    if (err) {
        lfs->mlist = dir.next;
        return err;
    }

    lfs->mlist = dir.next;
    if (lfs_tag_type3(tag) == LFS_TYPE_DIR) {
        // fix orphan
        err = lfs_fs_preporphans(lfs, -1);
        if (err) {
            return err;
        }

        err = lfs_fs_pred(lfs, dir.m.pair, &cwd);
        if (err) {
            return err;
        }

        err = lfs_dir_drop(lfs, &cwd, &dir.m);
        if (err) {
            return err;
        }
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_rawrename(lfs_t *lfs, const char *oldpath, const char *newpath) {
    // deorphan if we haven't yet, needed at most once after poweron
    int err = lfs_fs_forceconsistency(lfs);
    if (err) {
        return err;
    }

    // find old entry
    lfs_mdir_t oldcwd;
    lfs_stag_t oldtag = lfs_dir_find(lfs, &oldcwd, &oldpath, NULL);
    if (oldtag < 0 || lfs_tag_id(oldtag) == 0x3ff) {
        return (oldtag < 0) ? (int)oldtag : LFS_ERR_INVAL;
    }

    // find new entry
    lfs_mdir_t newcwd;
    uint16_t newid;
    lfs_stag_t prevtag = lfs_dir_find(lfs, &newcwd, &newpath, &newid);
    if ((prevtag < 0 || lfs_tag_id(prevtag) == 0x3ff) &&
            !(prevtag == LFS_ERR_NOENT && newid != 0x3ff)) {
        return (prevtag < 0) ? (int)prevtag : LFS_ERR_INVAL;
    }

    // if we're in the same pair there's a few special cases...
    bool samepair = (lfs_pair_cmp(oldcwd.pair, newcwd.pair) == 0);
    uint16_t newoldid = lfs_tag_id(oldtag);

    struct lfs_mlist prevdir;
    prevdir.next = lfs->mlist;
    if (prevtag == LFS_ERR_NOENT) {
        // check that name fits
        lfs_size_t nlen = strlen(newpath);
        if (nlen > lfs->name_max) {
            return LFS_ERR_NAMETOOLONG;
        }

        // there is a small chance we are being renamed in the same
        // directory/ to an id less than our old id, the global update
        // to handle this is a bit messy
        if (samepair && newid <= newoldid) {
            newoldid += 1;
        }
    } else if (lfs_tag_type3(prevtag) != lfs_tag_type3(oldtag)) {
        return LFS_ERR_ISDIR;
    } else if (samepair && newid == newoldid) {
        // we're renaming to ourselves??
        return 0;
    } else if (lfs_tag_type3(prevtag) == LFS_TYPE_DIR) {
        // must be empty before removal
        lfs_block_t prevpair[2];
        lfs_stag_t res = lfs_dir_get(lfs, &newcwd, LFS_MKTAG(0x700, 0x3ff, 0),
                LFS_MKTAG(LFS_TYPE_STRUCT, newid, 8), prevpair);
        if (res < 0) {
            return (int)res;
        }
        lfs_pair_fromle32(prevpair);

        // must be empty before removal
        err = lfs_dir_fetch(lfs, &prevdir.m, prevpair);
        if (err) {
            return err;
        }

        if (prevdir.m.count > 0 || prevdir.m.split) {
            return LFS_ERR_NOTEMPTY;
        }

        // mark fs as orphaned
        err = lfs_fs_preporphans(lfs, +1);
        if (err) {
            return err;
        }

        // I know it's crazy but yes, dir can be changed by our parent's
        // commit (if predecessor is child)
        prevdir.type = 0;
        prevdir.id = 0;
        lfs->mlist = &prevdir;
    }

    if (!samepair) {
        lfs_fs_prepmove(lfs, newoldid, oldcwd.pair);
    }

    // move over all attributes
    err = lfs_dir_commit(lfs, &newcwd, LFS_MKATTRS(
            {LFS_MKTAG_IF(prevtag != LFS_ERR_NOENT,
                LFS_TYPE_DELETE, newid, 0), NULL},
            {LFS_MKTAG(LFS_TYPE_CREATE, newid, 0), NULL},
            {LFS_MKTAG(lfs_tag_type3(oldtag), newid, strlen(newpath)), newpath},
            {LFS_MKTAG(LFS_FROM_MOVE, newid, lfs_tag_id(oldtag)), &oldcwd},
            {LFS_MKTAG_IF(samepair,
                LFS_TYPE_DELETE, newoldid, 0), NULL}));
    if (err) {
        lfs->mlist = prevdir.next;
        return err;
    }

    // let commit clean up after move (if we're different! otherwise move
    // logic already fixed it for us)
    if (!samepair && lfs_gstate_hasmove(&lfs->gstate)) {
        // prep gstate and delete move id
        lfs_fs_prepmove(lfs, 0x3ff, NULL);
        err = lfs_dir_commit(lfs, &oldcwd, LFS_MKATTRS(
                {LFS_MKTAG(LFS_TYPE_DELETE, lfs_tag_id(oldtag), 0), NULL}));
        if (err) {
            lfs->mlist = prevdir.next;
            return err;
        }
    }

    lfs->mlist = prevdir.next;
    if (prevtag != LFS_ERR_NOENT
            && lfs_tag_type3(prevtag) == LFS_TYPE_DIR) {
        // fix orphan
        err = lfs_fs_preporphans(lfs, -1);
        if (err) {
            return err;
        }

        err = lfs_fs_pred(lfs, prevdir.m.pair, &newcwd);
        if (err) {
            return err;
        }

        err = lfs_dir_drop(lfs, &newcwd, &prevdir.m);
        if (err) {
            return err;
        }
    }

    return 0;
}
#endif

static lfs_ssize_t lfs_rawgetattr(lfs_t *lfs, const char *path,
        uint8_t type, void *buffer, lfs_size_t size) {
    lfs_mdir_t cwd;
    lfs_stag_t tag = lfs_dir_find(lfs, &cwd, &path, NULL);
    if (tag < 0) {
        return tag;
    }

    uint16_t id = lfs_tag_id(tag);
    if (id == 0x3ff) {
        // special case for root
        id = 0;
        int err = lfs_dir_fetch(lfs, &cwd, lfs->root);
        if (err) {
            return err;
        }
    }

    tag = lfs_dir_get(lfs, &cwd, LFS_MKTAG(0x7ff, 0x3ff, 0),
            LFS_MKTAG(LFS_TYPE_USERATTR + type,
                id, lfs_min(size, lfs->attr_max)),
            buffer);
    if (tag < 0) {
        if (tag == LFS_ERR_NOENT) {
            return LFS_ERR_NOATTR;
        }

        return tag;
    }

    return lfs_tag_size(tag);
}

#ifndef LFS_READONLY
static int lfs_commitattr(lfs_t *lfs, const char *path,
        uint8_t type, const void *buffer, lfs_size_t size) {
    lfs_mdir_t cwd;
    lfs_stag_t tag = lfs_dir_find(lfs, &cwd, &path, NULL);
    if (tag < 0) {
        return tag;
    }

    uint16_t id = lfs_tag_id(tag);
    if (id == 0x3ff) {
        // special case for root
        id = 0;
        int err = lfs_dir_fetch(lfs, &cwd, lfs->root);
        if (err) {
            return err;
        }
    }

    return lfs_dir_commit(lfs, &cwd, LFS_MKATTRS(
            {LFS_MKTAG(LFS_TYPE_USERATTR + type, id, size), buffer}));
}
#endif

#ifndef LFS_READONLY
static int lfs_rawsetattr(lfs_t *lfs, const char *path,
        uint8_t type, const void *buffer, lfs_size_t size) {
    if (size > lfs->attr_max) {
        return LFS_ERR_NOSPC;
    }

    return lfs_commitattr(lfs, path, type, buffer, size);
}
#endif

#ifndef LFS_READONLY
static int lfs_rawremoveattr(lfs_t *lfs, const char *path, uint8_t type) {
    return lfs_commitattr(lfs, path, type, NULL, 0x3ff);
}
#endif


/// Filesystem operations ///
static int lfs_init(lfs_t *lfs, const struct lfs_config *cfg) {
    lfs->cfg = cfg;
    int err = 0;

    // validate that the lfs-cfg sizes were initiated properly before
    // performing any arithmetic logics with them
    LFS_ASSERT(lfs->cfg->read_size != 0);
    LFS_ASSERT(lfs->cfg->prog_size != 0);
    LFS_ASSERT(lfs->cfg->cache_size != 0);

    // check that block size is a multiple of cache size is a multiple
    // of prog and read sizes
    LFS_ASSERT(lfs->cfg->cache_size % lfs->cfg->read_size == 0);
    LFS_ASSERT(lfs->cfg->cache_size % lfs->cfg->prog_size == 0);
    LFS_ASSERT(lfs->cfg->block_size % lfs->cfg->cache_size == 0);

    // check that the block size is large enough to fit ctz pointers
    LFS_ASSERT(4*lfs_npw2(0xffffffff / (lfs->cfg->block_size-2*4))
            <= lfs->cfg->block_size);

    // block_cycles = 0 is no longer supported.
    //
    // block_cycles is the number of erase cycles before littlefs evicts
    // metadata logs as a part of wear leveling. Suggested values are in the
    // range of 100-1000, or set block_cycles to -1 to disable block-level
    // wear-leveling.
    LFS_ASSERT(lfs->cfg->block_cycles != 0);


    // setup read cache
    if (lfs->cfg->read_buffer) {
        lfs->rcache.buffer = lfs->cfg->read_buffer;
    } else {
        lfs->rcache.buffer = lfs_malloc(lfs->cfg->cache_size);
        if (!lfs->rcache.buffer) {
            err = LFS_ERR_NOMEM;
            goto cleanup;
        }
    }

    // setup program cache
    if (lfs->cfg->prog_buffer) {
        lfs->pcache.buffer = lfs->cfg->prog_buffer;
    } else {
        lfs->pcache.buffer = lfs_malloc(lfs->cfg->cache_size);
        if (!lfs->pcache.buffer) {
            err = LFS_ERR_NOMEM;
            goto cleanup;
        }
    }

    // zero to avoid information leaks
    lfs_cache_zero(lfs, &lfs->rcache);
    lfs_cache_zero(lfs, &lfs->pcache);

    // setup lookahead, must be multiple of 64-bits, 32-bit aligned
    LFS_ASSERT(lfs->cfg->lookahead_size > 0);
    LFS_ASSERT(lfs->cfg->lookahead_size % 8 == 0 &&
            (uintptr_t)lfs->cfg->lookahead_buffer % 4 == 0);
    if (lfs->cfg->lookahead_buffer) {
        lfs->free.buffer = lfs->cfg->lookahead_buffer;
    } else {
        lfs->free.buffer = lfs_malloc(lfs->cfg->lookahead_size);
        if (!lfs->free.buffer) {
            err = LFS_ERR_NOMEM;
            goto cleanup;
        }
    }

    // check that the size limits are sane
    LFS_ASSERT(lfs->cfg->name_max <= LFS_NAME_MAX);
    lfs->name_max = lfs->cfg->name_max;
    if (!lfs->name_max) {
        lfs->name_max = LFS_NAME_MAX;
    }

    LFS_ASSERT(lfs->cfg->file_max <= LFS_FILE_MAX);
    lfs->file_max = lfs->cfg->file_max;
    if (!lfs->file_max) {
        lfs->file_max = LFS_FILE_MAX;
    }

    LFS_ASSERT(lfs->cfg->attr_max <= LFS_ATTR_MAX);
    lfs->attr_max = lfs->cfg->attr_max;
    if (!lfs->attr_max) {
        lfs->attr_max = LFS_ATTR_MAX;
    }

    LFS_ASSERT(lfs->cfg->metadata_max <= lfs->cfg->block_size);

    // setup default state
    lfs->root[0] = LFS_BLOCK_NULL;
    lfs->root[1] = LFS_BLOCK_NULL;
    lfs->mlist = NULL;
    lfs->seed = 0;
    lfs->gdisk = (lfs_gstate_t){0};
    lfs->gstate = (lfs_gstate_t){0};
    lfs->gdelta = (lfs_gstate_t){0};
#ifdef LFS_MIGRATE
    lfs->lfs1 = NULL;
#endif

    return 0;

cleanup:
    lfs_deinit(lfs);
    return err;
}

static int lfs_deinit(lfs_t *lfs) {
    // free allocated memory
    if (!lfs->cfg->read_buffer) {
        lfs_free(lfs->rcache.buffer);
    }

    if (!lfs->cfg->prog_buffer) {
        lfs_free(lfs->pcache.buffer);
    }

    if (!lfs->cfg->lookahead_buffer) {
        lfs_free(lfs->free.buffer);
    }

    return 0;
}

#ifndef LFS_READONLY
static int lfs_rawformat(lfs_t *lfs, const struct lfs_config *cfg) {
    int err = 0;
    {
        err = lfs_init(lfs, cfg);
        if (err) {
            return err;
        }

        // create free lookahead
        memset(lfs->free.buffer, 0, lfs->cfg->lookahead_size);
        lfs->free.off = 0;
        lfs->free.size = lfs_min(8*lfs->cfg->lookahead_size,
                lfs->cfg->block_count);
        lfs->free.i = 0;
        lfs_alloc_ack(lfs);

        // create root dir
        lfs_mdir_t root;
        err = lfs_dir_alloc(lfs, &root);
        if (err) {
            goto cleanup;
        }

        // write one superblock
        lfs_superblock_t superblock = {
            .version     = LFS_DISK_VERSION,
            .block_size  = lfs->cfg->block_size,
            .block_count = lfs->cfg->block_count,
            .name_max    = lfs->name_max,
            .file_max    = lfs->file_max,
            .attr_max    = lfs->attr_max,
        };

        lfs_superblock_tole32(&superblock);
        err = lfs_dir_commit(lfs, &root, LFS_MKATTRS(
                {LFS_MKTAG(LFS_TYPE_CREATE, 0, 0), NULL},
                {LFS_MKTAG(LFS_TYPE_SUPERBLOCK, 0, 8), "littlefs"},
                {LFS_MKTAG(LFS_TYPE_INLINESTRUCT, 0, sizeof(superblock)),
                    &superblock}));
        if (err) {
            goto cleanup;
        }

        // force compaction to prevent accidentally mounting any
        // older version of littlefs that may live on disk
        root.erased = false;
        err = lfs_dir_commit(lfs, &root, NULL, 0);
        if (err) {
            goto cleanup;
        }

        // sanity check that fetch works
        err = lfs_dir_fetch(lfs, &root, (const lfs_block_t[2]){0, 1});
        if (err) {
            goto cleanup;
        }
    }

cleanup:
    lfs_deinit(lfs);
    return err;

}
#endif

static int lfs_rawmount(lfs_t *lfs, const struct lfs_config *cfg) {
    int err = lfs_init(lfs, cfg);
    if (err) {
        return err;
    }

    // scan directory blocks for superblock and any global updates
    lfs_mdir_t dir = {.tail = {0, 1}};
    lfs_block_t tortoise[2] = {LFS_BLOCK_NULL, LFS_BLOCK_NULL};
    lfs_size_t tortoise_i = 1;
    lfs_size_t tortoise_period = 1;
    while (!lfs_pair_isnull(dir.tail)) {
        // detect cycles with Brent's algorithm
        if (lfs_pair_issync(dir.tail, tortoise)) {
            LFS_WARN("Cycle detected in tail list");
            err = LFS_ERR_CORRUPT;
            goto cleanup;
        }
        if (tortoise_i == tortoise_period) {
            tortoise[0] = dir.tail[0];
            tortoise[1] = dir.tail[1];
            tortoise_i = 0;
            tortoise_period *= 2;
        }
        tortoise_i += 1;

        // fetch next block in tail list
        lfs_stag_t tag = lfs_dir_fetchmatch(lfs, &dir, dir.tail,
                LFS_MKTAG(0x7ff, 0x3ff, 0),
                LFS_MKTAG(LFS_TYPE_SUPERBLOCK, 0, 8),
                NULL,
                lfs_dir_find_match, &(struct lfs_dir_find_match){
                    lfs, "littlefs", 8});
        if (tag < 0) {
            err = tag;
            goto cleanup;
        }

        // has superblock?
        if (tag && !lfs_tag_isdelete(tag)) {
            // update root
            lfs->root[0] = dir.pair[0];
            lfs->root[1] = dir.pair[1];

            // grab superblock
            lfs_superblock_t superblock;
            tag = lfs_dir_get(lfs, &dir, LFS_MKTAG(0x7ff, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_INLINESTRUCT, 0, sizeof(superblock)),
                    &superblock);
            if (tag < 0) {
                err = tag;
                goto cleanup;
            }
            lfs_superblock_fromle32(&superblock);

            // check version
            uint16_t major_version = (0xffff & (superblock.version >> 16));
            uint16_t minor_version = (0xffff & (superblock.version >>  0));
            if ((major_version != LFS_DISK_VERSION_MAJOR ||
                 minor_version > LFS_DISK_VERSION_MINOR)) {
                LFS_ERROR("Invalid version v%"PRIu16".%"PRIu16,
                        major_version, minor_version);
                err = LFS_ERR_INVAL;
                goto cleanup;
            }

            // check superblock configuration
            if (superblock.name_max) {
                if (superblock.name_max > lfs->name_max) {
                    LFS_ERROR("Unsupported name_max (%"PRIu32" > %"PRIu32")",
                            superblock.name_max, lfs->name_max);
                    err = LFS_ERR_INVAL;
                    goto cleanup;
                }

                lfs->name_max = superblock.name_max;
            }

            if (superblock.file_max) {
                if (superblock.file_max > lfs->file_max) {
                    LFS_ERROR("Unsupported file_max (%"PRIu32" > %"PRIu32")",
                            superblock.file_max, lfs->file_max);
                    err = LFS_ERR_INVAL;
                    goto cleanup;
                }

                lfs->file_max = superblock.file_max;
            }

            if (superblock.attr_max) {
                if (superblock.attr_max > lfs->attr_max) {
                    LFS_ERROR("Unsupported attr_max (%"PRIu32" > %"PRIu32")",
                            superblock.attr_max, lfs->attr_max);
                    err = LFS_ERR_INVAL;
                    goto cleanup;
                }

                lfs->attr_max = superblock.attr_max;
            }

            if (superblock.block_count != lfs->cfg->block_count) {
                LFS_ERROR("Invalid block count (%"PRIu32" != %"PRIu32")",
                        superblock.block_count, lfs->cfg->block_count);
                err = LFS_ERR_INVAL;
                goto cleanup;
            }

            if (superblock.block_size != lfs->cfg->block_size) {
                LFS_ERROR("Invalid block size (%"PRIu32" != %"PRIu32")",
                        superblock.block_size, lfs->cfg->block_size);
                err = LFS_ERR_INVAL;
                goto cleanup;
            }
        }

        // has gstate?
        err = lfs_dir_getgstate(lfs, &dir, &lfs->gstate);
        if (err) {
            goto cleanup;
        }
    }

    // found superblock?
    if (lfs_pair_isnull(lfs->root)) {
        err = LFS_ERR_INVAL;
        goto cleanup;
    }

    // update littlefs with gstate
    if (!lfs_gstate_iszero(&lfs->gstate)) {
        LFS_DEBUG("Found pending gstate 0x%08"PRIx32"%08"PRIx32"%08"PRIx32,
                lfs->gstate.tag,
                lfs->gstate.pair[0],
                lfs->gstate.pair[1]);
    }
    lfs->gstate.tag += !lfs_tag_isvalid(lfs->gstate.tag);
    lfs->gdisk = lfs->gstate;

    // setup free lookahead, to distribute allocations uniformly across
    // boots, we start the allocator at a random location
    lfs->free.off = lfs->seed % lfs->cfg->block_count;
    lfs_alloc_drop(lfs);

    return 0;

cleanup:
    lfs_rawunmount(lfs);
    return err;
}

static int lfs_rawunmount(lfs_t *lfs) {
    return lfs_deinit(lfs);
}


/// Filesystem filesystem operations ///
int lfs_fs_rawtraverse(lfs_t *lfs,
        int (*cb)(void *data, lfs_block_t block), void *data,
        bool includeorphans) {
    // iterate over metadata pairs
    lfs_mdir_t dir = {.tail = {0, 1}};

#ifdef LFS_MIGRATE
    // also consider v1 blocks during migration
    if (lfs->lfs1) {
        int err = lfs1_traverse(lfs, cb, data);
        if (err) {
            return err;
        }

        dir.tail[0] = lfs->root[0];
        dir.tail[1] = lfs->root[1];
    }
#endif

    lfs_block_t tortoise[2] = {LFS_BLOCK_NULL, LFS_BLOCK_NULL};
    lfs_size_t tortoise_i = 1;
    lfs_size_t tortoise_period = 1;
    while (!lfs_pair_isnull(dir.tail)) {
        // detect cycles with Brent's algorithm
        if (lfs_pair_issync(dir.tail, tortoise)) {
            LFS_WARN("Cycle detected in tail list");
            return LFS_ERR_CORRUPT;
        }
        if (tortoise_i == tortoise_period) {
            tortoise[0] = dir.tail[0];
            tortoise[1] = dir.tail[1];
            tortoise_i = 0;
            tortoise_period *= 2;
        }
        tortoise_i += 1;

        for (int i = 0; i < 2; i++) {
            int err = cb(data, dir.tail[i]);
            if (err) {
                return err;
            }
        }

        // iterate through ids in directory
        int err = lfs_dir_fetch(lfs, &dir, dir.tail);
        if (err) {
            return err;
        }

        for (uint16_t id = 0; id < dir.count; id++) {
            struct lfs_ctz ctz;
            lfs_stag_t tag = lfs_dir_get(lfs, &dir, LFS_MKTAG(0x700, 0x3ff, 0),
                    LFS_MKTAG(LFS_TYPE_STRUCT, id, sizeof(ctz)), &ctz);
            if (tag < 0) {
                if (tag == LFS_ERR_NOENT) {
                    continue;
                }
                return tag;
            }
            lfs_ctz_fromle32(&ctz);

            if (lfs_tag_type3(tag) == LFS_TYPE_CTZSTRUCT) {
                err = lfs_ctz_traverse(lfs, NULL, &lfs->rcache,
                        ctz.head, ctz.size, cb, data);
                if (err) {
                    return err;
                }
            } else if (includeorphans &&
                    lfs_tag_type3(tag) == LFS_TYPE_DIRSTRUCT) {
                for (int i = 0; i < 2; i++) {
                    err = cb(data, (&ctz.head)[i]);
                    if (err) {
                        return err;
                    }
                }
            }
        }
    }

#ifndef LFS_READONLY
    // iterate over any open files
    for (lfs_file_t *f = (lfs_file_t*)lfs->mlist; f; f = f->next) {
        if (f->type != LFS_TYPE_REG) {
            continue;
        }

        if ((f->flags & LFS_F_DIRTY) && !(f->flags & LFS_F_INLINE)) {
            int err = lfs_ctz_traverse(lfs, &f->cache, &lfs->rcache,
                    f->ctz.head, f->ctz.size, cb, data);
            if (err) {
                return err;
            }
        }

        if ((f->flags & LFS_F_WRITING) && !(f->flags & LFS_F_INLINE)) {
            int err = lfs_ctz_traverse(lfs, &f->cache, &lfs->rcache,
                    f->block, f->pos, cb, data);
            if (err) {
                return err;
            }
        }
    }
#endif

    return 0;
}

#ifndef LFS_READONLY
static int lfs_fs_pred(lfs_t *lfs,
        const lfs_block_t pair[2], lfs_mdir_t *pdir) {
    // iterate over all directory directory entries
    pdir->tail[0] = 0;
    pdir->tail[1] = 1;
    lfs_block_t tortoise[2] = {LFS_BLOCK_NULL, LFS_BLOCK_NULL};
    lfs_size_t tortoise_i = 1;
    lfs_size_t tortoise_period = 1;
    while (!lfs_pair_isnull(pdir->tail)) {
        // detect cycles with Brent's algorithm
        if (lfs_pair_issync(pdir->tail, tortoise)) {
            LFS_WARN("Cycle detected in tail list");
            return LFS_ERR_CORRUPT;
        }
        if (tortoise_i == tortoise_period) {
            tortoise[0] = pdir->tail[0];
            tortoise[1] = pdir->tail[1];
            tortoise_i = 0;
            tortoise_period *= 2;
        }
        tortoise_i += 1;

        if (lfs_pair_cmp(pdir->tail, pair) == 0) {
            return 0;
        }

        int err = lfs_dir_fetch(lfs, pdir, pdir->tail);
        if (err) {
            return err;
        }
    }

    return LFS_ERR_NOENT;
}
#endif

#ifndef LFS_READONLY
struct lfs_fs_parent_match {
    lfs_t *lfs;
    const lfs_block_t pair[2];
};
#endif

#ifndef LFS_READONLY
static int lfs_fs_parent_match(void *data,
        lfs_tag_t tag, const void *buffer) {
    struct lfs_fs_parent_match *find = data;
    lfs_t *lfs = find->lfs;
    const struct lfs_diskoff *disk = buffer;
    (void)tag;

    lfs_block_t child[2];
    int err = lfs_bd_read(lfs,
            &lfs->pcache, &lfs->rcache, lfs->cfg->block_size,
            disk->block, disk->off, &child, sizeof(child));
    if (err) {
        return err;
    }

    lfs_pair_fromle32(child);
    return (lfs_pair_cmp(child, find->pair) == 0) ? LFS_CMP_EQ : LFS_CMP_LT;
}
#endif

#ifndef LFS_READONLY
static lfs_stag_t lfs_fs_parent(lfs_t *lfs, const lfs_block_t pair[2],
        lfs_mdir_t *parent) {
    // use fetchmatch with callback to find pairs
    parent->tail[0] = 0;
    parent->tail[1] = 1;
    lfs_block_t tortoise[2] = {LFS_BLOCK_NULL, LFS_BLOCK_NULL};
    lfs_size_t tortoise_i = 1;
    lfs_size_t tortoise_period = 1;
    while (!lfs_pair_isnull(parent->tail)) {
        // detect cycles with Brent's algorithm
        if (lfs_pair_issync(parent->tail, tortoise)) {
            LFS_WARN("Cycle detected in tail list");
            return LFS_ERR_CORRUPT;
        }
        if (tortoise_i == tortoise_period) {
            tortoise[0] = parent->tail[0];
            tortoise[1] = parent->tail[1];
            tortoise_i = 0;
            tortoise_period *= 2;
        }
        tortoise_i += 1;

        lfs_stag_t tag = lfs_dir_fetchmatch(lfs, parent, parent->tail,
                LFS_MKTAG(0x7ff, 0, 0x3ff),
                LFS_MKTAG(LFS_TYPE_DIRSTRUCT, 0, 8),
                NULL,
                lfs_fs_parent_match, &(struct lfs_fs_parent_match){
                    lfs, {pair[0], pair[1]}});
        if (tag && tag != LFS_ERR_NOENT) {
            return tag;
        }
    }

    return LFS_ERR_NOENT;
}
#endif

#ifndef LFS_READONLY
static int lfs_fs_preporphans(lfs_t *lfs, int8_t orphans) {
    LFS_ASSERT(lfs_tag_size(lfs->gstate.tag) > 0x000 || orphans >= 0);
    LFS_ASSERT(lfs_tag_size(lfs->gstate.tag) < 0x3ff || orphans <= 0);
    lfs->gstate.tag += orphans;
    lfs->gstate.tag = ((lfs->gstate.tag & ~LFS_MKTAG(0x800, 0, 0)) |
            ((uint32_t)lfs_gstate_hasorphans(&lfs->gstate) << 31));

    return 0;
}
#endif

#ifndef LFS_READONLY
static void lfs_fs_prepmove(lfs_t *lfs,
        uint16_t id, const lfs_block_t pair[2]) {
    lfs->gstate.tag = ((lfs->gstate.tag & ~LFS_MKTAG(0x7ff, 0x3ff, 0)) |
            ((id != 0x3ff) ? LFS_MKTAG(LFS_TYPE_DELETE, id, 0) : 0));
    lfs->gstate.pair[0] = (id != 0x3ff) ? pair[0] : 0;
    lfs->gstate.pair[1] = (id != 0x3ff) ? pair[1] : 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_fs_demove(lfs_t *lfs) {
    if (!lfs_gstate_hasmove(&lfs->gdisk)) {
        return 0;
    }

    // Fix bad moves
    LFS_DEBUG("Fixing move {0x%"PRIx32", 0x%"PRIx32"} 0x%"PRIx16,
            lfs->gdisk.pair[0],
            lfs->gdisk.pair[1],
            lfs_tag_id(lfs->gdisk.tag));

    // no other gstate is supported at this time, so if we found something else
    // something most likely went wrong in gstate calculation
    LFS_ASSERT(lfs_tag_type3(lfs->gdisk.tag) == LFS_TYPE_DELETE);

    // fetch and delete the moved entry
    lfs_mdir_t movedir;
    int err = lfs_dir_fetch(lfs, &movedir, lfs->gdisk.pair);
    if (err) {
        return err;
    }

    // prep gstate and delete move id
    uint16_t moveid = lfs_tag_id(lfs->gdisk.tag);
    lfs_fs_prepmove(lfs, 0x3ff, NULL);
    err = lfs_dir_commit(lfs, &movedir, LFS_MKATTRS(
            {LFS_MKTAG(LFS_TYPE_DELETE, moveid, 0), NULL}));
    if (err) {
        return err;
    }

    return 0;
}
#endif

#ifndef LFS_READONLY
static int lfs_fs_deorphan(lfs_t *lfs, bool powerloss) {
    if (!lfs_gstate_hasorphans(&lfs->gstate)) {
        return 0;
    }

    int8_t found = 0;

    // Check for orphans in two separate passes:
    // - 1 for half-orphans (relocations)
    // - 2 for full-orphans (removes/renames)
    //
    // Two separate passes are needed as half-orphans can contain outdated
    // references to full-orphans, effectively hiding them from the deorphan
    // search.
    //
    int pass = 0;
    while (pass < 2) {
        // Fix any orphans
        lfs_mdir_t pdir = {.split = true, .tail = {0, 1}};
        lfs_mdir_t dir;
        bool moreorphans = false;

        // iterate over all directory directory entries
        while (!lfs_pair_isnull(pdir.tail)) {
            int err = lfs_dir_fetch(lfs, &dir, pdir.tail);
            if (err) {
                return err;
            }

            // check head blocks for orphans
            if (!pdir.split) {
                // check if we have a parent
                lfs_mdir_t parent;
                lfs_stag_t tag = lfs_fs_parent(lfs, pdir.tail, &parent);
                if (tag < 0 && tag != LFS_ERR_NOENT) {
                    return tag;
                }

                if (pass == 0 && tag != LFS_ERR_NOENT) {
                    lfs_block_t pair[2];
                    lfs_stag_t state = lfs_dir_get(lfs, &parent,
                            LFS_MKTAG(0x7ff, 0x3ff, 0), tag, pair);
                    if (state < 0) {
                        return state;
                    }
                    lfs_pair_fromle32(pair);

                    if (!lfs_pair_issync(pair, pdir.tail)) {
                        // we have desynced
                        LFS_DEBUG("Fixing half-orphan "
                                "{0x%"PRIx32", 0x%"PRIx32"} "
                                "-> {0x%"PRIx32", 0x%"PRIx32"}",
                                pdir.tail[0], pdir.tail[1], pair[0], pair[1]);

                        // fix pending move in this pair? this looks like an
                        // optimization but is in fact _required_ since
                        // relocating may outdate the move.
                        uint16_t moveid = 0x3ff;
                        if (lfs_gstate_hasmovehere(&lfs->gstate, pdir.pair)) {
                            moveid = lfs_tag_id(lfs->gstate.tag);
                            LFS_DEBUG("Fixing move while fixing orphans "
                                    "{0x%"PRIx32", 0x%"PRIx32"} 0x%"PRIx16"\n",
                                    pdir.pair[0], pdir.pair[1], moveid);
                            lfs_fs_prepmove(lfs, 0x3ff, NULL);
                        }

                        lfs_pair_tole32(pair);
                        state = lfs_dir_orphaningcommit(lfs, &pdir, LFS_MKATTRS(
                                {LFS_MKTAG_IF(moveid != 0x3ff,
                                    LFS_TYPE_DELETE, moveid, 0), NULL},
                                {LFS_MKTAG(LFS_TYPE_SOFTTAIL, 0x3ff, 8),
                                    pair}));
                        lfs_pair_fromle32(pair);
                        if (state < 0) {
                            return state;
                        }

                        found += 1;

                        // did our commit create more orphans?
                        if (state == LFS_OK_ORPHANED) {
                            moreorphans = true;
                        }

                        // refetch tail
                        continue;
                    }
                }

                // note we only check for full orphans if we may have had a
                // power-loss, otherwise orphans are created intentionally
                // during operations such as lfs_mkdir
                if (pass == 1 && tag == LFS_ERR_NOENT && powerloss) {
                    // we are an orphan
                    LFS_DEBUG("Fixing orphan {0x%"PRIx32", 0x%"PRIx32"}",
                            pdir.tail[0], pdir.tail[1]);

                    // steal state
                    err = lfs_dir_getgstate(lfs, &dir, &lfs->gdelta);
                    if (err) {
                        return err;
                    }

                    // steal tail
                    lfs_pair_tole32(dir.tail);
                    int state = lfs_dir_orphaningcommit(lfs, &pdir, LFS_MKATTRS(
                            {LFS_MKTAG(LFS_TYPE_TAIL + dir.split, 0x3ff, 8),
                                dir.tail}));
                    lfs_pair_fromle32(dir.tail);
                    if (state < 0) {
                        return state;
                    }

                    found += 1;

                    // did our commit create more orphans?
                    if (state == LFS_OK_ORPHANED) {
                        moreorphans = true;
                    }

                    // refetch tail
                    continue;
                }
            }

            pdir = dir;
        }

        pass = moreorphans ? 0 : pass+1;
    }

    // mark orphans as fixed
    return lfs_fs_preporphans(lfs, -lfs_min(
            lfs_gstate_getorphans(&lfs->gstate),
            found));
}
#endif

#ifndef LFS_READONLY
static int lfs_fs_forceconsistency(lfs_t *lfs) {
    int err = lfs_fs_demove(lfs);
    if (err) {
        return err;
    }

    err = lfs_fs_deorphan(lfs, true);
    if (err) {
        return err;
    }

    return 0;
}
#endif

static int lfs_fs_size_count(void *p, lfs_block_t block) {
    (void)block;
    lfs_size_t *size = p;
    *size += 1;
    return 0;
}

static lfs_ssize_t lfs_fs_rawsize(lfs_t *lfs) {
    lfs_size_t size = 0;
    int err = lfs_fs_rawtraverse(lfs, lfs_fs_size_count, &size, false);
    if (err) {
        return err;
    }

    return size;
}

#ifdef LFS_MIGRATE
////// Migration from littelfs v1 below this //////

/// Version info ///

// Software library version
// Major (top-nibble), incremented on backwards incompatible changes
// Minor (bottom-nibble), incremented on feature additions
#define LFS1_VERSION 0x00010007
#define LFS1_VERSION_MAJOR (0xffff & (LFS1_VERSION >> 16))
#define LFS1_VERSION_MINOR (0xffff & (LFS1_VERSION >>  0))

// Version of On-disk data structures
// Major (top-nibble), incremented on backwards incompatible changes
// Minor (bottom-nibble), incremented on feature additions
#define LFS1_DISK_VERSION 0x00010001
#define LFS1_DISK_VERSION_MAJOR (0xffff & (LFS1_DISK_VERSION >> 16))
#define LFS1_DISK_VERSION_MINOR (0xffff & (LFS1_DISK_VERSION >>  0))


/// v1 Definitions ///

// File types
enum lfs1_type {
    LFS1_TYPE_REG        = 0x11,
    LFS1_TYPE_DIR        = 0x22,
    LFS1_TYPE_SUPERBLOCK = 0x2e,
};

typedef struct lfs1 {
    lfs_block_t root[2];
} lfs1_t;

typedef struct lfs1_entry {
    lfs_off_t off;

    struct lfs1_disk_entry {
        uint8_t type;
        uint8_t elen;
        uint8_t alen;
        uint8_t nlen;
        union {
            struct {
                lfs_block_t head;
                lfs_size_t size;
            } file;
            lfs_block_t dir[2];
        } u;
    } d;
} lfs1_entry_t;

typedef struct lfs1_dir {
    struct lfs1_dir *next;
    lfs_block_t pair[2];
    lfs_off_t off;

    lfs_block_t head[2];
    lfs_off_t pos;

    struct lfs1_disk_dir {
        uint32_t rev;
        lfs_size_t size;
        lfs_block_t tail[2];
    } d;
} lfs1_dir_t;

typedef struct lfs1_superblock {
    lfs_off_t off;

    struct lfs1_disk_superblock {
        uint8_t type;
        uint8_t elen;
        uint8_t alen;
        uint8_t nlen;
        lfs_block_t root[2];
        uint32_t block_size;
        uint32_t block_count;
        uint32_t version;
        char magic[8];
    } d;
} lfs1_superblock_t;


/// Low-level wrappers v1->v2 ///
static void lfs1_crc(uint32_t *crc, const void *buffer, size_t size) {
    *crc = lfs_crc(*crc, buffer, size);
}

static int lfs1_bd_read(lfs_t *lfs, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    // if we ever do more than writes to alternating pairs,
    // this may need to consider pcache
    return lfs_bd_read(lfs, &lfs->pcache, &lfs->rcache, size,
            block, off, buffer, size);
}

static int lfs1_bd_crc(lfs_t *lfs, lfs_block_t block,
        lfs_off_t off, lfs_size_t size, uint32_t *crc) {
    for (lfs_off_t i = 0; i < size; i++) {
        uint8_t c;
        int err = lfs1_bd_read(lfs, block, off+i, &c, 1);
        if (err) {
            return err;
        }

        lfs1_crc(crc, &c, 1);
    }

    return 0;
}


/// Endian swapping functions ///
static void lfs1_dir_fromle32(struct lfs1_disk_dir *d) {
    d->rev     = lfs_fromle32(d->rev);
    d->size    = lfs_fromle32(d->size);
    d->tail[0] = lfs_fromle32(d->tail[0]);
    d->tail[1] = lfs_fromle32(d->tail[1]);
}

static void lfs1_dir_tole32(struct lfs1_disk_dir *d) {
    d->rev     = lfs_tole32(d->rev);
    d->size    = lfs_tole32(d->size);
    d->tail[0] = lfs_tole32(d->tail[0]);
    d->tail[1] = lfs_tole32(d->tail[1]);
}

static void lfs1_entry_fromle32(struct lfs1_disk_entry *d) {
    d->u.dir[0] = lfs_fromle32(d->u.dir[0]);
    d->u.dir[1] = lfs_fromle32(d->u.dir[1]);
}

static void lfs1_entry_tole32(struct lfs1_disk_entry *d) {
    d->u.dir[0] = lfs_tole32(d->u.dir[0]);
    d->u.dir[1] = lfs_tole32(d->u.dir[1]);
}

static void lfs1_superblock_fromle32(struct lfs1_disk_superblock *d) {
    d->root[0]     = lfs_fromle32(d->root[0]);
    d->root[1]     = lfs_fromle32(d->root[1]);
    d->block_size  = lfs_fromle32(d->block_size);
    d->block_count = lfs_fromle32(d->block_count);
    d->version     = lfs_fromle32(d->version);
}


///// Metadata pair and directory operations ///
static inline lfs_size_t lfs1_entry_size(const lfs1_entry_t *entry) {
    return 4 + entry->d.elen + entry->d.alen + entry->d.nlen;
}

static int lfs1_dir_fetch(lfs_t *lfs,
        lfs1_dir_t *dir, const lfs_block_t pair[2]) {
    // copy out pair, otherwise may be aliasing dir
    const lfs_block_t tpair[2] = {pair[0], pair[1]};
    bool valid = false;

    // check both blocks for the most recent revision
    for (int i = 0; i < 2; i++) {
        struct lfs1_disk_dir test;
        int err = lfs1_bd_read(lfs, tpair[i], 0, &test, sizeof(test));
        lfs1_dir_fromle32(&test);
        if (err) {
            if (err == LFS_ERR_CORRUPT) {
                continue;
            }
            return err;
        }

        if (valid && lfs_scmp(test.rev, dir->d.rev) < 0) {
            continue;
        }

        if ((0x7fffffff & test.size) < sizeof(test)+4 ||
            (0x7fffffff & test.size) > lfs->cfg->block_size) {
            continue;
        }

        uint32_t crc = 0xffffffff;
        lfs1_dir_tole32(&test);
        lfs1_crc(&crc, &test, sizeof(test));
        lfs1_dir_fromle32(&test);
        err = lfs1_bd_crc(lfs, tpair[i], sizeof(test),
                (0x7fffffff & test.size) - sizeof(test), &crc);
        if (err) {
            if (err == LFS_ERR_CORRUPT) {
                continue;
            }
            return err;
        }

        if (crc != 0) {
            continue;
        }

        valid = true;

        // setup dir in case it's valid
        dir->pair[0] = tpair[(i+0) % 2];
        dir->pair[1] = tpair[(i+1) % 2];
        dir->off = sizeof(dir->d);
        dir->d = test;
    }

    if (!valid) {
        LFS_ERROR("Corrupted dir pair at {0x%"PRIx32", 0x%"PRIx32"}",
                tpair[0], tpair[1]);
        return LFS_ERR_CORRUPT;
    }

    return 0;
}

static int lfs1_dir_next(lfs_t *lfs, lfs1_dir_t *dir, lfs1_entry_t *entry) {
    while (dir->off + sizeof(entry->d) > (0x7fffffff & dir->d.size)-4) {
        if (!(0x80000000 & dir->d.size)) {
            entry->off = dir->off;
            return LFS_ERR_NOENT;
        }

        int err = lfs1_dir_fetch(lfs, dir, dir->d.tail);
        if (err) {
            return err;
        }

        dir->off = sizeof(dir->d);
        dir->pos += sizeof(dir->d) + 4;
    }

    int err = lfs1_bd_read(lfs, dir->pair[0], dir->off,
            &entry->d, sizeof(entry->d));
    lfs1_entry_fromle32(&entry->d);
    if (err) {
        return err;
    }

    entry->off = dir->off;
    dir->off += lfs1_entry_size(entry);
    dir->pos += lfs1_entry_size(entry);
    return 0;
}

/// littlefs v1 specific operations ///
int lfs1_traverse(lfs_t *lfs, int (*cb)(void*, lfs_block_t), void *data) {
    if (lfs_pair_isnull(lfs->lfs1->root)) {
        return 0;
    }

    // iterate over metadata pairs
    lfs1_dir_t dir;
    lfs1_entry_t entry;
    lfs_block_t cwd[2] = {0, 1};

    while (true) {
        for (int i = 0; i < 2; i++) {
            int err = cb(data, cwd[i]);
            if (err) {
                return err;
            }
        }

        int err = lfs1_dir_fetch(lfs, &dir, cwd);
        if (err) {
            return err;
        }

        // iterate over contents
        while (dir.off + sizeof(entry.d) <= (0x7fffffff & dir.d.size)-4) {
            err = lfs1_bd_read(lfs, dir.pair[0], dir.off,
                    &entry.d, sizeof(entry.d));
            lfs1_entry_fromle32(&entry.d);
            if (err) {
                return err;
            }

            dir.off += lfs1_entry_size(&entry);
            if ((0x70 & entry.d.type) == (0x70 & LFS1_TYPE_REG)) {
                err = lfs_ctz_traverse(lfs, NULL, &lfs->rcache,
                        entry.d.u.file.head, entry.d.u.file.size, cb, data);
                if (err) {
                    return err;
                }
            }
        }

        // we also need to check if we contain a threaded v2 directory
        lfs_mdir_t dir2 = {.split=true, .tail={cwd[0], cwd[1]}};
        while (dir2.split) {
            err = lfs_dir_fetch(lfs, &dir2, dir2.tail);
            if (err) {
                break;
            }

            for (int i = 0; i < 2; i++) {
                err = cb(data, dir2.pair[i]);
                if (err) {
                    return err;
                }
            }
        }

        cwd[0] = dir.d.tail[0];
        cwd[1] = dir.d.tail[1];

        if (lfs_pair_isnull(cwd)) {
            break;
        }
    }

    return 0;
}

static int lfs1_moved(lfs_t *lfs, const void *e) {
    if (lfs_pair_isnull(lfs->lfs1->root)) {
        return 0;
    }

    // skip superblock
    lfs1_dir_t cwd;
    int err = lfs1_dir_fetch(lfs, &cwd, (const lfs_block_t[2]){0, 1});
    if (err) {
        return err;
    }

    // iterate over all directory directory entries
    lfs1_entry_t entry;
    while (!lfs_pair_isnull(cwd.d.tail)) {
        err = lfs1_dir_fetch(lfs, &cwd, cwd.d.tail);
        if (err) {
            return err;
        }

        while (true) {
            err = lfs1_dir_next(lfs, &cwd, &entry);
            if (err && err != LFS_ERR_NOENT) {
                return err;
            }

            if (err == LFS_ERR_NOENT) {
                break;
            }

            if (!(0x80 & entry.d.type) &&
                 memcmp(&entry.d.u, e, sizeof(entry.d.u)) == 0) {
                return true;
            }
        }
    }

    return false;
}

/// Filesystem operations ///
static int lfs1_mount(lfs_t *lfs, struct lfs1 *lfs1,
        const struct lfs_config *cfg) {
    int err = 0;
    {
        err = lfs_init(lfs, cfg);
        if (err) {
            return err;
        }

        lfs->lfs1 = lfs1;
        lfs->lfs1->root[0] = LFS_BLOCK_NULL;
        lfs->lfs1->root[1] = LFS_BLOCK_NULL;

        // setup free lookahead
        lfs->free.off = 0;
        lfs->free.size = 0;
        lfs->free.i = 0;
        lfs_alloc_ack(lfs);

        // load superblock
        lfs1_dir_t dir;
        lfs1_superblock_t superblock;
        err = lfs1_dir_fetch(lfs, &dir, (const lfs_block_t[2]){0, 1});
        if (err && err != LFS_ERR_CORRUPT) {
            goto cleanup;
        }

        if (!err) {
            err = lfs1_bd_read(lfs, dir.pair[0], sizeof(dir.d),
                    &superblock.d, sizeof(superblock.d));
            lfs1_superblock_fromle32(&superblock.d);
            if (err) {
                goto cleanup;
            }

            lfs->lfs1->root[0] = superblock.d.root[0];
            lfs->lfs1->root[1] = superblock.d.root[1];
        }

        if (err || memcmp(superblock.d.magic, "littlefs", 8) != 0) {
            LFS_ERROR("Invalid superblock at {0x%"PRIx32", 0x%"PRIx32"}",
                    0, 1);
            err = LFS_ERR_CORRUPT;
            goto cleanup;
        }

        uint16_t major_version = (0xffff & (superblock.d.version >> 16));
        uint16_t minor_version = (0xffff & (superblock.d.version >>  0));
        if ((major_version != LFS1_DISK_VERSION_MAJOR ||
             minor_version > LFS1_DISK_VERSION_MINOR)) {
            LFS_ERROR("Invalid version v%d.%d", major_version, minor_version);
            err = LFS_ERR_INVAL;
            goto cleanup;
        }

        return 0;
    }

cleanup:
    lfs_deinit(lfs);
    return err;
}

static int lfs1_unmount(lfs_t *lfs) {
    return lfs_deinit(lfs);
}

/// v1 migration ///
static int lfs_rawmigrate(lfs_t *lfs, const struct lfs_config *cfg) {
    struct lfs1 lfs1;
    int err = lfs1_mount(lfs, &lfs1, cfg);
    if (err) {
        return err;
    }

    {
        // iterate through each directory, copying over entries
        // into new directory
        lfs1_dir_t dir1;
        lfs_mdir_t dir2;
        dir1.d.tail[0] = lfs->lfs1->root[0];
        dir1.d.tail[1] = lfs->lfs1->root[1];
        while (!lfs_pair_isnull(dir1.d.tail)) {
            // iterate old dir
            err = lfs1_dir_fetch(lfs, &dir1, dir1.d.tail);
            if (err) {
                goto cleanup;
            }

            // create new dir and bind as temporary pretend root
            err = lfs_dir_alloc(lfs, &dir2);
            if (err) {
                goto cleanup;
            }

            dir2.rev = dir1.d.rev;
            dir1.head[0] = dir1.pair[0];
            dir1.head[1] = dir1.pair[1];
            lfs->root[0] = dir2.pair[0];
            lfs->root[1] = dir2.pair[1];

            err = lfs_dir_commit(lfs, &dir2, NULL, 0);
            if (err) {
                goto cleanup;
            }

            while (true) {
                lfs1_entry_t entry1;
                err = lfs1_dir_next(lfs, &dir1, &entry1);
                if (err && err != LFS_ERR_NOENT) {
                    goto cleanup;
                }

                if (err == LFS_ERR_NOENT) {
                    break;
                }

                // check that entry has not been moved
                if (entry1.d.type & 0x80) {
                    int moved = lfs1_moved(lfs, &entry1.d.u);
                    if (moved < 0) {
                        err = moved;
                        goto cleanup;
                    }

                    if (moved) {
                        continue;
                    }

                    entry1.d.type &= ~0x80;
                }

                // also fetch name
                char name[LFS_NAME_MAX+1];
                memset(name, 0, sizeof(name));
                err = lfs1_bd_read(lfs, dir1.pair[0],
                        entry1.off + 4+entry1.d.elen+entry1.d.alen,
                        name, entry1.d.nlen);
                if (err) {
                    goto cleanup;
                }

                bool isdir = (entry1.d.type == LFS1_TYPE_DIR);

                // create entry in new dir
                err = lfs_dir_fetch(lfs, &dir2, lfs->root);
                if (err) {
                    goto cleanup;
                }

                uint16_t id;
                err = lfs_dir_find(lfs, &dir2, &(const char*){name}, &id);
                if (!(err == LFS_ERR_NOENT && id != 0x3ff)) {
                    err = (err < 0) ? err : LFS_ERR_EXIST;
                    goto cleanup;
                }

                lfs1_entry_tole32(&entry1.d);
                err = lfs_dir_commit(lfs, &dir2, LFS_MKATTRS(
                        {LFS_MKTAG(LFS_TYPE_CREATE, id, 0), NULL},
                        {LFS_MKTAG_IF_ELSE(isdir,
                            LFS_TYPE_DIR, id, entry1.d.nlen,
                            LFS_TYPE_REG, id, entry1.d.nlen),
                                name},
                        {LFS_MKTAG_IF_ELSE(isdir,
                            LFS_TYPE_DIRSTRUCT, id, sizeof(entry1.d.u),
                            LFS_TYPE_CTZSTRUCT, id, sizeof(entry1.d.u)),
                                &entry1.d.u}));
                lfs1_entry_fromle32(&entry1.d);
                if (err) {
                    goto cleanup;
                }
            }

            if (!lfs_pair_isnull(dir1.d.tail)) {
                // find last block and update tail to thread into fs
                err = lfs_dir_fetch(lfs, &dir2, lfs->root);
                if (err) {
                    goto cleanup;
                }

                while (dir2.split) {
                    err = lfs_dir_fetch(lfs, &dir2, dir2.tail);
                    if (err) {
                        goto cleanup;
                    }
                }

                lfs_pair_tole32(dir2.pair);
                err = lfs_dir_commit(lfs, &dir2, LFS_MKATTRS(
                        {LFS_MKTAG(LFS_TYPE_SOFTTAIL, 0x3ff, 8), dir1.d.tail}));
                lfs_pair_fromle32(dir2.pair);
                if (err) {
                    goto cleanup;
                }
            }

            // Copy over first block to thread into fs. Unfortunately
            // if this fails there is not much we can do.
            LFS_DEBUG("Migrating {0x%"PRIx32", 0x%"PRIx32"} "
                        "-> {0x%"PRIx32", 0x%"PRIx32"}",
                    lfs->root[0], lfs->root[1], dir1.head[0], dir1.head[1]);

            err = lfs_bd_erase(lfs, dir1.head[1]);
            if (err) {
                goto cleanup;
            }

            err = lfs_dir_fetch(lfs, &dir2, lfs->root);
            if (err) {
                goto cleanup;
            }

            for (lfs_off_t i = 0; i < dir2.off; i++) {
                uint8_t dat;
                err = lfs_bd_read(lfs,
                        NULL, &lfs->rcache, dir2.off,
                        dir2.pair[0], i, &dat, 1);
                if (err) {
                    goto cleanup;
                }

                err = lfs_bd_prog(lfs,
                        &lfs->pcache, &lfs->rcache, true,
                        dir1.head[1], i, &dat, 1);
                if (err) {
                    goto cleanup;
                }
            }

            err = lfs_bd_flush(lfs, &lfs->pcache, &lfs->rcache, true);
            if (err) {
                goto cleanup;
            }
        }

        // Create new superblock. This marks a successful migration!
        err = lfs1_dir_fetch(lfs, &dir1, (const lfs_block_t[2]){0, 1});
        if (err) {
            goto cleanup;
        }

        dir2.pair[0] = dir1.pair[0];
        dir2.pair[1] = dir1.pair[1];
        dir2.rev = dir1.d.rev;
        dir2.off = sizeof(dir2.rev);
        dir2.etag = 0xffffffff;
        dir2.count = 0;
        dir2.tail[0] = lfs->lfs1->root[0];
        dir2.tail[1] = lfs->lfs1->root[1];
        dir2.erased = false;
        dir2.split = true;

        lfs_superblock_t superblock = {
            .version     = LFS_DISK_VERSION,
            .block_size  = lfs->cfg->block_size,
            .block_count = lfs->cfg->block_count,
            .name_max    = lfs->name_max,
            .file_max    = lfs->file_max,
            .attr_max    = lfs->attr_max,
        };

        lfs_superblock_tole32(&superblock);
        err = lfs_dir_commit(lfs, &dir2, LFS_MKATTRS(
                {LFS_MKTAG(LFS_TYPE_CREATE, 0, 0), NULL},
                {LFS_MKTAG(LFS_TYPE_SUPERBLOCK, 0, 8), "littlefs"},
                {LFS_MKTAG(LFS_TYPE_INLINESTRUCT, 0, sizeof(superblock)),
                    &superblock}));
        if (err) {
            goto cleanup;
        }

        // sanity check that fetch works
        err = lfs_dir_fetch(lfs, &dir2, (const lfs_block_t[2]){0, 1});
        if (err) {
            goto cleanup;
        }

        // force compaction to prevent accidentally mounting v1
        dir2.erased = false;
        err = lfs_dir_commit(lfs, &dir2, NULL, 0);
        if (err) {
            goto cleanup;
        }
    }

cleanup:
    lfs1_unmount(lfs);
    return err;
}

#endif


/// Public API wrappers ///

// Here we can add tracing/thread safety easily

// Thread-safe wrappers if enabled
#ifdef LFS_THREADSAFE
#define LFS_LOCK(cfg)   cfg->lock(cfg)
#define LFS_UNLOCK(cfg) cfg->unlock(cfg)
#else
#define LFS_LOCK(cfg)   ((void)cfg, 0)
#define LFS_UNLOCK(cfg) ((void)cfg)
#endif

// Public API
#ifndef LFS_READONLY
int lfs_format(lfs_t *lfs, const struct lfs_config *cfg) {
    int err = LFS_LOCK(cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_format(%p, %p {.context=%p, "
                ".read=%p, .prog=%p, .erase=%p, .sync=%p, "
                ".read_size=%"PRIu32", .prog_size=%"PRIu32", "
                ".block_size=%"PRIu32", .block_count=%"PRIu32", "
                ".block_cycles=%"PRIu32", .cache_size=%"PRIu32", "
                ".lookahead_size=%"PRIu32", .read_buffer=%p, "
                ".prog_buffer=%p, .lookahead_buffer=%p, "
                ".name_max=%"PRIu32", .file_max=%"PRIu32", "
                ".attr_max=%"PRIu32"})",
            (void*)lfs, (void*)cfg, cfg->context,
            (void*)(uintptr_t)cfg->read, (void*)(uintptr_t)cfg->prog,
            (void*)(uintptr_t)cfg->erase, (void*)(uintptr_t)cfg->sync,
            cfg->read_size, cfg->prog_size, cfg->block_size, cfg->block_count,
            cfg->block_cycles, cfg->cache_size, cfg->lookahead_size,
            cfg->read_buffer, cfg->prog_buffer, cfg->lookahead_buffer,
            cfg->name_max, cfg->file_max, cfg->attr_max);

    err = lfs_rawformat(lfs, cfg);

    LFS_TRACE("lfs_format -> %d", err);
    LFS_UNLOCK(cfg);
    return err;
}
#endif

int lfs_mount(lfs_t *lfs, const struct lfs_config *cfg) {
    int err = LFS_LOCK(cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_mount(%p, %p {.context=%p, "
                ".read=%p, .prog=%p, .erase=%p, .sync=%p, "
                ".read_size=%"PRIu32", .prog_size=%"PRIu32", "
                ".block_size=%"PRIu32", .block_count=%"PRIu32", "
                ".block_cycles=%"PRIu32", .cache_size=%"PRIu32", "
                ".lookahead_size=%"PRIu32", .read_buffer=%p, "
                ".prog_buffer=%p, .lookahead_buffer=%p, "
                ".name_max=%"PRIu32", .file_max=%"PRIu32", "
                ".attr_max=%"PRIu32"})",
            (void*)lfs, (void*)cfg, cfg->context,
            (void*)(uintptr_t)cfg->read, (void*)(uintptr_t)cfg->prog,
            (void*)(uintptr_t)cfg->erase, (void*)(uintptr_t)cfg->sync,
            cfg->read_size, cfg->prog_size, cfg->block_size, cfg->block_count,
            cfg->block_cycles, cfg->cache_size, cfg->lookahead_size,
            cfg->read_buffer, cfg->prog_buffer, cfg->lookahead_buffer,
            cfg->name_max, cfg->file_max, cfg->attr_max);

    err = lfs_rawmount(lfs, cfg);

    LFS_TRACE("lfs_mount -> %d", err);
    LFS_UNLOCK(cfg);
    return err;
}

int lfs_unmount(lfs_t *lfs) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_unmount(%p)", (void*)lfs);

    err = lfs_rawunmount(lfs);

    LFS_TRACE("lfs_unmount -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

#ifndef LFS_READONLY
int lfs_remove(lfs_t *lfs, const char *path) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_remove(%p, \"%s\")", (void*)lfs, path);

    err = lfs_rawremove(lfs, path);

    LFS_TRACE("lfs_remove -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

#ifndef LFS_READONLY
int lfs_rename(lfs_t *lfs, const char *oldpath, const char *newpath) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_rename(%p, \"%s\", \"%s\")", (void*)lfs, oldpath, newpath);

    err = lfs_rawrename(lfs, oldpath, newpath);

    LFS_TRACE("lfs_rename -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

int lfs_stat(lfs_t *lfs, const char *path, struct lfs_info *info) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_stat(%p, \"%s\", %p)", (void*)lfs, path, (void*)info);

    err = lfs_rawstat(lfs, path, info);

    LFS_TRACE("lfs_stat -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

lfs_ssize_t lfs_getattr(lfs_t *lfs, const char *path,
        uint8_t type, void *buffer, lfs_size_t size) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_getattr(%p, \"%s\", %"PRIu8", %p, %"PRIu32")",
            (void*)lfs, path, type, buffer, size);

    lfs_ssize_t res = lfs_rawgetattr(lfs, path, type, buffer, size);

    LFS_TRACE("lfs_getattr -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

#ifndef LFS_READONLY
int lfs_setattr(lfs_t *lfs, const char *path,
        uint8_t type, const void *buffer, lfs_size_t size) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_setattr(%p, \"%s\", %"PRIu8", %p, %"PRIu32")",
            (void*)lfs, path, type, buffer, size);

    err = lfs_rawsetattr(lfs, path, type, buffer, size);

    LFS_TRACE("lfs_setattr -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

#ifndef LFS_READONLY
int lfs_removeattr(lfs_t *lfs, const char *path, uint8_t type) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_removeattr(%p, \"%s\", %"PRIu8")", (void*)lfs, path, type);

    err = lfs_rawremoveattr(lfs, path, type);

    LFS_TRACE("lfs_removeattr -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

#ifndef LFS_NO_MALLOC
int lfs_file_open(lfs_t *lfs, lfs_file_t *file, const char *path, int flags) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_open(%p, %p, \"%s\", %x)",
            (void*)lfs, (void*)file, path, flags);
    LFS_ASSERT(!lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    err = lfs_file_rawopen(lfs, file, path, flags);

    LFS_TRACE("lfs_file_open -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

int lfs_file_opencfg(lfs_t *lfs, lfs_file_t *file,
        const char *path, int flags,
        const struct lfs_file_config *cfg) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_opencfg(%p, %p, \"%s\", %x, %p {"
                 ".buffer=%p, .attrs=%p, .attr_count=%"PRIu32"})",
            (void*)lfs, (void*)file, path, flags,
            (void*)cfg, cfg->buffer, (void*)cfg->attrs, cfg->attr_count);
    LFS_ASSERT(!lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    err = lfs_file_rawopencfg(lfs, file, path, flags, cfg);

    LFS_TRACE("lfs_file_opencfg -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

int lfs_file_close(lfs_t *lfs, lfs_file_t *file) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_close(%p, %p)", (void*)lfs, (void*)file);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    err = lfs_file_rawclose(lfs, file);

    LFS_TRACE("lfs_file_close -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

#ifndef LFS_READONLY
int lfs_file_sync(lfs_t *lfs, lfs_file_t *file) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_sync(%p, %p)", (void*)lfs, (void*)file);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    err = lfs_file_rawsync(lfs, file);

    LFS_TRACE("lfs_file_sync -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

lfs_ssize_t lfs_file_read(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_read(%p, %p, %p, %"PRIu32")",
            (void*)lfs, (void*)file, buffer, size);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    lfs_ssize_t res = lfs_file_rawread(lfs, file, buffer, size);

    LFS_TRACE("lfs_file_read -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

#ifndef LFS_READONLY
lfs_ssize_t lfs_file_write(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_write(%p, %p, %p, %"PRIu32")",
            (void*)lfs, (void*)file, buffer, size);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    lfs_ssize_t res = lfs_file_rawwrite(lfs, file, buffer, size);

    LFS_TRACE("lfs_file_write -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}
#endif

lfs_soff_t lfs_file_seek(lfs_t *lfs, lfs_file_t *file,
        lfs_soff_t off, int whence) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_seek(%p, %p, %"PRId32", %d)",
            (void*)lfs, (void*)file, off, whence);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    lfs_soff_t res = lfs_file_rawseek(lfs, file, off, whence);

    LFS_TRACE("lfs_file_seek -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

#ifndef LFS_READONLY
int lfs_file_truncate(lfs_t *lfs, lfs_file_t *file, lfs_off_t size) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_truncate(%p, %p, %"PRIu32")",
            (void*)lfs, (void*)file, size);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    err = lfs_file_rawtruncate(lfs, file, size);

    LFS_TRACE("lfs_file_truncate -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

lfs_soff_t lfs_file_tell(lfs_t *lfs, lfs_file_t *file) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_tell(%p, %p)", (void*)lfs, (void*)file);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    lfs_soff_t res = lfs_file_rawtell(lfs, file);

    LFS_TRACE("lfs_file_tell -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

int lfs_file_rewind(lfs_t *lfs, lfs_file_t *file) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_rewind(%p, %p)", (void*)lfs, (void*)file);

    err = lfs_file_rawrewind(lfs, file);

    LFS_TRACE("lfs_file_rewind -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

lfs_soff_t lfs_file_size(lfs_t *lfs, lfs_file_t *file) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_file_size(%p, %p)", (void*)lfs, (void*)file);
    LFS_ASSERT(lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)file));

    lfs_soff_t res = lfs_file_rawsize(lfs, file);

    LFS_TRACE("lfs_file_size -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

#ifndef LFS_READONLY
int lfs_mkdir(lfs_t *lfs, const char *path) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_mkdir(%p, \"%s\")", (void*)lfs, path);

    err = lfs_rawmkdir(lfs, path);

    LFS_TRACE("lfs_mkdir -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}
#endif

int lfs_dir_open(lfs_t *lfs, lfs_dir_t *dir, const char *path) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_dir_open(%p, %p, \"%s\")", (void*)lfs, (void*)dir, path);
    LFS_ASSERT(!lfs_mlist_isopen(lfs->mlist, (struct lfs_mlist*)dir));

    err = lfs_dir_rawopen(lfs, dir, path);

    LFS_TRACE("lfs_dir_open -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

int lfs_dir_close(lfs_t *lfs, lfs_dir_t *dir) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_dir_close(%p, %p)", (void*)lfs, (void*)dir);

    err = lfs_dir_rawclose(lfs, dir);

    LFS_TRACE("lfs_dir_close -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

int lfs_dir_read(lfs_t *lfs, lfs_dir_t *dir, struct lfs_info *info) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_dir_read(%p, %p, %p)",
            (void*)lfs, (void*)dir, (void*)info);

    err = lfs_dir_rawread(lfs, dir, info);

    LFS_TRACE("lfs_dir_read -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

int lfs_dir_seek(lfs_t *lfs, lfs_dir_t *dir, lfs_off_t off) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_dir_seek(%p, %p, %"PRIu32")",
            (void*)lfs, (void*)dir, off);

    err = lfs_dir_rawseek(lfs, dir, off);

    LFS_TRACE("lfs_dir_seek -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

lfs_soff_t lfs_dir_tell(lfs_t *lfs, lfs_dir_t *dir) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_dir_tell(%p, %p)", (void*)lfs, (void*)dir);

    lfs_soff_t res = lfs_dir_rawtell(lfs, dir);

    LFS_TRACE("lfs_dir_tell -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

int lfs_dir_rewind(lfs_t *lfs, lfs_dir_t *dir) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_dir_rewind(%p, %p)", (void*)lfs, (void*)dir);

    err = lfs_dir_rawrewind(lfs, dir);

    LFS_TRACE("lfs_dir_rewind -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

lfs_ssize_t lfs_fs_size(lfs_t *lfs) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_fs_size(%p)", (void*)lfs);

    lfs_ssize_t res = lfs_fs_rawsize(lfs);

    LFS_TRACE("lfs_fs_size -> %"PRId32, res);
    LFS_UNLOCK(lfs->cfg);
    return res;
}

int lfs_fs_traverse(lfs_t *lfs, int (*cb)(void *, lfs_block_t), void *data) {
    int err = LFS_LOCK(lfs->cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_fs_traverse(%p, %p, %p)",
            (void*)lfs, (void*)(uintptr_t)cb, data);

    err = lfs_fs_rawtraverse(lfs, cb, data, true);

    LFS_TRACE("lfs_fs_traverse -> %d", err);
    LFS_UNLOCK(lfs->cfg);
    return err;
}

#ifdef LFS_MIGRATE
int lfs_migrate(lfs_t *lfs, const struct lfs_config *cfg) {
    int err = LFS_LOCK(cfg);
    if (err) {
        return err;
    }
    LFS_TRACE("lfs_migrate(%p, %p {.context=%p, "
                ".read=%p, .prog=%p, .erase=%p, .sync=%p, "
                ".read_size=%"PRIu32", .prog_size=%"PRIu32", "
                ".block_size=%"PRIu32", .block_count=%"PRIu32", "
                ".block_cycles=%"PRIu32", .cache_size=%"PRIu32", "
                ".lookahead_size=%"PRIu32", .read_buffer=%p, "
                ".prog_buffer=%p, .lookahead_buffer=%p, "
                ".name_max=%"PRIu32", .file_max=%"PRIu32", "
                ".attr_max=%"PRIu32"})",
            (void*)lfs, (void*)cfg, cfg->context,
            (void*)(uintptr_t)cfg->read, (void*)(uintptr_t)cfg->prog,
            (void*)(uintptr_t)cfg->erase, (void*)(uintptr_t)cfg->sync,
            cfg->read_size, cfg->prog_size, cfg->block_size, cfg->block_count,
            cfg->block_cycles, cfg->cache_size, cfg->lookahead_size,
            cfg->read_buffer, cfg->prog_buffer, cfg->lookahead_buffer,
            cfg->name_max, cfg->file_max, cfg->attr_max);

    err = lfs_rawmigrate(lfs, cfg);

    LFS_TRACE("lfs_migrate -> %d", err);
    LFS_UNLOCK(cfg);
    return err;
}
#endif

