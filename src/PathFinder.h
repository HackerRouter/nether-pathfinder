#pragma once

#include <vector>
#include <optional>
#include <unordered_set>

#include <jni.h>

#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "PathNode.h"
#include "ChunkGen.h"
#include "Allocator.h"

enum class FakeChunkMode {
    GENERATE = 0
    ,AIR = 1
    ,SOLID = 2
};

struct Path {
    enum class Type {
        SEGMENT,
        FINISHED
    };

    Type type;
    BlockPos start;
    BlockPos goal; // where the path wants to go, not necessarily where it ends
    std::vector<BlockPos> blocks;
    std::vector<std::unique_ptr<PathNode>> nodes;
    cache_t chunkCache;

    [[nodiscard]] const BlockPos& getEndPos() const {
        // This should basically never be empty
        return !blocks.empty() ? blocks.back() : this->start;
    }
};

struct Context {
    ChunkGeneratorHell generator;
    std::optional<std::string> baritoneCache;
    std::mutex cacheMutex;
    std::unique_ptr<Allocator<Chunk>> chunkAllocator;
    cache_t chunkCache;
    ParallelExecutor<4> topExecutor;
    std::array<ChunkGenExec, 4> executors;
    std::atomic_flag cancelFlag;
    std::unordered_set<RegionPos> checkedRegions;
    // All native pathfinding coordinates are normalized to [0, height).
    // minY is only used at the JNI boundary to translate Minecraft world coordinates.
    int minY;
    int height;
    Dimension dimension;


    explicit Context(int64_t seed, std::optional<std::string>&& cacheDir, Dimension dim, int minY, int height, bool pageAllocator):
        generator(ChunkGeneratorHell::fromSeed(seed)), baritoneCache(cacheDir), minY(minY), height(height), dimension(dim)
        {
            if (height <= 0 || height > 384) {
                throw std::range_error("bad world height");
            }
            if (pageAllocator && getPageSize() == 4096) {
                chunkAllocator = std::make_unique<PageAllocator<Chunk>>();
            } else {
                chunkAllocator = std::make_unique<Allocator<Chunk>>();
            }
        }
    explicit Context(int64_t seed, Dimension dim, int minY, int height, bool pageAllocator): Context(seed, std::nullopt, dim, minY, height, pageAllocator) {}
    explicit Context(int64_t seed, std::string&& cacheDir, Dimension dim, int minY, int height, bool pageAllocator): Context(seed, std::optional{cacheDir}, dim, minY, height, pageAllocator) {}

    // Source compatibility for native callers that used the pre-minY API.
    explicit Context(int64_t seed, Dimension dim, int height, bool pageAllocator): Context(seed, dim, 0, height, pageAllocator) {}
    explicit Context(int64_t seed, std::string&& cacheDir, Dimension dim, int height, bool pageAllocator): Context(seed, std::move(cacheDir), dim, 0, height, pageAllocator) {}

    [[nodiscard]] bool containsWorldY(double worldY) const {
        return worldY >= minY && worldY < static_cast<double>(minY) + height;
    }

    [[nodiscard]] int toInternalY(int worldY) const {
        return worldY - minY;
    }

    [[nodiscard]] double toInternalY(double worldY) const {
        return worldY - minY;
    }

    [[nodiscard]] int toWorldY(int internalY) const {
        return internalY + minY;
    }
    ~Context() {
        // useless optimization
        if (!chunkAllocator->auto_frees_on_destroy()) {
            for (auto &p: chunkCache) {
                if (p.second.second) {
                    chunkAllocator->free(p.second.second);
                }
            }
        }
    }
};

// long name but I do not care
// simply calls getRealChunkOrDefault or getOrGenChunk depending on mode
const Chunk& getRealChunkFromCacheOrFakeChunkMaybeGen(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos, FakeChunkMode mode);
// gets from cache, or generates and inserts into cache
const Chunk& getOrGenChunk(Context& ctx, ChunkGenExec& executor, const ChunkPos& pos);
const Chunk& getRealChunkOrDefault(Context& ctx, const ChunkPos& pos, bool solid);

std::optional<Path> findPathFull(Context& ctx, const NodePos& start, const NodePos& goal, double fakeChunkCost);
std::optional<Path> findPathSegment(Context& ctx, const NodePos& start, const NodePos& goal, bool x4Min, int failTimeoutMs, bool airIfFake, double fakeChunkCost);

template<Size size>
std::optional<NodePos> findAir(Context& ctx, const BlockPos& start1x, FakeChunkMode fakeChunkMode = FakeChunkMode::GENERATE);
