/**
 * @file cache.cpp
 * @brief File containing the implementation of the cache class for the virtual machine
 * @author Vishank Singh, https://github.com/VishankSingh
 */
#include "vm/cache/cache.h"
#include <bits/stdc++.h>
#include <random>
#include <memory>

#include "utils.h"
#include "globals.h"
#include "common/instructions.h"
#include "config.h"
namespace cache {

    uint64_t random_index(uint64_t num)
    {
        std:: mt19937 gen{std::random_device{}()};
        std:: uniform_int_distribution<uint64_t> dist(0, num-1);
        return dist(gen);
    }


    class CacheSetstorer{
        public:
        struct LRUandFIFO{
            uint64_t forlru=0;
            uint64_t forfifo=0;
        };
        std:: vector<CacheSet> sets;
        std:: vector<std:: vector<LRUandFIFO>> lru_fifo;  

        uint64_t fifo_counter=0;
        uint64_t lru_counter=0;
        CacheSetstorer(CacheConfig& cc)
        {
            uint64_t numset = cc.lines/cc.associativity;
            sets.resize(numset);
            for(uint64_t i=0;i<numset;i++)
            {
                sets[i]=CacheSet(cc.associativity);
            }
            lru_fifo.resize(numset);
            for(uint64_t i=0;i<numset;i++)
            {
                lru_fifo[i].resize(cc.associativity);
            }
        }
    };


    Cache:: Cache()
    {
        enabled= true;
        type =CacheType::Data;
        config = CacheConfig();
        stats.accesses=stats.hits=stats.misses=0;
        impl_=nullptr;
    }
    Cache:: Cache(const CacheConfig& cc)
    {
        enabled= true;
        type =(cc.cache_type);
        config = cc;
        stats.accesses=stats.hits=stats.misses=0;
        impl_=std::make_unique<CacheSetstorer>(cc);
    }
    void Cache::SetMemory(MemoryController* mem) {
    memory_ = mem;
}

    Cache::~Cache() = default;
    inline uint64_t Cache::BlockSizeBytes() const {
          return config.words_per_line * sizeof(uint64_t);
        }
    inline uint64_t Cache::NumSets() const {
    return config.lines / config.associativity; 
        }
    uint64_t Cache::GetIndex(uint64_t addr) const{
    // index is (block_number) % num_sets
    uint64_t block_size = BlockSizeBytes();
    uint64_t block_number = addr / block_size;
    return block_number % NumSets();
}

    uint64_t Cache::GetTag(uint64_t addr) const{
    // tag = block_number (full block number). Using full block number keeps tag comparisons simple.
    uint64_t block_size = BlockSizeBytes();
    return addr / block_size;
}


    uint64_t Cache::FindVictim(uint64_t idx)
    {
        CacheSetstorer &k=*impl_;
        auto &ss= k.lru_fifo[idx];
        auto &set =k.sets[idx];

        for(uint64_t i=0;i<config.associativity;i++)
        {
            if(set.lines[i].state==CacheLineState::Invalid)
            return i;
        }
        switch(config.replacement_policy)
        {
            case ReplacementPolicy::Random:
            return random_index(config.associativity);

            case ReplacementPolicy::FIFO:
            uint64_t best=0;
            uint64_t bestorder=ss[0].forfifo;
            for(uint64_t i=1;i<config.associativity;i++)
            {
                if(ss[i].forfifo< bestorder)
                {
                    bestorder=ss[i].forfifo;
                    best=i;
                }
            }
            return best;

            case ReplacementPolicy:: LRU:
            default:
            uint64_t best=0;
            uint64_t purana=ss[0].forlru;
            for(uint64_t i=1;i<config.associativity;i++)
            {
                if(ss[i].forlru<purana)
                {
                    purana=ss[i].forlru;
                    best=i;
                }
            }
            return best;

        }
    }
    bool Cache::Read(uint64_t addr, size_t read_size, std::vector<uint8_t>& data)
    {
    if(!enabled || !impl_) return false;

    uint64_t index = GetIndex(addr);
    uint64_t tag   = GetTag(addr);

    auto& I = *impl_;
    auto& set = I.sets[index];
    auto& lrufifodata = I.lru_fifo[index];

    I.lru_counter++;

    uint64_t block_size   = BlockSizeBytes();
    uint64_t block_start  = (addr / block_size) * block_size;
    uint64_t offset_in_block = addr - block_start;

    if(offset_in_block + read_size > block_size)
        read_size = block_size - offset_in_block;

    data.clear();
    data.resize(read_size);

    // CHECK FOR HIT
    for(uint64_t i = 0; i < set.associativity; i++)
    {
        CacheLine& line = set.lines[i];

        if(line.state != CacheLineState::Invalid && line.tag == tag)
        {
            // HIT
            std::copy_n(
                line.data.begin() + offset_in_block,
                read_size,
                data.begin()
            );

            lrufifodata[i].forlru = I.lru_counter;
            return true;
        }
    }

    // MISS
    long victim = -1;
    for(uint64_t i = 0; i < set.associativity; i++)
    {
        if(set.lines[i].state == CacheLineState::Invalid)
        {
            victim = i;
            break;
        }
    }
    if(victim == -1) victim = FindVictim(index);

    CacheLine& line = set.lines[victim];

    // EVICT IF DIRTY
    if(line.state == CacheLineState::Dirty)
    {
        uint64_t old_block_start = line.tag * block_size;
        for(size_t j = 0; j < line.data.size(); j++)
            memory_->WriteByte(old_block_start + j, line.data[j]);
    }

    // LOAD NEW BLOCK
    std::vector<uint8_t> new_block(block_size);
    for(uint64_t j = 0; j < block_size; j++)
        new_block[j] = memory_->ReadByte(block_start + j);

    line.data = std::move(new_block);
    line.tag = tag;
    line.state = CacheLineState::Valid;

    lrufifodata[victim].forfifo = I.fifo_counter++;
    lrufifodata[victim].forlru  = I.lru_counter;

    // return requested bytes
    std::copy_n(
        line.data.begin() + offset_in_block,
        read_size,
        data.begin()
    );

    return false;
}


    bool Cache::Write(uint64_t addr, const std::vector<uint8_t>& data)
    {
    if(!enabled || !impl_) return false;
    // stats.accesses++;

    uint64_t index = GetIndex(addr);
    uint64_t tag = GetTag(addr);

    auto& I = *impl_;
    auto& set = I.sets[index];
    auto& lrufifodata = I.lru_fifo[index];

    I.lru_counter++;

    size_t write_len = data.size();
    if(write_len == 0) return true; 

    uint64_t block_size = BlockSizeBytes();
    uint64_t block_start = (addr / block_size) * block_size; // absolute byte address of line start
    uint64_t offset_in_block = addr - block_start; // starting offset

    // --- Try to find a hit first ---
    for (uint64_t i = 0; i < set.associativity; ++i)
    {
        CacheLine& line = set.lines[i];
        if(line.state != CacheLineState::Invalid && line.tag == tag)
        {
            // HIT: update only the bytes at offset_in_block
            if (offset_in_block + write_len > line.data.size()) {
                write_len = line.data.size() - offset_in_block;
            }

            // copy bytes into cache line
            std::copy_n(data.begin(), write_len, line.data.begin() + offset_in_block);

            // write policy handling
            if(config.write_hit_policy == WriteHitPolicy::WriteBack) {
                line.state = CacheLineState::Dirty;
            } else if(config.write_hit_policy == WriteHitPolicy::WriteThrough) {
                for(size_t j = 0; j < write_len; ++j) {
                    memory_->WriteByte(addr + j, data[j]);
                }
            }

            lrufifodata[i].forlru = I.lru_counter;
            return true;
        }
    }

    // --- MISS ---
    // stats.misses++;

    if(config.write_miss_policy == WriteMissPolicy::NoWriteAllocate) {
       
        for(size_t j = 0; j < write_len; ++j) {
            memory_->WriteByte(addr + j, data[j]);
        }
        return true; 
    }

    
    long victim = -1;
    for(uint64_t i = 0; i < set.associativity; ++i) {
        if(set.lines[i].state == CacheLineState::Invalid) { victim = i; break; }
    }
    if (victim == -1) victim = FindVictim(index);

    CacheLine& line = set.lines[victim];
    
    if(line.state == CacheLineState::Dirty)
    {
        uint64_t old_block_start = line.tag * block_size;
        for(size_t i = 0; i < line.data.size(); i++) {
            memory_->WriteByte(old_block_start + i, line.data[i]);
        }
    }

    // fetch the whole block from memory
    std::vector<uint8_t> new_block(block_size);
    for(uint64_t i = 0; i < block_size; ++i) {
        new_block[i] = memory_->ReadByte(block_start + i);
    }
    line.data = std::move(new_block);
    line.tag = tag;
    line.state = CacheLineState::Valid;

    // now perform the partial write into the freshly loaded block
    if (offset_in_block + write_len > line.data.size()) {
        write_len = line.data.size() - offset_in_block; // clip
    }
    std::copy_n(data.begin(), write_len, line.data.begin() + offset_in_block);


    if(config.write_hit_policy == WriteHitPolicy::WriteBack) {
        line.state = CacheLineState::Dirty;
    } else if(config.write_hit_policy == WriteHitPolicy::WriteThrough) {
        // update memory for the written bytes
        for(size_t j = 0; j < write_len; ++j) {
            memory_->WriteByte(addr + j, data[j]);
        }
    }

    // update replacement metadata
    lrufifodata[victim].forfifo = I.fifo_counter++;
    lrufifodata[victim].forlru = I.lru_counter;

    return true;
}


        void Cache::SetEnabled(bool e) { enabled = e; }
        bool Cache::IsEnabled() const { return enabled; }
        void Cache::ResetStats() { stats = {0,0,0}; }
        CacheStats Cache::GetStats() const { return stats; }
}