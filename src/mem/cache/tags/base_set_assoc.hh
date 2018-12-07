/*
 * Copyright (c) 2012-2014,2017 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
#define __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__

#include <functional>
#include <map>
#include <string>
#include <unordered_set>

#include "base/logging.hh"
#include "base/random.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "params/BaseSetAssoc.hh"

extern int access_at;
extern int hit_count;
extern int miss_count;

using namespace std;
/**
 * A basic cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc placement policy divides the cache into s sets of w
 * cache lines (ways).
 */
class BaseSetAssoc : public BaseTags
{
  protected:
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** The cache blocks. */
    std::vector<CacheBlk> blks;

    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** Replacement policy */
    BaseReplacementPolicy *replacementPolicy;

    std::map<Addr, unordered_set<Addr>> Atcache;
  public:
    /** Convenience typedef. */
     typedef BaseSetAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    BaseSetAssoc(const Params *p);

    /**
     * Destructor
     */
    virtual ~BaseSetAssoc() {};

    /**
     * Initialize blocks as CacheBlk instances.
     */
    void tagsInit() override;

    /**
     * This function updates the tags when a block is invalidated. It also
     * updates the replacement data.
     *
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache access and
     * should only be used as such. Returns the tag lookup latency as a side
     * effect.
     *
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param lat The latency of the tag lookup.
     * @return Pointer to the cache block if found.
     */
    bool hitTaginAtcache(Addr address)
    {
       Addr set_addr = indexingPolicy->extractSet(address);
       auto it = Atcache.find(set_addr);
       if (it == Atcache.end())
           return false;
       Addr tag = extractTag(address);
       return (it->second).find(tag) != (it->second).end();

    }

    void insertTaginAtcache(Addr address)
    {
       Addr set_addr = indexingPolicy->extractSet(address);
       auto it = Atcache.find(set_addr);
       if (it == Atcache.end()) {
            unordered_set<Addr> item;
            item.insert(extractTag(address));
            Atcache[set_addr] = item;
            return;
       }

       if ((it->second).size() >= 16) {
            /***
               Randomly pickup an item in the certain set.
               Follow the code from this approach:
               https://goo.gl/GTurg7
            ***/
            int bucket, bucket_size;
            do {
                bucket = random_mt.random(0,
                       (int)(it->second).bucket_count() - 1);
            } while ( (bucket_size = (it->second).bucket_size(bucket)) == 0 );
            auto block = std::next((it->second).begin(bucket),
                               random_mt.random(0, bucket_size - 1));
            (it->second).erase(*block);
        }
        (it->second).insert(extractTag(address));
    }

    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) override
    {
        bool TagInAtcache = false;
        if (numEntriesAtcache != 0) {
            TagInAtcache = hitTaginAtcache(addr);
            if (!TagInAtcache) {
                insertTaginAtcache(addr);
            }
        }
        CacheBlk *blk = findBlock(addr, is_secure);

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                dataAccesses += 1;
            }
        } else {
            dataAccesses += allocAssoc;
        }

        // If a cache hit
        if (blk != nullptr) {
            // Update number of references to accessed block
            blk->refCount++;

            // Update replacement data of accessed block
            replacementPolicy->touch(blk->replacementData);
        }

        // The tag lookup latency is the same for a hit or a miss
        if (numEntriesAtcache != 0 ) {
            access_at++;
            lat = AtcacheLookupLatency;
            if (!TagInAtcache) {
                miss_count++;
                lat += lookupLatency;
            } else {
                hit_count++;
            }
        } else {
            lat = lookupLatency;
        }
        return blk;
    }

    /**
     * Find replacement victim based on address. The list of evicted blocks
     * only contains the victim.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const override
    {
        // Get possible entries to be victimized
        const std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(addr);

        // Choose replacement victim from replacement candidates
        CacheBlk* victim = static_cast<CacheBlk*>(replacementPolicy->getVictim(
                                entries));

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        return victim;
    }

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param addr Address of the block.
     * @param is_secure Whether the block is in secure space or not.
     * @param src_master_ID The source requestor ID.
     * @param task_ID The new task ID.
     * @param blk The block to update.
     */
    void insertBlock(const Addr addr, const bool is_secure,
                     const int src_master_ID, const uint32_t task_ID,
                     CacheBlk *blk) override
    {
        // Insert block
        BaseTags::insertBlock(addr, is_secure, src_master_ID, task_ID, blk);

        // Increment tag counter
        tagsInUse++;

        // Update replacement policy
        replacementPolicy->reset(blk->replacementData);
    }

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Regenerate the block address from the tag and indexing location.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return indexingPolicy->regenerateAddr(blk->tag, blk);
    }

    void forEachBlk(std::function<void(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            visitor(blk);
        }
    }

    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            if (visitor(blk)) {
                return true;
            }
        }
        return false;
    }
};

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
