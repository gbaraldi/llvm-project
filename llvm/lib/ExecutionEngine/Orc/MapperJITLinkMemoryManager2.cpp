//=== MapperJITLinkMemoryManager2.cpp - Memory management with MemoryMapper ===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/ExecutionEngine/Orc/MapperJITLinkMemoryManager2.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/ExecutionEngine/JITLink/JITLink.h"
#include "llvm/ExecutionEngine/Orc/Shared/ExecutorAddress.h"
#include "llvm/ExecutionEngine/Orc/Shared/MemoryFlags.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/Process.h"
#include <cassert>
#include <cstddef>
#include <optional>

using namespace llvm::jitlink;

namespace llvm {
namespace orc {

#define MIN_INTERVAL_SIZE 128
class MapperJITLinkMemoryManager2::InFlightAlloc
    : public JITLinkMemoryManager::InFlightAlloc {
public:
  InFlightAlloc(MapperJITLinkMemoryManager2 &Parent, LinkGraph &G,
                ExecutorAddr AllocAddr,
                std::vector<MemoryMapper::AllocInfo::SegInfo> Segs)
      : Parent(Parent), G(G), AllocAddr(AllocAddr), Segs(std::move(Segs)) {}

  void finalize(OnFinalizedFunction OnFinalize) override {
    MemoryMapper::AllocInfo AI;
    AI.MappingBase = AllocAddr;

    std::swap(AI.Segments, Segs);
    std::swap(AI.Actions, G.allocActions());

    Parent.Mapper->initialize(AI, [OnFinalize = std::move(OnFinalize)](
                                      Expected<ExecutorAddr> Result) mutable {
      if (!Result) {
        OnFinalize(Result.takeError());
        return;
      }

      OnFinalize(FinalizedAlloc(*Result));
    });
  }

  void abandon(OnAbandonedFunction OnFinalize) override {
    Parent.Mapper->release({AllocAddr}, std::move(OnFinalize));
  }

private:
  MapperJITLinkMemoryManager2 &Parent;
  LinkGraph &G;
  ExecutorAddr AllocAddr;
  std::vector<MemoryMapper::AllocInfo::SegInfo> Segs;
};
  Slab::SlabSection::SlabSection(ExecutorAddrRange Span, MemProt Prot) : AvailableMemory(AMAllocator), Prot(Prot) {
    AvailableMemory.insert(Span.Start, Span.End - 1, true);
    assert(Span.Start <= Span.End && "Invalid span");
}

std::optional<ExecutorAddrRange> Slab::SlabSection::allocate(uint64_t Size, uint64_t Align) {

    ExecutorAddrRange SelectedRange{};
    AvailableMemoryMap::iterator BestFit;
    uint64_t LargestInterval = 0;
    // Best fit search.
    // Look through all the intervals and find the smallest one large enough to fit the allocation.
    // Also take note of the largest interval size for fast canFit checks.
    // One could also try a first fits approach which is faster but wastes more memory and might fragment the memory more.
    for (AvailableMemoryMap::iterator It = AvailableMemory.begin();
                                      It != AvailableMemory.end(); It++) {
        uint64_t AlignedStart = alignTo(It.start().getValue(), Align);
        if ((AlignedStart + Size) <= (It.stop().getValue() + 1)) {
            uint64_t AllocSize = It.stop().getValue() - AlignedStart + 1;
            if (SelectedRange.empty() || AllocSize < SelectedRange.size()) {
                SelectedRange = ExecutorAddrRange(It.start(), It.stop() + 1);
                auto OldBestFit = BestFit;
                BestFit = It;
                if (LargestInterval < OldBestFit.stop() - OldBestFit.start()) {
                    LargestInterval = OldBestFit.start() - OldBestFit.stop();
                }
            }
        } else {
            if (LargestInterval < It.stop() - It.start()) {
                LargestInterval = It.stop() - It.start();
            }
        }
    }
    this->LargestInterval = LargestInterval;
    if (SelectedRange.empty()) {
        // This really shouldn't happen, the caller should do a canFit check first.
        return std::nullopt;
    }

    // Check if we want to save the remaining memory for reuse in next allocation(s)
    uint64_t AlignedEnd = alignTo(BestFit.start().getValue(), Align) + Size;
    if ((BestFit.stop().getValue() - 1) - (AlignedEnd) > MIN_INTERVAL_SIZE) { // Don't want to make tiny intervals
        SelectedRange = ExecutorAddrRange(BestFit.start(), ExecutorAddr(AlignedEnd));
        auto IntervalEnd = BestFit.stop();
        BestFit.erase();
        AvailableMemory.insert(ExecutorAddr(AlignedEnd), IntervalEnd - 1, true);
    } else
        BestFit.erase();

    return SelectedRange;
}

MapperJITLinkMemoryManager2::MapperJITLinkMemoryManager2(
    size_t ReservationGranularity, std::unique_ptr<MemoryMapper> Mapper)
    : ReservationUnits(ReservationGranularity), AvailableMemory(AMAllocator),
      Mapper(std::move(Mapper)) {}

void MapperJITLinkMemoryManager2::allocate(const JITLinkDylib *JD, LinkGraph &G,
                                          OnAllocatedFunction OnAllocated) {
  BasicLayout BL(G);

  // find required address space
  auto SegsSizes = BL.getSplitPageBasedLayoutSizes(Mapper->getPageSize());


  if (!SegsSizes) {
    OnAllocated(SegsSizes.takeError());
    return;
  }

  auto TotalSize = SegsSizes->total();

  auto CompleteAllocation = [this, &G, BL = std::move(BL),
                             OnAllocated = std::move(OnAllocated)](
                                Expected<ExecutorAddrRange> Result) mutable {
    if (!Result) {
      Mutex.unlock();
      return OnAllocated(Result.takeError());
    }

    auto NextSegAddr = Result->Start;

    std::vector<MemoryMapper::AllocInfo::SegInfo> SegInfos;

    for (auto &KV : BL.segments()) {
      auto &AG = KV.first;
      auto &Seg = KV.second;

      auto TotalSize = Seg.ContentSize + Seg.ZeroFillSize;

      Seg.Addr = NextSegAddr;
      Seg.WorkingMem = Mapper->prepare(NextSegAddr, TotalSize);

      NextSegAddr += alignTo(TotalSize, Mapper->getPageSize());

      MemoryMapper::AllocInfo::SegInfo SI;
      SI.Offset = Seg.Addr - Result->Start;
      SI.ContentSize = Seg.ContentSize;
      SI.ZeroFillSize = Seg.ZeroFillSize;
      SI.AG = AG;
      SI.WorkingMem = Seg.WorkingMem;

      SegInfos.push_back(SI);
    }

    UsedMemory.insert({Result->Start, NextSegAddr - Result->Start});

    if (NextSegAddr < Result->End) {
      // Save the remaining memory for reuse in next allocation(s)
      AvailableMemory.insert(NextSegAddr, Result->End - 1, true);
    }
    Mutex.unlock();

    if (auto Err = BL.apply()) {
      OnAllocated(std::move(Err));
      return;
    }

    OnAllocated(std::make_unique<InFlightAlloc>(*this, G, Result->Start,
                                                std::move(SegInfos)));
  };

  Mutex.lock();

  // find an already reserved range that is large enough
  ExecutorAddrRange SelectedRange{};

  for (AvailableMemoryMap::iterator It = AvailableMemory.begin();
       It != AvailableMemory.end(); It++) {
    if (It.stop() - It.start() + 1 >= TotalSize) {
      SelectedRange = ExecutorAddrRange(It.start(), It.stop() + 1);
      It.erase();
      break;
    }
  }

  if (SelectedRange.empty()) { // no already reserved range was found
    auto TotalAllocation = alignTo(TotalSize, ReservationUnits);
    Mapper->reserve(TotalAllocation, std::move(CompleteAllocation));
  } else {
    CompleteAllocation(SelectedRange);
  }
}

void MapperJITLinkMemoryManager2::deallocate(
    std::vector<FinalizedAlloc> Allocs, OnDeallocatedFunction OnDeallocated) {
  std::vector<ExecutorAddr> Bases;
  Bases.reserve(Allocs.size());
  for (auto &FA : Allocs) {
    ExecutorAddr Addr = FA.getAddress();
    Bases.push_back(Addr);
  }

  Mapper->deinitialize(Bases, [this, Allocs = std::move(Allocs),
                               OnDeallocated = std::move(OnDeallocated)](
                                  llvm::Error Err) mutable {
    // TODO: How should we treat memory that we fail to deinitialize?
    // We're currently bailing out and treating it as "burned" -- should we
    // require that a failure to deinitialize still reset the memory so that
    // we can reclaim it?
    if (Err) {
      for (auto &FA : Allocs)
        FA.release();
      OnDeallocated(std::move(Err));
      return;
    }

    {
      std::lock_guard<std::mutex> Lock(Mutex);

      for (auto &FA : Allocs) {
        ExecutorAddr Addr = FA.getAddress();
        ExecutorAddrDiff Size = UsedMemory[Addr];

        UsedMemory.erase(Addr);
        AvailableMemory.insert(Addr, Addr + Size - 1, true);

        FA.release();
      }
    }

    OnDeallocated(Error::success());
  });
}

} // end namespace orc
} // end namespace llvm
