//===--------------- MapperJITLinkMemoryManager2.h -*- C++ -*---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements JITLinkMemoryManager using MemoryMapper
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_EXECUTIONENGINE_ORC_MAPPERJITLINKMEMORYMANAGER2_H
#define LLVM_EXECUTIONENGINE_ORC_MAPPERJITLINKMEMORYMANAGER2_H

#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/IntervalMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ExecutionEngine/JITLink/JITLinkMemoryManager.h"
#include "llvm/ExecutionEngine/Orc/MemoryMapper.h"
#include "llvm/Support/MathExtras.h"
#include <cstddef>
#include <cstdint>
#include <limits>

namespace llvm {
namespace orc {
  const size_t MaxSlabSections = 8; // Maybe this could be tunable but for now we will just use 8

  struct Slab {
    struct SlabSection {
    public:
      SlabSection(Slab& Parent,ExecutorAddrRange Span, MemProt Prot);
      ExecutorAddrRange span() const { return Span; }
      ExecutorAddr lastIdx() const { return LastIdx; }
      bool availableBytes() const { return Span.End - LastIdx; }
      bool canFit(size_t Size, size_t Align) const;
      MemProt prot() const { return Prot; }
      std::optional<ExecutorAddrRange> allocate(size_t Size, size_t Align);
      bool deallocate(ExecutorAddr Addr);
    private:
      using AvailableMemoryMap = IntervalMap<ExecutorAddr, bool>;
      AvailableMemoryMap::Allocator AMAllocator;
      IntervalMap<ExecutorAddr, bool> AvailableMemory;
      // Ranges that have been reserved in executor and already allocated
      DenseMap<ExecutorAddr, ExecutorAddrDiff> UsedMemory;
      MemProt Prot;
      Slab &Parent;
    };

    std::optional<ExecutorAddrRange> allocate() {
      // for (auto &SlabSection : Sections) {
      //   if (SlabSection.prot() == Prot) {
      //     if (auto Range = SlabSection.allocate(Size, Align))
      //       return Range;
      //   }
      // }
      return std::nullopt;
    }
    private:
      size_t page_size;

      ExecutorAddrRange Span;
        // At any one point all these sections must be part of one large slab of memory.
      SlabSection ROSections;
      SlabSection RWSections;
      SlabSection ExecutableSections;
      // // Finalize sections are special because they are deallocated after finalization so we separate a small scratch space for them.
      // // This likely needs to be a freelist to support multiple allocations in flight to be able to reuse the space. For now we will just waste memory.
      // SlabSection RODataFinalize;
      // SlabSection RWDataFinalize;
      // SlabSection ExecutableFinalize;
};

class MapperJITLinkMemoryManager2 : public jitlink::JITLinkMemoryManager {
public:

  MapperJITLinkMemoryManager2(size_t ReservationGranularity,
                             std::unique_ptr<MemoryMapper> Mapper);

  template <class MemoryMapperType, class... Args>
  static Expected<std::unique_ptr<MapperJITLinkMemoryManager2>>
  CreateWithMapper(size_t ReservationGranularity, Args &&...A) {
    auto Mapper = MemoryMapperType::Create(std::forward<Args>(A)...);
    if (!Mapper)
      return Mapper.takeError();

    return std::make_unique<MapperJITLinkMemoryManager2>(ReservationGranularity,
                                                        std::move(*Mapper));
  }

  void allocate(const jitlink::JITLinkDylib *JD, jitlink::LinkGraph &G,
                OnAllocatedFunction OnAllocated) override;
  // synchronous overload
  using JITLinkMemoryManager::allocate;

  void deallocate(std::vector<FinalizedAlloc> Allocs,
                  OnDeallocatedFunction OnDeallocated) override;
  // synchronous overload
  using JITLinkMemoryManager::deallocate;

  private:
    class InFlightAlloc;
  std::mutex Mutex;

  // We reserve multiples of this from the executor address space
  size_t ReservationUnits;

  // Ranges that have been reserved in executor but not yet allocated
  using AvailableMemoryMap = IntervalMap<ExecutorAddr, bool>;
  AvailableMemoryMap::Allocator AMAllocator;
  IntervalMap<ExecutorAddr, bool> AvailableMemory;
  // Ranges that have been reserved in executor and already allocated
  DenseMap<ExecutorAddr, ExecutorAddrDiff> UsedMemory;
  std::unique_ptr<MemoryMapper> Mapper;
};

} // end namespace orc
} // end namespace llvm

#endif // LLVM_EXECUTIONENGINE_ORC_MAPPERJITLINKMEMORYMANAGER2_H
