/*
 * Copyright (c) 2001-2005   The Regents of The University of Michigan
 * Copyright (c) 2007        MIPS Technologies, Inc.
 * Copyright (c) 2007-2008   The Florida State University
 * Copyright (c) 2009        The University of Edinburgh
 * Copyright (c) 2014 - 2015 Sven Karlsson
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Stephen Hines
 *          Timothy M. Jones
 *          Sven Karlsson
 */

#ifndef __ARCH_RISCV_TLB_HH__
#define __ARCH_RISCV_TLB_HH__

#include <map>

#include "arch/generic/tlb.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/utility.hh"
#include "arch/riscv/vtophys.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/RiscvTLB.hh"

class ThreadContext;

namespace RiscvISA {

/* Dummy implementation. Riscv does not (yet) support a TLB. */
struct TlbEntry
{
 Addr _pageStart;

 inline
 TlbEntry()
 {
 }

 inline
 TlbEntry(Addr asn, Addr vaddr, Addr paddr,
           bool uncacheable, bool read_only)
        : _pageStart(paddr)
 {
  if (uncacheable || read_only)
   warn("Riscv TlbEntry does not support uncacheable"
        " or read-only mappings\n");

 }

 inline void
 updateVaddr(Addr new_vaddr)
 {
  panic("unimplemented");
 }

 inline Addr
 pageStart()
 {
  return _pageStart;
 }

 inline void
 serialize(std::ostream &os)
 {
  SERIALIZE_SCALAR(_pageStart);
 }

 inline void
 unserialize(Checkpoint *cp, const std::string &section)
 {
  UNSERIALIZE_SCALAR(_pageStart);
 }
};

class TLB : public BaseTLB
{
  protected:
    int size;                   // TLB Size

    /* Stat containers. Taken from src/arch/arm/tlb.hh */
    mutable Stats::Scalar instHits;
    mutable Stats::Scalar instMisses;
    mutable Stats::Scalar readHits;
    mutable Stats::Scalar readMisses;
    mutable Stats::Scalar writeHits;
    mutable Stats::Scalar writeMisses;
    mutable Stats::Scalar inserts;
    mutable Stats::Scalar flushTlb;
    mutable Stats::Scalar flushTlbMva;
    mutable Stats::Scalar flushTlbMvaAsid;
    mutable Stats::Scalar flushTlbAsid;
    mutable Stats::Scalar flushedEntries;
    mutable Stats::Scalar alignFaults;
    mutable Stats::Scalar prefetchFaults;
    mutable Stats::Scalar domainFaults;
    mutable Stats::Scalar permsFaults;

    Stats::Formula readAccesses;
    Stats::Formula writeAccesses;
    Stats::Formula instAccesses;
    Stats::Formula hits;
    Stats::Formula misses;
    Stats::Formula accesses;

    Addr uartAddr;
    Addr expectedAddr;


  public:
    typedef RiscvTLBParams Params;

    TLB(const Params *p);
    virtual ~TLB();

    inline void
    takeOverFrom(BaseTLB *otlb)
    {
        panic("Riscv: takeOverFrom unimplemented.\n");
    }

    inline void
    flushAll()
    {
        panic("Riscv: flushAll unimplemented.\n");
    }

    inline void
    demapPage(Addr vaddr, uint64_t asn)
    {
        panic("Riscv: demapPage unimplemented.\n");
    }

    Fault translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode);


    /** Stub function for CheckerCPU compilation support.  Riscv ISA not
     *  supported by Checker at the moment
     */
    inline Fault
    translateFunctional(RequestPtr req, ThreadContext *tc, Mode mode)
    {
        panic("Riscv: translateFunctional Not implemented\n");
        return NoFault;
    }

    Fault finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const;

    // Checkpointing
    inline void
    serialize(std::ostream &os)
    {
        SERIALIZE_SCALAR(size);
    }

    inline void
    unserialize(Checkpoint *cp, const std::string &section)
    {
        UNSERIALIZE_SCALAR(size);
    }

    void regStats();

};

} // namespace RiscvISA

#endif // __ARCH_RISCV_TLB_HH__
