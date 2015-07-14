/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2014 Sven Karlsson
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
 *          Jaidev Patwardhan
 *          Stephen Hines
 *          Timothy M. Jones
 *          Sven Karlsson
 */

#include "arch/riscv/tlb.hh"
#include "arch/riscv/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "params/RiscvTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

using namespace std;
using namespace RiscvISA;


TLB::TLB(const Params *p)
    : BaseTLB(p), size(p->size)
{
    uartAddr = 0x20000025;
    expectedAddr = 0x200000af;
}

TLB::~TLB()
{
}

Fault
TLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode)
{

	
    //if(req->getVaddr() == expectedAddr || req->getVaddr() == 0x200000ef || req->getVaddr() == 0x20000000) { req->setPaddr(uartAddr); return NoFault; }

    req->setPaddr(req->getVaddr());	

    return NoFault;
}

void
TLB::regStats()
{
    instHits
        .name(name() + ".inst_hits")
        .desc("ITB inst hits")
        ;

    instMisses
        .name(name() + ".inst_misses")
        .desc("ITB inst misses")
        ;

    instAccesses
        .name(name() + ".inst_accesses")
        .desc("ITB inst accesses")
        ;

    readHits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    readMisses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;

    readAccesses
        .name(name() + ".read_accesses")
        .desc("DTB read accesses")
        ;

    writeHits
        .name(name() + ".write_hits")
        .desc("DTB write hits")
        ;

    writeMisses
        .name(name() + ".write_misses")
        .desc("DTB write misses")
        ;

    writeAccesses
        .name(name() + ".write_accesses")
        .desc("DTB write accesses")
        ;

    hits
        .name(name() + ".hits")
        .desc("DTB hits")
        ;

    misses
        .name(name() + ".misses")
        .desc("DTB misses")
        ;

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    flushTlb
        .name(name() + ".flush_tlb")
        .desc("Number of times complete TLB was flushed")
        ;

    flushTlbMva
        .name(name() + ".flush_tlb_mva")
        .desc("Number of times TLB was flushed by MVA")
        ;

    flushTlbMvaAsid
        .name(name() + ".flush_tlb_mva_asid")
        .desc("Number of times TLB was flushed by MVA & ASID")
        ;

    flushTlbAsid
        .name(name() + ".flush_tlb_asid")
        .desc("Number of times TLB was flushed by ASID")
        ;

    flushedEntries
        .name(name() + ".flush_entries")
        .desc("Number of entries that have been flushed from TLB")
        ;

    alignFaults
        .name(name() + ".align_faults")
        .desc("Number of TLB faults due to alignment restrictions")
        ;

    prefetchFaults
        .name(name() + ".prefetch_faults")
        .desc("Number of TLB faults due to prefetch")
        ;

    domainFaults
        .name(name() + ".domain_faults")
        .desc("Number of TLB faults due to domain restrictions")
        ;

    permsFaults
        .name(name() + ".perms_faults")
        .desc("Number of TLB faults due to permissions restrictions")
        ;

    instAccesses = instHits + instMisses;
    readAccesses = readHits + readMisses;
    writeAccesses = writeHits + writeMisses;
    hits = readHits + writeHits + instHits;
    misses = readMisses + writeMisses + instMisses;
    accesses = readAccesses + writeAccesses + instAccesses;
}

RiscvISA::TLB *
RiscvTLBParams::create()
{
    return new RiscvISA::TLB(this);
}
