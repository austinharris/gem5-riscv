/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 *          Timothy M. Jones
 *          Sven Karlsson
 */

#ifndef __ARCH_RISCV_ISA_HH__
#define __ARCH_RISCV_ISA_HH__

#include "arch/riscv/registers.hh"
#include "arch/riscv/types.hh"
#include "base/misc.hh"
#include "sim/sim_object.hh"

struct RiscvISAParams;
class ThreadContext;
class Checkpoint;
class EventManager;

namespace RiscvISA
{

class ISA : public SimObject
{
  protected:

    std::vector<std::vector<MiscReg> > miscRegFile;

  public:
    typedef RiscvISAParams Params;
    static std::string miscRegNames[NumMiscRegs];


    void
    clear();

    MiscReg
    readMiscRegNoEffect(int misc_reg) const;

    MiscReg
    readMiscReg(int misc_reg, ThreadContext *tc);

    void
    setMiscRegNoEffect(int misc_reg, const MiscReg &val);

    void
    setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc);

    int
    flattenIntIndex(int reg) const
    {
        return reg;
    }

    int
    flattenFloatIndex(int reg) const
    {
        return reg;
    }

    // dummy
    int
    flattenCCIndex(int reg) const
    {
        return reg;
    }

    int
    flattenMiscIndex(int reg) const
    {
        return reg;
    }

    void startup(ThreadContext *tc) {}

    /// Explicitly import the otherwise hidden startup
    using SimObject::startup;

    const Params *params() const;

    ISA(Params *p);
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_ISA_HH__
