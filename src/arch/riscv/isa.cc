/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2014 Sven Karlsson
 * All rights reserved
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
 * Authors: Andreas Sandberg
 *          Sven Karlsson
 */

#include "arch/riscv/isa.hh"
#include "params/RiscvISA.hh"
#include "debug/RiscvPRA.hh"

namespace RiscvISA
{

std::string
ISA::miscRegNames[NumMiscRegs] =
{
    "Hartid"
};

ISA::ISA(Params *p)
    : SimObject(p)
{
    miscRegFile.resize(NumMiscRegs);
    for (int i =0; i < NumMiscRegs; i++) {
        miscRegFile[i].resize(1);
    }
    clear();
}

void
ISA::clear()
{
    for(int i = 0; i < NumMiscRegs; i++)
        for (int j = 0; j < miscRegFile[i].size(); j++)
            miscRegFile[i][j] = 0;
}

const RiscvISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

MiscReg
ISA::readMiscRegNoEffect(int misc_reg) const
{
    unsigned reg_sel = 0;
    DPRINTF(RiscvPRA, "Reading CSR:%u Select:%u (%s) (%lx).\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg],
            miscRegFile[misc_reg][reg_sel]);
    return miscRegFile[misc_reg][reg_sel];
}

//@TODO: MIPS MT's register view automatically connects
//       Status to TCStatus depending on current thread
//template <class TC>
MiscReg
ISA::readMiscReg(int misc_reg, ThreadContext *tc)
{
    unsigned reg_sel = 0;
    DPRINTF(RiscvPRA,
            "Reading CP0 Register:%u Select:%u (%s) with effect (%lx).\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg],
            miscRegFile[misc_reg][reg_sel]);

    return miscRegFile[misc_reg][reg_sel];
}

void
ISA::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    unsigned reg_sel = 0;
    DPRINTF(RiscvPRA,
            "Setting (direct set) CP0 Register:%u "
            "Select:%u (%s) to %#x.\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);

    miscRegFile[misc_reg][reg_sel] = val;
}

void
ISA::setMiscReg(int misc_reg, const MiscReg &val,
                ThreadContext *tc)
{
    int reg_sel = 0;

    DPRINTF(RiscvPRA,
            "Setting CP0 Register:%u "
            "Select:%u (%s) to %#x, with effect.\n",
            misc_reg / 8, misc_reg % 8, miscRegNames[misc_reg], val);


    miscRegFile[misc_reg][reg_sel] = val;

    //    scheduleCP0Update(tc->getCpuPtr(), Cycles(1));
}

}

RiscvISA::ISA *
RiscvISAParams::create()
{
    return new RiscvISA::ISA(this);
}
