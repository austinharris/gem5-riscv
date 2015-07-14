/*
 * Copyright (c) 2013        ARM Limited
 * Copyright (c) 2014 - 2015 Sven Karlsson
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
 * Authors: Andreas Hansson
 *          Sven Karlsson
 */

#ifndef __ARCH_RISCV_REGISTERS_HH__
#define __ARCH_RISCV_REGISTERS_HH__

#include "arch/riscv/generated/max_inst_regs.hh"
#include "base/types.hh"

namespace RiscvISA {

using RiscvISAInst::MaxInstSrcRegs;
using RiscvISAInst::MaxInstDestRegs;

typedef uint_fast8_t RegIndex;
typedef uint32_t IntReg;
typedef uint32_t FloatRegBits; //! \todo needs to be updated to reflect FPU
typedef float FloatReg;        //! \todo needs to be updated to reflect FPU
typedef uint8_t CCReg;         // Not applicable to Riscv
typedef uint32_t MiscReg;

const int NumIntRegs = 32;
const int NumFloatRegs = 0;
const int NumCCRegs = 0;
const int NumMiscRegs = 1;

// These help enumerate all the registers for dependence tracking.
const int FP_Reg_Base = NumIntRegs;
const int CC_Reg_Base = FP_Reg_Base + NumFloatRegs;
const int Misc_Reg_Base = CC_Reg_Base + NumCCRegs;
const int Max_Reg_Index = Misc_Reg_Base + NumMiscRegs;


// Semantically meaningful register indices
const int ZeroReg = 0;
const int ReturnValueReg = 8;
const int ArgumentReg0 = 8;
const int ArgumentReg1 = 9;
const int ArgumentReg2 = 10;
const int ArgumentReg3 = 11;
const int ArgumentReg4 = 12;
const int FramePointerReg = 125;
const int StackPointerReg = 126;

const int SyscallNumReg = 8;
const int SyscallPseudoReturnReg = 8;
const int SyscallSuccessReg = 8;

enum MiscRegIndex{
    MISCREG_HARTID = 0
};

}

#endif // __ARCH_RISCV_REGISTERS_HH__
