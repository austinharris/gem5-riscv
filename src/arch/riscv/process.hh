/*
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009      The University of Edinburgh
 * Copyright (c) 2015      Sven Karlsson
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
 * Authors: Stephen Hines
 *          Timothy M. Jones
 *          Sven Karlsson
 */

#ifndef __RISCV_PROCESS_HH__
#define __RISCV_PROCESS_HH__

#include <string>
#include <vector>

#include "base/loader/object_file.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/registers.hh"
#include "sim/process.hh"
#include "sim/byteswap.hh"
#include "mem/port_proxy.hh"

class LiveProcess;
class ObjectFile;
class System;

/*! \todo This entire class is just a placeholder. It has to be implemented. */

class RiscvLiveProcess : public LiveProcess
{
	public:
		void initState()
		{
        LiveProcess::initState();

    		/*const Addr PageShift = 21;
          const Addr PageBytes = ULL(1) << PageShift;

          stack_base = 0x80000000;
		
          // Set pointer for next thread stack.  Reserve 8M for main stack.
          next_thread_stack_base = stack_base - (256 * 1024 * 1024);

          // Set up break point (Top of Heap)
          brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
          brk_point = roundUp(brk_point, PageBytes);

          // Set up region for mmaps.  Start it 1GB above the top of the heap.
          mmap_start = mmap_end = brk_point + 0x40000000L;

          // load object file into target memory
          objFile->loadSections(initVirtMem);
	
          int space_needed = 0x4000000;

          // set bottom of stack
          stack_min = stack_base - space_needed;
          // align it
          stack_min = roundDown(stack_min, PageBytes);
          stack_size = stack_base - stack_min;
          // map memory
          allocateMem(stack_min, roundUp(stack_size, PageBytes));
          allocateMem(0x20000000, 0xffff);*/

        PortProxy systemPort(system->getSystemPort(),system->cacheLineSize());

        //USE A SIMPLE SYSTEM PORT PROXY INSTEAD OF THE normal, derived seTranslatingPortProxy.
        objFile->loadSections(systemPort);
        ThreadContext *tc = system->getThreadContext(contextIds[0]);
        tc->pcState(objFile->entryPoint());
        //tc->setIntReg(TheISA::StackPointerReg,0x7FFF0FFF);
			
		}


  protected:
    inline
    RiscvLiveProcess(LiveProcessParams * params, ObjectFile *objFile)
        : LiveProcess(params, objFile)
    {
   
    }
};

/* No architectural page table defined for this ISA */
typedef NoArchPageTable ArchPageTable;

#endif // __RISCV_PROCESS_HH__

