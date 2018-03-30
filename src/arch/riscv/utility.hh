/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
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
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Andreas Hansson
 *          Sven Karlsson
 *          Alec Roelke
 */

#ifndef __ARCH_RISCV_UTILITY_HH__
#define __ARCH_RISCV_UTILITY_HH__

#include <cmath>
#include <cstdint>

#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

namespace RiscvISA
{

template<typename T> inline bool
isquietnan(T val)
{
    return false;
}

template<> inline bool
isquietnan<float>(float val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint32_t&>(val)&0x00400000);
}

template<> inline bool
isquietnan<double>(double val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint64_t&>(val)&0x0008000000000000ULL);
}

template<typename T> inline bool
issignalingnan(T val)
{
    return false;
}

template<> inline bool
issignalingnan<float>(float val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint32_t&>(val)&0x00200000);
}

template<> inline bool
issignalingnan<double>(double val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint64_t&>(val)&0x0004000000000000ULL);
}

inline PCState
buildRetPC(const PCState &curPC, const PCState &callPC)
{
    PCState retPC = callPC;
    retPC.advance();
    retPC.pc(curPC.npc());
    return retPC;
}

inline uint64_t
getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp)
{
    return 0;
}

inline void startupCPU(ThreadContext *tc, int cpuId)
{
}

inline void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    // First loop through the integer registers.
    for (int i = 0; i < NumIntRegs; ++i)
        dest->setIntReg(i, src->readIntReg(i));

    // Lastly copy PC/NPC
    dest->pcState(src->pcState());
}

inline void
skipFunction(ThreadContext *tc)
{
    panic("Not Implemented for Riscv");
}

inline void
advancePC(PCState &pc, const StaticInstPtr &inst)
{
    inst->advancePC(pc);
}

static inline bool
inUserMode(ThreadContext *tc)
{
    return true;
}

inline uint64_t
getExecutingAsid(ThreadContext *tc)
{
    return 0;
}

inline void
initCPU(ThreadContext *, int cpuId)
{
    panic("initCPU not implemented for Riscv.\n");
}

} // namespace RiscvISA

#endif // __ARCH_RISCV_UTILITY_HH__
