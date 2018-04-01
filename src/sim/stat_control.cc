/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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
 *          Sascha Bischoff
 */

// This file will contain default statistics for the simulator that
// don't really belong to a specific simulator object

#include "sim/stat_control.hh"

#include <fstream>
#include <iostream>
#include <list>

#include "base/callback.hh"
#include "base/hostinfo.hh"
#include "base/statistics.hh"
#include "base/time.hh"
#include "cpu/base.hh"
#include "sim/global_event.hh"

using namespace std;

Stats::Formula simSeconds;
Stats::Value simTicks;
Stats::Value finalTick;
Stats::Value simFreq;

//gxj 声明sdhMap和instMap； regStatsSDH()函数的实现 以及calculateSDH()的实现；
#include <map>

Stats::SparseHistogram Stack_Distance_Histogram;
Stats::Scalar hits_fgl;//ZYT
//Stats::Scalar accesses_fgl; //ZYT
//Stats::Scalar misses_fgl; //ZYT
//Stats::SparseHistogram Stack_Distance_HistogramO3;  //fglO3 定义Stack_Distance_HistogramO3

map<unsigned int, Addr> sdhMap;
map<unsigned int, list<Addr> > instMap;
//map<unsigned int, Addr> sdhMapO3; //fglO3  定义sdhMapO3
//map<unsigned int, list<Addr> > instMapO3;   //fglO3 定义instMapO3

extern unsigned globeSetMask;//已知地址求组时的掩码；
extern int globeSetShift;    //已知地址求组时的偏移量；

void regStatsSDH()
{
    Stack_Distance_Histogram
	.init(0)
	.name("inorder_stack_distance_histogram")  //统计信息的前缀
	.desc("stack distance histogram in software profiling")   //统计信息的后缀
	;

	hits_fgl
		.name("dcache_hit_number")
		.desc("ZYT")
		;

	/*accesses_fgl
		.name("dcache_access_number")
		.desc("ZYT")
		;

	misses_fgl
		.name("dcache_misses_number")
		.desc("ZYT")
		;*/
}
//fglO3 实现regStatsSDHO3() 
/*void sumstats()
{
	sum_fgl
	.init(0)
	.name(name()+fgl ")  //统计信息的前缀
	.desc("stack distance histogram in software profiling")   //统计信息的后缀
	;
}//fglO3 end;*/

Addr extractSet(Addr addr)
{
    return ((addr >> globeSetShift) & globeSetMask);//???
}


void calculateSDH()
{
	bool flag = false;  //fglceshi
	//int SDD = 40;//ZYT
	//static long unsigned int sum_fgl = 0;
    map<unsigned int, Addr>::iterator mapPtr;
    for (mapPtr = sdhMap.begin(); mapPtr != sdhMap.end(); mapPtr++)
    {
		flag = false;
		Addr cacheAddr = mapPtr->second;
		Addr cacheSet = extractSet(cacheAddr);

		if (instMap.find(cacheSet) != instMap.end())
		{
			list<Addr>::iterator listPtr;
			int stackDist = 1;
			for (listPtr = instMap[cacheSet].end(); listPtr != instMap[cacheSet].begin(); ) //倒着索引
			{
				listPtr--;
				if (*listPtr == cacheAddr)
				{
					Stack_Distance_Histogram.sample(stackDist);//这个SDH.sample是对分布中值为stackDish的柱+1；
					if(stackDist <= 4)
						hits_fgl++;
					flag = true;  //fglceshi
					instMap[cacheSet].erase(listPtr);  //如果找到相同地址值，则把旧地址值从list中删除。
					break;
				}
				else
					stackDist++;
			}
			//SDD=stackDist;//ZYT
		}
		if (instMap[cacheSet].size() >= 40) //如果list的size到达上限40，则
			instMap[cacheSet].erase(instMap[cacheSet].begin()); //删除最旧的地址；
		instMap[cacheSet].push_back(cacheAddr);  //把新出现的地址追加到list的末尾；
		//accesses_fgl++;  //fgl
		if (flag == false) Stack_Distance_Histogram.sample(40); //fglceshi
	}
	//misses_fgl = accesses_fgl - hits_fgl;  //fgl;
    instMap.clear();
    sdhMap.clear();
}

//fglO3 calculateSDHO3()的实现
/*void calculateSDHO3()           
{
	map<unsigned int, Addr>::iterator mapPtr;
	for (mapPtr = sdhMapO3.begin(); mapPtr != sdhMapO3.end(); mapPtr++)
	{

		Addr cacheAddr = mapPtr->second;
		Addr cacheSet = extractSet(cacheAddr);
		if (instMapO3.find(cacheSet) != instMapO3.end())
		{
			list<Addr>::iterator listPtr;  
			int stackDist = 1;
			for (listPtr = instMapO3[cacheSet].end(); listPtr != instMapO3[cacheSet].begin(); ) //倒着索引
			{
				listPtr--;
				if (*listPtr == cacheAddr)
				{
					Stack_Distance_HistogramO3.sample(stackDist);//这个SDH.sample是对分布中值为stackDist的柱+1；
					instMapO3[cacheSet].erase(listPtr);  //如果找到相同地址值，则把旧地址值从list中删除。
					break;
			    }
				else
					stackDist++;
																
			 }

		}
		if (instMapO3[cacheSet].size() >= 40) //如果list的size到达上限40，则
		instMapO3[cacheSet].erase(instMapO3[cacheSet].begin()); //删除最旧的地址；
		instMapO3[cacheSet].push_back(cacheAddr);  //把新出现的地址追加到list的末尾；
	}
	instMapO3.clear();
	sdhMapO3.clear();
}//fgl  end  */

namespace Stats {

Time statTime(true);
Tick startTick;

GlobalEvent *dumpEvent;

struct SimTicksReset : public Callback
{
    void process()
    {
        statTime.setTimer();
        startTick = curTick();
    }
};

double
statElapsedTime()
{
    Time now;
    now.setTimer();

    Time elapsed = now - statTime;
    return elapsed;
}

Tick
statElapsedTicks()
{
    return curTick() - startTick;
}

Tick
statFinalTick()
{
    return curTick();
}

SimTicksReset simTicksReset;

struct Global
{
    Stats::Formula hostInstRate;
    Stats::Formula hostOpRate;
    Stats::Formula hostTickRate;
    Stats::Value hostMemory;
    Stats::Value hostSeconds;

    Stats::Value simInsts;
    Stats::Value simOps;

    Global();
};

Global::Global()
{
    simInsts
        .functor(BaseCPU::numSimulatedInsts) //追sim_insts，没有找到++的位置，在numSimlatedInsts接着往下找
        .name("sim_insts")
        .desc("Number of instructions simulated")
        .precision(0)
        .prereq(simInsts)
        ;

    simOps
        .functor(BaseCPU::numSimulatedOps)
        .name("sim_ops")
        .desc("Number of ops (including micro ops) simulated")
        .precision(0)
        .prereq(simOps)
        ;

    simSeconds
        .name("sim_seconds")
        .desc("Number of seconds simulated")
        ;

    simFreq
        .scalar(SimClock::Frequency)
        .name("sim_freq")
        .desc("Frequency of simulated ticks")
        ;

    simTicks
        .functor(statElapsedTicks)
        .name("sim_ticks")
        .desc("Number of ticks simulated")
        ;

    finalTick
        .functor(statFinalTick)
        .name("final_tick")
        .desc("Number of ticks from beginning of simulation "
              "(restored from checkpoints and never reset)")
        ;

    hostInstRate
        .name("host_inst_rate")
        .desc("Simulator instruction rate (inst/s)")
        .precision(0)
        .prereq(simInsts)
        ;

    hostOpRate
        .name("host_op_rate")
        .desc("Simulator op (including micro ops) rate (op/s)")
        .precision(0)
        .prereq(simOps)
        ;

    hostMemory
        .functor(memUsage)
        .name("host_mem_usage")
        .desc("Number of bytes of host memory used")
        .prereq(hostMemory)
        ;

    hostSeconds
        .functor(statElapsedTime)
        .name("host_seconds")
        .desc("Real time elapsed on the host")
        .precision(2)
        ;

    hostTickRate
        .name("host_tick_rate")
        .desc("Simulator tick rate (ticks/s)")
        .precision(0)
        ;

    simSeconds = simTicks / simFreq;
    hostInstRate = simInsts / hostSeconds;
    hostOpRate = simOps / hostSeconds;
    hostTickRate = simTicks / hostSeconds;

    registerResetCallback(&simTicksReset);
}

void
initSimStats()
{
    static Global global;
}

/**
 * Event to dump and/or reset the statistics.
 */
class StatEvent : public GlobalEvent
{
  private:
    bool dump;
    bool reset;
    Tick repeat;

  public:
    StatEvent(Tick _when, bool _dump, bool _reset, Tick _repeat)
        : GlobalEvent(_when, Stat_Event_Pri, 0),
          dump(_dump), reset(_reset), repeat(_repeat)
    {
    }

    virtual void
    process()
    {
		//gxj 在process()函数内调用calculateSDH计算堆栈距离分布；
		calculateSDH();
		//end

	//	calculateSDHO3();  //fglO3 在process()函数内调用calculateSDHO3()计算堆栈距离分布；

        if (dump)
            Stats::dump();

        if (reset)
            Stats::reset();

        if (repeat) {
            Stats::schedStatEvent(dump, reset, curTick() + repeat, repeat);
        }
    }

    const char *description() const { return "GlobalStatEvent"; }
};

void
schedStatEvent(bool dump, bool reset, Tick when, Tick repeat)
{
    // simQuantum is being added to the time when the stats would be
    // dumped so as to ensure that this event happens only after the next
    // sync amongst the event queues.  Asingle event queue simulation
    // should remain unaffected.
    dumpEvent = new StatEvent(when + simQuantum, dump, reset, repeat);
}

void
periodicStatDump(Tick period)
{
    /*
     * If the period is set to 0, then we do not want to dump periodically,
     * thus we deschedule the event. Else, if the period is not 0, but the event
     * has already been scheduled, we need to get rid of the old event before we
     * create a new one, as the old event will no longer be moved forward in the
     * event that we resume from a checkpoint.
     */
    if (dumpEvent != NULL && (period == 0 || dumpEvent->scheduled())) {
        // Event should AutoDelete, so we do not need to free it.
        dumpEvent->deschedule();
    }

    /*
     * If the period is not 0, we schedule the event. If this is called with a
     * period that is less than the current tick, then we shift the first dump
     * by curTick. This ensures that we do not schedule the event is the past.
     */
    if (period != 0) {
        // Schedule the event
        if (period >= curTick()) {
            schedStatEvent(true, true, (Tick)period, (Tick)period);
        } else {
            schedStatEvent(true, true, (Tick)period + curTick(), (Tick)period);
        }
    }
}

void
updateEvents()
{
    /*
     * If the dumpEvent has been scheduled, but is scheduled in the past, then
     * we need to shift the event to be at a valid point in time. Therefore, we
     * shift the event by curTick.
     */
    if (dumpEvent != NULL &&
        (dumpEvent->scheduled() && dumpEvent->when() < curTick())) {
        // shift by curTick() and reschedule
        Tick _when = dumpEvent->when();
        dumpEvent->reschedule(_when + curTick());
    }
}

} // namespace Stats
