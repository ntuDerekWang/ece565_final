# -*- coding: utf-8 -*-
# Copyright (c) 2015 Jason Power
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Jason Power

""" This file creates a single CPU and a two-level cache system.
This script takes a single parameter which specifies a binary to execute.
If none is provided it executes 'hello' by default (mostly used for testing)

See Part 1, Chapter 3: Adding cache to the configuration script in the
learning_gem5 book for more information about this script.
This file exports options for the L1 I/D and L2 cache sizes.

IMPORTANT: If you modify this file, it's likely that the Learning gem5 book
           also needs to be updated. For now, email Jason <power.jg@gmail.com>

"""

from __future__ import print_function
# import the m5 (gem5) library created when gem5 is built
import m5
# import all of the SimObjects
from m5.objects import *

# Add the common scripts to our path
m5.util.addToPath('../')
m5.util.addToPath('../common')
m5.util.addToPath('../ruby')
m5.util.addToPath('../topologies')
# import the caches which we made
from caches import *

# import the SimpleOpts module
from common import SimpleOpts
import spec2k6
import Options
import Simulation
import optparse

parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

# Benchmark options

parser.add_option("-b", "--benchmark", default="",
                 help="The benchmark to be loaded.")
parser.add_option("-z", "--enable_atacahe", default="",
                 help="Enable atcache.")

(options, args) = parser.parse_args()
if options.benchmark == 'perlbench':
   process = spec2k6.perlbench
elif options.benchmark == 'bzip2':
   process = spec2k6.bzip2
elif options.benchmark == 'gcc':
   process = spec2k6.gcc
elif options.benchmark == 'bwaves':
   process = spec2k6.bwaves
elif options.benchmark == 'gamess':
   process = spec2k6.gamess
elif options.benchmark == 'mcf':
   process = spec2k6.mcf
elif options.benchmark == 'milc':
   process = spec2k6.milc
elif options.benchmark == 'zeusmp':
   process = spec2k6.zeusmp
elif options.benchmark == 'gromacs':
   process = spec2k6.gromacs
elif options.benchmark == 'cactusADM':
   process = spec2k6.cactusADM
elif options.benchmark == 'leslie3d':
   process = spec2k6.leslie3d
elif options.benchmark == 'namd':
   process = spec2k6.namd
elif options.benchmark == 'gobmk':
   process = spec2k6.gobmk;
elif options.benchmark == 'dealII':
   process = spec2k6.dealII
elif options.benchmark == 'soplex':
   process = spec2k6.soplex
elif options.benchmark == 'povray':
   process = spec2k6.povray
elif options.benchmark == 'calculix':
   process = spec2k6.calculix
elif options.benchmark == 'hmmer':
   process = spec2k6.hmmer
elif options.benchmark == 'sjeng':
   process = spec2k6.sjeng
elif options.benchmark == 'GemsFDTD':
   process = spec2k6.GemsFDTD
elif options.benchmark == 'libquantum':
   process = spec2k6.libquantum
elif options.benchmark == 'h264ref':
   process = spec2k6.h264ref
elif options.benchmark == 'tonto':
   process = spec2k6.tonto
elif options.benchmark == 'lbm':
   process = spec2k6.lbm
elif options.benchmark == 'omnetpp':
   process = spec2k6.omnetpp
elif options.benchmark == 'astar':
   process = spec2k6.astar
elif options.benchmark == 'wrf':
   process = spec2k6.wrf
elif options.benchmark == 'sphinx3':
   process = spec2k6.sphinx3
elif options.benchmark == 'xalancbmk':
   process = spec2k6.xalancbmk
elif options.benchmark == 'specrand_i':
   process = spec2k6.specrand_i
elif options.benchmark == 'specrand_f':
   process = spec2k6.specrand_f

if options.enable_atacahe == 'True':
   enable_atacahe = True
elif options.enable_atacahe == 'False':
   enable_atacahe = False

'''
# Set the usage message to display
SimpleOpts.set_usage("usage: %prog [options] <binary to execute>")

# Finalize the arguments and grab the opts so we can pass it on to our objects
(opts, args) = SimpleOpts.parse_args()

# get ISA for the default binary to run. This is mostly for simple testing
isa = str(m5.defines.buildEnv['TARGET_ISA']).lower()

# Default to running 'hello', use the compiled ISA to find the binary
binary = 'tests/test-progs/hello/bin/' + isa + '/linux/hello'

# Check if there was a binary passed in via the command line and error if
# there are too many arguments
if len(args) == 1:
    binary = args[0]
elif len(args) > 1:
    SimpleOpts.print_help()
    m5.fatal("Expected a binary to execute as positional argument")
'''
# create the system we are going to simulate
system = System()

# Set the clock fequency of the system (and all of its children)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '3GHz'
system.clk_domain.voltage_domain = VoltageDomain()

# Set up the system
system.mem_mode = 'timing'               # Use timing accesses
system.mem_ranges = [AddrRange('2048MB')] # Create an address range

# Create a simple CPU
system.cpu = DerivO3CPU()

# Create an L1 instruction and data cache
system.cpu.icache = L1ICache()
system.cpu.dcache = L1DCache()

# Connect the instruction and data caches to the CPU
system.cpu.icache.connectCPU(system.cpu)
system.cpu.dcache.connectCPU(system.cpu)

# Create a memory bus, a coherent crossbar, in this case
system.l2bus = L2XBar()

# Hook the CPU ports up to the l2bus
system.cpu.icache.connectBus(system.l2bus)
system.cpu.dcache.connectBus(system.l2bus)

# Create an L2 cache and connect it to the l2bus
system.l2cache = L2Cache()
system.l2cache.connectCPUSideBus(system.l2bus)

# Create a memory bus
system.membus = SystemXBar()

# Connect the L2 cache to the membus
system.l2cache.connectMemSideBus(system.membus)
system.l2cache.tags.EnableAtcache = enable_atacahe
system.l2cache.tags.warmup_percentage = 90
system.l2cache.tags.AtcacheLookupLatency = 2
system.l2cache.sequential_access = True
system.l2cache.tag_latency = 35
system.l2cache.data_latency = 25
system.l2cache.response_latency = 35
system.l2cache.assoc = 16
system.l2cache.size = '256MB'

# create the interrupt controller for the CPU
system.cpu.createInterruptController()

# For x86 only, make sure the interrupts are connected to the memory
# Note: these are directly connected to the memory bus and are not cached
if m5.defines.buildEnv['TARGET_ISA'] == "x86":
    system.cpu.interrupts[0].pio = system.membus.master
    system.cpu.interrupts[0].int_master = system.membus.slave
    system.cpu.interrupts[0].int_slave = system.membus.master

# Connect the system up to the membus
system.system_port = system.membus.slave

# Create a DDR3 memory controller
system.mem_ctrl = DDR3_1600_8x8()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.master

# Create a process for a simple "Hello World" application
#process = Process()
#process = spec2k6.povray
#process = spec2k6.gcc

# Set the command
# cmd is a list which begins with the executable (like argv)
#process.cmd = [binary]
# Set the cpu to use the process as its workload and create thread contexts
system.cpu.workload = process
system.cpu.createThreads()
system.cpu.max_insts_any_thread = 10000000000
# set up the root SimObject and start the simulation
root = Root(full_system = False, system = system)
# instantiate all of the objects we've created above
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print('Exiting @ tick %i because %s' % (m5.curTick(), exit_event.getCause()))
