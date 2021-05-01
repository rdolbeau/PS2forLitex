#
# This file is part of LiteX.
#
# Wrapper around the PS2Host PS/2 controller
# to implement control/data CSRs & interrupt
#
# Copyright (C) 2021 Romain Dolbeau <romain@dolbeau.org>
# SPDX-License-Identifier: BSD-2-Clause

import os
#from os import path

from migen import *
from migen.genlib.fifo import SyncFIFOBuffered

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *

from PS2Host import PS2Host

def _to_signal(obj):
    return obj.raw_bits() if isinstance(obj, Record) else obj

class PS2HostWrapper(Module, AutoCSR):
    def __init__(self, platform, pads, sys_clk_freq = 100e6):
        self.platform = platform
        
        tx_data_i = Signal(8)
        send_req_i = Signal()

        # data -> 0-7: data; 31: valid
        self.rx_data = CSRStorage(size = 32, reset = 0, write_from_dev=True)
        # data -> 0-7: data; 31: valid
        self.tx_data = CSRStorage(size = 32, reset = 0, write_from_dev=True)
        # ctrl -> 0: interrupt enable, 1: reset, 31 -> interrupt
        self.ctrl = CSRStorage(size = 32, reset = 0, write_from_dev=True)
        in_ctrl_1_n_d = Signal()
        self.comb += in_ctrl_1_n_d.eq(~self.ctrl.storage[1:2])
        
        self.submodules.ps2_host = PS2Host(sys_clk = ClockSignal("sys"), sys_clk_freq = sys_clk_freq, sys_rst = in_ctrl_1_n_d,
                                           ps2_clk = pads.ps2_clk, ps2_data = pads.ps2_data,
                                           tx_data = tx_data_i, send_req = send_req_i)

        busy_o = self.ps2_host.busy
        rx_data_o = self.ps2_host.rx_data
        ready_o = self.ps2_host.ready
        error_o = self.ps2_host.error
        
        self.submodules.rx_fifo = rx_fifo = SyncFIFOBuffered(width=8, depth=32) ## no fill control
        ready_o_cur = Signal()
        ready_o_prev = Signal()
        self.comb += ready_o_cur.eq(ready_o)
        self.sync += ready_o_prev.eq(ready_o_cur)
        self.comb += rx_fifo.din.eq(rx_data_o)
        # edge trigger
        self.comb += rx_fifo.we.eq(ready_o_cur & ~ready_o_prev)
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
                self.rx_data.we.eq(0),
                self.tx_data.we.eq(0),
                self.ctrl.we.eq(0),
                rx_fifo.re.eq(0),
                send_req_i.eq(0),
                NextState("RESETKBD"))
        fsm.act("RESETKBD",
                self.rx_data.dat_w.eq(0xDEADBEEF), # recognizable value
                self.rx_data.we.eq(1),
                self.ctrl.dat_w.eq(0x00000002), # stop reset to verilog block
                self.ctrl.we.eq(1),
                NextState("RUN"))
        fsm.act("RUN",
                self.rx_data.we.eq(0),
                self.tx_data.we.eq(0),
                self.ctrl.we.eq(0),
                rx_fifo.re.eq(0),
                send_req_i.eq(0),
                If(rx_fifo.readable & ~self.ctrl.storage[31:32],
                   self.rx_data.dat_w.eq(0x80000000 | rx_fifo.dout),
                   self.rx_data.we.eq(1),
                   self.ctrl.dat_w.eq(0x80000003), # must keep irq & !reset
                   self.ctrl.we.eq(1),
                   rx_fifo.re.eq(1)),
                If(~rx_fifo.readable & ~self.ctrl.storage[31:32],
                   self.rx_data.dat_w.eq(0),
                   self.rx_data.we.eq(1)),
                If(self.tx_data.storage[31:32] & ~busy_o,
                   tx_data_i.eq(self.tx_data.storage[0:8]), # put data
                   send_req_i.eq(1), # send data
                   NextState("TX")))
        fsm.act("TX",
                send_req_i.eq(0), # stop send after one cycle
                If(busy_o, NextState("TXFINISH")))
        fsm.act("TXFINISH",
                self.tx_data.dat_w.eq(0),
                self.tx_data.we.eq(1),
                NextState("RUN"))
        ## irq
        self.submodules.ev = EventManager()
        in_ctrl_31_d = Signal()
        in_ctrl_0_d = Signal()
        self.comb += in_ctrl_31_d.eq(self.ctrl.storage[31:32])
        self.comb += in_ctrl_0_d.eq(self.ctrl.storage[0:1])
        self.ev.keyev = EventSourceProcess(description="Key Event", edge="rising")
        self.ev.finalize()
        self.comb += self.ev.keyev.trigger.eq(in_ctrl_31_d & in_ctrl_0_d)
