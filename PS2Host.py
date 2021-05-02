#
# This file is part of LiteX.
#
# Migen port & fix of the PS/2 controller at
#  https://opencores.org/projects/ps2_host_controller
#
# Copyright (C) 2021 Romain Dolbeau <romain@dolbeau.org>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from migen.fhdl.specials import Tristate

def h_xor_8(data):
    return data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7]

def h_or_11(data):
    return data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] | data[8] | data[9] | data[10]

def rev_8(data):
    return Cat(data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0])

class PS2HostClkCtrl(Module):
    def __init__(self, sys_clk, sys_clk_freq, sys_rst, ps2_clk_i, send_req):
        self.ps2_clk_posedge = Signal()
        self.ps2_clk_negedge = Signal()

        self.ps2_data_o = Signal()
        self.ps2_clk_o = Signal()

        ps2_clk_samples = Signal(2)

        self.sync += If(sys_rst,
                        ps2_clk_samples.eq(0x3)).Else(
                            ps2_clk_samples.eq(Cat(ps2_clk_i, ps2_clk_samples[0:1])))
        self.comb += self.ps2_clk_posedge.eq(~ps2_clk_samples[1:2] &  ps2_clk_samples[0:1])
        self.comb += self.ps2_clk_negedge.eq( ps2_clk_samples[1:2] & ~ps2_clk_samples[0:1])

        cycles_for_125us = int(125e-6 * sys_clk_freq)
        cycles_for_20us = int(20e-6 * sys_clk_freq)
        inhibit_timer = Signal(log2_int(cycles_for_125us, need_pow2 = False))
        timer_is_zero = Signal()
        self.comb += timer_is_zero.eq(inhibit_timer == 0x0000)
        
        self.sync += If((sys_rst | (~send_req & timer_is_zero)),
                        inhibit_timer.eq(0)).Else(
                            If(timer_is_zero,
                               inhibit_timer.eq(cycles_for_125us)).Else(
                                   inhibit_timer.eq(inhibit_timer - 1)))

        self.comb += If(((inhibit_timer > 0) & (inhibit_timer < cycles_for_20us)),
                        self.ps2_data_o.eq(0)).Else(
                            self.ps2_data_o.eq(1))
        
        self.comb += If((inhibit_timer == 0),
                        self.ps2_clk_o.eq(1)).Else(
                            self.ps2_clk_o.eq(0))

class PS2HostWatchdog(Module):
    def __init__(self, sys_clk, sys_clk_freq, sys_rst, ps2_clk_posedge, ps2_clk_negedge, watchdog_delay_us = 200):
        self.watchdog_rst = Signal()
        watchdog_active = Signal()
        
        cycles_for_Xus = int(watchdog_delay_us * 1e-6 * sys_clk_freq)
        watchdog_timer = Signal(log2_int(cycles_for_Xus, need_pow2 = False))
        ps2_clk_edge = Signal()
        
        self.comb += ps2_clk_edge.eq(ps2_clk_posedge | ps2_clk_negedge)
        self.sync += If((sys_rst | self.watchdog_rst | ~(watchdog_active | ps2_clk_edge)),
                        watchdog_active.eq(0)).Else(
                            watchdog_active.eq(1))
        self.sync += If((sys_rst | self.watchdog_rst | ~watchdog_active | ps2_clk_edge),
                        watchdog_timer.eq(cycles_for_Xus)).Else(
                            watchdog_timer.eq(watchdog_timer - 1))
        self.comb += If(watchdog_timer == 0,
                        self.watchdog_rst.eq(1)).Else(
                            self.watchdog_rst.eq(0))

class PS2HostRX(Module):
    def __init__(self, sys_clk, sys_clk_freq, sys_rst, ps2_clk_negedge, ps2_data):
        self.rx_data = Signal(8)
        self.ready = Signal()
        self.error = Signal()
        frame = Signal(12)

        self.sync += If((sys_rst | self.ready),
                        frame.eq(0x001)).Else(
                            If(ps2_clk_negedge,
                               frame.eq(Cat(ps2_data, frame[0:11]))).Else(
                                   frame.eq(frame)))
        self.sync += If(sys_rst,
                        self.ready.eq(0)).Else(
                            self.ready.eq(frame[11:12]))
        self.sync += If(sys_rst,
                        self.rx_data.eq(0x00)).Else(
                            If(frame[11:12],
                               self.rx_data.eq(rev_8(frame[2:10]))).Else(
                                   self.rx_data.eq(self.rx_data)))
        self.sync += If(sys_rst,
                        self.error.eq(0)).Else(
                            If(frame[11:12],
                               self.error.eq(~(~frame[10:11] & (~frame[1:2] == h_xor_8(frame[2:10])) & frame[0:1]))).Else(
                                   self.error.eq(self.error)))

class PS2HostTX(Module):
    def __init__(self, sys_clk, sys_clk_freq, sys_rst, ps2_clk_posedge, tx_data, send_req):
        self.ps2_clk_posedge = ps2_clk_posedge
        self.tx_data = tx_data
        self.send_req = send_req

        self.busy = Signal()
        frame = Signal(12)
        self.ps2_data_o = Signal()

        self.sync += If(sys_rst | (~send_req & (frame == 0x000)),
                        frame.eq(0x000)).Else(
                            If((frame == 0x000),
                               frame.eq(Cat(1, ~h_xor_8(tx_data), rev_8(tx_data), 0, 0))).Else(
                                    If(ps2_clk_posedge,
                                       frame.eq(Cat(0, frame[0:11]))).Else(
                                           frame.eq(frame))))
        self.comb += If((~h_or_11(frame[0:11]) | frame[0]),
                        self.ps2_data_o.eq(1)).Else(
                            self.ps2_data_o.eq(frame[11]))
        self.comb += If(frame == 0x000,
                        self.busy.eq(0)).Else(
                            self.busy.eq(1))

class PS2Host(Module):
    def __init__(self, sys_clk, sys_clk_freq, sys_rst, ps2_clk, ps2_data, tx_data, send_req):
        sys_rst = sys_rst
        self.ps2_clk = ps2_clk
        ps2_data_i = Signal()
        ps2_data_o = Signal()
        ps2_data_oe = Signal()
        self.specials += Tristate(ps2_data, ps2_data_o, ps2_data_oe, ps2_data_i)
        ps2_clk_i = Signal()
        ps2_clk_o = Signal()
        ps2_clk_oe = Signal()
        self.specials += Tristate(ps2_clk, ps2_clk_o, ps2_clk_oe, ps2_clk_i)
        self.busy = Signal()

        # Clock tracking
        self.submodules.clk_ctrl = PS2HostClkCtrl(sys_clk, sys_clk_freq, sys_rst, ps2_clk_i, send_req)
        self.ps2_clk_posedge = self.clk_ctrl.ps2_clk_posedge
        self.ps2_clk_negedge = self.clk_ctrl.ps2_clk_negedge

        # Watchdog
        self.submodules.watchdog = PS2HostWatchdog(sys_clk, sys_clk_freq, sys_rst, self.ps2_clk_posedge, self.ps2_clk_negedge, watchdog_delay_us = 400)
        self.watchdog_rst = self.watchdog.watchdog_rst

        # RX data
        rx_rst = Signal()
        self.comb += rx_rst.eq(sys_rst | self.busy | self.watchdog_rst)
        self.submodules.rx = PS2HostRX(sys_clk, sys_clk_freq, rx_rst, self.ps2_clk_negedge, ps2_data_i)
        self.rx_data = self.rx.rx_data
        self.ready = self.rx.ready
        self.error = self.rx.error

        # TX Data
        tx_rst = Signal()
        self.comb += tx_rst.eq(sys_rst | self.watchdog_rst)
        self.submodules.tx = PS2HostTX(sys_clk, sys_clk_freq, tx_rst, self.ps2_clk_posedge, tx_data, send_req)
        self.comb += self.busy.eq(self.tx.busy)

        self.comb += ps2_data_o.eq(self.clk_ctrl.ps2_data_o & self.tx.ps2_data_o) # if either is 0, then 0
        self.comb += ps2_data_oe.eq(~(self.clk_ctrl.ps2_data_o & self.tx.ps2_data_o)) # only output 0 (1 is Z)
        self.comb += ps2_clk_o.eq(self.clk_ctrl.ps2_clk_o)
        self.comb += ps2_clk_oe.eq(~self.clk_ctrl.ps2_clk_o)
