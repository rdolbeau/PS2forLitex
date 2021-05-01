# PS2forLitex

PS/2 (keyboard, maybe mouse but untested) controller for the [LiteX](https://github.com/enjoy-digital/litex/) SoC generator.

Files in there:

PS2Host.py: the actual controller, reimplemented in Migen from the PS/2 specifications and a verilog host controller from opencores

PS2Wrapper.py: the CSR/interrupt to bridge the controller with LiteX

0001.litex-ps2.patch: Linux driver for the controller (derived from the Altera PS/2 driver)

PS2_json2dts_fragment.txt: a bit of code to add to the json2dts python script to add the proper DTS entry (it pretends the interrupt is optional, but the driver won't work without the interrupt)

The pins should be added to the 'platform' in litex-boards, e.g. for the digilent PS/2 PMod:

```
        ("ps2kbd", 0,
            Subsignal("ps2_data", Pins(f"{pmod}:0 ")),
            Subsignal("ps2_clk",  Pins(f"{pmod}:2")),
            Misc("DRIVE=16"),
            IOStandard("LVCMOS33"),
        ),
```

Then the controller can be added in the 'target', e.g.:

```
        # Keyboard ---------------------------------------------------------------------------------
        if ps2kbd:
            self.submodules.ps2kbd = PS2HostWrapper(platform = platform, pads = platform.request("ps2kbd"), sys_clk_freq = sys_clk_freq)
            self.irq.add("ps2kbd", use_loc_if_exists=True)

```
