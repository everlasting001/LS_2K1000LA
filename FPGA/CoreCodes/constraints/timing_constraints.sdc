//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11.02 (64-bit)
//Project: FourAxis SCARA Robot Arm

// 50MHz system clock
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]
