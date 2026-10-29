# MES50HP SoC Hardware Test - PDS Build Script
# Usage: pds_shell -file build.tcl
# Device: PGL50H-6IFBG484 (Logos)
# ============================================================

set_arch -family Logos -device PGL50H -speedgrade -6 -package FBG484

add_design -verilog "rtl/system_top.v"
add_design -verilog "rtl/axi_crossbar.v"
add_design -verilog "rtl/axi_dma.v"
add_design -verilog "rtl/axi_fir_wrap.v"
add_design -verilog "custom/fir_81tap.v"

add_design -verilog "core/cortexm0ds/ahb_axi_define.v"
add_design -verilog "core/cortexm0ds/ahb2axi4_burst.v"
add_design -verilog "core/cortexm0ds/ahb2axi4_fifo.v"
add_design -verilog "core/cortexm0ds/ahb2axi4_if.v"
add_design -verilog "core/cortexm0ds/ahb2axi4_ahb.v"
add_design -verilog "core/cortexm0ds/ahb2axi4_axi.v"
add_design -verilog "core/cortexm0ds/cortexm0ds_logic.v"
add_design -verilog "core/cortexm0ds/cortexm0ds.v"
add_design -verilog "core/models/cm0_dbg_reset_sync.v"
add_design -verilog "core/cortexm0_integration/cortexm0_systick.v"
add_design -verilog "core/cortexm0_integration/cortexm0_wic.v"
add_design -verilog "core/cortexm0_integration/cortexm0integration.v"
add_design -verilog "core/cortexm0_dap/cortexm0dap.v"

add_design -verilog "mcu/cmsdk_mcu_defs.v"
add_design -verilog "mcu/models/cmsdk_clock_gate.v"
add_design -verilog "mcu/cmsdk_mcu_pin_mux.v"
add_design -verilog "mcu/cmsdk_mcu_clkctrl.v"
add_design -verilog "mcu/cmsdk_mcu.v"

add_design -verilog "mcu_system/cmsdk_axi_memory/cmsdk_axi_memory_defs.v"
add_design -verilog "mcu_system/cmsdk_axi_memory/cmsdk_axi_flash.v"
add_design -verilog "mcu_system/cmsdk_axi_memory/cmsdk_axi_sram.v"
add_design -verilog "mcu_system/cmsdk_axi_memory/cmsdk_axi_ram_beh.v"
add_design -verilog "mcu_system/cmsdk_axi_memory/cmsdk_ahb_ram_beh.v"

add_design -verilog "mcu_system/cmsdk_axi_slave_mux/cmsdk_axi_addr_decode.v"
add_design -verilog "mcu_system/cmsdk_axi_slave_mux/cmsdk_axi_slave_mux.v"

add_design -verilog "mcu_system/cmsdk_axi_to_apb/axi2apb_if.v"
add_design -verilog "mcu_system/cmsdk_axi_to_apb/axi2apb_axi.v"
add_design -verilog "mcu_system/cmsdk_axi_to_apb/axi2apb_apb.v"

add_design -verilog "mcu_system/cmsdk_apb_slave_mux/cmsdk_apb_slave_mux.v"
add_design -verilog "mcu_system/cmsdk_apb_timer/cmsdk_apb_timer.v"
add_design -verilog "mcu_system/cmsdk_apb_dualtimers/cmsdk_apb_dualtimers_defs.v"
add_design -verilog "mcu_system/cmsdk_apb_dualtimers/cmsdk_apb_dualtimers_frc.v"
add_design -verilog "mcu_system/cmsdk_apb_dualtimers/cmsdk_apb_dualtimers.v"
add_design -verilog "mcu_system/cmsdk_apb_uart/cmsdk_apb_uart.v"
add_design -verilog "mcu_system/cmsdk_apb_watchdog/cmsdk_apb_watchdog_defs.v"
add_design -verilog "mcu_system/cmsdk_apb_watchdog/cmsdk_apb_watchdog_frc.v"
add_design -verilog "mcu_system/cmsdk_apb_watchdog/cmsdk_apb_watchdog.v"
add_design -verilog "mcu_system/cmsdk_axi2apb_subsystem/cmsdk_irq_sync.v"
add_design -verilog "mcu_system/cmsdk_axi2apb_subsystem/cmsdk_apb_test_slave.v"
add_design -verilog "mcu_system/cmsdk_axi2apb_subsystem/cmsdk_axi2apb_subsystem.v"

add_design -verilog "mcu_system/cmsdk_axi_gpio/cmsdk_axi_gpio.v"
add_design -verilog "mcu_system/cmsdk_axi_gpio/cmsdk_axi_to_iop.v"
add_design -verilog "mcu_system/cmsdk_axi_gpio/cmsdk_iop_gpio.v"

add_design -verilog "mcu_system/cmsdk_mcu_system/cmsdk_axi_default_slave.v"
add_design -verilog "mcu_system/cmsdk_mcu_system/cmsdk_axi_sysrom_table.v"
add_design -verilog "mcu_system/cmsdk_mcu_system/cmsdk_axi_sysctrl.v"
add_design -verilog "mcu_system/cmsdk_mcu_system/cmsdk_mcu_system.v"

# Firmware image: rebuild + sync image.hex + image_words.hex (board build, 9600)
exec mingw32-make -B -C software
file copy -force software/hello_soc.hex image.hex
exec python -X utf8 tools/hex2words.py image.hex image_words.hex

add_constraint -logic -fdc "pnr/mes50hp_soc.fdc"

compile -top_module system_top

puts "==== COMPILE DONE ===="
synthesize -ads -selected_syn_tool_opt 2 
dev_map

pnr -force_to_run -gplace_times 4 -parallel 2 -enable_route_constraint_iterate -route_constraint_slack 0.5 -optimize_multi_corner_timing -report_timing
report_timing
gen_bit_stream -force_to_run
