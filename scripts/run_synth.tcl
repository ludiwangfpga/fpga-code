# 用法: vivado -mode batch -source run_synth.tcl
# part 需与 create_project.tcl 保持一致
set proj_dir [file dirname [file dirname [file normalize [info script]]]]
set part xc7z100ffg900-2
set vsrc {}
foreach d {rtl/eth rtl/eth/arbiter rtl/eth/hash rtl/eth/mem} {
    set vsrc [concat $vsrc [glob -nocomplain $proj_dir/$d/*.v]]
}
if {[llength $vsrc] == 0} { puts "SYNTH_FAILED: no eth sources"; exit 1 }
read_verilog $vsrc
read_verilog $proj_dir/rtl/impedance/part1_config.v
read_verilog $proj_dir/rtl/impedance/cordic_sincos.v
read_verilog $proj_dir/rtl/impedance/cordic_atan2.v
read_verilog $proj_dir/rtl/impedance/fk_engine.v
read_verilog $proj_dir/rtl/impedance/analytical_jacobian_serial.v
read_verilog $proj_dir/rtl/impedance/impedance_torque.v
read_verilog $proj_dir/rtl/impedance/wrench_calc.v
read_verilog $proj_dir/rtl/impedance/cart_impedance.v
read_verilog $proj_dir/rtl/impedance/adaptive_impedance_v2.v
read_verilog $proj_dir/rtl/foc/torque_to_iq.v
read_verilog $proj_dir/rtl/foc/sincos.v
read_verilog $proj_dir/rtl/foc/cartesian2polar.v
read_verilog $proj_dir/rtl/foc/clark_tr.v
read_verilog $proj_dir/rtl/foc/park_tr.v
read_verilog $proj_dir/rtl/foc/pi_controller.v
read_verilog $proj_dir/rtl/foc/hold_detect.v
read_verilog $proj_dir/rtl/foc/svpwm.v
read_verilog $proj_dir/rtl/foc/foc_top.v
read_verilog $proj_dir/rtl/foc/foc_channel.v
read_verilog $proj_dir/rtl/foc/foc_array.v
read_verilog $proj_dir/rtl/top/udp_fk_cmd.v
read_verilog $proj_dir/rtl/top/top_udp_add.v
read_verilog $proj_dir/rtl/top/impedance_control_top.v
read_ip      $proj_dir/ip/clk_wiz_0/clk_wiz_0.xci
if {[catch {synth_ip [get_ips clk_wiz_0]} e]} { puts "SYNTH_FAILED: ip $e"; exit 1 }
read_xdc     $proj_dir/constraints/top_udp_add.xdc
if {[catch {synth_design -top impedance_control_top -part $part} e]} { puts "SYNTH_FAILED: $e"; exit 1 }
write_checkpoint -force $proj_dir/top_udp_add_synth.dcp
report_utilization -hierarchical
puts "SYNTH_OK"
