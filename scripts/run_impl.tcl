# 用法: vivado -mode batch -source run_impl.tcl
# 单进程非工程流程: synth -> opt -> place -> phys_opt -> route -> phys_opt -> bitstream
# （phys_opt_design 抢救 WNS; 关键路径还靠流水化: FK 矩阵乘 + 雅可比叉乘均拆多拍）
# （避免 launch_runs/launch_simulation 在本机派生子进程导致 Broken pipe）
# part 需与 create_project.tcl 保持一致
set proj_dir [file dirname [file dirname [file normalize [info script]]]]
set part xc7z100ffg900-2
set vsrc {}
foreach d {rtl/eth rtl/eth/arbiter rtl/eth/hash rtl/eth/mem} {
    set vsrc [concat $vsrc [glob -nocomplain $proj_dir/$d/*.v]]
}
if {[llength $vsrc] == 0} { puts "IMPL_FAILED: no eth sources"; exit 1 }
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
if {[catch {synth_ip [get_ips clk_wiz_0]} e]} { puts "IMPL_FAILED: ip $e"; exit 1 }
read_xdc     $proj_dir/constraints/top_udp_add.xdc
if {[catch {synth_design -top impedance_control_top -part $part} e]} { puts "IMPL_FAILED: synth $e"; exit 1 }
if {[catch {opt_design}   e]} { puts "IMPL_FAILED: opt $e";   exit 1 }
if {[catch {place_design} e]} { puts "IMPL_FAILED: place $e"; exit 1 }
if {[catch {phys_opt_design} e]} { puts "PHYS_OPT_POSTPLACE_SKIP: $e" }
if {[catch {route_design} e]} { puts "IMPL_FAILED: route $e"; exit 1 }
if {[catch {phys_opt_design} e]} { puts "PHYS_OPT_POSTROUTE_SKIP: $e" }
report_timing_summary -file $proj_dir/timing_summary.rpt
if {[catch {set wns [get_property SLACK [lindex [get_timing_paths -max_paths 1 -nworst 1 -setup] 0]]} e]} { set wns "n/a" }
puts "WNS=$wns"
# FOC 的 PWM/ADC/编码器引脚本阶段未绑真实管脚(仅仿真/监控,接真电机时补 xdc)。
# 把未约束端口的 DRC(UCIO-1)与未指定 I/O 标准(NSTD-1)降为 warning,允许出 bit。
catch {set_property SEVERITY {Warning} [get_drc_checks UCIO-1]}
catch {set_property SEVERITY {Warning} [get_drc_checks NSTD-1]}
write_checkpoint -force $proj_dir/top_udp_add_routed.dcp
if {[catch {write_bitstream -force $proj_dir/top_udp_add.bit} e]} { puts "IMPL_FAILED: bit $e"; exit 1 }
puts "BITSTREAM_OK"
