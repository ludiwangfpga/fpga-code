# 用法: vivado -mode batch -source create_project.tcl
set proj_dir [file dirname [file dirname [file normalize [info script]]]]
set part     xc7z100ffg900-2

create_project udp_add_eth $proj_dir/vivado -part $part -force

add_files -norecurse [glob -nocomplain $proj_dir/rtl/eth/*.v]
add_files -norecurse [glob -nocomplain $proj_dir/rtl/eth/arbiter/*.v]
add_files -norecurse [glob -nocomplain $proj_dir/rtl/eth/hash/*.v]
add_files -norecurse [glob -nocomplain $proj_dir/rtl/eth/mem/*.v]
add_files -norecurse $proj_dir/rtl/impedance/part1_config.v
add_files -norecurse $proj_dir/rtl/impedance/cordic_sincos.v
add_files -norecurse $proj_dir/rtl/impedance/cordic_atan2.v
add_files -norecurse $proj_dir/rtl/impedance/fk_engine.v
add_files -norecurse $proj_dir/rtl/impedance/analytical_jacobian_serial.v
add_files -norecurse $proj_dir/rtl/impedance/impedance_torque.v
add_files -norecurse $proj_dir/rtl/impedance/wrench_calc.v
add_files -norecurse $proj_dir/rtl/impedance/cart_impedance.v
add_files -norecurse $proj_dir/rtl/impedance/adaptive_impedance_v2.v
add_files -norecurse $proj_dir/rtl/foc/torque_to_iq.v
add_files -norecurse $proj_dir/rtl/foc/sincos.v
add_files -norecurse $proj_dir/rtl/foc/cartesian2polar.v
add_files -norecurse $proj_dir/rtl/foc/clark_tr.v
add_files -norecurse $proj_dir/rtl/foc/park_tr.v
add_files -norecurse $proj_dir/rtl/foc/pi_controller.v
add_files -norecurse $proj_dir/rtl/foc/hold_detect.v
add_files -norecurse $proj_dir/rtl/foc/svpwm.v
add_files -norecurse $proj_dir/rtl/foc/foc_top.v
add_files -norecurse $proj_dir/rtl/foc/foc_channel.v
add_files -norecurse $proj_dir/rtl/foc/foc_array.v
add_files -norecurse $proj_dir/rtl/top/udp_fk_cmd.v
add_files -norecurse $proj_dir/rtl/top/top_udp_add.v
add_files -norecurse $proj_dir/rtl/top/impedance_control_top.v

add_files -norecurse $proj_dir/ip/clk_wiz_0/clk_wiz_0.xci
add_files -fileset constrs_1 -norecurse $proj_dir/constraints/top_udp_add.xdc
add_files -fileset sim_1 -norecurse $proj_dir/sim/tb_udp_fk_cmd.v

set_property top impedance_control_top  [get_filesets sources_1]
set_property top tb_udp_fk_cmd [get_filesets sim_1]

catch { upgrade_ip [get_ips clk_wiz_0] }
generate_target all [get_files $proj_dir/ip/clk_wiz_0/clk_wiz_0.xci]

puts "PROJECT_CREATED_OK"
