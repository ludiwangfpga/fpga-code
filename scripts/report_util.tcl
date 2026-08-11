set proj_dir [file dirname [file dirname [file normalize [info script]]]]
open_checkpoint $proj_dir/top_udp_add_routed.dcp
report_utilization -file $proj_dir/util_flat.rpt
report_utilization -hierarchical -file $proj_dir/util_hier.rpt
puts "UTIL_DONE"
