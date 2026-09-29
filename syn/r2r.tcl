project_open ap68030
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -from [get_registers *] -to [get_registers *] -npaths 30 -detail summary -file worst_r2r_summary.txt
report_timing -setup -from [get_registers *] -to [get_registers *] -npaths 6 -detail full_path -file worst_r2r.txt
delete_timing_netlist
project_close
