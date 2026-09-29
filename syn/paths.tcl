project_open ap68030
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -npaths 12 -detail path_only -panel_name "Worst setup" -file worst_setup.txt
report_timing -setup -npaths 400 -detail summary -file worst_setup_summary.txt
delete_timing_netlist
project_close
