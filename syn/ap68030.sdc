# AP68030 timing constraints: 50 MHz processor clock
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty
# asynchronous inputs are sampled on the falling edge by the bus controller
set_input_delay  -clock clk -max 2.0 [all_inputs]
set_input_delay  -clock clk -min 0.0 [all_inputs]
set_output_delay -clock clk -max 2.0 [all_outputs]
set_output_delay -clock clk -min 0.0 [all_outputs]
set_false_path -from [get_ports {reset_n_i cdis_n mmudis_n ipl_n[*] br_n bgack_n}]
