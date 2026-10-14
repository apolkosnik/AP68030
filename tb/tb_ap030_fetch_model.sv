// Lazy-prefetch comparison boundary test. From the repository root:
// $ verilator --binary --timing -Wno-fatal -Wno-PINMISSING -Irtl -Irtl/core \
//   --top-module tb_ap030_fetch_model --Mdir /tmp/opencode/ap030-fetch-model \
//   tb/tb_ap030_fetch_model.sv rtl/ap030_core.v rtl/ap030_regfile.v \
//   rtl/ap030_alu.v rtl/ap030_muldiv.v
// /tmp/opencode/ap030-fetch-model/Vtb_ap030_fetch_model
module tb_ap030_fetch_model;
reg [31:0] stop_addr, scanned;
reg stop_valid, scan_valid;
ap030_core dut (
	.clk(1'b0), .ce(1'b0), .ce_f(1'b0), .rst(1'b0),
	.fetch_stop_v(stop_valid), .fetch_stop(stop_addr),
	.fetch_scan_v(scan_valid), .fetch_scan_to(scanned)
);

// Independent widened-arithmetic oracle: neither threshold addition may
// wrap, whereas advancing the 32-bit instruction address must wrap.
function automatic expected(input [31:0] addr, input [31:0] next_fetch);
	reg [32:0] a, stop_at, scan_to;
	begin
		a = {1'b0, addr}; stop_at = {1'b0, stop_addr}; scan_to = {1'b0, scanned};
		expected = (!stop_valid || a + 33'd2 < stop_at) &&
		           (!scan_valid || scanned >= next_fetch || a + 33'd4 <= scan_to);
	end
endfunction

integer checks = 0;
task automatic check_window(input [31:0] scan_pc, input [31:0] saved, input [31:0] next_fetch);
	reg [31:0] after_pop;
	begin
		dut.scan_pc = scan_pc;
		dut.due_scan = saved;
		dut.fetch_pc = next_fetch;
		#1;
		for (integer n = 0; n < 5; n = n + 1) begin
			after_pop = (n == 4) ? saved : scan_pc + 32'(2 * n);
			if (dut.fetch_model_allow[n] !== expected(after_pop, next_fetch))
				$fatal(1, "fetch model mismatch pop=%0d scan=%08x saved=%08x next=%08x stop=%08x scanned=%08x flags=%b%b",
				       n, scan_pc, saved, next_fetch, stop_addr, scanned, stop_valid, scan_valid);
			checks = checks + 1;
		end
	end
endtask

reg [31:0] edge_addr [0:15];
initial begin
	edge_addr[0] = 0; edge_addr[1] = 1; edge_addr[2] = 2; edge_addr[3] = 3;
	edge_addr[4] = 4; edge_addr[5] = 5; edge_addr[6] = 6; edge_addr[7] = 7;
	edge_addr[8] = 32'h7ffffffe; edge_addr[9] = 32'h80000000;
	edge_addr[10] = 32'hfffffffa; edge_addr[11] = 32'hfffffffb;
	edge_addr[12] = 32'hfffffffc; edge_addr[13] = 32'hfffffffd;
	edge_addr[14] = 32'hfffffffe; edge_addr[15] = 32'hffffffff;
	for (integer flags = 0; flags < 4; flags = flags + 1) begin
		{stop_valid, scan_valid} = 2'(flags);
		for (integer a = 0; a < 16; a = a + 1)
			for (integer b = 0; b < 16; b = b + 1)
				for (integer c = 0; c < 16; c = c + 1) begin
					stop_addr = edge_addr[b]; scanned = edge_addr[c];
					check_window(edge_addr[a], edge_addr[(a+b+c) % 16], edge_addr[(a+c+7) % 16]);
				end
	end
	for (integer n = 0; n < 20000; n = n + 1) begin
		{stop_valid, scan_valid} = 2'($urandom);
		stop_addr = $urandom; scanned = $urandom;
		check_window($urandom, $urandom, $urandom);
	end
	$display("ALL TESTS PASSED (%0d lazy-prefetch boundary/random checks)", checks);
	$finish;
end
endmodule
