//--------------------------------------------------------------------------//
// AP68030 - tb_ap030_program.sv                                            //
//                                                                          //
// Runs an assembled self-checking program against the processor on a       //
// pin-level MC68030 bus.  Memory is a 32-bit synchronous (STERM) port with  //
// burst support; a 16-bit and an 8-bit asynchronous (DSACK) window exercise //
// dynamic bus sizing.  The program reports through memory-mapped registers: //
//   $F100 word  failing test number                                          //
//   $F102 word  $BAD0 = failed, $600D = all tests passed                     //
//   $F110 word  interrupt request level (0 releases IPL)                     //
//   $F112 word  interrupt vector for the next IACK (0 = autovector)          //
//   $F120 byte  write must arrive with FC = 1 (MOVES/DFC check)             //
//   $F130 long  bus error trigger address (0 disables)                       //
//   $F140 word  wait states for the memory port                             //
//   $F170 word  bit 0 asserts MMUDIS, bit 1 asserts CDIS                      //
//   $F174 long  read: bus cycles run with CIOUT asserted                     //
//   $F178 long  read: bus cycles                                             //
//   $F17C long  read: last effective address the coprocessor model received //
//   $F180 word  coprocessor save CIR format word (read/write)               //
//   $F184 word  read: last control CIR value; $F188/$F18C: operands 0 and 1  //
// Memory map (24-bit decode):                                               //
//   $000000-$0FFFFF  RAM, 32-bit synchronous, burst                          //
//   $200000-$2FFFFF  RAM alias, 16-bit asynchronous                          //
//   $300000-$3FFFFF  RAM alias, 8-bit asynchronous                          //
//   $400000-$4FFFFF  RAM alias, 32-bit asynchronous, CIIN asserted           //
//   $F00000-$F0FFFF  test registers (8-bit lanes on D31-D24, 32-bit port)    //
//   $FF0000-$FFFFFF  bus error                                               //
// Loaded with +prog=<hex>; +waits=<n> sets the initial wait states;         //
// +expect_halt passes when the processor halts (double bus fault test).     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap030_defs.svh"

module tb_ap030_program;

reg clk = 0;
always #10 clk = ~clk;
reg reset_n = 0;

wire [31:0] a, d_o;
wire  [2:0] fc;
wire  [1:0] siz;
wire        rw, rmc_n, as_n, ds_n, dben_n, ecs_n, ocs_n, ciout_n, cbreq_n, bus_oe, d_oe, bg_n, ipend_n;
wire        reset_n_oe, refill_n, status_n, dbg_halted, dbg_inst;
integer     insts = 0;
always @(posedge clk) if (dbg_inst) insts = insts + 1;
wire [31:0] dbg_pc;
wire [15:0] dbg_sr;
wire  [7:0] dbg_state;
reg  [31:0] d_i;
reg         dsack0_n = 1, dsack1_n = 1, sterm_n = 1, berr_n = 1, halt_n = 1, avec_n = 1, ciin_n = 1, cback_n = 1;
reg         br_n = 1, bgack_n = 1, cdis_n = 1, mmudis_n = 1;
reg   [2:0] ipl_n = 3'b111;

ap030_top dut (
	.clk(clk),
	.a(a), .fc(fc), .siz(siz), .rw(rw), .rmc_n(rmc_n), .as_n(as_n), .ds_n(ds_n), .dben_n(dben_n),
	.ecs_n(ecs_n), .ocs_n(ocs_n), .ciout_n(ciout_n), .cbreq_n(cbreq_n), .bus_oe(bus_oe),
	.d_o(d_o), .d_oe(d_oe), .d_i(d_i),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(sterm_n), .berr_n(berr_n), .halt_n(halt_n),
	.avec_n(avec_n), .ciin_n(ciin_n), .cback_n(cback_n), .br_n(br_n), .bg_n(bg_n), .bgack_n(bgack_n),
	.ipl_n(ipl_n), .ipend_n(ipend_n), .reset_n_i(reset_n), .reset_n_oe(reset_n_oe),
	.cdis_n(cdis_n), .mmudis_n(mmudis_n), .refill_n(refill_n), .status_n(status_n),
	.dbg_pc(dbg_pc), .dbg_sr(dbg_sr), .dbg_state(dbg_state), .dbg_halted(dbg_halted), .dbg_inst(dbg_inst)
);

//---------------------------------------------------------------------------
// memory
//---------------------------------------------------------------------------
reg [7:0] mem [0:(1<<20)-1];
integer wait_states = 0;
integer errors = 0;
integer clocks = 0;
always @(posedge clk) clocks = clocks + 1;

wire as_asserted = ~as_n & bus_oe;
wire ds_asserted = ~ds_n & bus_oe;
wire [3:0] region = a[23:20];
wire is_ram32   = (region == 4'h0);
wire is_ram16   = (region == 4'h2);
wire is_ram8    = (region == 4'h3);
wire is_ram32a  = (region == 4'h4);
wire is_regs    = (region == 4'hF) && (a[19:16] == 4'h0);
wire is_berr    = (region == 4'hF) && (a[19:16] == 4'hF);
wire is_iack    = (fc == 3'd7) && (a[19:16] == 4'hF);
wire is_sync    = is_ram32 && (fc != 3'd7);
wire is_async   = !is_sync;
wire [19:0] ma  = a[19:0];

integer as_cnt = 0;
always @(posedge clk) as_cnt <= as_asserted ? as_cnt + 1 : 0;

// test registers
reg  [2:0] irq_level = 0;
reg  [7:0] irq_vector = 0;      // 0 = AVEC, $FF = BERR (spurious), else the vector
reg [31:0] berr_addr = 0;
reg [15:0] irq_delay = 0;       // clocks until irq_delay_level is applied
reg  [2:0] irq_delay_level = 0;
reg [15:0] bkpt_op = 0;         // breakpoint acknowledge: 0 = BERR, else the opcode
integer    ciout_cycles = 0;    // bus cycles run with CIOUT asserted
integer    bus_cycles = 0;      // bus cycles (AS assertions)
reg        as_was = 0;
always @(posedge clk) begin
	as_was <= as_asserted;
	if (as_was && !as_asserted) begin
		bus_cycles <= bus_cycles + 1;
		if (ciout_seen) ciout_cycles <= ciout_cycles + 1;
	end
end
reg ciout_seen = 0;
always @(negedge clk) if (as_asserted) ciout_seen <= ~ciout_n; else ciout_seen <= 0;
integer    reset_len = 0;       // length of the last RESET instruction pulse
integer    reset_cnt = 0;
reg        berr_hit;
always @* berr_hit = (berr_addr != 0) && as_asserted && ({a[31:2], 2'b00} == {berr_addr[31:2], 2'b00}) && (fc != 3'd7);
always @* ipl_n = ~irq_level;
always @(posedge clk) begin
	if (irq_delay != 0) begin
		irq_delay <= irq_delay - 1;
		if (irq_delay == 1) irq_level <= irq_delay_level;
	end
	if (reset_n_oe) reset_cnt <= reset_cnt + 1;
	else if (reset_cnt != 0) begin reset_len <= reset_cnt; reset_cnt <= 0; end
end
// CPU space (UM 7.4): interrupt acknowledge, breakpoint acknowledge, coprocessor
wire is_cpu    = (fc == 3'd7);
wire is_bkpt   = is_cpu && (a[19:16] == 4'h0);
wire is_cpsp   = is_cpu && (a[19:16] == 4'h2);
wire is_cp1    = is_cpsp && (a[15:13] == 3'd1);   // coprocessor 1: the model below

// synchronous port: STERM after the wait states, burst with CBACK
wire sync_ready = is_sync && bus_oe && (as_cnt >= wait_states);
reg  [1:0] beat_idx = 0;
reg        sterm_rec = 0;
always @* begin
	sterm_n = 1; cback_n = 1;
	if (is_sync && bus_oe && !as_n) begin
		sterm_n = ~sync_ready;
		cback_n = 0;
	end
end
always @(posedge clk) sterm_rec <= ~sterm_n & as_asserted & rw & (~cbreq_n | burst_active);
reg burst_active = 0;
always @(negedge clk) begin
	if (!as_asserted) begin beat_idx <= a[3:2]; burst_active <= 0; end
	else if (sterm_rec) begin burst_active <= 1; beat_idx <= beat_idx + 1; end
end

// asynchronous ports
always @* begin
	dsack0_n = 1; dsack1_n = 1; berr_n = 1; avec_n = 1; ciin_n = 1;
	if (as_asserted && (wait_states == 0 || as_cnt >= wait_states + 1) && is_async) begin
		if (is_iack) begin
			if (irq_vector == 0) avec_n = 0;
			else if (irq_vector == 8'hFF) berr_n = 0;
			else {dsack1_n, dsack0_n} = 2'b00;
		end else if (is_bkpt) begin
			if (bkpt_op == 0) berr_n = 0; else {dsack1_n, dsack0_n} = 2'b00;
		end else if (is_cp1) begin
			{dsack1_n, dsack0_n} = 2'b00;
		end else if (is_cpu) berr_n = 0;
		else if (is_berr || berr_hit) berr_n = 0;
		else if (is_ram16) {dsack1_n, dsack0_n} = 2'b01;
		else if (is_ram8) {dsack1_n, dsack0_n} = 2'b10;
		else {dsack1_n, dsack0_n} = 2'b00;
		if (is_ram32a || is_regs) ciin_n = 0;      // I/O and the CIIN window are not cachable
	end
	if (berr_hit && is_sync && as_asserted) begin berr_n = 0; end
end

// read data: the full port width
reg [31:0] rdata;
wire [19:0] lw_base = {ma[19:2], 2'b00};
always @* begin
	rdata = 32'hxxxxxxxx;
	if (is_iack) rdata = {24'd0, irq_vector};
	else if (is_bkpt) rdata = {bkpt_op, 16'd0};
	else if (is_cp1) rdata = cp_rdata;
	else if (is_regs) begin
		case (a[7:0])
			8'h10: rdata = {13'd0, irq_level, 16'd0};
			8'h40: rdata = {wait_states[15:0], 16'd0};
			8'h50: rdata = clocks[31:0];
			8'h60: rdata = reset_len[31:0];
			8'h74: rdata = ciout_cycles[31:0];
			8'h78: rdata = bus_cycles[31:0];
			8'h7C: rdata = cp_last_ea;
			8'h80: rdata = {cp_save_fmt, 16'd0};
			8'h84: rdata = {cp_ctrl, 16'd0};
			8'h88: rdata = cp_operand[0];
			8'h8C: rdata = cp_operand[1];
			default: rdata = 32'd0;
		endcase
	end else if (is_sync) rdata = {mem[{ma[19:4], beat_idx, 2'b00}], mem[{ma[19:4], beat_idx, 2'b01}],
	                               mem[{ma[19:4], beat_idx, 2'b10}], mem[{ma[19:4], beat_idx, 2'b11}]};
	else if (is_ram16) rdata = {mem[{ma[19:1], 1'b0}], mem[{ma[19:1], 1'b1}], 16'hxxxx};
	else if (is_ram8) rdata = {mem[ma], 24'hxxxxxx};
	else rdata = {mem[lw_base], mem[lw_base+1], mem[lw_base+2], mem[lw_base+3]};
end
always @* d_i = rw ? rdata : 32'hxxxxxxxx;

// writes with the byte enables of UM Table 7-7
function [3:0] byte_en;
	input [1:0] p; input [1:0] s; input [1:0] o;
	reg [2:0] n; reg [2:0] cap;
	begin
		n = (s == `SIZ_BYTE) ? 3'd1 : (s == `SIZ_WORD) ? 3'd2 : (s == `SIZ_3BYTE) ? 3'd3 : 3'd4;
		case (p)
			`PORT_32: begin cap = 3'd4 - {1'b0,o}; if (n < cap) cap = n; byte_en = ((4'b1111 >> (4 - cap)) << o); end
			`PORT_16: begin cap = 3'd2 - {2'b0,o[0]}; if (n < cap) cap = n; byte_en = (cap == 3'd2) ? 4'b0011 : (o[0] ? 4'b0010 : 4'b0001); end
			default: byte_en = 4'b0001;
		endcase
	end
endfunction
task capture_write;
	reg [3:0] be; reg [19:0] base; reg [1:0] p; integer i;
	begin
		p = is_ram16 ? `PORT_16 : is_ram8 ? `PORT_8 : `PORT_32;
		be = byte_en(p, siz, a[1:0]);
		case (p)
			`PORT_32: base = {ma[19:2], 2'b00};
			`PORT_16: base = {ma[19:1], 1'b0};
			default:  base = ma;
		endcase
		if ($test$plusargs("bustrace")) $display("%8d WR a=%08x fc=%0d siz=%b be=%b d=%08x", clocks, a, fc, siz, be, d_o);
		if (is_cp1) cp_write(a[4:0], be, d_o);
		else if (is_regs) begin
			// registers: byte lanes per address (byte 0 on D31-D24)
			for (i = 0; i < 4; i = i + 1) if (be[i]) reg_write({a[7:2], 2'b00} + i[7:0], d_o[31 - 8*i -: 8]);
		end else begin
			for (i = 0; i < 4; i = i + 1) if (be[i]) mem[base + i] = d_o[31 - 8*i -: 8];
		end
	end
endtask
reg [15:0] fail_num = 0;
reg        done = 0;
task reg_write;
	input [7:0] off; input [7:0] v;
	begin
		case (off)
			8'h00: fail_num[15:8] = v;
			8'h01: fail_num[7:0] = v;
			8'h02: ;
			8'h03: begin
				if (v == 8'h0D) begin done = 1; $display("program reports: ALL TESTS PASSED (%0d clocks, %0d instructions, %0.2f clocks per instruction)", clocks, insts, $itor(clocks) / $itor(insts)); end
				else if (v == 8'hD0) begin done = 1; errors = errors + 1; $display("program reports: TEST FAILED number %0d", fail_num); end
			end
			8'h11: irq_level = v[2:0];
			8'h13: irq_vector = v;
			8'h14: irq_delay[15:8] = v;
			8'h15: irq_delay[7:0] = v;
			8'h17: irq_delay_level = v[2:0];
			8'h18: bkpt_op[15:8] = v;
			8'h19: bkpt_op[7:0] = v;
			8'h20: if (fc != 3'd1) begin errors = errors + 1; $display("FAIL: $F120 written with FC=%0d", fc); end
			8'h30: berr_addr[31:24] = v;
			8'h31: berr_addr[23:16] = v;
			8'h32: berr_addr[15:8] = v;
			8'h33: berr_addr[7:0] = v;
			8'h41: wait_states = v;
			8'h71: begin mmudis_n = ~v[0]; cdis_n = ~v[1]; end
			8'h80: cp_save_fmt[15:8] = v;
			8'h81: cp_save_fmt[7:0] = v;
			default: ;
		endcase
	end
endtask
// a write is captured once per cycle (wait states keep DS asserted for
// several clocks; the coprocessor model's operand list must not repeat)
reg wr_seen = 0;
always @(posedge clk) begin
	if (is_async && as_asserted && ds_asserted && !rw && d_oe && !wr_seen) begin capture_write; wr_seen <= 1; end
	if (!as_asserted) wr_seen <= 0;
end
`include "tb_cp_model.svh"
always @(posedge clk) if ($test$plusargs("bustrace") && as_asserted && rw && (!sterm_n || !dsack0_n || !dsack1_n))
	$display("%8d RD a=%08x fc=%0d siz=%b d=%08x burst=%0d", clocks, a, fc, siz, rdata, burst_active);
always @(negedge clk) if (is_sync && as_asserted && !sterm_n && !rw && d_oe) capture_write;

//---------------------------------------------------------------------------
// run
//---------------------------------------------------------------------------
reg [1023:0] progfile;
integer i, w;
initial begin
	for (i = 0; i < (1<<20); i = i + 1) mem[i] = 8'h00;
	if (!$value$plusargs("prog=%s", progfile)) begin
		$display("TEST FAILED: no +prog= given"); $finish;
	end
	$readmemh(progfile, mem);
	if ($value$plusargs("waits=%d", w)) wait_states = w;
	repeat (20) @(posedge clk);
	reset_n = 1;
	while (!done && !dbg_halted) @(posedge clk);
	repeat (10) @(posedge clk);
	if (dbg_halted) begin
		// +expect_halt: the program ends in a double bus fault on purpose
		if ($test$plusargs("expect_halt")) $display("program reports: processor halted as expected (%0d clocks)", clocks);
		else begin errors = errors + 1; $display("FAIL: processor halted (double bus fault) at pc %08x state %0d", dbg_pc, dbg_state); end
	end else if ($test$plusargs("expect_halt")) begin
		errors = errors + 1; $display("FAIL: the processor did not halt");
	end
	if (errors != 0) begin
		$display("registers: D0-D7 %08x %08x %08x %08x %08x %08x %08x %08x", dut.core.rf.r[0], dut.core.rf.r[1], dut.core.rf.r[2], dut.core.rf.r[3],
		         dut.core.rf.r[4], dut.core.rf.r[5], dut.core.rf.r[6], dut.core.rf.r[7]);
		$display("           A0-A7 %08x %08x %08x %08x %08x %08x %08x %08x  sr %04x", dut.core.rf.r[8], dut.core.rf.r[9], dut.core.rf.r[10], dut.core.rf.r[11],
		         dut.core.rf.r[12], dut.core.rf.r[13], dut.core.rf.r[14], dut.core.rf.a7, dut.core.sr);
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else $display("TEST FAILED: %0d errors", errors);
	$finish;
end

// sequencer state every clock with +strace
always @(posedge clk) if ($test$plusargs("strace")) $display("%8d st=%0d pc=%08x ds=%0d is=%0d wb=%b breq=%b bbusy=%b bdone=%b own=%0d bst=%0d as=%b", clocks, dbg_state, dbg_pc, dut.memsys.ds, dut.memsys.is,
	dut.memsys.wb_valid, dut.memsys.b_req, dut.memsys.b_busy, dut.memsys.b_done, dut.memsys.owner, dut.memsys.bus.bst, ~as_n);

// instruction cache activity with +itrace
always @(posedge clk) if ($test$plusargs("itrace")) begin
	if (dut.memsys.ic_fi_we) $display("%8d IC-FILL a=%08x fc=%0d d=%08x", clocks, {dut.memsys.ic_fi_addr, 2'b00}, dut.memsys.ic_fi_fc, dut.memsys.ic_fi_data);
	if (dut.memsys.ilk_act) $display("%8d IC-LK la=%08x hit=%b tag=%b en=%b is=%0d", clocks, dut.memsys.ci_addr, dut.memsys.ic_hit, dut.memsys.ic_tag_hit, dut.memsys.ic_en, dut.memsys.is);
	if (dut.core.cacr_ci || dut.core.cacr_cei) $display("%8d IC-CLEAR ci=%b cei=%b caar=%08x", clocks, dut.core.cacr_ci, dut.core.cacr_cei, dut.core.caar);
end
// data cache activity with +ctrace
always @(posedge clk) if ($test$plusargs("ctrace")) begin
	if (dut.memsys.dc_wr_we) $display("%8d DC-WR la=%08x fc=%0d be=%b d=%08x hit=%b tag=%b store=%b", clocks, dut.memsys.dc_wr_la, dut.memsys.dc_wr_fc,
		dut.memsys.dc_wr_be, dut.memsys.dc_wr_data, dut.memsys.dcache.wr_hit, dut.memsys.dcache.wr_tag_hit, dut.memsys.dcache.wr_store);
	if (dut.memsys.dc_fi_we) $display("%8d DC-FILL a=%08x fc=%0d d=%08x", clocks, {dut.memsys.dc_fi_addr, 2'b00}, dut.memsys.dc_fi_fc, dut.memsys.dc_fi_data);
	if (dut.memsys.lk_act) $display("%8d DC-LK la=%08x rw=%b hit=%b tag=%b ds=%0d", clocks, dut.memsys.c_addr, dut.memsys.d_rw, dut.memsys.dc_hit, dut.memsys.dc_tag_hit, dut.memsys.ds);
	if (dut.memsys.ds == 3) $display("%8d DC-HIT data=%08x", clocks, dut.memsys.dc_data);
end
// exception entries with +trace
reg [7:0] exc_prev = 0;
always @(posedge clk) begin
	if ($test$plusargs("trace") && dut.core.state == 8'd74 && exc_prev != 8'd74)
		$display("%8d EXC vec=%0d fmt=%0h pc=%08x ia=%08x sr=%04x ssw=%04x fa=%08x rk=%0d rs=%0d", clocks, dut.core.exc_vec, dut.core.exc_fmt, dut.core.exc_pc, dut.core.exc_ia, dut.core.exc_sr, dut.core.exc_ssw, dut.core.exc_fa, dut.core.exc_rk, dut.core.exc_rs);
	exc_prev <= dut.core.state;
end
// trace of instruction starts with +trace (every dispatch changes pc_i)
reg [31:0] pc_prev = 32'hFFFFFFFF;
reg [7:0] prev_state = 0;
always @(posedge clk) begin
	if ($test$plusargs("trace") && dut.core.pc_i != pc_prev)
		$display("%8d pc=%08x ir=%04x sr=%04x d0=%08x a7=%08x", clocks, dut.core.pc_i, dut.core.ir, dut.core.sr,
		         dut.core.rf.r[0], dut.core.rf.a7);
	pc_prev <= dut.core.pc_i;
	if ($test$plusargs("rftrace") && dut.core.rf_we)
		$display("%8d RF[%0d] <= %08x", clocks, dut.core.rf_waddr, dut.core.rf_wdata);
	prev_state <= dut.core.state;
end

initial begin
	#200000000;
	$display("TEST FAILED: timeout at pc %08x state %0d", dbg_pc, dbg_state);
	$finish;
end

endmodule
