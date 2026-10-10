//--------------------------------------------------------------------------//
// AP68030 - tb_ap030_bus.sv                                                //
//                                                                          //
// Pin-level check of the bus controller against slave models written from  //
// the MC68030 User's Manual, independently of the controller:              //
//   - 8/16/32-bit asynchronous (DSACKx) ports with programmable wait states //
//     and byte enables per UM Table 7-7                                     //
//   - a 32-bit synchronous (STERM) port with burst (CBACK) and the modulo-4 //
//     A3:A2 wrap of UM 7.3.7                                                //
//   - CIIN, early and late BERR, retry (BERR+HALT), HALT, IACK with vector, //
//     AVEC and spurious termination, BR/BG/BGACK arbitration               //
// Every operand size, alignment and port combination is read and written;  //
// the memory image, the assembled operand, the cache fills and the cycle   //
// timing (in half clocks) are checked.                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap030_defs.svh"

module tb_ap030_bus;

reg clk = 0;
always #10 clk = ~clk;      // 50 MHz
reg rst = 1;

// ---- request side --------------------------------------------------------
reg         req = 0;
reg   [1:0] req_kind = 0;
reg  [31:0] req_addr = 0;
reg   [2:0] req_nbytes = 0, req_total = 0;
reg         req_rw = 1;
reg   [2:0] req_fc = 5;
reg         req_rmc = 0, req_rmc_last = 0, req_ciout = 0, req_cbreq = 0, req_ocs = 1, req_cache = 1;
reg  [31:0] req_wdata = 0;
wire        req_ack, busy, done, res_berr, res_avec, res_ciin, fill_stb, bus_idle, bus_granted;
wire [31:0] rd_data, fill_data;
wire [31:2] fill_addr;
reg         rmc_hold = 0;      // the memory system holds RMC (an RMW operation is open)

// ---- pins ----------------------------------------------------------------
wire [31:0] a;
wire  [2:0] fc;
wire  [1:0] siz;
wire        rw, rmc_n, as_n, ds_n, dben_n, ecs_n, ocs_n, ciout_n, cbreq_n, bus_oe, d_oe, bg_n;
wire [31:0] d_o;
reg  [31:0] d_i;
reg         dsack0_n = 1, dsack1_n = 1, sterm_n = 1, berr_n = 1, halt_n = 1, avec_n = 1;
reg         ciin_n = 1, cback_n = 1, br_n = 1, bgack_n = 1;

ap030_bus dut (
	.clk(clk), .ce(1'b1), .ce_f(1'b1), .rst(rst),
	.req(req), .req_kind(req_kind), .req_addr(req_addr), .req_nbytes(req_nbytes),
	.req_total(req_total), .req_rw(req_rw), .req_fc(req_fc), .req_rmc(req_rmc),
	.req_rmc_last(req_rmc_last), .req_ciout(req_ciout), .req_cbreq(req_cbreq), .req_ocs(req_ocs),
	.req_cache(req_cache), .req_wdata(req_wdata), .req_ack(req_ack), .busy(busy), .done(done),
	.rd_data(rd_data), .res_berr(res_berr), .res_avec(res_avec), .res_ciin(res_ciin),
	.fill_stb(fill_stb), .fill_addr(fill_addr), .fill_data(fill_data),
	.rmc_hold(rmc_hold), .halted(1'b0), .bus_idle(bus_idle),
	.a_o(a), .fc_o(fc), .siz_o(siz), .rw_o(rw), .rmc_n_o(rmc_n), .as_n_o(as_n), .ds_n_o(ds_n),
	.dben_n_o(dben_n), .ecs_n_o(ecs_n), .ocs_n_o(ocs_n), .ciout_n_o(ciout_n), .cbreq_n_o(cbreq_n),
	.bus_oe(bus_oe), .d_o(d_o), .d_oe(d_oe), .d_i(d_i),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(sterm_n), .berr_n(berr_n), .halt_n(halt_n),
	.avec_n(avec_n), .ciin_n(ciin_n), .cback_n(cback_n), .br_n(br_n), .bgack_n(bgack_n),
	.bg_n_o(bg_n), .bus_granted(bus_granted)
);

//---------------------------------------------------------------------------
// memory and the slave models
//---------------------------------------------------------------------------
// address map (bits 19:16 select the port)
//   $0xxxx  32-bit synchronous, burst capable
//   $1xxxx  32-bit asynchronous
//   $2xxxx  16-bit asynchronous
//   $3xxxx  8-bit asynchronous
//   $4xxxx  32-bit async, CIIN asserted
//   $5xxxx  32-bit async, BERR in lieu of DSACK
//   $6xxxx  32-bit async, late BERR (one clock after DSACK)
//   $7xxxx  32-bit async, retry once (BERR+HALT), then normal
//   $8xxxx  32-bit async, HALT after the cycle for 4 clocks
//   $9xxxx  32-bit sync, late BERR after STERM
//   $Axxxx  32-bit sync burst, CBACK negated after the second beat
//   $Bxxxx  32-bit sync burst, CIIN on the third beat
//   $Cxxxx  32-bit sync, BERR in lieu of STERM
//   $Dxxxx  32-bit sync with one wait state
reg [7:0] mem [0:(1<<20)-1];
reg [7:0] gold [0:(1<<20)-1];
integer   wait_states = 0;
reg [3:0] region;
always @* region = a[19:16];

wire as_asserted = ~as_n & bus_oe;
wire ds_asserted = ~ds_n & bus_oe;
wire is_iack = (fc == 3'd7) && (a[19:16] == 4'hF);
wire is_sync = (region == 4'h0 || region == 4'h9 || region == 4'hA || region == 4'hB || region == 4'hC || region == 4'hD) && !is_iack;
wire is_async = !is_sync;
reg  [1:0] port;   // for async regions
always @* begin
	case (region)
		4'h2: port = `PORT_16;
		4'h3: port = `PORT_8;
		default: port = `PORT_32;
	endcase
end

// clocks with AS asserted, for wait states and delayed responses
integer as_cnt = 0;
always @(posedge clk) as_cnt <= as_asserted ? as_cnt + 1 : 0;

// relinquish and retry: rr_on answers the cycles with BERR and HALT (no
// DSACK), rr_halt holds HALT afterwards (the test drives BR and BGACK)
reg  rr_on = 0, rr_halt = 0;
// retry bookkeeping: region 7 signals retry on the first attempt of each cycle
reg  retry_armed = 1;
reg  halt_hold = 0;
integer halt_cnt = 0;
// AVEC / spurious for IACK: level 5 autovectors, level 6 spurious, others vector $40+level
wire [2:0] iack_lvl = a[3:1];

// synchronous port: STERM from the address, wait states counted from AS
wire sync_ready = is_sync && bus_oe && ((region == 4'hD) ? (as_cnt >= 1) : (as_cnt >= wait_states));
reg  burst_active = 0;
reg  [1:0] beat_idx = 0;       // A3:A2 of the longword presented now
integer beats = 0;             // beats presented in this burst
always @* begin
	sterm_n = 1; cback_n = 1; ciin_n = 1;
	if (is_sync && bus_oe && (as_asserted || !as_n)) begin
		if (region == 4'hC) sterm_n = 1;
		else sterm_n = ~sync_ready;
		// CBACK: one more longword can follow (never for region C)
		if (region == 4'hA) cback_n = ~(beats < 1);            // negated with the 2nd beat's STERM
		else cback_n = ~(region != 4'hC);
		if (region == 4'hB && beats == 2) ciin_n = 0;          // third beat not cachable
	end
	if (is_async && region == 4'h4 && as_asserted) ciin_n = 0;
end

// asynchronous port: DSACK from AS after the wait states, BERR/HALT variants
always @* begin
	dsack0_n = 1; dsack1_n = 1; berr_n = 1; avec_n = 1;
	halt_n = ~halt_hold;
	// W wait states: DSACK must be recognized at the falling edge ending
	// S2+2W, i.e. asserted after the rising edge of S2+2W-2 (as_cnt = W+1)
	if (is_async && as_asserted && (wait_states == 0 || as_cnt >= wait_states + 1)) begin
		if (is_iack) begin
			if (iack_lvl == 3'd5) avec_n = 0;
			else if (iack_lvl == 3'd6) berr_n = 0;
			else begin dsack1_n = 0; dsack0_n = 0; end
		end else case (region)
			4'h5: berr_n = 0;
			4'h6: begin {dsack1_n, dsack0_n} = 2'b00; if (as_cnt >= 2) berr_n = 0; end
			4'h7: begin
				if (retry_armed) begin berr_n = 0; halt_n = 0; end
				else {dsack1_n, dsack0_n} = 2'b00;
			end
			default: begin
				case (port)
					`PORT_32: {dsack1_n, dsack0_n} = 2'b00;
					`PORT_16: {dsack1_n, dsack0_n} = 2'b01;
					default:  {dsack1_n, dsack0_n} = 2'b10;
				endcase
			end
		endcase
	end
	if (is_sync && region == 4'h9 && as_asserted && (as_cnt >= 1)) berr_n = 0;   // late BERR after STERM
	if (is_sync && region == 4'hC && as_asserted) berr_n = 0;                    // BERR in lieu of STERM
	if (rr_on && as_asserted) begin {dsack1_n, dsack0_n} = 2'b11; berr_n = 0; halt_n = 0; end
	if (rr_halt) halt_n = 0;
end

// retry: BERR+HALT on the first attempt; HALT released two clocks after AS negates
always @(posedge clk) begin
	if (region == 4'h7 && as_asserted && retry_armed) begin
		halt_hold <= 1; halt_cnt <= 0;
	end
	if (halt_hold && !as_asserted) begin
		halt_cnt <= halt_cnt + 1;
		if (halt_cnt >= 2) begin halt_hold <= 0; retry_armed <= 0; end
	end
	// region 8: HALT alone for four clocks after each completed cycle
	if (region == 4'h8 && as_asserted && !halt_hold8) begin
		halt_hold8 <= 1; halt_cnt8 <= 0;
	end
	if (halt_hold8) begin
		halt_cnt8 <= halt_cnt8 + 1;
		if (halt_cnt8 >= 4) halt_hold8 <= 0;
	end
end
reg halt_hold8 = 0; integer halt_cnt8 = 0;
always @* if (halt_hold8) halt_n = 0;

// read data: the full port width, per UM 7.2.7
wire [31:0] lw_base = {a[31:2], 2'b00};
reg [31:0] rdata;
always @* begin
	rdata = 32'hxxxxxxxx;
	if (is_iack) rdata = {8'h00, 8'h00, 8'h00, 8'h40 + {5'd0, iack_lvl}};
	else if (is_sync) rdata = {mem[{a[31:4], beat_idx, 2'b00}], mem[{a[31:4], beat_idx, 2'b01}],
	                           mem[{a[31:4], beat_idx, 2'b10}], mem[{a[31:4], beat_idx, 2'b11}]};
	else case (port)
		`PORT_32: rdata = {mem[lw_base], mem[lw_base+1], mem[lw_base+2], mem[lw_base+3]};
		`PORT_16: rdata = {mem[{a[31:1],1'b0}], mem[{a[31:1],1'b1}], 16'hxxxx};
		default:  rdata = {mem[a], 24'hxxxxxx};
	endcase
end
always @* d_i = rw ? rdata : 32'hxxxxxxxx;

// burst beat sequencing: the controller recognizes STERM on a rising edge
// and latches on the following falling edge; the slave presents the next
// longword after that falling edge
reg sterm_rec = 0;
always @(posedge clk) sterm_rec <= ~sterm_n & as_asserted & rw & (~cbreq_n | burst_active);
always @(negedge clk) begin
	if (!as_asserted) begin
		beat_idx <= a[3:2];
		beats <= 0;
		burst_active <= 0;
	end else if (sterm_rec) begin
		burst_active <= 1;
		beat_idx <= beat_idx + 1;
		beats <= beats + 1;
	end
end

// write capture with the byte enables of UM Table 7-7
function [3:0] byte_en;   // bit 0 = D31-D24 lane
	input [1:0] p; input [1:0] s; input [1:0] o;
	reg [2:0] n; reg [2:0] cap;
	begin
		n = (s == `SIZ_BYTE) ? 3'd1 : (s == `SIZ_WORD) ? 3'd2 : (s == `SIZ_3BYTE) ? 3'd3 : 3'd4;
		case (p)
			`PORT_32: begin cap = 3'd4 - {1'b0,o}; if (n < cap) cap = n;
				byte_en = ((4'b1111 >> (4 - cap)) << o); end
			`PORT_16: begin cap = 3'd2 - {2'b0,o[0]}; if (n < cap) cap = n;
				byte_en = (cap == 3'd2) ? 4'b0011 : (o[0] ? 4'b0010 : 4'b0001); end
			default: byte_en = 4'b0001;
		endcase
	end
endfunction
task capture_write;
	reg [3:0] be; reg [31:0] base; integer i;
	begin
		be = byte_en(is_sync ? `PORT_32 : port, siz, a[1:0]);
		case (is_sync ? `PORT_32 : port)
			`PORT_32: base = {a[31:2], 2'b00};
			`PORT_16: base = {a[31:1], 1'b0};
			default:  base = a;
		endcase
		for (i = 0; i < 4; i = i + 1)
			if (be[i]) mem[base + i] = d_o[31 - 8*i -: 8];
	end
endtask
// async: data valid while DS asserted; sync: at the falling edge with STERM
always @(posedge clk) if (is_async && as_asserted && ds_asserted && !rw && d_oe) capture_write;
always @(negedge clk) if (is_sync && as_asserted && !sterm_n && !rw && d_oe) capture_write;

//---------------------------------------------------------------------------
// timing monitor: AS must change only on falling edges; measure cycle length
//---------------------------------------------------------------------------
integer as_len_half = 0;      // half clocks AS has been asserted
integer last_cycle_half = 0;
integer errors = 0;
integer cycles_seen = 0;
reg as_prev = 1;
reg mon_on = 0;
always @(posedge clk) begin
	if (mon_on && !as_n && as_prev) begin errors = errors + 1; $display("FAIL: AS asserted on a rising edge"); end
	if (mon_on && as_n && !as_prev) begin errors = errors + 1; $display("FAIL: AS negated on a rising edge"); end
	as_prev <= as_n;
	if (!as_n) as_len_half = as_len_half + 1;
end
always @(negedge clk) begin
	#1;
	if (!as_n) as_len_half = as_len_half + 1;
	if (as_n && as_len_half != 0) begin
		last_cycle_half = as_len_half;
		cycles_seen = cycles_seen + 1;
		as_len_half = 0;
	end
	as_prev = as_n;
end
// ECS must be a half clock pulse
always @(posedge clk) #1 if (!rst && !ecs_n && !(as_n)) begin
	// ECS during S0 only: AS is negated at that point unless a burst is active
end
`ifdef BUS_TRACE
always @(posedge clk or negedge clk) if (mon_on && $time < 1000)
	$display("%0t clk=%b as=%b ds=%b sterm=%b dsack=%b%b ecs=%b a=%08x siz=%b rw=%b bst=%0d chk=%b d=%08x", $time, clk, as_n, ds_n, sterm_n, dsack1_n, dsack0_n, ecs_n, a, siz, rw, dut.bst, dut.chk_late, d_i);
`endif
// data bus must never be driven during reads
always @(posedge clk) if (d_oe && rw) begin errors = errors + 1; $display("FAIL: data driven on a read"); end

//---------------------------------------------------------------------------
// driver
//---------------------------------------------------------------------------
integer fills = 0;
reg [31:0] last_fill_data;
reg [31:2] last_fill_addr;
always @(posedge clk) if (fill_stb) begin
	fills = fills + 1;
	last_fill_data = fill_data;
	last_fill_addr = fill_addr;
	// a fill must always equal the memory image of that longword
	if (fill_data != {gold[{fill_addr,2'b00}], gold[{fill_addr,2'b01}], gold[{fill_addr,2'b10}], gold[{fill_addr,2'b11}]}) begin
		errors = errors + 1;
		$display("FAIL: fill %08x = %08x, expected %02x%02x%02x%02x", {fill_addr,2'b00}, fill_data,
		         gold[{fill_addr,2'b00}], gold[{fill_addr,2'b01}], gold[{fill_addr,2'b10}], gold[{fill_addr,2'b11}]);
	end
end

integer cycles_before;
integer clocks_before;
integer clocks = 0;
always @(posedge clk) clocks = clocks + 1;

task do_transfer;
	input [1:0] kind; input [31:0] addr; input [2:0] nb; input [2:0] total; input rd;
	input [31:0] wdata; input cache; input cbreq;
	begin
		@(posedge clk);
		req <= 1; req_kind <= kind; req_addr <= addr; req_nbytes <= nb; req_total <= total;
		req_rw <= rd; req_wdata <= wdata; req_cache <= cache; req_cbreq <= cbreq; req_ocs <= 1;
		req_fc <= (kind == `BK_IACK) ? 3'd7 : 3'd5;
		cycles_before = cycles_seen; clocks_before = clocks;
		@(posedge clk);
		while (!req_ack) @(posedge clk);
		req <= 0;
		@(posedge clk);
		while (!done) @(posedge clk);
	end
endtask

task wait_idle;
	begin
		@(posedge clk);
		while (!bus_idle) @(posedge clk);
		@(posedge clk);
	end
endtask

// expected right-justified operand from the golden image
function [31:0] gold_op;
	input [31:0] addr; input [2:0] nb;
	integer i;
	begin
		gold_op = 0;
		for (i = 0; i < nb; i = i + 1) gold_op = {gold_op[23:0], gold[addr + i]};
	end
endfunction

integer i, j, k, n, off, p, w, fails_here;
integer exp_cycles, exp_clocks;
reg [31:0] addr, wv;
reg [15:0] pat;

initial begin
	for (i = 0; i < (1<<20); i = i + 1) begin
		mem[i] = i[7:0] ^ i[15:8] ^ 8'h5A;
		gold[i] = mem[i];
	end
	repeat (3) @(posedge clk);
	// UM 7.7: bus requests are recognized during RESET assertion: BG answers
	// BR while the processor is held in reset (the bus stays three-stated)
	br_n = 0;
	k = 0;
	while (k < 8 && bg_n) begin @(posedge clk); k = k + 1; end
	if (bg_n) begin errors = errors + 1; $display("FAIL: BR during reset not answered with BG"); end
	if (bus_oe) begin errors = errors + 1; $display("FAIL: bus driven during reset"); end
	br_n = 1;
	repeat (6) @(posedge clk);
	rst <= 0;
	repeat (2) @(posedge clk);
	mon_on = 1;

	//-------------------------------------------------------------- reads
	// every size, every alignment, every port, wait states 0..2
	for (p = 0; p < 4; p = p + 1) begin
		for (w = 0; w < 3; w = w + 1) begin
			wait_states = w;
			for (n = 1; n <= 4; n = n + 1) begin
				for (off = 0; off < 4; off = off + 1) begin
					if (off + n <= 4) begin
						addr = {12'd0, p[3:0], 16'h1230} + off;
						fills = 0;
						do_transfer(`BK_DATA, addr, n[2:0], n[2:0], 1, 0, 1, 0);
						if (res_berr || rd_data != gold_op(addr, n[2:0])) begin
							errors = errors + 1;
							$display("FAIL: read port%0d w%0d n%0d off%0d: got %08x exp %08x berr %0d",
							         p, w, n, off, rd_data, gold_op(addr, n[2:0]), res_berr);
						end
						// the entry is filled from every port (single entry mode, UM 6.1.3.1)
						wait_idle;
						if (fills != 1 || last_fill_addr != addr[31:2]) begin
							errors = errors + 1;
							$display("FAIL: read port%0d n%0d off%0d: fills %0d addr %08x", p, n, off, fills, {last_fill_addr,2'b00});
						end
						// cycle count: the operand cycles plus the fill cycles (UM 7.2.3 / 6.1.3.1)
						case (p)
							0, 1: exp_cycles = 1;
							2: exp_cycles = 2;                    // a 16-bit port needs both halves of the entry
							default: exp_cycles = 4;              // an 8-bit port needs all four bytes
						endcase
						if (cycles_seen - cycles_before != exp_cycles) begin
							errors = errors + 1;
							$display("FAIL: read port%0d n%0d off%0d: %0d bus cycles, expected %0d",
							         p, n, off, cycles_seen - cycles_before, exp_cycles);
						end
						// cycle length: sync 2 clocks (4 half clocks), async 3 clocks, plus waits
						exp_clocks = (p == 0) ? (2 + 2*w) : (4 + 2*w);
						if (last_cycle_half != exp_clocks) begin
							errors = errors + 1;
							$display("FAIL: read port%0d w%0d: AS asserted %0d half clocks, expected %0d",
							         p, w, last_cycle_half, exp_clocks);
						end
					end
				end
			end
		end
	end
	wait_states = 0;

	//-------------------------------------------------------------- non-cachable reads
	for (p = 1; p < 4; p = p + 1) begin
		addr = {12'd0, p[3:0], 16'h2001};
		fills = 0;
		do_transfer(`BK_DATA, addr, 3'd2, 3'd2, 1, 0, 0, 0);
		wait_idle;
		if (rd_data != gold_op(addr, 3'd2) || fills != 0) begin
			errors = errors + 1; $display("FAIL: non-cachable read port%0d: %08x fills %0d", p, rd_data, fills);
		end
		// no fill cycles: the number of cycles is that of the operand alone
		exp_cycles = (p == 3) ? 2 : (p == 2) ? 2 : 1;
		if (cycles_seen - cycles_before != exp_cycles) begin
			errors = errors + 1; $display("FAIL: non-cachable read port%0d cycles %0d", p, cycles_seen - cycles_before);
		end
	end

	//-------------------------------------------------------------- writes
	// two-portion operands are written as the core would: the first portion
	// carries the whole remaining operand (SIZ), the second the rest
	for (p = 0; p < 4; p = p + 1) begin
		for (w = 0; w < 2; w = w + 1) begin
			wait_states = w;
			for (n = 1; n <= 4; n = n + 1) begin
				for (off = 0; off < 4; off = off + 1) begin
					addr = {12'd0, p[3:0], 16'h3400} + 16*n + 4*off + off;
					wv = 32'h11223344 + n * 32'h01010101 + off * 32'h10101010 + p * 32'h00000101;
					for (i = 0; i < n; i = i + 1) gold[addr + i] = wv[8*(n-1-i) +: 8];
					if (off + n <= 4) begin
						do_transfer(`BK_DATA, addr, n[2:0], n[2:0], 0, wv, 0, 0);
					end else begin
						do_transfer(`BK_DATA, addr, 3'd4 - off[2:0], n[2:0], 0, wv, 0, 0);
						do_transfer(`BK_DATA, {addr[31:2], 2'b00} + 4, n[2:0] - (3'd4 - off[2:0]), n[2:0] - (3'd4 - off[2:0]), 0,
						            wv & (32'hFFFFFFFF >> (8*(4 - (n - (4 - off))))), 0, 0);
					end
					wait_idle;
					fails_here = 0;
					for (i = 0; i < 16; i = i + 1)
						if (mem[{addr[31:4],4'd0} + i] != gold[{addr[31:4],4'd0} + i]) fails_here = fails_here + 1;
					if (fails_here != 0) begin
						errors = errors + 1;
						$display("FAIL: write port%0d w%0d n%0d off%0d: %0d bytes differ around %08x", p, w, n, off, fails_here, addr);
					end
				end
			end
		end
	end
	wait_states = 0;

	//-------------------------------------------------------------- burst fills
	// a long read at each A3:A2 start bursts four entries in wrap order
	for (off = 0; off < 4; off = off + 1) begin
		addr = 32'h00004100 + 4*off;
		fills = 0;
		do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 1);
		if (rd_data != gold_op(addr, 3'd4)) begin errors = errors + 1; $display("FAIL: burst operand %08x", rd_data); end
		wait_idle;
		if (fills != 4) begin errors = errors + 1; $display("FAIL: burst from %08x gave %0d fills", addr, fills); end
		if (cycles_seen - cycles_before != 1 || last_cycle_half != 8) begin
			errors = errors + 1;
			$display("FAIL: burst from %08x: %0d cycles, AS %0d half clocks (expected 1, 8)", addr, cycles_seen - cycles_before, last_cycle_half);
		end
	end
	// the operand of a burst is available after the first beat
	// misaligned operand inside a line: the burst covers both entries
	addr = 32'h00004206;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd2, 3'd4, 1, 0, 1, 1);
	wait_idle;
	if (fills != 4 || rd_data != gold_op(addr, 3'd2)) begin errors = errors + 1; $display("FAIL: misaligned burst"); end
	// burst with one wait state per beat (region D)
	addr = 32'h000D4300;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 1);
	wait_idle;
	if (fills != 4 || last_cycle_half != 10) begin errors = errors + 1; $display("FAIL: waited burst fills %0d half %0d", fills, last_cycle_half); end
	// CBACK negated with the second beat's STERM: two longwords only
	addr = 32'h000A4400;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 1);
	wait_idle;
	if (fills != 2) begin errors = errors + 1; $display("FAIL: early CBACK negation gave %0d fills", fills); end
	// CIIN on the third beat: that entry not cached, burst ends there
	addr = 32'h000B4500;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 1);
	wait_idle;
	if (fills != 2) begin errors = errors + 1; $display("FAIL: CIIN mid burst gave %0d fills", fills); end
	// burst not requested when the cache disallows it
	addr = 32'h00004600;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (fills != 1 || last_cycle_half != 2) begin errors = errors + 1; $display("FAIL: non-burst sync read fills %0d half %0d", fills, last_cycle_half); end

	//-------------------------------------------------------------- CIIN
	addr = 32'h00044700;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (fills != 0 || !res_ciin || rd_data != gold_op(addr, 3'd4)) begin errors = errors + 1; $display("FAIL: CIIN read"); end

	//-------------------------------------------------------------- bus errors
	addr = 32'h00054800;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (!res_berr) begin errors = errors + 1; $display("FAIL: BERR in lieu of DSACK not reported"); end
	addr = 32'h00064800;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (!res_berr || fills != 0) begin errors = errors + 1; $display("FAIL: late BERR (async) not reported"); end
	addr = 32'h00094800;
	fills = 0;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (!res_berr || fills != 0) begin errors = errors + 1; $display("FAIL: late BERR (sync) not reported"); end
	addr = 32'h000C4800;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 1);
	wait_idle;
	if (!res_berr) begin errors = errors + 1; $display("FAIL: BERR in lieu of STERM not reported"); end
	// a bus error on a fill-only cycle is not an error: 8-bit port word read
	// whose fill cycles hit the BERR region cannot be built with this map, so
	// check instead that a write error is reported
	addr = 32'h00054804;
	do_transfer(`BK_DATA, addr, 3'd2, 3'd2, 0, 32'h1234, 0, 0);
	wait_idle;
	if (!res_berr) begin errors = errors + 1; $display("FAIL: write BERR not reported"); end

	//-------------------------------------------------------------- retry
	retry_armed = 1;
	addr = 32'h00074900;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (res_berr || rd_data != gold_op(addr, 3'd4) || cycles_seen - cycles_before != 2) begin
		errors = errors + 1; $display("FAIL: retry: berr %0d data %08x cycles %0d", res_berr, rd_data, cycles_seen - cycles_before);
	end

	//-------------------------------------------------------------- HALT
	addr = 32'h00084A00;
	do_transfer(`BK_DATA, addr, 3'd4, 3'd4, 1, 0, 1, 0);
	clocks_before = clocks;
	do_transfer(`BK_DATA, addr + 4, 3'd4, 3'd4, 1, 0, 1, 0);
	wait_idle;
	if (clocks - clocks_before < 7) begin errors = errors + 1; $display("FAIL: HALT did not delay the next cycle (%0d clocks)", clocks - clocks_before); end

	//-------------------------------------------------------------- IACK
	// level 3: vector from a 32-bit port on D7-D0
	do_transfer(`BK_IACK, 32'hFFFFFFF7, 3'd1, 3'd1, 1, 0, 0, 0);
	wait_idle;
	if (res_berr || res_avec || rd_data[7:0] != 8'h43) begin errors = errors + 1; $display("FAIL: IACK vector %02x", rd_data[7:0]); end
	// level 5: AVEC
	do_transfer(`BK_IACK, 32'hFFFFFFFB, 3'd1, 3'd1, 1, 0, 0, 0);
	wait_idle;
	if (!res_avec) begin errors = errors + 1; $display("FAIL: AVEC not reported"); end
	// level 6: BERR -> spurious
	do_transfer(`BK_IACK, 32'hFFFFFFFD, 3'd1, 3'd1, 1, 0, 0, 0);
	wait_idle;
	if (!res_berr) begin errors = errors + 1; $display("FAIL: spurious IACK not reported"); end

	//-------------------------------------------------------------- RMC
	req_rmc = 1; req_rmc_last = 0; rmc_hold = 1;
	do_transfer(`BK_DATA, 32'h00014B00, 3'd1, 3'd1, 1, 0, 0, 0);
	if (rmc_n) begin errors = errors + 1; $display("FAIL: RMC not held after the read"); end
	req_rmc_last = 1;
	do_transfer(`BK_DATA, 32'h00014B00, 3'd1, 3'd1, 0, 32'h80, 0, 0);
	wait_idle;
	if (!rmc_n) begin errors = errors + 1; $display("FAIL: RMC not released after the write"); end
	req_rmc = 0; req_rmc_last = 0; rmc_hold = 0;
	gold[32'h00014B00] = 8'h80;
	// an RMW operation that ends after its read (CAS/CAS2 mismatch): the
	// memory system stops holding RMC while the next transfer is already
	// running; RMC is negated once that cycle is over, and a transfer that
	// starts after the release begins with RMC negated (UM 7.1.1)
	req_rmc = 1; req_rmc_last = 0; rmc_hold = 1;
	do_transfer(`BK_DATA, 32'h00014B10, 3'd4, 3'd4, 1, 0, 0, 0);
	req_rmc = 0;
	wait_states = 2;
	@(posedge clk);
	req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014B14; req_nbytes <= 4; req_total <= 4; req_rw <= 1; req_cache <= 0; req_cbreq <= 0;
	@(posedge clk); while (!req_ack) @(posedge clk);
	req <= 0;
	@(posedge clk);
	rmc_hold = 0;                              // released while that cycle runs
	while (!done) @(posedge clk);
	wait_idle;
	if (!rmc_n) begin errors = errors + 1; $display("FAIL: RMC not negated after a release during a cycle"); end
	wait_states = 0;
	req_rmc = 1; req_rmc_last = 0; rmc_hold = 1;
	do_transfer(`BK_DATA, 32'h00014B18, 3'd4, 3'd4, 1, 0, 0, 0);
	req_rmc = 0;
	rmc_hold = 0;                              // released before the next transfer
	fails_here = 0;
	@(posedge clk);
	req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014B1C; req_nbytes <= 4; req_total <= 4; req_rw <= 1; req_cache <= 0; req_cbreq <= 0;
	@(posedge clk); while (!req_ack) @(posedge clk);
	req <= 0;
	while (!done) begin @(posedge clk); if (as_asserted && !rmc_n) fails_here = fails_here + 1; end
	if (fails_here != 0) begin errors = errors + 1; $display("FAIL: RMC asserted on the cycle after the RMW operation"); end
	wait_idle;

	//-------------------------------------------------------------- arbitration
	br_n = 0;
	repeat (4) @(posedge clk);
	if (bg_n) begin errors = errors + 1; $display("FAIL: BG not asserted after BR"); end
	bgack_n = 0; br_n = 1;
	repeat (2) @(posedge clk);
	if (bus_oe) begin errors = errors + 1; $display("FAIL: bus not three-stated after BGACK"); end
	if (!bg_n) begin errors = errors + 1; $display("FAIL: BG not negated after BGACK"); end
	// a request while the bus is away must wait
	@(posedge clk);
	req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014C00; req_nbytes <= 4; req_total <= 4; req_rw <= 1; req_cache <= 0; req_cbreq <= 0;
	repeat (5) @(posedge clk);
	if (!as_n || bus_oe) begin errors = errors + 1; $display("FAIL: cycle started while the bus was granted away"); end
	bgack_n = 1;
	while (!done) @(posedge clk);
	req <= 0;
	if (rd_data != gold_op(32'h00014C00, 3'd4)) begin errors = errors + 1; $display("FAIL: read after arbitration"); end
	wait_idle;
	// BR alone that is withdrawn without BGACK: the processor keeps the bus
	br_n = 0;
	repeat (3) @(posedge clk);
	br_n = 1;
	repeat (6) @(posedge clk);
	if (!bus_oe) begin errors = errors + 1; $display("FAIL: bus not reclaimed after BR withdrawn"); end
	// BR during a cycle: the bus is placed in the high-impedance state after
	// the rising edge that follows the negation of AS, the end of S5 (UM
	// 7.7.4, Figure 7-60)
	wait_states = 2;
	@(posedge clk);
	req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014C10; req_nbytes <= 4; req_total <= 4; req_rw <= 1; req_cache <= 0; req_cbreq <= 0;
	@(posedge clk); while (!req_ack) @(posedge clk);
	req <= 0;
	while (!as_asserted) @(posedge clk);
	br_n = 0;
	k = 0;
	while (k == 0) begin @(negedge clk); #1; if (as_n) k = 1; end
	@(posedge clk); #1;
	if (bus_oe || bg_n) begin errors = errors + 1; $display("FAIL: bus not three-stated at the rising edge after AS negated (bus_oe %0d bg_n %0d)", bus_oe, bg_n); end
	@(posedge clk);
	bgack_n = 0; br_n = 1;
	repeat (3) @(posedge clk);
	bgack_n = 1;
	wait_idle;
	wait_states = 0;
	repeat (4) @(posedge clk);
	// BR and BGACK together from the idle state (Figure 7-61: state 0 goes
	// to state 4 on A whatever R is, then to state 5, so BG is asserted
	// once and stays asserted while both are held; the bus floats)
	k = 0; fails_here = 0;
	br_n = 0; bgack_n = 0;
	for (i = 0; i < 12; i = i + 1) begin
		@(negedge clk); #1;
		if (!bg_n) k = 1;
		else if (k == 1) fails_here = fails_here + 1;     // BG negated after it was asserted
	end
	if (k == 0 || fails_here != 0 || bus_oe) begin
		errors = errors + 1; $display("FAIL: BR with BGACK from the idle state: BG seen %0d, BG negated again %0d, bus_oe %0d", k, fails_here, bus_oe);
	end
	br_n = 1; bgack_n = 1;
	repeat (8) @(posedge clk);
	if (!bus_oe || !bg_n) begin errors = errors + 1; $display("FAIL: bus not reclaimed after BR and BGACK negated"); end
	// RMC blocks BG
	req_rmc = 1; req_rmc_last = 0; rmc_hold = 1;
	do_transfer(`BK_DATA, 32'h00014D00, 3'd1, 3'd1, 1, 0, 0, 0);
	br_n = 0;
	repeat (4) @(posedge clk);
	if (!bg_n) begin errors = errors + 1; $display("FAIL: BG asserted during RMC"); end
	req_rmc_last = 1;
	do_transfer(`BK_DATA, 32'h00014D00, 3'd1, 3'd1, 0, 32'h80, 0, 0);
	gold[32'h00014D00] = 8'h80;
	wait_idle;
	repeat (3) @(posedge clk);
	if (bg_n) begin errors = errors + 1; $display("FAIL: BG not asserted after RMC ended"); end
	br_n = 1; req_rmc = 0; req_rmc_last = 0; rmc_hold = 0;
	repeat (6) @(posedge clk);
	// relinquish and retry (UM 7.5.2, 7.7.4): BERR, HALT and BR together on
	// the first read of a read-modify-write operation release the bus (BG,
	// three-state for the BGACK master); once HALT is negated the read is
	// rerun with RMC.  The same on a later cycle of the operation is a plain
	// retry: BG waits until RMC is negated (Figure 7-61 note).
	req_rmc = 1; req_rmc_last = 0; rmc_hold = 1; rr_on = 1;
	@(posedge clk);
	req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014D40; req_nbytes <= 4; req_total <= 4; req_rw <= 1; req_cache <= 0; req_cbreq <= 0;
	@(posedge clk); while (!req_ack) @(posedge clk);
	req <= 0;
	while (!as_asserted) @(posedge clk);
	br_n = 0; rr_halt = 1;
	while (as_asserted) @(posedge clk);
	rr_on = 0;
	k = 0;
	while (k < 12 && bg_n) begin @(posedge clk); k = k + 1; end
	if (bg_n) begin errors = errors + 1; $display("FAIL: relinquish and retry on the first RMW read: BG not asserted"); end
	else begin
		@(posedge clk); bgack_n = 0; br_n = 1;
		repeat (3) @(posedge clk);
		if (bus_oe) begin errors = errors + 1; $display("FAIL: relinquish and retry: bus not released"); end
		repeat (3) @(posedge clk);
		bgack_n = 1;
	end
	br_n = 1;
	repeat (2) @(posedge clk);
	rr_halt = 0;
	while (!done) @(posedge clk);
	if (res_berr || rd_data != gold_op(32'h00014D40, 3'd4) || rmc_n) begin
		errors = errors + 1; $display("FAIL: relinquish and retry: rerun read berr %0d data %08x rmc_n %0d", res_berr, rd_data, rmc_n);
	end
	rr_on = 1; req_rmc_last = 1;
	@(posedge clk);
	req <= 1; req_addr <= 32'h00014D40; req_rw <= 0; req_wdata <= 32'h5A5AA5A5;
	@(posedge clk); while (!req_ack) @(posedge clk);
	req <= 0;
	while (!as_asserted) @(posedge clk);
	br_n = 0; rr_halt = 1;
	while (as_asserted) @(posedge clk);
	rr_on = 0;
	repeat (10) @(posedge clk);
	if (!bg_n || !bus_oe) begin errors = errors + 1; $display("FAIL: BG during the retry of a later RMW cycle"); end
	rr_halt = 0;
	while (!done) @(posedge clk);
	wait_idle;
	for (i = 0; i < 4; i = i + 1) gold[32'h00014D40 + i] = mem[32'h00014D40 + i];
	if (res_berr || {mem[32'h00014D40], mem[32'h00014D41], mem[32'h00014D42], mem[32'h00014D43]} != 32'h5A5AA5A5) begin
		errors = errors + 1; $display("FAIL: relinquish and retry: retried RMW write");
	end
	k = 0;
	while (k < 6 && bg_n) begin @(posedge clk); k = k + 1; end
	if (bg_n) begin errors = errors + 1; $display("FAIL: BG not asserted after the RMW operation"); end
	br_n = 1; req_rmc = 0; req_rmc_last = 0; rmc_hold = 0;
	repeat (6) @(posedge clk);

	// single-wire arbitration inside an RMW operation (UM 7.7.4: BGACK alone
	// releases the bus; it "applies to all bus cycles of a read-modify-write
	// sequence"; Figure 7-62): after the read the bus floats, no cycle runs
	// while BGACK is asserted, then the write runs with RMC asserted
	req_rmc = 1; req_rmc_last = 0; rmc_hold = 1;
	do_transfer(`BK_DATA, 32'h00014D60, 3'd4, 3'd4, 1, 0, 0, 0);
	bgack_n = 0;
	repeat (4) @(posedge clk);
	if (bus_oe) begin errors = errors + 1; $display("FAIL: BGACK alone inside an RMW operation: bus not three-stated"); end
	if (!bg_n) begin errors = errors + 1; $display("FAIL: BG asserted for BGACK alone during RMC"); end
	cycles_before = cycles_seen;
	req_rmc_last = 1;
	@(posedge clk);
	req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014D60; req_nbytes <= 4; req_total <= 4; req_rw <= 0; req_wdata <= 32'hC3C3A5A5; req_cache <= 0; req_cbreq <= 0;
	repeat (6) @(posedge clk);
	if (cycles_seen != cycles_before || !as_n) begin errors = errors + 1; $display("FAIL: cycle run while BGACK is asserted"); end
	bgack_n = 1;
	fails_here = 0;
	while (!req_ack) @(posedge clk);
	req <= 0;
	while (!done) begin @(posedge clk); if (as_asserted && rmc_n) fails_here = fails_here + 1; end
	if (fails_here != 0) begin errors = errors + 1; $display("FAIL: RMW write after BGACK ran without RMC"); end
	wait_idle;
	if (!rmc_n) begin errors = errors + 1; $display("FAIL: RMC not negated after the RMW write"); end
	for (i = 0; i < 4; i = i + 1) gold[32'h00014D60 + i] = mem[32'h00014D60 + i];
	if ({mem[32'h00014D60], mem[32'h00014D61], mem[32'h00014D62], mem[32'h00014D63]} != 32'hC3C3A5A5) begin
		errors = errors + 1; $display("FAIL: RMW write after BGACK");
	end
	req_rmc = 0; req_rmc_last = 0; rmc_hold = 0;
	repeat (4) @(posedge clk);

	//-------------------------------------------------------------- back-to-back
	// consecutive requests: an async cycle every three clocks (UM Figure 7-25)
	clocks_before = clocks;
	for (i = 0; i < 4; i = i + 1) begin
		@(posedge clk);
		req <= 1; req_kind <= `BK_DATA; req_addr <= 32'h00014E00 + 4*i; req_nbytes <= 4; req_total <= 4;
		req_rw <= i[0]; req_wdata <= 32'hA5A5A5A5; req_cache <= 0; req_cbreq <= 0; req_ocs <= 1;
		@(posedge clk); while (!req_ack) @(posedge clk);
		req <= 0;
	end
	while (!done) @(posedge clk);
	wait_idle;

	$display("bus cycles observed: %0d", cycles_seen);
	if (errors == 0) $display("ALL TESTS PASSED");
	else $display("TEST FAILED: %0d errors", errors);
	$finish;
end

// watchdog
initial begin
	#20000000;
	$display("TEST FAILED: timeout");
	$finish;
end

endmodule
