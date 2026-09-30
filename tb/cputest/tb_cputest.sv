// AP030 against the WinUAE cputest corpus (68030, data format v20).
//
// Port of the AP040 replay driver (Minimig tests/ap040/tb_dat_replay.v) to
// the AP030's MC68030 pin bus.  replay_gen.py expands one corpus slice into
// APR2 records; each record runs on the real ap030_top.  No instruction
// behaviour is modelled here.  Per round the bench:
//   - resets the processor with the round's PC in the reset vector and
//     holds its first opcode fetch while it injects the complete register
//     state (D0-D7, A0-A6, USP/ISP/MSP, SR, VBR),
//   - serves the corpus memory images (low memory, test memory) on a
//     zero-wait 32-bit synchronous port, a synthetic vector table at CAPV
//     and a handler page at CAPH (NOPs; vector 9's handler is RTE so a
//     stacked trace can complete),
//   - answers interrupt acknowledge with AVEC and every other CPU space
//     cycle with BERR (no coprocessor: F-line; BKPT: illegal),
//   - captures the exception entry (vector, SR, registers) at S_EXC0 and
//     checks registers, SR, memory writes and the exception frame against
//     the corpus.
`timescale 1ns/1ns

module tb_cputest;

localparam [31:0] TMEM_MAX = 32'h0020_0000;
reg [31:0] TBASE;
reg [31:0] TSIZE;
localparam [31:0] CAPV  = 32'h4210_0000;
localparam [31:0] CAPH  = 32'h4211_0000;
localparam [31:0] RND2  = 32'h524E4432;
localparam [7:0]  S_EXC0 = 8'd74;
localparam [7:0]  S_EXC_JMP = 8'd78;    // S_EXC4: vector fetched, jumping to the handler

localparam [31:0] F_FPU          = 32'h0000_0001;
localparam [31:0] F_IGNORE_EXC   = 32'h0000_0002;
localparam [31:0] F_TRACE_STACK  = 32'h0000_0004;
localparam [31:0] F_TRACE_ALONE  = 32'h0000_0008;
localparam [31:0] F_CHECK_FPIAR  = 32'h0000_0020;
localparam [31:0] F_NORMAL_END   = 32'h0000_0040;
localparam [31:0] F_V20          = 32'h0000_0080;
localparam integer EXEC_TIMEOUT = 10000;

reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0;

//--------------------------------------------------------------------------
// the processor
//--------------------------------------------------------------------------
wire [31:0] a, d_o;
wire  [2:0] fc;
wire  [1:0] siz;
wire        rw, as_n, ds_n, d_oe, bus_oe, cbreq_n;
reg  [31:0] d_i;
reg         sterm_n, berr_n, avec_n, dsack0_n, dsack1_n;
reg   [2:0] ipl;               // active low
wire [31:0] dbg_pc;

ap030_top dut (
	.clk(clk),
	.a(a), .fc(fc), .siz(siz), .rw(rw), .rmc_n(), .as_n(as_n), .ds_n(ds_n), .dben_n(),
	.ecs_n(), .ocs_n(), .ciout_n(), .cbreq_n(cbreq_n), .bus_oe(bus_oe),
	.d_o(d_o), .d_oe(d_oe), .d_i(d_i),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(sterm_n), .berr_n(berr_n), .halt_n(1'b1),
	.avec_n(avec_n), .ciin_n(1'b1), .cback_n(1'b1), .br_n(1'b1), .bg_n(), .bgack_n(1'b1),
	.ipl_n(ipl), .ipend_n(), .reset_n_i(nreset), .reset_n_oe(),
	.cdis_n(1'b1), .mmudis_n(1'b1), .refill_n(), .status_n(),
	.dbg_pc(dbg_pc), .dbg_sr(), .dbg_state(), .dbg_inst(), .dbg_halted(),
	.dbg_vbr(), .dbg_cacr(), .dbg_cache_clear(),
	.snoop_we(1'b0), .snoop_addr(32'd0), .nmi_vec_nocache(1'b0)
);

wire [31:0] dbg_a7 = dut.core.rf.a7;
wire [7:0]  core_state = dut.core.state;

//--------------------------------------------------------------------------
// corpus memory and the synthetic reset/vector/handler overlays
//--------------------------------------------------------------------------
reg [7:0] lmem [0:32767];
reg [7:0] tmem [0:TMEM_MAX-1];

reg         hold_fetch;
reg         boot_overlay;
reg         round_active;
reg  [31:0] cur_pc;
reg  [31:0] boot_msp;
reg  [31:0] boot_pc;
reg  [31:0] odd_vector;

function [7:0] be_byte;
	input [31:0] value;
	input [1:0] lane;
	begin
		case (lane)
			2'd0: be_byte = value[31:24];
			2'd1: be_byte = value[23:16];
			2'd2: be_byte = value[15:8];
			default: be_byte = value[7:0];
		endcase
	end
endfunction

function [7:0] rd8;
	input [31:0] ad;
	reg [7:0] vec;
	reg [31:0] vv, hoff;
	begin
		if (boot_overlay && ad < 8)
			rd8 = be_byte(ad < 4 ? boot_msp : boot_pc, ad[1:0]);
		else if (ad[31:15] == 0)
			rd8 = lmem[ad[14:0]];
		else if (ad >= TBASE && ad < TBASE + TSIZE)
			rd8 = tmem[ad - TBASE];
		else if (ad >= CAPV && ad < CAPV + 32'h400) begin
			vec = (ad - CAPV) >> 2;
			// a standalone trace is recorded by the generator as handled
			// (stored trace), not through the odd vector
			vv = (odd_vector != 0 && vec >= 4 && !(vec == 9 && e_trace == 2)) ? odd_vector
			                                                                   : CAPH + {21'd0, vec, 3'd0};
			rd8 = be_byte(vv, ad[1:0]);
		end else if (ad >= CAPH && ad < CAPH + 32'h800) begin
			hoff = ad - CAPH;
			// vector 9's handler is RTE; all other slots are NOPs
			if (hoff[10:3] == 9 && hoff[2:0] == 0) rd8 = 8'h4e;
			else if (hoff[10:3] == 9 && hoff[2:0] == 1) rd8 = 8'h73;
			else if (!hoff[0]) rd8 = 8'h4e;
			else rd8 = 8'h71;
		end else
			rd8 = 8'h00;
	end
endfunction

wire as_asserted = !as_n && bus_oe;
wire cpu_space   = (fc == 3'd7);
wire prog_fetch  = (fc == 3'd2 || fc == 3'd6);
wire in_hand     = (a >= CAPH) && (a < CAPH + 32'h800);
wire held        = hold_fetch && prog_fetch && ({a[31:2], 2'b00} == {cur_pc[31:2], 2'b00});

// zero-wait synchronous port (no bursts); CPU space terminates asynchronously
always @* begin
	sterm_n = 1'b1; berr_n = 1'b1; avec_n = 1'b1; dsack0_n = 1'b1; dsack1_n = 1'b1;
	if (as_asserted) begin
		if (cpu_space) begin
			if (a[19:16] == 4'hF) avec_n = 1'b0;     // interrupt acknowledge: autovector
			else berr_n = 1'b0;                        // breakpoint, coprocessor: absent
		end else if (!held)
			sterm_n = 1'b0;
	end
end
always @* d_i = {rd8({a[31:2], 2'b00}), rd8({a[31:2], 2'b01}), rd8({a[31:2], 2'b10}), rd8({a[31:2], 2'b11})};

// declared before first use
reg [31:0] patch_addr;
integer jf, jn, jr;
reg [31:0] flags, test_idx, round_idx;
integer errors /* verilator public_flat_rw */;

task write_byte;
	input [31:0] ad;
	input [7:0] v;
	begin
		if (ad[31:15] == 0) lmem[ad[14:0]] = v;
		else if (ad >= TBASE && ad < TBASE + TSIZE) tmem[ad - TBASE] = v;
		else begin
			$display("HARNESS: write outside corpus memory: %08x", ad);
			errors = errors + 1;
		end
	end
endtask

// processor writes: the byte lanes of a 32-bit port (UM Table 7-7)
reg wr_seen;
function [3:0] lanes; input [1:0] off; input [1:0] sz;
	reg [3:0] m;
	begin
		case (sz)
			2'b01: m = 4'b1000;
			2'b10: m = 4'b1100;
			2'b11: m = 4'b1110;
			default: m = 4'b1111;
		endcase
		lanes = m >> off;
	end
endfunction
integer wi;
always @(negedge clk) begin
	if (!as_asserted) wr_seen <= 0;
	else if (!cpu_space && !rw && !sterm_n && d_oe && !wr_seen && nreset) begin
		wr_seen <= 1;
		for (wi = 0; wi < 4; wi = wi + 1)
			if (lanes(a[1:0], siz)[3 - wi]) begin
				if ({a[31:2], 2'b00} + wi < 32'h8000 ||
				    ({a[31:2], 2'b00} + wi >= TBASE && {a[31:2], 2'b00} + wi < TBASE + TSIZE))
					write_byte({a[31:2], 2'b00} + wi, d_o[31 - 8*wi -: 8]);
			end
	end
end

//--------------------------------------------------------------------------
// exception entry snapshot: S_EXC0 is entered before the processor changes
// SR or its stack; the register file is sampled two clocks later so that a
// register write issued by the faulting state has landed
//--------------------------------------------------------------------------
reg exc_seen, cap_pend, cap_pend2;
reg [7:0] cap_vec, latest_exc_vec;
reg [31:0] cap_regs [0:15];
reg [15:0] cap_sr;
reg [31:0] cap_sp;
reg [7:0] expected_exc_live;
reg [7:0] first_exc_vec;          // the first exception the core takes in a round
integer irq_order_skips;
reg [7:0] e_trace;
integer ci;

always @(posedge clk) begin
	if (!round_active) begin
		exc_seen <= 0; cap_pend <= 0; cap_pend2 <= 0;
	end else begin
		if (cap_pend) begin cap_pend <= 0; cap_pend2 <= 1; end
		else if (cap_pend2) begin
			cap_pend2 <= 0;
			for (ci = 0; ci < 15; ci = ci + 1) cap_regs[ci] <= dut.core.rf.r[ci];
			cap_regs[15] <= dut.core.rf.usp;
		end
		if (core_state != S_EXC0) exc_seen <= 0;
		if (core_state == S_EXC0 && !exc_seen) begin
			exc_seen <= 1;
			latest_exc_vec <= dut.core.exc_vec;
			if (first_exc_vec == 8'hff) first_exc_vec <= dut.core.exc_vec;
			// a trace handler executes RTE and is not the final snapshot,
			// unless the trace is the recorded result
			if (dut.core.exc_vec != 9 || expected_exc_live == 9 || e_trace == 2) begin
				cap_vec <= dut.core.exc_vec;
				cap_sr <= dut.core.sr;
				cap_pend <= 1;
			end
			// release the interrupt once the processor has accepted it
			if (dut.core.exc_is_irq) ipl <= 3'b111;
		end
	end
end

//--------------------------------------------------------------------------
// APR2 input
//--------------------------------------------------------------------------
integer ran /* verilator public_flat_rw */;
integer mism /* verilator public_flat_rw */;
integer report_lim, timeout;
integer trace_round;
integer k, n, fgot;
integer lmfd, tmfd;
reg [2047:0] job_file, lmem_file, tmem_file;

function [7:0] jread8; input integer dummy; integer r;
	begin
		r = $fgetc(jf);
		if (r < 0) begin $display("FAIL: unexpected EOF"); $finish; end
		jread8 = r[7:0];
	end endfunction
function [15:0] jread16; input integer dummy; begin jread16 = {jread8(0), jread8(0)}; end endfunction
function [31:0] jread32; input integer dummy; begin jread32 = {jread16(0), jread16(0)}; end endfunction

reg [31:0] i_regs [0:15];
reg [31:0] i_sr, i_pc, i_ssp, i_msp, i_fpcr, i_fpsr, i_fpiar;
reg [31:0] e_regs [0:15];
reg [31:0] e_sr, e_srmask, e_fpcr, e_fpsr, e_fpiar, e_pc;
reg [7:0] e_exc, e_group2, i_level;
reg [15:0] e_trace_sr, e_trace_srmask;
reg [31:0] e_trace_pc;
reg [7:0] frame_b [0:255];
reg [7:0] frame_m [0:255];
integer frame_len;
reg [31:0] em_a [0:255];
reg [7:0] em_sz [0:255];
reg [31:0] em_v [0:255];
reg [31:0] em_old [0:255];
integer em_cnt;
reg [31:0] post_a [0:255];
reg [15:0] post_n [0:255];
reg [15:0] post_off [0:255];
reg [7:0] post_b [0:8191];
integer post_cnt, post_bytes;
reg [31:0] clean_a [0:255];
reg [15:0] clean_n [0:255];
reg [15:0] clean_off [0:255];
reg [7:0] clean_b [0:8191];
integer clean_cnt, clean_bytes;

task apply_value; input [31:0] ad; input [7:0] sz; input [31:0] v; integer nb, bi;
	begin
		nb = (sz == 0) ? 1 : (sz == 1) ? 2 : 4;
		for (bi = 0; bi < nb; bi = bi + 1) write_byte(ad + bi, v >> (8 * (nb - 1 - bi)));
	end endtask

function [31:0] read_value; input [31:0] ad; input [7:0] sz;
	begin
		if (sz == 0) read_value = {24'd0, rd8(ad)};
		else if (sz == 1) read_value = {16'd0, rd8(ad), rd8(ad+1)};
		else read_value = {rd8(ad), rd8(ad+1), rd8(ad+2), rd8(ad+3)};
	end endfunction

task read_apply_patches; integer pcnt, pi, pj, plen; reg [31:0] pa;
	begin
		pcnt = jread16(0);
		for (pi = 0; pi < pcnt; pi = pi + 1) begin
			pa = jread32(0); plen = jread16(0);
			for (pj = 0; pj < plen; pj = pj + 1) write_byte(pa + pj, jread8(0));
		end
	end endtask

task read_post_patches; integer pi, pj;
	begin
		post_cnt = jread16(0); post_bytes = 0;
		for (pi = 0; pi < post_cnt; pi = pi + 1) begin
			post_a[pi] = jread32(0); post_n[pi] = jread16(0); post_off[pi] = post_bytes;
			for (pj = 0; pj < post_n[pi]; pj = pj + 1) begin
				if (post_bytes >= 8192) begin $display("FAIL: post patch overflow"); $finish; end
				post_b[post_bytes] = jread8(0); post_bytes = post_bytes + 1;
			end
		end
	end endtask

task read_clean_patches; integer pi, pj;
	begin
		clean_cnt = jread16(0); clean_bytes = 0;
		for (pi = 0; pi < clean_cnt; pi = pi + 1) begin
			clean_a[pi] = jread32(0); clean_n[pi] = jread16(0); clean_off[pi] = clean_bytes;
			for (pj = 0; pj < clean_n[pi]; pj = pj + 1) begin
				if (clean_bytes >= 8192) begin $display("FAIL: cleanup patch overflow"); $finish; end
				clean_b[clean_bytes] = jread8(0); clean_bytes = clean_bytes + 1;
			end
		end
	end endtask

task apply_deferred; integer pi, pj;
	begin
		for (pi = 0; pi < post_cnt; pi = pi + 1)
			for (pj = 0; pj < post_n[pi]; pj = pj + 1) write_byte(post_a[pi] + pj, post_b[post_off[pi] + pj]);
		for (pi = 0; pi < clean_cnt; pi = pi + 1)
			for (pj = 0; pj < clean_n[pi]; pj = pj + 1) write_byte(clean_a[pi] + pj, clean_b[clean_off[pi] + pj]);
	end endtask

task mismatch; input [8*20:1] what; input [31:0] exp; input [31:0] got;
	begin
		mism = mism + 1;
		if (mism <= report_lim)
			$display("MISMATCH j%0d t%0d r%0d %0s: expected %08x got %08x (op=%02x%02x%02x%02x%02x%02x%02x%02x sr_in=%04x)",
			         jr, test_idx, round_idx, what, exp, got,
			         rd8(i_pc + 0), rd8(i_pc + 1), rd8(i_pc + 2), rd8(i_pc + 3),
			         rd8(i_pc + 4), rd8(i_pc + 5), rd8(i_pc + 6), rd8(i_pc + 7), i_sr[15:0]);
	end endtask

task check_trace_frame; reg [31:0] sp, pcv; reg [15:0] srv;
	begin
		sp = dbg_a7;
		srv = read_value(sp, 1);
		pcv = read_value(sp + 2, 2);
		if (e_trace == 2) begin
			if (((srv ^ e_trace_sr) & e_trace_srmask) != 0) mismatch("trace SR", e_trace_sr, srv);
			if (pcv !== e_trace_pc) mismatch("trace PC", e_trace_pc, pcv);
		end else if (e_trace == 0 && (flags & F_NORMAL_END)) begin
			// the corpus records no trace here because the native runner does
			// not check one; the 68030 traces the instruction, stacking the
			// post-instruction SR and the address of the terminating ILLEGAL
			if (((srv ^ e_sr[15:0]) & e_srmask[15:0]) != 0) mismatch("trace SR", e_sr, srv);
			if (pcv !== e_pc) mismatch("trace PC", e_pc, pcv);
		end
	end endtask

task check_final; integer fi; reg [31:0] sp;
	begin
		if (!(flags & F_IGNORE_EXC) && cap_vec !== e_exc) mismatch("exception", e_exc, cap_vec);
		for (fi = 0; fi < 16; fi = fi + 1)
			if (cap_regs[fi] !== e_regs[fi])
				mismatch(fi < 8 ? "D register" : "A register", e_regs[fi], cap_regs[fi]);
		if (((cap_sr ^ e_sr[15:0]) & e_srmask[15:0]) != 0) mismatch("SR", e_sr, cap_sr);
		sp = cap_sp;
		if (frame_len != 0) begin
			for (fi = 0; fi < frame_len; fi = fi + 1)
				if (((rd8(sp + fi)) ^ frame_b[fi]) & frame_m[fi]) begin
					if (mism < report_lim) $display("  frame byte %0d at %08x mask=%02x", fi, sp + fi, frame_m[fi]);
					mismatch("exception frame", frame_b[fi], rd8(sp + fi));
				end
		end else if (!(flags & F_IGNORE_EXC) && e_exc == 4) begin
			if (read_value(sp + 2, 2) !== e_pc) mismatch("end PC", e_pc, read_value(sp + 2, 2));
		end
		for (fi = 0; fi < em_cnt; fi = fi + 1) begin
			if (read_value(em_a[fi], em_sz[fi]) !== em_v[fi])
				mismatch("memory write", em_v[fi], read_value(em_a[fi], em_sz[fi]));
			apply_value(em_a[fi], em_sz[fi], em_old[fi]);
		end
	end endtask

task inject_state; integer ii;
	begin
		// the native runner enters every test through a format $0 RTE; for a
		// supervisor-mode round it copies the corpus stack image to the ISP
		if (i_sr[13]) for (ii = 0; ii < 32; ii = ii + 1) write_byte(i_ssp + ii, rd8(i_regs[15] + ii));
		for (ii = 0; ii < 15; ii = ii + 1) dut.core.rf.r[ii] = i_regs[ii];
		dut.core.rf.usp = i_regs[15];
		dut.core.rf.isp = i_ssp;
		dut.core.rf.msp = i_msp;
		dut.core.sr = i_sr[15:0];
		dut.core.tr_t1 = i_sr[15];
		dut.core.tr_t0 = i_sr[14];
		dut.core.vbr = CAPV;
		dut.core.cacr = 0;
		dut.core.sfc = 0; dut.core.dfc = 0;
	end endtask

// interrupts: the request is raised once the tested instruction has
// started, so it is recognised at the boundary after it
reg ipl_after;
reg [2:0] ipl_after_lvl;
// +trace_round=N: list every instruction boundary and exception of record N
always @(posedge clk) if (round_active && jr == trace_round) begin
	if (dut.dbg_inst)
		$display("  trace j%0d: inst pc=%08x ir=%04x sr=%04x d0=%08x a7=%08x",
		         jr, dbg_pc, dut.core.ir, dut.core.sr, dut.core.rf.r[0], dbg_a7);
	if (core_state == S_EXC0 && !exc_seen)
		$display("  trace j%0d: exception %0d sr=%04x", jr, dut.core.exc_vec, dut.core.sr);
end

// The native runner enters the test through RTE with the request already
// pending, and the 68030 takes it at the boundary after the tested
// instruction.  The bench raises the pins in the clock after the tested
// instruction is dispatched and loads the synchronizer and the recognized
// level with them (after that negedge's updates), so the request is pending
// before the next boundary however short the instruction is.
always @(negedge clk) if (round_active && ipl_after && dut.dbg_inst) begin
	#1;
	ipl = ~ipl_after_lvl;
	dut.core.ipl_s1 = ipl_after_lvl;
	dut.core.ipl_s2 = ipl_after_lvl;
	if (ipl_after_lvl == 3'd7 && dut.core.irq_lvl != 3'd7) dut.core.nmi_edge = 1'b1;
	dut.core.irq_lvl = ipl_after_lvl;
	ipl_after = 0;
end

task run_round;
	integer vec, saw_primary, saw_trace, trace_bits, primary_vec;
	integer primary_frame_done;
	reg [31:0] ha;
	begin
		boot_msp = i_ssp; boot_pc = i_pc;
		cur_pc = i_pc;
		hold_fetch = 1; boot_overlay = 1; round_active = 0;
		cap_vec = 8'hff; latest_exc_vec = 8'hff; first_exc_vec = 8'hff;
		ipl_after = 0;
		// the 68030 corpus records the tested instruction as executed before
		// the interrupt is taken (registers, SR and memory all show its
		// result), including the odd-vector interrupt rounds
		// v24 (current generator): the request is pending from the start and
		// is taken before the tested instruction unless the SR masks it
		ipl = 3'b111;
		if (i_level != 0) begin
			if (flags & F_V20) begin ipl_after = 1; ipl_after_lvl = i_level[2:0]; end
			else ipl = ~i_level[2:0];
		end
		nreset = 0;
		repeat (12) @(posedge clk);
		nreset = 1;

		timeout = 0;
		while (!(as_asserted && held) && timeout < 4000) begin
			@(posedge clk); timeout = timeout + 1;
		end
		if (timeout >= 4000) begin
			$display("FAIL j%0d t%0d r%0d: start-fetch timeout, pc=%h", jr, test_idx, round_idx, dbg_pc);
			errors = errors + 1;
			disable run_round;
		end
		inject_state;
		boot_overlay = 0;
		round_active = 1;
		@(posedge clk);
		hold_fetch = 0;

		saw_primary = 0; saw_trace = 0; primary_vec = -1;
		primary_frame_done = 0; timeout = 0;
		trace_bits = (i_sr[15:14] != 0) || (e_sr[15:14] != 0);
		while (timeout < EXEC_TIMEOUT) begin
			@(posedge clk); timeout = timeout + 1;
			if (e_trace == 0 && trace_bits && e_exc != 0 && e_exc != 9 &&
			    latest_exc_vec == e_exc && core_state == S_EXC_JMP) begin
				primary_frame_done = 1; timeout = EXEC_TIMEOUT;
			end
			if (frame_len == 0 && e_exc == 4 && latest_exc_vec == 4 && core_state == S_EXC_JMP) begin
				primary_frame_done = 1; timeout = EXEC_TIMEOUT;
			end
			if (saw_trace && e_exc != 9 && latest_exc_vec == e_exc && core_state == S_EXC_JMP) begin
				primary_frame_done = 1; timeout = EXEC_TIMEOUT;
			end
			if (as_asserted && prog_fetch && in_hand && a[2:0] == 3'b000) begin
				ha = a - CAPH;
				vec = ha >> 3;
				if ((!saw_trace && vec == latest_exc_vec) ||
				    (saw_primary && saw_trace && vec == primary_vec) ||
				    (!saw_primary && saw_trace && latest_exc_vec != 9 && vec == latest_exc_vec)) begin
					if (vec == 9 && (e_trace != 0 || e_exc == 9 || (flags & F_NORMAL_END))) begin
						check_trace_frame;
						if (!saw_primary && cap_vec != 8'hff && cap_vec != 9) begin
							saw_primary = 1; primary_vec = cap_vec;
						end
						saw_trace = 1;
						if (e_exc == 9) timeout = EXEC_TIMEOUT;
						else while (as_asserted && prog_fetch && a == CAPH + vec*8) @(posedge clk);
					end else if (vec == 9) begin
						// a request the corpus records despite the new SR mask
						// (see the irq-order classification below): the 68030
						// ignores it and traces as the SR now says
						if (!((flags & F_V20) && i_level != 0 && e_exc >= 25 && e_exc <= 31 && e_sr[10:8] >= i_level[2:0] &&
						      i_level[2:0] != 3'd7))
							mismatch("unexpected trace", 32'd0, {16'd0, dut.core.sr});
						timeout = EXEC_TIMEOUT;
					end else if ((e_trace == 1 || (e_exc == 4 && trace_bits)) && !saw_primary && !saw_trace) begin
						saw_primary = 1; primary_vec = vec;
						while (as_asserted && prog_fetch && a == CAPH + vec*8) @(posedge clk);
					end else begin
						timeout = EXEC_TIMEOUT;
					end
				end
			end
		end
		if (!primary_frame_done && !(as_asserted && prog_fetch && in_hand)) begin
			$display("FAIL j%0d t%0d r%0d: execution timeout, pc=%h state=%0d", jr, test_idx, round_idx, dbg_pc, core_state);
			errors = errors + 1;
			round_active = 0;
			disable run_round;
		end
		cap_sp = dbg_a7;
		nreset = 0;
		if (e_trace != 0 && !saw_trace) mismatch("missing trace", 9, 0);
		#1;
		// The v20 corpus generator (WinUAE cputest, mid 2020) runs the tested
		// instruction, then records the pending interrupt unconditionally:
		// it ignores the SR interrupt mask and replaces an exception of the
		// instruction itself (privilege violation, trace, trap) with the
		// interrupt, stacking the user-mode SR.  The MC68030 takes those in
		// priority order (UM 8.1, table 8-4) and does not take a masked
		// request, so such rounds are counted and not compared.
		if ((flags & F_V20) && i_level != 0 && ((e_exc >= 25 && e_exc <= 31) || (odd_vector != 0 && e_exc == 3)) &&
		    ((e_exc >= 25 && e_exc <= 31 && e_sr[10:8] >= i_level[2:0] && i_level[2:0] != 3'd7) ||
		     first_exc_vec == 8'd5 || first_exc_vec == 8'd6 || first_exc_vec == 8'd7 ||
		     first_exc_vec == 8'd8 || first_exc_vec == 8'd9 ||
		     (first_exc_vec >= 8'd32 && first_exc_vec <= 8'd47))) begin
			irq_order_skips = irq_order_skips + 1;
			for (k = 0; k < em_cnt; k = k + 1) apply_value(em_a[k], em_sz[k], em_old[k]);
		end else begin
			check_final;
			ran = ran + 1;
		end
		round_active = 0;
		ipl = 3'b111;
	end
endtask

integer limit, start_record;
reg [31:0] job_version, job_tbase, job_tsize;
reg [31:0] toggle_a, toggle_v;
reg [7:0] toggle_kind;
reg [31:0] dummy_fp;

initial begin
	hold_fetch = 0; boot_overlay = 0; round_active = 0; wr_seen = 0;
	ipl = 3'b111; nreset = 0; errors = 0; ran = 0; mism = 0; ipl_after = 0; irq_order_skips = 0;
	report_lim = 40;
	if (!$value$plusargs("trace_round=%d", trace_round)) trace_round = -1;
	for (k = 0; k < 32768; k = k + 1) lmem[k] = 0;
	for (k = 0; k < TMEM_MAX; k = k + 1) tmem[k] = 0;
	if (!$value$plusargs("job=%s", job_file) || !$value$plusargs("lmem=%s", lmem_file) ||
	    !$value$plusargs("tmem=%s", tmem_file)) begin
		$display("FAIL: require +job= +lmem= +tmem="); $finish;
	end
	if (!$value$plusargs("limit=%d", limit)) limit = 32'h7fffffff;
	if (!$value$plusargs("start=%d", start_record)) start_record = 0;
	jf = $fopen(job_file, "rb");
	if (!jf) begin $display("FAIL: cannot open APR2 job"); $finish; end
	if (jread32(0) !== "APR2") begin $display("FAIL: bad job magic"); $finish; end
	job_version = jread32(0); jn = jread32(0);
	job_tbase = jread32(0); job_tsize = jread32(0); odd_vector = jread32(0);
	if (job_version != 3 || job_tsize > TMEM_MAX) begin $display("FAIL: unsupported APR2 geometry/version"); $finish; end
	TBASE = job_tbase; TSIZE = job_tsize;
	lmfd = $fopen(lmem_file, "rb"); tmfd = $fopen(tmem_file, "rb");
	if (!lmfd || !tmfd) begin $display("FAIL: cannot open corpus memory images"); $finish; end
	fgot = $fread(lmem, lmfd); $fclose(lmfd);
	fgot = $fread(tmem, tmfd); $fclose(tmfd);
	if (jn > limit) jn = limit;
	$display("tb_cputest: %0d APR2 records", jn);

	for (jr = 0; jr < jn; jr = jr + 1) begin
		if (jread32(0) !== RND2) begin $display("FAIL: record desync at %0d", jr); $finish; end
		test_idx = jread32(0); round_idx = jread32(0); flags = jread32(0);
		for (k = 0; k < 16; k = k + 1) i_regs[k] = jread32(0);
		i_sr = jread32(0); i_pc = jread32(0); i_ssp = jread32(0); i_msp = jread32(0);
		for (k = 0; k < 8; k = k + 1) begin dummy_fp = jread32(0); dummy_fp = jread32(0); dummy_fp = jread32(0); end
		i_fpcr = jread32(0); i_fpsr = jread32(0); i_fpiar = jread32(0);
		i_level = jread8(0);
		read_apply_patches;
		n = jread16(0);
		for (k = 0; k < n; k = k + 1) begin
			toggle_a = jread32(0); toggle_kind = jread8(0);
			toggle_v = read_value(toggle_a, 2);
			if (toggle_kind == 1) apply_value(toggle_a, 2, {toggle_v[15:0], toggle_v[31:16]});
			else if (toggle_kind == 2) apply_value(toggle_a, 1, toggle_v[31:16] == 16'h2048 ? 16'h4afc : 16'h2048);
		end
		for (k = 0; k < 16; k = k + 1) e_regs[k] = jread32(0);
		e_sr = jread32(0); e_srmask = jread32(0);
		for (k = 0; k < 8; k = k + 1) begin dummy_fp = jread32(0); dummy_fp = jread32(0); dummy_fp = jread32(0); end
		e_fpcr = jread32(0); e_fpsr = jread32(0); e_fpiar = jread32(0);
		e_exc = jread8(0); expected_exc_live = e_exc; e_pc = jread32(0);
		e_trace = jread8(0); e_group2 = jread8(0);
		e_trace_sr = jread16(0); e_trace_srmask = jread16(0);
		e_trace_pc = jread32(0);
		frame_len = jread16(0);
		for (k = 0; k < frame_len; k = k + 1) frame_b[k] = jread8(0);
		for (k = 0; k < frame_len; k = k + 1) frame_m[k] = jread8(0);
		em_cnt = jread16(0);
		for (k = 0; k < em_cnt; k = k + 1) begin
			em_a[k] = jread32(0); em_sz[k] = jread8(0); em_v[k] = jread32(0); em_old[k] = jread32(0);
		end
		read_post_patches;
		read_clean_patches;
		if ((flags & F_IGNORE_EXC) || (flags & F_FPU) || jr < start_record) apply_deferred;
		else begin
			run_round;
			apply_deferred;
		end
	end
	$display("irq-order rounds not compared: %0d", irq_order_skips);
	$display("dat replay: %0d rounds, %0d mismatches, %0d harness errors", ran, mism, errors);
	if (mism == 0 && errors == 0) $display("ALL TESTS PASSED");
	else $display("TEST FAILED with %0d errors", mism + errors);
	$finish;
end

// a wedged round, not a long slice: jr must advance at least every 200 ms
integer watchdog_jr;
initial begin
	watchdog_jr = -1;
	forever begin
		#200_000_000;
		if (jr === watchdog_jr) begin $display("FAIL: no corpus progress at record %0d", jr); $finish; end
		watchdog_jr = jr;
	end
end

endmodule
