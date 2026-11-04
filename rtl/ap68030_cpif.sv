//--------------------------------------------------------------------------//
// AP68030 (070) - the pipelined core with the MC68030 bus                  //
//                                                                          //
// ap68030_cpif.sv - the MC68030 coprocessor interface (UM section 10) for  //
// the build without the on-chip FPU: F-line instructions with CpID 1-7    //
// run the coprocessor protocol over CPU space type 2 cycles.               //
//                                                                          //
// A port of AP68030's coprocessor states (rtl/core/ap030_exec_c.vh) to a   //
// sequential engine beside the pipeline.  The back end starts it with the  //
// instruction's operation word and address once every older instruction   //
// has completed (its uop waits at WB, the pipeline behind it is discarded  //
// afterwards); the engine reads the instruction stream itself (the         //
// coprocessor decides how many words an operand takes), evaluates EAs,     //
// moves operands and registers, and ends with the next PC or with an       //
// exception for the back end to take:                                      //
//   fin      the instruction is done: npc, and SR if sr_we                 //
//   xreq     an exception: xvec, xfmt ($0 $2 $9 $B), xpc, xia (format     //
//            $2/$9 instruction address, $B the fault address), xint/xea    //
//            ($9: the protocol state and the evaluated EA), xfslw ($B)     //
//   xirq     the pending interrupt is taken now: xfmt $0 (busy: the       //
//            instruction restarts, xpc = its address) or $9 (come again    //
//            with IA: the mid-instruction frame, xpc = scanPC)             //
// RTE of a $9 frame resumes the dialogue (resume, with the frame's words). //
//                                                                          //
// The register file is read through rra/rrd (combinational, used the next  //
// clock) and written through rwe/rwa/rwd; its A7 is the active stack       //
// pointer of SR.  Bus transfers are naturally aligned (the BIU's): an      //
// operand is split into pieces here.                                       //
//--------------------------------------------------------------------------//
module ap68030_cpif
	import ap68060_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,

	input  logic        start,          // pulse: run the instruction at pc_i
	input  logic        resume,         // pulse: RTE of a $9 frame
	input  logic [15:0] op,             // start: the operation word
	input  logic [31:0] pc_i,           // the instruction's address
	input  logic [31:0] rs_pc,          // resume: the frame's PC (scanPC)
	input  logic [31:0] rs_int,         // resume: the frame's protocol state word
	input  logic [31:0] rs_ea,          // resume: the frame's EA
	input  logic [15:0] sr,             // the SR (with the CCR) the instruction sees
	input  logic        irq_pend,       // an interrupt above the mask is pending

	output logic        fin,
	output logic        fin_tr,         // ... and its trace is due (T1; T0 with a change of flow)
	output logic [31:0] npc,
	output logic        sr_we,
	output logic [15:0] sr_wv,
	output logic        xreq,
	output logic        xirq,
	output logic  [7:0] xvec,
	output logic  [3:0] xfmt,
	output logic [31:0] xpc,
	output logic [31:0] xia,
	output logic [31:0] xint,
	output logic [31:0] xea,
	output logic [31:0] xfslw,
	output logic        x_tr,           // the exception's handler entry is traced (an instruction
	                                    // trap, a post-instruction exception)
	output logic        busy,

	// the back register file
	output logic  [4:0] rra,
	input  logic [31:0] rrd,
	output logic        rwe,
	output logic  [4:0] rwa,
	output logic [31:0] rwd,
	// the control registers (transfer main processor control register)
	input  logic  [2:0] c_sfc, c_dfc,
	input  logic [31:0] c_cacr, c_vbr, c_caar,
	output logic        cw_we,
	output logic  [7:0] cw_sel,         // the special register selector (MOVEC's)
	output logic [31:0] cw_val,

	// the bus: a BIU requester
	output logic        b_req,
	output busreq_t     b_breq,
	output logic [127:0] b_wdata,
	input  logic        b_gnt,
	input  logic        b_done,
	input  logic        b_err,
	input  logic        b_rvalid,
	input  logic [31:0] b_rdata
);

localparam logic [7:0] VEC_BERR = 8'd2, VEC_ADDR = 8'd3, VEC_PRIV = 8'd8, VEC_FLINE = 8'd11,
                       VEC_CPPROTO = 8'd13, VEC_FMTERR = 8'd14, VEC_TRAPCC = 8'd7;

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
typedef enum logic [6:0] {
	C_IDLE, C_EXT, C_EAPRE, C_CP0, C_CP1, C_CP2, C_CPWAIT, C_ABORT, C_EXC, C_NULL,
	C_BCC2, C_DBCC2, C_DBCC3, C_SCC2, C_SCCWR, C_TRAP2,
	C_EAX, C_ISTREAM, C_XEA, C_XEA2, C_XFER, C_XFER2, C_XFER3,
	C_TOS, C_TOS2, C_TOSR, C_REG, C_REG2, C_CTRL, C_CTRL2,
	C_REGS, C_REGS2, C_REGS3, C_MULT, C_MULT2, C_MULT3, C_MULT4, C_MSAVE, C_MREST,
	C_SR, C_SR2, C_SR3,
	C_SAVE0, C_SAVE1, C_SVBODY, C_SVWR, C_REST0, C_REST1, C_REST2, C_REST3, C_RSBODY, C_RSWR,
	C_EA, C_EA1, C_EA2, C_EA3, C_EA4, C_EA5, C_EA6, C_EA7,
	C_ACC, C_ACCW, C_RDREG
} cst_t;
cst_t        st, ret, ea_ret, acc_ret, rd_ret;

logic [15:0] ir, ext;
logic [31:0] pci, scan, ea, ea2, tmp, tmp2;
logic  [3:0] cp_kind;                // 0 cpGEN 1 cpBcc 2 cpDBcc 3 cpScc 4 cpTRAPcc
logic  [2:0] cp_id;
logic        cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, flt_fline;
logic [15:0] cp_resp;
logic  [7:0] cp_len, cp_pos, cnt;
logic [15:0] mm_mask;
logic  [1:0] xsub;                   // C_EXC: 0 pre, 1 mid, 2 post
logic  [7:0] avec;                   // C_ABORT: the vector
logic        tr_t1, tr_t0, flow;

// the EA field of the operation word
wire  [2:0] em = ir[5:3], er = ir[2:0];
wire  ea_dn   = (em == 3'd0);
wire  ea_an   = (em == 3'd1);
wire  ea_ind  = (em == 3'd2);
wire  ea_pi   = (em == 3'd3);
wire  ea_pd   = (em == 3'd4);
wire  ea_d16  = (em == 3'd5);
wire  ea_idx  = (em == 3'd6);
wire  ea_absw = (em == 3'd7) && (er == 3'd0);
wire  ea_absl = (em == 3'd7) && (er == 3'd1);
wire  ea_pcd  = (em == 3'd7) && (er == 3'd2);
wire  ea_pci  = (em == 3'd7) && (er == 3'd3);
wire  ea_imm  = (em == 3'd7) && (er == 3'd4);
wire  ea_bad  = (em == 3'd7) && (er > 3'd4);
wire  ea_mem     = ea_ind | ea_pi | ea_pd | ea_d16 | ea_idx | ea_absw | ea_absl | ea_pcd | ea_pci;
wire  ea_memalt  = ea_ind | ea_pi | ea_pd | ea_d16 | ea_idx | ea_absw | ea_absl;
wire  ea_ctrl    = ea_ind | ea_d16 | ea_idx | ea_absw | ea_absl | ea_pcd | ea_pci;
wire  ea_ctrlalt = ea_ind | ea_d16 | ea_idx | ea_absw | ea_absl;
wire  ea_data    = ea_dn | ea_mem | ea_imm;
wire  ea_dataalt = ea_dn | ea_memalt;
wire  ea_alt     = ea_dn | ea_an | ea_memalt;
wire  ea_any     = !ea_bad;

function automatic logic cp_ea_ok(input logic [2:0] cat);
	case (cat)
		3'b000:  cp_ea_ok = ea_ctrlalt;
		3'b001:  cp_ea_ok = ea_dataalt;
		3'b010:  cp_ea_ok = ea_memalt;
		3'b011:  cp_ea_ok = ea_alt;
		3'b100:  cp_ea_ok = ea_ctrl;
		3'b101:  cp_ea_ok = ea_data;
		3'b110:  cp_ea_ok = ea_mem | ea_imm;
		default: cp_ea_ok = ea_any;
	endcase
endfunction

// register numbers: D0-D7, A0-A6, A7 = the active stack pointer
wire  [4:0] sp_reg = !sr[13] ? R_USP : sr[12] ? R_MSP : R_ISP;
function automatic logic [4:0] rnum(input logic [3:0] r);   // {A, n}
	rnum = r[3] ? ((r[2:0] == 3'd7) ? sp_reg : (R_A0 + {2'b00, r[2:0]})) : {2'b00, r[2:0]};
endfunction
wire  [2:0] fc_data = sr[13] ? 3'd5 : 3'd1;
wire  [2:0] fc_prog = sr[13] ? 3'd6 : 3'd2;

// the CIR address of offset o
function automatic logic [31:0] cir(input logic [4:0] o);
	cir = 32'h0002_0000 | {16'd0, cp_id, 13'd0} | {27'd0, o};
endfunction

function automatic logic [31:0] sext16(input logic [15:0] v);
	sext16 = {{16{v[15]}}, v};
endfunction

//--------------------------------------------------------------------------
// the access unit: n (1-4) bytes at a (any alignment), FC fc, as aligned
// pieces; a read's bytes come back right justified in acc_rd
//--------------------------------------------------------------------------
logic [31:0] acc_a, acc_wd, acc_rd;
logic  [2:0] acc_n, acc_fc, acc_pn;
logic        acc_w, acc_err, acc_cir, acc_on;
logic  [2:0] acc_got;
wire  [2:0] acc_np = acc_a[0] ? 3'd1 :
                     acc_a[1] ? ((acc_n >= 3'd2) ? 3'd2 : 3'd1) :
                     (acc_n >= 3'd4) ? 3'd4 : (acc_n >= 3'd2) ? 3'd2 : 3'd1;
// the piece's write data on the lanes (the BIU takes them placed)
wire [31:0] acc_pw = acc_wd >> (8 * (acc_n - acc_np));
always_comb begin
	b_breq      = '0;
	b_breq.addr = acc_a;
	b_breq.siz  = (acc_np == 3'd4) ? SIZ_L : (acc_np == 3'd2) ? SIZ_W : SIZ_B;
	b_breq.rd   = !acc_w;
	b_breq.tt   = TT_NORMAL;
	b_breq.tm   = acc_fc;
	b_wdata     = {96'd0, (acc_np == 3'd1) ? {4{acc_pw[7:0]}} :
	                      (acc_np == 3'd2) ? {2{acc_pw[15:0]}} : acc_pw};
end
// a piece's bytes from the lanes
function automatic logic [31:0] grab(input logic [31:0] l, input logic [2:0] n, input logic [1:0] a);
	case (n)
		3'd1:    grab = {24'd0, l[31 - 8 * a -: 8]};
		3'd2:    grab = {16'd0, a[1] ? l[15:0] : l[31:16]};
		default: grab = l;
	endcase
endfunction

//--------------------------------------------------------------------------
// the sequencer
//--------------------------------------------------------------------------
logic [31:0] eb, ex;                 // EA calculation: base, index
logic [15:0] eext;
logic        e_pc, e_full;
logic  [1:0] e_bds, e_ods;
logic  [2:0] e_iis;
logic        e_is, e_bs;
logic        s_wait;                 // C_RDREG / stream word: one clock for rrd

// helper tasks (blocking: they set the state machine's next step)
task automatic acc(input logic [31:0] a, input logic [2:0] n, input logic w, input logic [31:0] wd,
                   input logic [2:0] fc, input logic is_cir, input cst_t r);
	acc_a <= a; acc_n <= n; acc_w <= w; acc_wd <= wd; acc_fc <= fc; acc_cir <= is_cir;
	acc_rd <= 32'd0; acc_got <= 3'd0; acc_ret <= r;
	st <= C_ACC;
endtask
task automatic cir_rd(input logic [4:0] o, input logic [2:0] n, input cst_t r);
	acc(cir(o), n, 1'b0, 32'd0, 3'd7, 1'b1, r);
endtask
task automatic cir_wr(input logic [4:0] o, input logic [2:0] n, input logic [31:0] v, input cst_t r);
	acc(cir(o), n, 1'b1, v, 3'd7, 1'b1, r);
endtask
task automatic rdw(input cst_t r);                 // the next instruction stream word
	acc(scan, 3'd2, 1'b0, 32'd0, fc_prog, 1'b0, r);
	scan <= scan + 32'd2;
endtask
task automatic regrd(input logic [3:0] r, input cst_t nx);
	rra <= rnum(r); rd_ret <= nx; st <= C_RDREG;
endtask
task automatic regwr(input logic [3:0] r, input logic [31:0] v);
	rwe <= 1'b1; rwa <= rnum(r); rwd <= v;
endtask
task automatic xgo(input logic [7:0] v, input logic [3:0] f, input logic [31:0] p, input logic [31:0] ia);
	xreq <= 1'b1; xvec <= v; xfmt <= f; xpc <= p; xia <= ia;
	xint <= {tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id, 1'b0, ir};
	xea  <= ea;
	x_tr <= 1'b0;
	st   <= C_IDLE;
endtask
task automatic proto;
	xgo(VEC_CPPROTO, 4'h9, scan, pci);
endtask
// (the trace: T1, T0 with a change of flow, or one an SR transfer made due)
task automatic finish(input logic [31:0] p);
	fin <= 1'b1; npc <= p; st <= C_IDLE;
	fin_tr <= tr_t1 || (tr_t0 && flow) || cp_trace_wait;
endtask
task automatic goto_pc(input logic [31:0] p);
	if (p[0]) xgo(VEC_ADDR, 4'hB, pci, p);   // an odd target: the address error (frame PC: the instruction)
	else begin finish(p); fin_tr <= tr_t1 || tr_t0 || cp_trace_wait; end
endtask

assign busy = (st != C_IDLE);

always_ff @(posedge clk) begin
	fin <= 1'b0; xreq <= 1'b0; xirq <= 1'b0; rwe <= 1'b0; cw_we <= 1'b0; sr_we <= 1'b0;
	if (!nreset) begin
		st <= C_IDLE; b_req <= 1'b0; acc_on <= 1'b0;
		ir <= '0; ext <= '0; pci <= '0; scan <= '0; ea <= '0; ea2 <= '0; tmp <= '0; tmp2 <= '0;
		cp_kind <= '0; cp_id <= '0; cp_cond <= 1'b0; cp_dr <= 1'b0; cp_ca <= 1'b0; cp_pcbit <= 1'b0;
		cp_trace_wait <= 1'b0; flt_fline <= 1'b0; cp_resp <= '0; cp_len <= '0; cp_pos <= '0; cnt <= '0;
		mm_mask <= '0; xsub <= '0; avec <= '0; tr_t1 <= 1'b0; tr_t0 <= 1'b0; flow <= 1'b0;
		npc <= '0; fin_tr <= 1'b0; x_tr <= 1'b0; xvec <= '0; xfmt <= '0; xpc <= '0; xia <= '0; xint <= '0; xea <= '0; xfslw <= '0;
		rra <= '0; rwa <= '0; rwd <= '0; cw_sel <= '0; cw_val <= '0; sr_wv <= '0;
		acc_a <= '0; acc_wd <= '0; acc_rd <= '0; acc_n <= '0; acc_fc <= '0; acc_pn <= '0;
		acc_w <= 1'b0; acc_err <= 1'b0; acc_cir <= 1'b0; acc_got <= '0;
		ret <= C_IDLE; ea_ret <= C_IDLE; acc_ret <= C_IDLE; rd_ret <= C_IDLE;
		eb <= '0; ex <= '0; eext <= '0; e_pc <= 1'b0; e_full <= 1'b0; e_bds <= '0; e_ods <= '0;
		e_iis <= '0; e_is <= 1'b0; e_bs <= 1'b0; s_wait <= 1'b0;
	end
	else begin
		case (st)
		//==============================================================
		// start: the second word where the instruction has one, then the
		// memory EA of cpSAVE/cpRESTORE
		//==============================================================
		C_IDLE: begin
			if (start) begin
				ir <= op; pci <= pc_i; scan <= pc_i + 32'd2; cnt <= 8'd0;
				cp_id <= op[11:9];
				cp_cond <= 1'b0; cp_dr <= 1'b0; cp_ca <= 1'b0; cp_pcbit <= 1'b0; cp_trace_wait <= 1'b0;
				tr_t1 <= sr[15]; tr_t0 <= sr[14]; flow <= 1'b0;
				ea <= 32'd0;
				case (op[8:6])
					3'b000, 3'b001: begin
						// (scan is set this clock: the address directly)
						acc(pc_i + 32'd2, 3'd2, 1'b0, 32'd0, sr[13] ? 3'd6 : 3'd2, 1'b0, C_EXT);
						scan <= pc_i + 32'd4;
					end
					3'b010, 3'b011: st <= C_CP0;
					default: st <= C_EAPRE;
				endcase
			end
			else if (resume) begin
				// RTE of a $9 frame: the dialogue again from the response
				ir <= rs_int[15:0]; pci <= pc_i; scan <= rs_pc; ea <= rs_ea;
				{tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id} <= rs_int[31:17];
				tr_t1 <= sr[15]; tr_t0 <= sr[14];
				st <= C_CP1;
			end
		end
		C_EXT: begin ext <= acc_rd[15:0]; st <= C_CP0; end
		C_EAPRE: begin
			// cpSAVE (-(An) or control alterable), cpRESTORE ((An)+ or control)
			if (ir[8:6] == 3'b100) begin
				if (ea_pd) st <= C_SAVE0;
				else begin ea_ret <= C_SAVE0; st <= C_EA; end
			end
			else begin
				if (ea_pi) regrd({1'b1, ir[2:0]}, C_REST0);
				else begin ea_ret <= C_REST0; st <= C_EA; end
			end
		end

		//==============================================================
		// instruction start: the command or condition to the coprocessor
		//==============================================================
		C_CP0: begin
			flt_fline <= 1'b1;
			case (ir[8:6])
				3'b000: begin
					cp_kind <= 4'd0; cp_cond <= 1'b0;
					cir_wr(5'h0A, 3'd2, {16'd0, ext}, C_CP1);
				end
				3'b001: begin
					cp_cond <= 1'b1;
					cp_kind <= ea_an ? 4'd2 : (em == 3'd7 && er[2:1] != 2'b00) ? 4'd4 : 4'd3;
					cir_wr(5'h0E, 3'd2, {16'd0, ext}, C_CP1);
				end
				default: begin
					cp_cond <= 1'b1; cp_kind <= 4'd1;
					cir_wr(5'h0E, 3'd2, {16'd0, ir}, C_CP1);
				end
			endcase
		end

		//==============================================================
		// the response dialogue
		//==============================================================
		C_CP1: begin
			flt_fline <= 1'b0;
			cir_rd(5'h00, 3'd2, C_CP2);
		end
		C_CP2: begin
			logic [15:0] r;
			r = acc_rd[15:0];
			cp_resp <= r;
			cp_ca <= r[15]; cp_dr <= r[13];
			if (r[14] && !cp_pcbit) begin
				// PC bit: the instruction address to its CIR first
				cp_pcbit <= 1'b1;
				acc(cir(5'h18), 3'd4, 1'b1, pci, 3'd7, 1'b1, C_CP2);
				acc_rd <= {16'd0, r};             // (the response, for C_CP2 again)
			end
			else begin
				cp_pcbit <= 1'b0;
				casez (r[12:8])
					5'b00000: begin
						// write to previously evaluated EA
						if (cp_cond || !r[13]) proto();
						else begin cp_len <= r[7:0]; cp_pos <= 8'd0; cp_dr <= 1'b1; st <= C_XFER; end
					end
					5'b00001: begin
						// transfer multiple coprocessor registers
						if (cp_cond || r[0]) proto();
						else if (!(r[13] ? (ea_ctrlalt || ea_pd) : (ea_ctrl || ea_pi))) begin
							avec <= VEC_FLINE; cir_wr(5'h02, 3'd2, 32'h0001, C_ABORT);
						end
						else begin cp_len <= r[7:0]; st <= C_MULT; end
					end
					5'b00100: begin
						if (r[13]) begin
							// busy: an interrupt is taken with the instruction
							// restarted, else the instruction again
							if (irq_pend) begin xirq <= 1'b1; xfmt <= 4'h0; xpc <= pci; st <= C_IDLE; end
							else finish(pci);
						end
						else begin
							// supervisor check
							if (cp_cond && !r[15]) proto();
							else if (!sr[13]) begin avec <= VEC_PRIV; cir_wr(5'h02, 3'd2, 32'h0001, C_ABORT); end
							else st <= C_CP1;
						end
					end
					5'b00101: begin
						// take address and transfer data
						if (cp_cond && !r[15]) proto();
						else begin
							cp_len <= r[7:0]; cp_pos <= 8'd0;
							cir_rd(5'h1C, 3'd4, C_XEA2);
						end
					end
					5'b00110: begin
						// transfer multiple main processor registers
						if (cp_cond && !r[15]) proto();
						else cir_rd(5'h14, 3'd2, C_REGS);
					end
					5'b00111: begin
						// transfer operation word
						if (cp_cond && !r[15]) proto();
						else cir_wr(5'h08, 3'd2, {16'd0, ir}, C_CPWAIT);
					end
					5'b0100?: begin
						// null
						if (r[13]) proto();
						else st <= C_NULL;
					end
					5'b01010: begin
						// evaluate and transfer effective address
						if (cp_cond || r[13]) proto();
						else if (!ea_ctrlalt) begin avec <= VEC_FLINE; cir_wr(5'h02, 3'd2, 32'h0001, C_ABORT); end
						else begin ea_ret <= C_EAX; st <= C_EA; end
					end
					5'b01100: begin
						// transfer single main processor register
						if (cp_cond && !r[15]) proto();
						else st <= C_REG;
					end
					5'b01101: begin
						// transfer main processor control register
						if (cp_cond && !r[15]) proto();
						else cir_rd(5'h14, 3'd2, C_CTRL);
					end
					5'b01110: begin
						// transfer to/from top of stack
						if ((cp_cond && !r[15]) || !(r[7:0] == 8'd1 || r[7:0] == 8'd2 || r[7:0] == 8'd4)) proto();
						else begin cp_len <= r[7:0]; regrd(4'hF, C_TOS); end
					end
					5'b01111: begin
						// transfer from instruction stream
						if ((cp_cond && !r[15]) || r[0]) proto();
						else begin cp_len <= r[7:0]; cp_pos <= 8'd0; st <= C_ISTREAM; end
					end
					5'b0001?: begin
						// transfer status register and scanPC
						if (cp_cond) proto();
						else st <= C_SR;
					end
					5'b10???: begin
						// evaluate effective address and transfer data
						if (cp_cond) proto();
						else if (!cp_ea_ok(r[10:8])) begin
							avec <= VEC_FLINE; cir_wr(5'h02, 3'd2, 32'h0001, C_ABORT);
						end
						else if ((r[13] && !ea_alt) ||
						         ((ea_dn || ea_an) && !(r[7:0] == 8'd1 || r[7:0] == 8'd2 || r[7:0] == 8'd4)) ||
						         (ea_imm && r[0] && r[7:0] != 8'd1))
							proto();
						else begin cp_len <= r[7:0]; cp_pos <= 8'd0; st <= C_XEA; end
					end
					5'b11100: begin avec <= r[7:0]; xsub <= 2'd0; cir_wr(5'h02, 3'd2, 32'h0002, C_EXC); end
					5'b11101: begin avec <= r[7:0]; xsub <= 2'd1; cir_wr(5'h02, 3'd2, 32'h0002, C_EXC); end
					5'b11110: begin avec <= r[7:0]; xsub <= 2'd2; cir_wr(5'h02, 3'd2, 32'h0002, C_EXC); end
					default: proto();
				endcase
			end
		end
		C_CPWAIT: begin
			if (cp_ca) st <= C_CP1;
			else if (cp_cond) proto();
			else if (tr_t1 || cp_trace_wait) st <= C_CP1;
			else finish(scan);
		end
		C_ABORT: xgo(avec, 4'h0, pci, 32'd0);
		C_EXC: begin
			case (xsub)
				2'd0:    xgo(avec, 4'h0, pci, 32'd0);
				2'd1:    xgo(avec, 4'h9, scan, pci);
				default: begin xgo(avec, 4'h2, scan, pci); x_tr <= tr_t1 || cp_trace_wait; end
			endcase
		end
		C_NULL: begin
			if (cp_resp[15]) begin
				// come again; IA allows an interrupt: the mid-instruction frame
				if (cp_resp[8] && irq_pend) begin
					xirq <= 1'b1; xfmt <= 4'h9; xpc <= scan; xia <= pci;
					xint <= {tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id, 1'b0, ir};
					xea <= ea;
					st <= C_IDLE;
				end
				else st <= C_CP1;
			end
			else if (cp_cond) begin
				cnt <= 8'd0;
				case (cp_kind)
					4'd1:    st <= C_BCC2;
					4'd2:    st <= C_DBCC2;
					4'd3:    st <= C_SCC2;
					default: st <= C_TRAP2;
				endcase
			end
			else if ((tr_t1 || cp_trace_wait) && !cp_resp[1]) begin
				if (cp_resp[8] && irq_pend) begin
					xirq <= 1'b1; xfmt <= 4'h9; xpc <= scan; xia <= pci;
					xint <= {tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id, 1'b0, ir};
					xea <= ea;
					st <= C_IDLE;
				end
				else st <= C_CP1;
			end
			else finish(scan);
		end

		//==============================================================
		// conditional completions
		//==============================================================
		C_BCC2: begin
			// cpBcc: the displacement (one or two words) after the opword
			if (ir[6]) begin
				if (cnt == 8'd0) begin cnt <= 8'd1; rdw(C_BCC2); end
				else if (cnt == 8'd1) begin cnt <= 8'd2; tmp2 <= {16'd0, acc_rd[15:0]}; rdw(C_BCC2); end
				else begin
					cnt <= 8'd0;
					if (cp_resp[0]) goto_pc(pci + 32'd2 + {tmp2[15:0], acc_rd[15:0]});
					else finish(scan);
				end
			end
			else begin
				if (cnt == 8'd0) begin cnt <= 8'd1; rdw(C_BCC2); end
				else begin
					cnt <= 8'd0;
					if (cp_resp[0]) goto_pc(pci + 32'd2 + sext16(acc_rd[15:0]));
					else finish(scan);
				end
			end
		end
		C_DBCC2: begin tmp2 <= 32'd0; rdw(C_DBCC3); end     // the displacement word
		C_DBCC3: begin
			if (!s_wait) begin
				tmp2 <= {16'd0, acc_rd[15:0]};
				rra <= rnum({1'b0, ir[2:0]}); s_wait <= 1'b1;
			end
			else begin
				logic [15:0] nw;
				s_wait <= 1'b0;
				nw = rrd[15:0] - 16'd1;
				if (cp_resp[0]) finish(scan);
				else begin
					regwr({1'b0, ir[2:0]}, {rrd[31:16], nw});
					if (nw != 16'hFFFF) goto_pc(scan - 32'd2 + sext16(tmp2[15:0]));
					else finish(scan);
				end
			end
		end
		C_SCC2: begin
			if (ea_dn) begin
				if (!s_wait) begin rra <= rnum({1'b0, ir[2:0]}); s_wait <= 1'b1; end
				else begin
					s_wait <= 1'b0;
					regwr({1'b0, ir[2:0]}, {rrd[31:8], {8{cp_resp[0]}}});
					finish(scan);
				end
			end
			else if (ea_pi || ea_pd) begin
				// (An)+ / -(An): a byte (A7: a word step)
				if (!s_wait) begin rra <= rnum({1'b1, ir[2:0]}); s_wait <= 1'b1; end
				else begin
					logic [31:0] st8;
					s_wait <= 1'b0;
					st8 = (ir[2:0] == 3'd7) ? 32'd2 : 32'd1;
					ea <= ea_pi ? rrd : (rrd - st8);
					regwr({1'b1, ir[2:0]}, ea_pi ? (rrd + st8) : (rrd - st8));
					st <= C_SCCWR;
				end
			end
			else begin ea_ret <= C_SCCWR; st <= C_EA; end
		end
		C_SCCWR: begin
			if (!acc_on) begin
				acc_on <= 1'b1;
				acc(ea, 3'd1, 1'b1, {24'd0, {8{cp_resp[0]}}}, fc_data, 1'b0, C_SCCWR);
			end
			else begin acc_on <= 1'b0; finish(scan); end
		end
		C_TRAP2: begin
			// cpTRAPcc: its operand words skipped, the trap when true
			if (er == 3'd2 && cnt == 8'd0) begin cnt <= 8'd1; rdw(C_TRAP2); end
			else if (er == 3'd3 && cnt < 8'd2) begin cnt <= cnt + 8'd1; rdw(C_TRAP2); end
			else begin
				cnt <= 8'd0;
				if (cp_resp[0]) begin xgo(VEC_TRAPCC, 4'h2, scan, pci); x_tr <= tr_t1 || tr_t0; end
				else finish(scan);
			end
		end

		//==============================================================
		// transfers
		//==============================================================
		C_EAX: cir_wr(5'h1C, 3'd4, ea, C_CPWAIT);
		C_ISTREAM: begin
			// operand words from the instruction stream to the operand CIR
			logic [7:0] rem;
			rem = cp_len - cp_pos;
			if (!acc_on) begin
				if (rem == 8'd0) st <= C_CPWAIT;
				else if (rem >= 8'd4) begin
					acc_on <= 1'b1; cnt <= 8'd4;
					acc(scan, 3'd4, 1'b0, 32'd0, fc_prog, 1'b0, C_ISTREAM);
					scan <= scan + 32'd4;
				end
				else begin
					acc_on <= 1'b1; cnt <= 8'd2;
					acc(scan, 3'd2, 1'b0, 32'd0, fc_prog, 1'b0, C_ISTREAM);
					scan <= scan + 32'd2;
				end
			end
			else begin
				acc_on <= 1'b0;
				cp_pos <= cp_pos + cnt;
				cir_wr(5'h10, cnt[2:0], acc_rd, C_ISTREAM);
			end
		end
		C_XEA: begin
			// evaluate EA and transfer data: register direct, immediate, memory
			logic [31:0] bytes, stp;
			bytes = {24'd0, cp_len};
			stp = (cp_len == 8'd1 && ir[2:0] == 3'd7) ? 32'd2 : bytes;
			if (ea_dn || ea_an) begin
				if (!s_wait) begin rra <= rnum({ea_an, ir[2:0]}); s_wait <= 1'b1; end
				else begin
					s_wait <= 1'b0;
					tmp <= rrd;
					if (!cp_dr) cir_wr(5'h10, cp_len[2:0], rrd, C_CPWAIT);
					else cir_rd(5'h10, cp_len[2:0], C_REG2);
				end
			end
			else if (ea_imm) begin
				cp_pos <= 8'd0;
				if (cp_len == 8'd1) begin
					// a byte takes a word of the stream
					if (!acc_on) begin acc_on <= 1'b1; rdw(C_XEA); end
					else begin acc_on <= 1'b0; cir_wr(5'h10, 3'd1, {24'd0, acc_rd[7:0]}, C_CPWAIT); end
				end
				else st <= C_ISTREAM;
			end
			else if (ea_pi || ea_pd) begin
				if (!s_wait) begin rra <= rnum({1'b1, ir[2:0]}); s_wait <= 1'b1; end
				else begin
					s_wait <= 1'b0;
					ea <= ea_pi ? rrd : (rrd - stp);
					regwr({1'b1, ir[2:0]}, ea_pi ? (rrd + stp) : (rrd - stp));
					st <= C_XFER;
				end
			end
			else begin ea_ret <= C_XFER; st <= C_EA; end
		end
		C_XEA2: begin tmp2 <= acc_rd; st <= C_XFER; end      // take address: the operand address
		C_XFER: begin
			// memory <-> operand CIR in long word parts, ascending
			logic [7:0] rem;
			logic [2:0] chunk;
			rem = cp_len - cp_pos;
			chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
			if (rem == 8'd0) st <= C_CPWAIT;
			else if (cp_dr) cir_rd(5'h10, chunk, C_XFER3);
			else if (cp_resp[12:8] == 5'b00101)
				acc(tmp2 + {24'd0, cp_pos}, chunk, 1'b0, 32'd0, fc_data, 1'b0, C_XFER2);
			else
				acc(ea + {24'd0, cp_pos}, chunk, 1'b0, 32'd0, e_pc ? fc_prog : fc_data, 1'b0, C_XFER2);
		end
		C_XFER2: begin
			logic [7:0] rem; logic [2:0] chunk;
			rem = cp_len - cp_pos; chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
			cp_pos <= cp_pos + {5'd0, chunk};
			cir_wr(5'h10, chunk, acc_rd, C_XFER);
		end
		C_XFER3: begin
			logic [7:0] rem; logic [2:0] chunk;
			rem = cp_len - cp_pos; chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
			cp_pos <= cp_pos + {5'd0, chunk};
			acc(((cp_resp[12:8] == 5'b00101) ? tmp2 : ea) + {24'd0, cp_pos}, chunk, 1'b1, acc_rd,
			    fc_data, 1'b0, C_XFER);
		end
		C_TOS: begin
			// to/from the top of the stack (A7, the active one), data space
			logic [31:0] stp;
			stp = (cp_len == 8'd1) ? 32'd2 : {24'd0, cp_len};
			if (!cp_dr) begin
				regwr(4'hF, rrd + stp);
				acc(rrd, cp_len[2:0], 1'b0, 32'd0, fc_data, 1'b0, C_TOS2);
			end
			else begin
				tmp2 <= rrd - stp;
				regwr(4'hF, rrd - stp);
				cir_rd(5'h10, cp_len[2:0], C_TOSR);
			end
		end
		C_TOS2: cir_wr(5'h10, cp_len[2:0], acc_rd, C_CPWAIT);
		C_TOSR: acc(tmp2, cp_len[2:0], 1'b1, acc_rd, fc_data, 1'b0, C_CPWAIT);
		C_REG: begin
			// transfer single main processor register
			if (!s_wait) begin rra <= rnum(cp_resp[3:0]); s_wait <= 1'b1; end
			else begin
				s_wait <= 1'b0;
				tmp <= rrd;
				if (!cp_dr) cir_wr(5'h10, 3'd4, rrd, C_CPWAIT);
				else cir_rd(5'h10, 3'd4, C_REG2);
			end
		end
		C_REG2: begin
			// a register from the operand CIR: C_REG's (D/A, number) or
			// C_XEA's register direct EA (a data register keeps its bits
			// above the size)
			if (cp_resp[12:8] == 5'b01100) regwr(cp_resp[3:0], acc_rd);
			else if (ea_an) regwr({1'b1, ir[2:0]}, (cp_len == 8'd2) ? sext16(acc_rd[15:0]) :
			                                        (cp_len == 8'd1) ? {{24{acc_rd[7]}}, acc_rd[7:0]} : acc_rd);
			else regwr({1'b0, ir[2:0]}, (cp_len == 8'd1) ? {tmp[31:8], acc_rd[7:0]} :
			                            (cp_len == 8'd2) ? {tmp[31:16], acc_rd[15:0]} : acc_rd);
			st <= C_CPWAIT;
		end
		C_CTRL: begin
			// the control register select code (UM table 10-5)
			logic [11:0] sel;
			sel = acc_rd[11:0];
			tmp <= acc_rd;
			case (sel)
				12'h000: begin tmp2 <= {29'd0, c_sfc}; st <= C_CTRL2; end
				12'h001: begin tmp2 <= {29'd0, c_dfc}; st <= C_CTRL2; end
				12'h002: begin tmp2 <= c_cacr; st <= C_CTRL2; end
				12'h801: begin tmp2 <= c_vbr; st <= C_CTRL2; end
				12'h802: begin tmp2 <= c_caar; st <= C_CTRL2; end
				12'h800, 12'h803, 12'h804: begin
					rra <= (sel == 12'h800) ? R_USP : (sel == 12'h803) ? R_MSP : R_ISP;
					s_wait <= 1'b1; st <= C_CTRL2;
				end
				default: proto();
			endcase
		end
		C_CTRL2: begin
			logic [31:0] v;
			v = s_wait ? rrd : tmp2;
			s_wait <= 1'b0;
			if (!cp_dr) cir_wr(5'h10, 3'd4, v, C_CPWAIT);
			else if (!acc_on) begin acc_on <= 1'b1; cir_rd(5'h10, 3'd4, C_CTRL2); end
			else begin
				acc_on <= 1'b0;
				case (tmp[11:0])
					12'h800: begin rwe <= 1'b1; rwa <= R_USP; rwd <= acc_rd; end
					12'h803: begin rwe <= 1'b1; rwa <= R_MSP; rwd <= acc_rd; end
					12'h804: begin rwe <= 1'b1; rwa <= R_ISP; rwd <= acc_rd; end
					default: begin
						cw_we  <= 1'b1;
						cw_sel <= (tmp[11:0] == 12'h801) ? 8'h11 : (tmp[11:0] == 12'h802) ? 8'h12 : {6'd0, tmp[1:0]};
						cw_val <= acc_rd;
					end
				endcase
				st <= C_CPWAIT;
			end
		end
		C_REGS: begin mm_mask <= acc_rd[15:0]; st <= C_REGS2; end
		C_REGS2: begin
			// transfer multiple main processor registers: D0..D7, A0..A7
			logic [3:0] r;
			r = 4'd0;
			for (int i = 15; i >= 0; i--) if (mm_mask[i]) r = 4'(i);
			if (mm_mask == 16'd0) st <= C_CPWAIT;
			else begin
				mm_mask[r] <= 1'b0;
				cnt <= {4'd0, r};
				rra <= rnum(r);
				st <= C_REGS3;
			end
		end
		C_REGS3: begin
			if (!cp_dr) cir_wr(5'h10, 3'd4, rrd, C_REGS2);
			else if (!acc_on) begin acc_on <= 1'b1; cir_rd(5'h10, 3'd4, C_REGS3); end
			else begin acc_on <= 1'b0; regwr(cnt[3:0], acc_rd); st <= C_REGS2; end
		end
		C_MULT: begin
			// transfer multiple coprocessor registers: the EA, then the mask
			if (ea_pi || ea_pd) regrd({1'b1, ir[2:0]}, C_MULT2);
			else begin ea_ret <= C_MULT2; st <= C_EA; end
		end
		C_MULT2: begin
			if (ea_pi || ea_pd) ea <= rrd;
			cir_rd(5'h14, 3'd2, C_MULT3);
		end
		C_MULT3: begin
			logic [4:0] n;
			logic [11:0] tb;
			n = 5'd0;
			for (int i = 0; i < 16; i++) n = n + {4'd0, acc_rd[i]};
			tb = (n[0] ? {4'd0, cp_len} : 12'd0) + (n[1] ? {3'd0, cp_len, 1'b0} : 12'd0) +
			     (n[2] ? {2'd0, cp_len, 2'b00} : 12'd0) + (n[3] ? {1'd0, cp_len, 3'b000} : 12'd0) +
			     (n[4] ? {cp_len, 4'b0000} : 12'd0);
			cnt <= {3'd0, n};
			cp_pos <= 8'd0;
			if (ea_pi) regwr({1'b1, ir[2:0]}, ea + {20'd0, tb});
			if (ea_pd) begin
				regwr({1'b1, ir[2:0]}, ea - {20'd0, tb});
				ea <= ea - {24'd0, cp_len};
			end
			st <= C_MULT4;
		end
		C_MULT4: begin
			logic [7:0] rem; logic [2:0] chunk;
			rem = cp_len - cp_pos;
			chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
			if (cnt == 8'd0) st <= C_CPWAIT;
			else if (rem == 8'd0) begin
				cnt <= cnt - 8'd1;
				cp_pos <= 8'd0;
				if (cnt != 8'd1) ea <= ea_pd ? (ea - {24'd0, cp_len}) : (ea + {24'd0, cp_len});
			end
			else begin
				cp_pos <= cp_pos + {5'd0, chunk};
				tmp2 <= {29'd0, chunk};
				if (!cp_dr) acc(ea + {24'd0, cp_pos}, chunk, 1'b0, 32'd0, fc_data, 1'b0, C_MSAVE);
				else cir_rd(5'h10, chunk, C_MREST);
			end
		end
		C_MSAVE: cir_wr(5'h10, tmp2[2:0], acc_rd, C_MULT4);
		C_MREST: acc(ea + {24'd0, cp_pos} - {29'd0, tmp2[2:0]}, tmp2[2:0], 1'b1, acc_rd, fc_data, 1'b0, C_MULT4);
		C_SR: begin
			// transfer status register and scanPC
			if (!cp_dr) begin
				if (cp_resp[8]) cir_wr(5'h18, 3'd4, scan, C_SR2);
				else cir_wr(5'h10, 3'd2, {16'd0, sr}, C_CPWAIT);
			end
			else cir_rd(5'h10, 3'd2, C_SR2);
		end
		C_SR2: begin
			if (!cp_dr) cir_wr(5'h10, 3'd2, {16'd0, sr}, C_CPWAIT);
			else begin
				sr_we <= 1'b1; sr_wv <= acc_rd[15:0];
				if (tr_t0) cp_trace_wait <= 1'b1;
				if (cp_resp[8]) cir_rd(5'h18, 3'd4, C_SR3);
				else st <= C_CPWAIT;
			end
		end
		C_SR3: begin
			// a new scanPC (an odd one: the address error)
			if (acc_rd[0]) xgo(VEC_ADDR, 4'hB, pci, acc_rd);
			else begin scan <= acc_rd; st <= C_CPWAIT; end
		end

		//==============================================================
		// cpSAVE (UM 10.2.3.3)
		//==============================================================
		C_SAVE0: begin
			flt_fline <= 1'b1;
			if (ea_pd) regrd({1'b1, ir[2:0]}, C_SAVE1);
			else st <= C_SAVE1;
		end
		C_SAVE1: begin
			if (!acc_on) begin
				tmp2 <= ea_pd ? rrd : ea;
				acc_on <= 1'b1;
				cir_rd(5'h04, 3'd2, C_SAVE1);
			end
			else begin
				logic [31:0] base;
				acc_on <= 1'b0;
				flt_fline <= 1'b0;
				tmp <= acc_rd;
				case (acc_rd[15:8])
					8'h00: begin
						base = ea_pd ? (tmp2 - 32'd4) : tmp2;
						if (ea_pd) regwr({1'b1, ir[2:0]}, base);
						ea <= base; cp_len <= 8'd0; cp_pos <= 8'd0;
						acc(base, 3'd4, 1'b1, {acc_rd[15:0], 16'd0}, fc_data, 1'b0, C_SVBODY);
					end
					8'h01: begin
						// not ready: an interrupt first (the instruction restarts), else again
						if (irq_pend) begin xirq <= 1'b1; xfmt <= 4'h0; xpc <= pci; st <= C_IDLE; end
						else begin acc_on <= 1'b1; cir_rd(5'h04, 3'd2, C_SAVE1); end
					end
					default: begin
						if (acc_rd[15:12] == 4'h0 || acc_rd[1:0] != 2'b00) begin
							avec <= VEC_FMTERR;
							cir_wr(5'h02, 3'd2, 32'h0001, C_ABORT);
						end
						else begin
							base = ea_pd ? (tmp2 - 32'd4 - {24'd0, acc_rd[7:0]}) : tmp2;
							if (ea_pd) regwr({1'b1, ir[2:0]}, base);
							ea <= base;
							cp_len <= acc_rd[7:0]; cp_pos <= acc_rd[7:0];
							acc(base, 3'd4, 1'b1, {acc_rd[15:0], 16'd0}, fc_data, 1'b0, C_SVBODY);
						end
					end
				endcase
			end
		end
		C_SVBODY: begin
			// the state frame's entries from the operand CIR to descending addresses
			if (cp_pos == 8'd0) finish(scan);
			else cir_rd(5'h10, 3'd4, C_SVWR);
		end
		C_SVWR: begin
			cp_pos <= cp_pos - 8'd4;
			acc(ea + {24'd0, cp_pos}, 3'd4, 1'b1, acc_rd, fc_data, 1'b0, C_SVBODY);
		end

		//==============================================================
		// cpRESTORE (UM 10.2.3.4)
		//==============================================================
		C_REST0: begin
			if (ea_pi) ea <= rrd;
			acc(ea_pi ? rrd : ea, 3'd2, 1'b0, 32'd0, fc_data, 1'b0, C_REST1);
		end
		C_REST1: begin
			tmp <= acc_rd;
			flt_fline <= 1'b1;
			cir_wr(5'h06, 3'd2, {16'd0, acc_rd[15:0]}, C_REST2);
		end
		C_REST2: begin flt_fline <= 1'b0; cir_rd(5'h06, 3'd2, C_REST3); end
		C_REST3: begin
			case (acc_rd[15:8])
				8'h00: begin
					if (ea_pi) regwr({1'b1, ir[2:0]}, ea + 32'd4);
					finish(scan);
				end
				8'h01: cir_rd(5'h06, 3'd2, C_REST3);
				default: begin
					if (acc_rd[15:12] == 4'h0 || tmp[1:0] != 2'b00) begin
						avec <= VEC_FMTERR;
						cir_wr(5'h02, 3'd2, 32'h0001, C_ABORT);
					end
					else begin
						cp_len <= tmp[7:0]; cp_pos <= 8'd0;
						if (ea_pi) regwr({1'b1, ir[2:0]}, ea + 32'd4 + {24'd0, tmp[7:0]});
						st <= C_RSBODY;
					end
				end
			endcase
		end
		C_RSBODY: begin
			if (cp_pos == cp_len) finish(scan);
			else acc(ea + 32'd4 + {24'd0, cp_pos}, 3'd4, 1'b0, 32'd0, fc_data, 1'b0, C_RSWR);
		end
		C_RSWR: begin cp_pos <= cp_pos + 8'd4; cir_wr(5'h10, 3'd4, acc_rd, C_RSBODY); end

		//==============================================================
		// the effective address of a control (or memory) mode: ea; e_pc
		// marks a PC-relative one (its reads are program references)
		//==============================================================
		C_EA: begin
			e_pc <= 1'b0;
			if (ea_absw || ea_absl || ea_pcd || ea_pci) begin
				eb <= scan;                      // (the PC of the extension word)
				rdw(C_EA1);
			end
			else regrd({1'b1, ir[2:0]}, C_EA1);
		end
		C_EA1: begin
			if (ea_ind || ea_pi || ea_pd) begin ea <= rrd; st <= ea_ret; end
			else if (ea_d16) begin eb <= rrd; rdw(C_EA2); end
			else if (ea_idx) begin eb <= rrd; rdw(C_EA3); end
			else if (ea_absw) begin ea <= sext16(acc_rd[15:0]); st <= ea_ret; end
			else if (ea_absl) begin tmp2 <= {16'd0, acc_rd[15:0]}; rdw(C_EA2); end
			else if (ea_pcd) begin ea <= eb + sext16(acc_rd[15:0]); e_pc <= 1'b1; st <= ea_ret; end
			else begin eext <= acc_rd[15:0]; e_pc <= 1'b1; st <= C_EA4; end   // (d8,PC,Xn) / full
		end
		C_EA2: begin
			if (ea_d16) ea <= eb + sext16(acc_rd[15:0]);
			else ea <= {tmp2[15:0], acc_rd[15:0]};
			st <= ea_ret;
		end
		C_EA3: begin eext <= acc_rd[15:0]; st <= C_EA4; end
		C_EA4: begin
			// the index register
			rra <= rnum(eext[15:12]);
			e_full <= eext[8];
			e_bs <= eext[7]; e_is <= eext[6]; e_bds <= eext[5:4]; e_iis <= eext[2:0];
			st <= C_EA5;
		end
		C_EA5: begin
			logic [31:0] xi;
			xi = eext[11] ? rrd : sext16(rrd[15:0]);
			xi = xi << eext[10:9];
			ex <= (e_full && e_is) ? 32'd0 : xi;
			if (!e_full) begin
				ea <= eb + xi + {{24{eext[7]}}, eext[7:0]};
				st <= ea_ret;
			end
			else begin
				if (e_bs) eb <= 32'd0;
				tmp <= 32'd0; tmp2 <= 32'd0; cnt <= 8'd0;
				// the base displacement: 0, 1 or 2 words
				if (e_bds == 2'd2) rdw(C_EA6);
				else if (e_bds == 2'd3) begin cnt <= 8'd1; rdw(C_EA6); end
				else st <= C_EA6;
			end
		end
		C_EA6: begin
			// bd collected (cnt: words still to come), then the indirection
			logic [31:0] bd, a;
			if (cnt == 8'd1) begin tmp <= {16'd0, acc_rd[15:0]}; cnt <= 8'd2; rdw(C_EA6); end
			else begin
				bd = (e_bds == 2'd2) ? sext16(acc_rd[15:0]) : (e_bds == 2'd3) ? {tmp[15:0], acc_rd[15:0]} : 32'd0;
				cnt <= 8'd0;
				// (the pointer's address: preindexed with Xn, postindexed without)
				a = e_iis[2] ? (eb + bd) : (eb + bd + ex);
				if (e_iis == 3'd0 || e_iis == 3'd4) begin
					ea <= eb + bd + ex; st <= ea_ret;      // no memory indirection
				end
				else begin
					// the pointer, then the outer displacement
					tmp2 <= 32'd0;
					acc(a, 3'd4, 1'b0, 32'd0, e_pc ? fc_prog : fc_data, 1'b0, C_EA7);
				end
			end
		end
		C_EA7: begin
			// the pointer read: postindexed adds Xn now; od 0, 1 or 2 words
			logic [31:0] p;
			if (cnt == 8'd0) begin
				p = acc_rd + (e_iis[2] ? ex : 32'd0);
				tmp <= p;
				if (e_iis[1:0] == 2'd2) begin cnt <= 8'd3; rdw(C_EA7); end
				else if (e_iis[1:0] == 2'd3) begin cnt <= 8'd1; rdw(C_EA7); end
				else begin ea <= p; e_pc <= e_pc && !e_bs; st <= ea_ret; end
			end
			else if (cnt == 8'd1) begin tmp2 <= {16'd0, acc_rd[15:0]}; cnt <= 8'd2; rdw(C_EA7); end
			else begin
				ea <= tmp + ((cnt == 8'd3) ? sext16(acc_rd[15:0]) : {tmp2[15:0], acc_rd[15:0]});
				cnt <= 8'd0;
				// (a PC-based memory indirect operand stays a program reference
				// unless the base was suppressed, as the pipeline's EAs)
				e_pc <= e_pc && !e_bs;
				st <= ea_ret;
			end
		end

		//==============================================================
		// a register read: rrd is valid the clock after rra
		//==============================================================
		C_RDREG: st <= rd_ret;

		//==============================================================
		// the access unit
		//==============================================================
		C_ACC: begin
			b_req  <= 1'b1;
			acc_pn <= acc_np;
			st     <= C_ACCW;
		end
		C_ACCW: begin
			if (b_gnt) b_req <= 1'b0;
			if (b_rvalid && !acc_w) acc_rd <= (acc_rd << (8 * acc_pn)) | grab(b_rdata, acc_pn, acc_a[1:0]);
			if (b_err) begin
				// no coprocessor answering the first CIR access: F-line;
				// otherwise a bus error (the instruction restarts)
				b_req <= 1'b0;
				if (acc_cir && flt_fline) xgo(VEC_FLINE, 4'h0, pci, 32'd0);
				else begin
					xfslw <= (acc_w ? FSLW_W : FSLW_R) | FSLW_RE |
					         {11'd0, (acc_pn == 3'd1) ? 2'b01 : (acc_pn == 3'd2) ? 2'b10 : 2'b00, 2'b00, acc_fc, 16'd0};
					xgo(VEC_BERR, 4'hB, pci, acc_a);
				end
			end
			else if (b_done) begin
				if (acc_n == acc_pn) st <= acc_ret;
				else begin
					acc_a <= acc_a + {29'd0, acc_pn};
					acc_n <= acc_n - acc_pn;
					st    <= C_ACC;
				end
			end
		end
		default: st <= C_IDLE;
		endcase
	end
end

endmodule
