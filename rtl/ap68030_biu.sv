//--------------------------------------------------------------------------//
// AP68030 (070) - the pipelined core with the MC68030 bus                  //
//                                                                          //
// ap68030_biu.sv - bus interface unit: the pipeline's requester interface  //
// (that of the AP68060's BIU) on AP68030's MC68030 bus controller          //
// (ap030_bus.v: S0-S5, dynamic bus sizing, STERM, CBREQ/CBACK bursts, RMC, //
// BERR/HALT retry, arbitration).                                           //
//                                                                          //
// A transaction is one naturally aligned byte, word or long word, or a    //
// line of four long words.  A single transfer is one ap030_bus transfer    //
// (its portion is the whole operand, SIZ the operand's size).  A line      //
// read starts with the long word at addr[3:2] as a cachable read with     //
// CBREQ: a burst brings the rest (fill_stb); every long word a burst did  //
// not bring is then read on its own.  CIIN on the first long word ends     //
// the line there (rtci: the requester does not allocate it).  A line       //
// write is four long-word writes from addr[3:2], wrapping.                 //
//                                                                          //
// Function codes: the transfer mode TM is the function code for normal    //
// transfers (1 user data, 2 user program, 5 supervisor data, 6 supervisor //
// program); table searches and pushes are supervisor data (5); acknowledge//
// transfers are CPU space (7): an interrupt acknowledge (TM = the level)  //
// at $FFFFFFF1 | level << 1, a breakpoint acknowledge (TM 0) at the       //
// requester's address.  LOCK / LOCKE become RMC: asserted from the first  //
// locked transfer, held until the LOCKE transfer ends or the processor    //
// abandons the sequence (unlock).                                          //
//                                                                          //
// Requester interface (per client c), as the AP68060's BIU:               //
//   req[c]      level; breq[c], wdata[c] stable until gnt[c]              //
//   gnt[c]      pulse: the transaction is latched                          //
//   rvalid      pulse per long word read (rclient, rdata on the lanes      //
//               D31-D0, rbeat = its A3:A2, ravec, rtci with the first)     //
//   done[c]     pulse: finished without error                              //
//   err[c]      pulse: ended with a bus error (errbeat: the long word of   //
//               a line in transfer order, 0 = the first)                   //
// RESET instruction: rsto_req (pulse); RSTO for 512 clocks once no         //
// transfer is in progress (rsto_busy until then and while asserted).      //
// RTE of a bus fault frame whose read fault the handler completed (DF     //
// clear): sub_arm (pulse) with the fault address and the data input       //
// buffer; the next data read of that address (a single transfer, or the   //
// first long word of a line read, then ended as by CIIN) takes the buffer //
// instead of a bus cycle.  sub_clr (an exception) disarms it.             //
//--------------------------------------------------------------------------//
`include "ap030_defs.svh"
module ap68030_biu
	import ap68060_pkg::*;
#(
	parameter int NC = 2            // number of requesters, 0 = highest priority
)
(
	input  logic              clk,
	input  logic              nreset,
	// requesters
	input  logic [NC-1:0]     req,
	input  busreq_t           breq   [NC],
	input  logic [127:0]      wdata  [NC],
	output logic [NC-1:0]     gnt,
	output logic [NC-1:0]     done,
	output logic [NC-1:0]     err,
	output logic              rvalid,
	output logic [$clog2(NC)-1:0] rclient,
	output logic [31:0]       rdata,
	output logic  [1:0]       rbeat,
	output logic              ravec,
	output logic              rtci,
	output logic  [1:0]       errbeat,
	output logic              idle,
	output logic              own,
	input  logic              unlock,
	input  logic              sn_busy,
	input  logic              halt,
	// RESET instruction
	input  logic              rsto_req,
	input  logic              sub_arm,
	input  logic              sub_clr,
	input  logic [31:0]       sub_a,
	input  logic [31:0]       sub_d,         // right justified
	input  logic  [1:0]       sub_sz,        // SIZ code (00 long, 01 byte, 10 word)
	output logic              rsto_busy,
	output logic              rsto_n,
	// the MC68030 pins
	output logic [31:0]       a_o,
	output logic  [2:0]       fc_o,
	output logic  [1:0]       siz_o,
	output logic              rw_o,
	output logic              rmc_n_o,
	output logic              as_n_o,
	output logic              ds_n_o,
	output logic              dben_n_o,
	output logic              ecs_n_o,
	output logic              ocs_n_o,
	output logic              ciout_n_o,
	output logic              cbreq_n_o,
	output logic              bus_oe,
	output logic [31:0]       d_o,
	output logic              d_oe,
	input  logic [31:0]       d_i,
	input  logic              dsack0_n,
	input  logic              dsack1_n,
	input  logic              sterm_n,
	input  logic              berr_n,
	input  logic              halt_n,
	input  logic              avec_n,
	input  logic              ciin_n,
	input  logic              cback_n,
	input  logic              br_n,
	input  logic              bgack_n,
	output logic              bg_n_o
);

//--------------------------------------------------------------------------
// the transaction latched
//--------------------------------------------------------------------------
typedef enum logic [1:0] { S_IDLE, S_REQ, S_WAIT, S_SUB } st_t;
st_t         st;
logic [$clog2(NC)-1:0] cl;
busreq_t     q;
logic [127:0] qw;
logic  [3:0] got;               // line read: long words delivered (by A3:A2)
logic  [1:0] k;                 // the long word of the transfer in progress (A3:A2)
logic  [1:0] nth;               // ... its place in transfer order
logic        first;             // the line's first transfer
logic        lk_open;           // a locked sequence is open (RMC held)
logic        tci;               // CIIN on the line's first long word
logic        sub_v;             // a substitution is armed
logic [31:0] sub_ar, sub_lanes;

// a data read (TM 1 or 5) the armed substitution answers
function automatic logic sub_hit(input busreq_t r, input logic v, input logic [31:0] a);
	sub_hit = v && r.rd && (r.tt != TT_ACK) && (r.tm == TM_UDATA || r.tm == TM_SDATA) &&
	          ((r.siz == SIZ_LINE) ? (r.addr[31:2] == a[31:2]) : (r.addr == a));
endfunction

wire line   = (q.siz == SIZ_LINE);
wire ack_tt = (q.tt == TT_ACK);
// bytes of a single transfer
wire [2:0] nb = (q.siz == SIZ_B) ? 3'd1 : (q.siz == SIZ_W) ? 3'd2 : 3'd4;
// the function code
wire [2:0] fc = ack_tt ? `FC_CPU_SPACE :
                (q.tm == TM_UDATA || q.tm == TM_UCODE || q.tm == TM_SDATA || q.tm == TM_SCODE) ? q.tm :
                `FC_SUPER_DATA;
wire [1:0] kind = (ack_tt && q.tm != 3'd0) ? `BK_IACK :
                  (q.tm == TM_TBL_DATA || q.tm == TM_TBL_CODE) && !ack_tt ? `BK_TABLE : `BK_DATA;
// the address of this transfer
wire [31:0] t_addr = (ack_tt && q.tm != 3'd0) ? {28'hFFFFFFF, q.tm, 1'b1} :
                     line ? {q.addr[31:4], k, 2'b00} : q.addr;
// write data, right justified (single writes come placed on the lanes)
logic [31:0] t_wd;
always_comb begin
	if (line) t_wd = qw[127 - 32 * k -: 32];
	else case (q.siz)
		SIZ_B:   t_wd = {24'd0, qw[31 - 8 * q.addr[1:0] -: 8]};
		SIZ_W:   t_wd = {16'd0, q.addr[1] ? qw[15:0] : qw[31:16]};
		default: t_wd = qw[31:0];
	endcase
end

//--------------------------------------------------------------------------
// the 68030 bus controller
//--------------------------------------------------------------------------
logic        b_req, b_ack, b_busy, b_done, b_berr, b_avec, b_ciin, b_fill, b_idle, b_granted;
logic [31:0] b_rd, b_fill_data;
logic [31:2] b_fill_addr;
wire  [2:0]  t_nb = line ? 3'd4 : nb;

ap030_bus bus (
	.clk(clk), .ce(1'b1), .ce_f(1'b1), .rst(!nreset),
	.req(b_req), .req_kind(kind), .req_addr(t_addr), .req_nbytes(t_nb), .req_total(t_nb),
	.req_rw(q.rd), .req_fc(fc), .req_rmc(q.lock), .req_rmc_last(q.locke),
	.req_ciout(q.ci), .req_cbreq(line && q.rd && first && !q.ci), .req_ocs(1'b1),
	.req_cache(line && q.rd && !q.ci), .req_wdata(t_wd),
	.req_ack(b_ack), .busy(b_busy), .done(b_done), .rd_data(b_rd),
	.res_berr(b_berr), .res_avec(b_avec), .res_ciin(b_ciin),
	.fill_stb(b_fill), .fill_addr(b_fill_addr), .fill_data(b_fill_data),
	.rmc_hold(lk_open || (st != S_IDLE && q.lock)), .halted(halt), .bus_idle(b_idle),
	.a_o(a_o), .fc_o(fc_o), .siz_o(siz_o), .rw_o(rw_o), .rmc_n_o(rmc_n_o),
	.as_n_o(as_n_o), .ds_n_o(ds_n_o), .dben_n_o(dben_n_o), .ecs_n_o(ecs_n_o), .ocs_n_o(ocs_n_o),
	.ciout_n_o(ciout_n_o), .cbreq_n_o(cbreq_n_o), .bus_oe(bus_oe),
	.d_o(d_o), .d_oe(d_oe), .d_i(d_i),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(sterm_n), .berr_n(berr_n),
	.halt_n(halt_n), .avec_n(avec_n), .ciin_n(ciin_n), .cback_n(cback_n),
	.br_n(br_n), .bgack_n(bgack_n), .bg_n_o(bg_n_o), .bus_granted(b_granted)
);
assign b_req = (st == S_REQ);
assign own   = !b_granted;

// read data back on the lanes D31-D0 (the requesters pick their bytes by
// address); an acknowledge's vector on D7-D0
wire [31:0] lanes = (ack_tt && q.tm != 3'd0) ? {24'd0, b_rd[7:0]} :
                    (q.siz == SIZ_B) ? ({24'd0, b_rd[7:0]} << (8 * (3 - q.addr[1:0]))) :
                    (q.siz == SIZ_W) ? (q.addr[1] ? {16'd0, b_rd[15:0]} : {b_rd[15:0], 16'd0}) :
                    b_rd;

//--------------------------------------------------------------------------
// the sequencer
//--------------------------------------------------------------------------
// the next long word of a line still missing, in wrap order from k
function automatic logic [1:0] next_missing(input logic [3:0] g, input logic [1:0] from);
	logic [1:0] r;
	r = from;
	for (int i = 3; i >= 1; i--) if (!g[2'(from + 2'(i))]) r = 2'(from + 2'(i));
	next_missing = r;
endfunction

logic [9:0] rsto_cnt;
logic       rsto_pend;
assign rsto_busy = !rsto_n || rsto_pend;

always_ff @(posedge clk) begin
	gnt <= '0; done <= '0; err <= '0;
	rvalid <= 1'b0; ravec <= 1'b0; rtci <= 1'b0;
	if (!nreset) begin
		st <= S_IDLE; cl <= '0; q <= '0; qw <= '0; got <= '0; k <= '0; nth <= '0;
		first <= 1'b0; lk_open <= 1'b0; tci <= 1'b0;
		rclient <= '0; rdata <= '0; rbeat <= '0; errbeat <= '0;
		rsto_n <= 1'b1; rsto_cnt <= '0; rsto_pend <= 1'b0;
		sub_v <= 1'b0; sub_ar <= '0; sub_lanes <= '0;
	end
	else begin
		// RESET instruction: RSTO once nothing is in progress
		if (rsto_cnt != 10'd0) begin
			rsto_cnt <= rsto_cnt - 10'd1;
			if (rsto_cnt == 10'd1) rsto_n <= 1'b1;
		end
		if (rsto_req) rsto_pend <= 1'b1;
		if (rsto_pend && st == S_IDLE && b_idle) begin
			rsto_pend <= 1'b0;
			rsto_n    <= 1'b0;
			rsto_cnt  <= 10'd512;
		end
		if (unlock) lk_open <= 1'b0;
		if (sub_arm) begin
			sub_v  <= 1'b1;
			sub_ar <= sub_a;
			sub_lanes <= (sub_sz == SIZ_B) ? ({24'd0, sub_d[7:0]} << (8 * (3 - sub_a[1:0]))) :
			             (sub_sz == SIZ_W) ? (sub_a[1] ? {16'd0, sub_d[15:0]} : {sub_d[15:0], 16'd0}) :
			             sub_d;
		end
		else if (sub_clr) sub_v <= 1'b0;

		case (st)
		S_IDLE: begin
			// a new transaction: the highest-priority client asking (none
			// while snoop invalidations are pending, RSTO or a halt)
			if (!sn_busy && !rsto_busy && !rsto_req && !halt) begin
				for (int c = NC - 1; c >= 0; c--)
					if (req[c]) cl <= c[$clog2(NC)-1:0];
				if (|req) begin
					logic [$clog2(NC)-1:0] c0;
					c0 = '0;
					for (int c = NC - 1; c >= 0; c--) if (req[c]) c0 = c[$clog2(NC)-1:0];
					q     <= breq[c0];
					qw    <= wdata[c0];
					gnt[c0] <= 1'b1;
					k     <= breq[c0].addr[3:2];
					nth   <= 2'd0;
					got   <= '0;
					first <= 1'b1;
					tci   <= 1'b0;
					st    <= sub_hit(breq[c0], sub_v && !sub_arm, sub_ar) ? S_SUB : S_REQ;
				end
			end
		end
		S_REQ: if (b_ack) st <= S_WAIT;
		S_WAIT: begin
			logic [3:0] g;
			logic       fin;
			g   = got;
			fin = 1'b0;
			// burst beats after the first (a fill of another long word)
			if (b_fill && line && q.rd && b_fill_addr[3:2] != k && !g[b_fill_addr[3:2]]) begin
				rvalid  <= 1'b1; rclient <= cl; rdata <= b_fill_data; rbeat <= b_fill_addr[3:2];
				g[b_fill_addr[3:2]] = 1'b1;
			end
			if (b_done) begin
				if (b_berr) begin
					err[cl] <= 1'b1; errbeat <= nth;
					if (q.lock) lk_open <= 1'b0;
					st <= S_IDLE;
				end
				else begin
					if (q.lock) lk_open <= !q.locke;
					if (q.rd) begin
						rvalid  <= 1'b1; rclient <= cl; rdata <= line ? b_rd : lanes;
						rbeat   <= line ? k : q.addr[3:2];
						ravec   <= b_avec;
						rtci    <= first && b_ciin;
					end
					if (!line) begin
						done[cl] <= 1'b1;
						st <= S_IDLE;
					end
					else begin
						g[k] = 1'b1;
						if (first && b_ciin) tci <= 1'b1;
					end
				end
			end
			got <= g;
			// a line: the burst (or the single long word) over -- the next
			// missing long word, or done
			if (line && st == S_WAIT && !b_busy && !b_done && !(b_fill && q.rd)) begin
				if ((q.rd && (tci || g == 4'b1111)) || (!q.rd && nth == 2'd3)) begin
					done[cl] <= 1'b1;
					st <= S_IDLE;
				end
				else begin
					k     <= q.rd ? next_missing(g, k) : 2'(k + 2'd1);
					nth   <= nth + 2'd1;
					first <= 1'b0;
					st    <= S_REQ;
				end
			end
		end
		S_SUB: begin
			// the data input buffer for the read (a line: its first long
			// word only, not allocated)
			rvalid  <= 1'b1; rclient <= cl; rdata <= sub_lanes;
			rbeat   <= q.addr[3:2];
			rtci    <= 1'b1;
			done[cl] <= 1'b1;
			sub_v   <= 1'b0;
			st      <= S_IDLE;
		end
		default: st <= S_IDLE;
		endcase
	end
end
assign idle = (st == S_IDLE) && b_idle;

endmodule
