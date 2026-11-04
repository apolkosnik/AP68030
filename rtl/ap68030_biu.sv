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
//                                                                          //
// C030 = 1: the MC68030's own caches here (the pipeline's are off then):   //
// AP68030's ap030_cache, a 256-byte instruction cache for client 1's       //
// fetches and a 256-byte data cache for client 0's accesses outside CPU    //
// space (UM section 6: 16 lines of four long words, a valid bit each,      //
// logical tags with the FC; CACR EI/FI/IBE, ED/FD/DBE/WA, CI/CEI/CD/CED    //
// with CAAR).  A read hit answers from the cache, a cachable miss is a     //
// cachable bus read (CBREQ when the burst is enabled and the line has no   //
// valid entry of its tag) whose fill strobes write the entries; writes     //
// update the data cache (write-through; WA's allocation).  Nothing new     //
// starts until the bus is idle (a burst still filling).                  //
//--------------------------------------------------------------------------//
`include "ap030_defs.svh"
module ap68030_biu
	import ap68060_pkg::*;
#(
	parameter int NC = 2,           // number of requesters, 0 = highest priority
	parameter bit C030 = 1'b0       // the 68030's 256-byte caches here
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
	output logic              errm,          // with err: the PMMU denied it (no bus cycle)
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
	// C030: the 68030's CACR, its clear bits (pulse: CD CED CI CEI) and CAAR
	input  logic [31:0]       cacr30,
	input  logic  [3:0]       cclr,
	input  logic  [7:2]       caar_idx,
	input  logic              snoop_we,
	input  logic [31:0]       snoop_addr,
	input  logic              nc_v,          // a read of nc_a goes past the data cache
	input  logic [31:0]       nc_a,          // (the level 7 autovector, nmi_vec_nocache)
	// the PMMU (AP68030's ap030_mmu): an MMU instruction (pulse; WB waits
	// for mm_done) -- a register write (PMOVE to TC SRP CRP TT0 TT1 MMUSR)
	// or an operation (PLOAD PTEST PFLUSH) -- and the registers
	input  logic              mmudis,        // the MMUDIS and CDIS pins, synchronized
	input  logic              cdis,
	input  logic              mm_req,
	input  logic              mm_reg,
	input  logic  [2:0]       mm_sel,        // 0 TC 1 SRP 2 CRP 3 TT0 4 TT1 5 MMUSR
	input  logic              mm_fd,
	input  logic [31:0]       mm_hi,
	input  logic [31:0]       mm_lo,         // (an operation's logical address)
	input  logic  [2:0]       mm_kind,       // 0 PLOADR 1 PLOADW 2 PTESTR 3 PTESTW 4 PFLUSHA 5 PFLUSH fc 6 PFLUSH fc,ea
	input  logic  [2:0]       mm_level,
	input  logic  [2:0]       mm_fc,
	input  logic  [2:0]       mm_fcmask,
	output logic              mm_done,       // pulse
	output logic              mm_cfgerr,     // ... a register write's MMU configuration error
	output logic [31:0]       mm_desc,       // ... PTEST: the last descriptor's address
	output logic [31:0]       mmu_tc, mmu_srp_hi, mmu_srp_lo, mmu_crp_hi, mmu_crp_lo,
	output logic [31:0]       mmu_tt0, mmu_tt1,
	output logic [15:0]       mmu_sr,
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
typedef enum logic [3:0] { S_IDLE, S_REQ, S_WAIT, S_SUB, S_HIT, S_FLT, S_WALK, S_MMR } st_t;
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
logic        q_cache, q_cbreq;  // C030: the bus read is cachable / may burst
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
// (the requesters' TM is the function code -- MOVES's any; table searches
// are the PMMU's own cycles, below)
wire [2:0] fc = ack_tt ? `FC_CPU_SPACE : q.tm;
wire [1:0] kind = (ack_tt && q.tm != 3'd0) ? `BK_IACK : `BK_DATA;
// the address of this transfer
// (q_pa: the physical address the translation gave; the caches use q.addr)
logic [31:0] q_pa;
logic        q_trci;            // the page is cache inhibited (CIOUT)
wire [31:0] t_addr = (ack_tt && q.tm != 3'd0) ? {28'hFFFFFFF, q.tm, 1'b1} :
                     line ? {q_pa[31:4], k, 2'b00} : q_pa;
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

// the PMMU's table search cycles (S_WALK): long words, supervisor data,
// RMC held, never cached
logic        w_req, w_rw, w_active, w_ack, w_berr, w_pend, mmu_busy;
logic [31:0] w_addr, w_wdata, w_rdata;
wire         wk  = (st == S_WALK);
wire         qci = q.ci || q_trci;
ap030_bus bus (
	.clk(clk), .ce(1'b1), .ce_f(1'b1), .rst(!nreset),
	.req(b_req), .req_kind(wk ? `BK_TABLE : kind), .req_addr(wk ? w_addr : t_addr),
	.req_nbytes(wk ? 3'd4 : t_nb), .req_total(wk ? 3'd4 : t_nb),
	.req_rw(wk ? w_rw : q.rd), .req_fc(wk ? `FC_SUPER_DATA : fc),
	.req_rmc(wk ? 1'b1 : q.lock), .req_rmc_last(wk ? 1'b0 : q.locke),
	.req_ciout(wk ? 1'b0 : qci), .req_cbreq(!wk && ((line && q.rd && first && !qci) || q_cbreq)),
	.req_ocs(!wk), .req_cache(!wk && ((line && q.rd && !qci) || q_cache)),
	.req_wdata(wk ? w_wdata : t_wd),
	.req_ack(b_ack), .busy(b_busy), .done(b_done), .rd_data(b_rd),
	.res_berr(b_berr), .res_avec(b_avec), .res_ciin(b_ciin),
	.fill_stb(b_fill), .fill_addr(b_fill_addr), .fill_data(b_fill_data),
	.rmc_hold(lk_open || (st != S_IDLE && !wk && q.lock) || w_active), .halted(halt), .bus_idle(b_idle),
	.a_o(a_o), .fc_o(fc_o), .siz_o(siz_o), .rw_o(rw_o), .rmc_n_o(rmc_n_o),
	.as_n_o(as_n_o), .ds_n_o(ds_n_o), .dben_n_o(dben_n_o), .ecs_n_o(ecs_n_o), .ocs_n_o(ocs_n_o),
	.ciout_n_o(ciout_n_o), .cbreq_n_o(cbreq_n_o), .bus_oe(bus_oe),
	.d_o(d_o), .d_oe(d_oe), .d_i(d_i),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(sterm_n), .berr_n(berr_n),
	.halt_n(halt_n), .avec_n(avec_n), .ciin_n(ciin_n), .cback_n(cback_n),
	.br_n(br_n), .bgack_n(bgack_n), .bg_n_o(bg_n_o), .bus_granted(b_granted)
);
assign b_req = (st == S_REQ) || (wk && w_req && !w_ack && !w_pend);
assign own   = !b_granted;

//--------------------------------------------------------------------------
// C030: the 68030's caches
//--------------------------------------------------------------------------
// the client a new transaction comes from (0 = highest priority)
logic [$clog2(NC)-1:0] csel;
// (only the data client while a locked sequence is open: no other cycle,
// an instruction fetch's, runs inside the indivisible operation, UM 7.3.3)
wire [NC-1:0] req_ok = lk_open ? (req & {{(NC-1){1'b0}}, 1'b1}) : req;
always_comb begin
	csel = '0;
	for (int c = NC - 1; c >= 0; c--) if (req_ok[c]) csel = c[$clog2(NC)-1:0];
end
busreq_t     rs;
assign rs = breq[csel];
// (the data cache: client 0, the DMU, and client 2, the coprocessor
// interface's operands; the instruction cache: client 1, the fetch)
wire         rs_d   = (csel != 1) && (rs.tt != TT_ACK) && (rs.tm != 3'd7) && (rs.siz != SIZ_LINE) &&
                      !(nc_v && rs.rd && rs.addr == nc_a);
wire         rs_i   = (csel == 1) && (rs.siz != SIZ_LINE);
//--------------------------------------------------------------------------
// the PMMU: every transfer outside CPU space is translated as it starts
// (logical address and function code; the caches are logical); a walk
// first when the ATC has no entry, a bus error (no bus cycle) when the
// access is denied.  (A read that hits a cache needs no ATC entry, only no
// denial: UM 9.2.)
//--------------------------------------------------------------------------
logic        tr_ok, tr_fault, tr_walk, tr_ci, tr_use, walk_req, mmu_cfg;
logic [31:0] tr_pa, mmu_desc;
logic        mmu_opreq, mmu_regwe, mmu_opdone;
logic [31:0] wk_la;
logic  [2:0] wk_fc;
logic        wk_rw, wk_rmc, wk_op;          // S_WALK: a search for the requester / an MMU instruction
logic  [1:0] mmr_n;                          // S_MMR: clocks to wait for the configuration check
logic        mm_pend;
logic        walk_done;
wire         rs_tr  = (rs.tt != TT_ACK) && (rs.tm != 3'd7);
wire         tr_flt = rs_tr && tr_fault;
wire         tr_wk  = rs_tr && tr_walk && !tr_fault;
wire         tr_go  = !rs_tr || tr_ok;
wire         tr_inh = rs_tr && tr_ci;
ap030_mmu mmu (
	.clk(clk), .ce(1'b1), .rst(!nreset),
	.tr_la(rs.addr), .tr_fc(rs.tm), .tr_rw(rs.rd), .tr_rmc(rs.lock), .mmudis(mmudis),
	.tr_ok(tr_ok), .tr_fault(tr_fault), .tr_walk(tr_walk), .tr_pa(tr_pa), .tr_ci(tr_ci),
	.tr_use(tr_use),
	.walk_req(walk_req), .walk_la(wk_la), .walk_fc(wk_fc), .walk_rw(wk_rw), .walk_rmc(wk_rmc),
	.walk_done(walk_done),
	.op_req(mmu_opreq), .op_kind(mm_kind), .op_level(mm_level), .op_la(mm_lo), .op_fc(mm_fc),
	.op_fcmask(mm_fcmask), .op_done(mmu_opdone), .op_desc_addr(mmu_desc), .wpend(1'b0),
	.reg_we(mmu_regwe), .reg_sel(mm_sel), .reg_wdata_hi(mm_hi), .reg_wdata_lo(mm_lo),
	.reg_fd(mm_fd), .cfg_err(mmu_cfg), .tc(mmu_tc), .srp_hi(mmu_srp_hi), .srp_lo(mmu_srp_lo),
	.crp_hi(mmu_crp_hi), .crp_lo(mmu_crp_lo), .tt0(mmu_tt0), .tt1(mmu_tt1), .mmusr(mmu_sr),
	.w_req(w_req), .w_addr(w_addr), .w_rw(w_rw), .w_wdata(w_wdata), .w_ack(w_ack),
	.w_rdata(w_rdata), .w_berr(w_berr), .w_active(w_active), .busy(mmu_busy)
);

wire         ic_on  = cacr30[0] && !cdis, ic_fok = ic_on && !cacr30[1];
wire         dc_on  = cacr30[8] && !cdis, dc_fok = dc_on && !cacr30[9];
logic        ic_tag_hit, ic_hit, ic_empty, dc_tag_hit, dc_hit, dc_empty;
logic [31:0] ic_q, dc_q;
logic        cfill_d, cfill_i;     // the transaction's fill strobes go to that cache
logic        hit_i;                // S_HIT: from the instruction cache
logic        dc_iline, ic_iline;   // a burst's first cycle ended in a bus error: the line invalid
// a read hit (a locked read never: it is always a bus cycle)
wire         c_dhit = C030 && rs_d && rs.rd && !rs.lock && dc_on && dc_hit && !tr_flt;
wire         c_ihit = C030 && rs_i && rs.rd && ic_on && ic_hit && !tr_flt;
// a write: the data cache's update (hit; WA allocation)
wire  [3:0]  rs_be  = (rs.siz == SIZ_B) ? (4'b1000 >> rs.addr[1:0]) :
                      (rs.siz == SIZ_W) ? (rs.addr[1] ? 4'b0011 : 4'b1100) : 4'b1111;
// a new transaction starts this clock (C030: once the bus is idle -- a
// burst's fill strobes still belong to the last one)
// (and not in a fill strobe's clock: the cache's one tag comparator is the
// fill's then)
wire         mm_go  = (st == S_IDLE) && mm_pend && !mmu_busy && b_idle && !b_fill;
wire         c_go   = (st == S_IDLE) && !mm_pend && !sn_busy && !rsto_busy && !rsto_req && !halt &&
                      (|req_ok) && (!C030 || (b_idle && !b_fill));
// (the transaction is granted: not one that must wait for a search)
wire         c_sub  = sub_hit(rs, sub_v && !sub_arm, sub_ar);
wire         c_take = c_go && !(tr_wk && !(c_dhit || c_ihit) && !c_sub) && !(tr_wk && mmu_busy);
// (a write: the data cache's update, once translated to a page that is not
// cache inhibited; a write the MMU denies invalidates the entry, UM 6.1.2)
wire         c_wr   = C030 && c_take && rs_d && !rs.rd && !c_sub && tr_go && !tr_inh;
wire         c_inv  = C030 && c_take && rs_d && !rs.rd && tr_flt;

wire         fill_d = C030 && b_fill && cfill_d;
wire         fill_i = C030 && b_fill && cfill_i;
ap030_cache #(.FC_BITS(1)) icache (
	.clk(clk), .ce(1'b1), .rst(!nreset),
	.lk_la(rs.addr), .lk_fc(rs.tm), .lk_tag_hit(ic_tag_hit), .lk_hit(ic_hit),
	.lk_line_empty(ic_empty), .lk_data(ic_q),
	.fi_we(fill_i), .fi_addr({q.addr[31:4], b_fill_addr[3:2]}), .fi_fc(q.tm), .fi_data(b_fill_data),
	.wr_we(1'b0), .wr_la(32'd0), .wr_fc(3'd0), .wr_be(4'd0), .wr_data(32'd0),
	.wr_wa(1'b0), .wr_allow_fill(1'b0),
	.inv_we(1'b0), .inv_la(32'd0), .inv_line(ic_iline), .inv_line_idx(q.addr[7:4]),
	.snp_we(1'b0), .snp_la(32'd0), .fw_start(1'b0), .fw_line(4'd0),
	.clr_all(cclr[1]), .clr_entry(cclr[0]), .clr_index(caar_idx)
);
ap030_cache #(.FC_BITS(3)) dcache (
	.clk(clk), .ce(1'b1), .rst(!nreset),
	.lk_la(rs.addr), .lk_fc(rs.tm), .lk_tag_hit(dc_tag_hit), .lk_hit(dc_hit),
	.lk_line_empty(dc_empty), .lk_data(dc_q),
	.fi_we(fill_d), .fi_addr({q.addr[31:4], b_fill_addr[3:2]}), .fi_fc(q.tm), .fi_data(b_fill_data),
	.wr_we(c_wr), .wr_la(rs.addr), .wr_fc(rs.tm), .wr_be(rs_be), .wr_data(wdata[csel][31:0]),
	.wr_wa(cacr30[13]), .wr_allow_fill(dc_fok),
	.inv_we(c_inv), .inv_la(rs.addr), .inv_line(dc_iline), .inv_line_idx(q.addr[7:4]),
	// (a snoop between the bus read and its fills clears them again)
	.snp_we(snoop_we), .snp_la(snoop_addr),
	.fw_start(C030 && b_req && b_ack && cfill_d), .fw_line(q.addr[7:4]),
	.clr_all(cclr[3]), .clr_entry(cclr[2]), .clr_index(caar_idx)
);


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
	gnt <= '0; done <= '0; err <= '0; errm <= 1'b0;
	rvalid <= 1'b0; ravec <= 1'b0; rtci <= 1'b0;
	dc_iline <= 1'b0; ic_iline <= 1'b0;
	walk_req <= 1'b0; tr_use <= 1'b0; mmu_opreq <= 1'b0; mmu_regwe <= 1'b0;
	w_ack <= 1'b0; mm_done <= 1'b0;
	if (!nreset) begin
		st <= S_IDLE; cl <= '0; q <= '0; qw <= '0; got <= '0; k <= '0; nth <= '0;
		first <= 1'b0; lk_open <= 1'b0; tci <= 1'b0;
		rclient <= '0; rdata <= '0; rbeat <= '0; errbeat <= '0;
		rsto_n <= 1'b1; rsto_cnt <= '0; rsto_pend <= 1'b0;
		sub_v <= 1'b0; sub_ar <= '0; sub_lanes <= '0;
		cfill_d <= 1'b0; cfill_i <= 1'b0; q_cache <= 1'b0; q_cbreq <= 1'b0; hit_i <= 1'b0;
		dc_iline <= 1'b0; ic_iline <= 1'b0;
		q_pa <= '0; q_trci <= 1'b0; mm_pend <= 1'b0; mm_cfgerr <= 1'b0; mm_desc <= '0;
		wk_la <= '0; wk_fc <= '0; wk_rw <= 1'b0; wk_rmc <= 1'b0; wk_op <= 1'b0; mmr_n <= '0;
		w_pend <= 1'b0; w_rdata <= '0; w_berr <= 1'b0;
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
		if (mm_req) mm_pend <= 1'b1;
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
			if (mm_go) begin
				// an MMU instruction: a register write (then the MMU's
				// configuration check), or an operation (its search on the bus)
				mm_pend <= 1'b0;
				mm_cfgerr <= 1'b0;
				if (mm_reg) begin
					mmu_regwe <= 1'b1;
					mmr_n     <= 2'd2;
					st        <= S_MMR;
				end
				else begin
					mmu_opreq <= 1'b1;
					wk_op     <= 1'b1;
					st        <= S_WALK;
				end
			end
			else if (c_go && !c_take) begin
				// no ATC entry: the table search first, then the transaction
				// again (the request stays presented)
				if (!mmu_busy) begin
					walk_req <= 1'b1;
					wk_la  <= rs.addr; wk_fc <= rs.tm; wk_rw <= rs.rd; wk_rmc <= rs.lock;
					wk_op  <= 1'b0;
					st     <= S_WALK;
				end
			end
			else if (c_go) begin
				cl    <= csel;
				q_pa  <= (rs_tr && tr_ok) ? tr_pa : rs.addr;
				q_trci <= tr_inh;
				tr_use <= rs_tr && tr_ok;
				q     <= rs;
				qw    <= wdata[csel];
				gnt[csel] <= 1'b1;
				k     <= rs.addr[3:2];
				nth   <= 2'd0;
				got   <= '0;
				first <= 1'b1;
				tci   <= 1'b0;
				// C030: a read hit, else the bus read's cache attributes
				hit_i   <= c_ihit;
				cfill_d <= C030 && rs_d && rs.rd && !rs.ci && !tr_inh && dc_fok;
				cfill_i <= C030 && rs_i && rs.rd && !rs.ci && !tr_inh && ic_fok;
				q_cache <= C030 && rs.rd && !rs.ci && !tr_inh && ((rs_d && dc_fok) || (rs_i && ic_fok));
				q_cbreq <= C030 && rs.rd && !rs.ci && !tr_inh &&
				           ((rs_d && dc_fok && cacr30[12] && !rs.lock && !rs.nobst && (!dc_tag_hit || dc_empty)) ||
				            (rs_i && ic_fok && cacr30[4] && (!ic_tag_hit || ic_empty)));
				st    <= c_sub ? S_SUB : (c_dhit || c_ihit) ? S_HIT : tr_flt ? S_FLT : S_REQ;
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
					// (C030, UM 6.1.3.2, 7.5.1: on the first cycle of a burst
					// the whole line is left invalid)
					if (q_cbreq && cfill_d) dc_iline <= 1'b1;
					if (q_cbreq && cfill_i) ic_iline <= 1'b1;
					if (q.lock) lk_open <= 1'b0;
					st <= S_IDLE;
				end
				else begin
					if (q.lock) lk_open <= !q.locke;
					if (q.rd) begin
						rvalid  <= 1'b1; rclient <= cl; rdata <= line ? b_rd : lanes;
						rbeat   <= line ? k : q.addr[3:2];
						ravec   <= b_avec;
						// (a page the PMMU marks CI is not cached either)
						rtci    <= first && (b_ciin || q_trci);
					end
					if (!line) begin
						done[cl] <= 1'b1;
						st <= S_IDLE;
					end
					else begin
						g[k] = 1'b1;
						if (first && (b_ciin || q_trci)) tci <= 1'b1;
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
		S_FLT: begin
			// the PMMU denied the access: a bus error, no bus cycle
			err[cl] <= 1'b1; errm <= 1'b1; errbeat <= 2'd0;
			st      <= S_IDLE;
		end
		S_WALK: begin
			// the PMMU's search: its cycles on the bus, one per w_req
			if (b_req && b_ack) w_pend <= 1'b1;
			if (w_pend && b_done) begin
				w_ack   <= 1'b1;
				w_rdata <= b_rd;
				w_berr  <= b_berr;
				w_pend  <= 1'b0;
			end
			if (!wk_op && walk_done) st <= S_IDLE;
			if (wk_op && mmu_opdone) begin
				mm_done <= 1'b1;
				mm_desc <= mmu_desc;
				st      <= S_IDLE;
			end
		end
		S_MMR: begin
			// the configuration check comes in the second clock after the write
			if (mmu_cfg) mm_cfgerr <= 1'b1;
			mmr_n <= mmr_n - 2'd1;
			if (mmr_n == 2'd0) begin
				mm_done <= 1'b1;
				st      <= S_IDLE;
			end
		end
		S_HIT: begin
			// C030: the cache's entry, read in the lookup clock
			rvalid  <= 1'b1; rclient <= cl; rdata <= hit_i ? ic_q : dc_q;
			rbeat   <= q.addr[3:2];
			done[cl] <= 1'b1;
			st      <= S_IDLE;
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
