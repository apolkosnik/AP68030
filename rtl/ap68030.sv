//--------------------------------------------------------------------------//
// AP68030 (070) - the AP68060's pipeline as an MC68030                     //
//                                                                          //
// ap68030.sv - top level with the MC68030 pin set of AP68030's ap030_top   //
// (MC68030UM section 5): the bus through ap68030_biu / ap030_bus.          //
// Bidirectional pins are split into input, output and enable: D31-D0       //
// d_i/d_o/d_oe; A, FC, SIZ, R/W, RMC, AS, DS, DBEN, CIOUT, CBREQ driven    //
// when bus_oe; RESET reset_n_i in, reset_n_oe drives it low (the RESET     //
// instruction).  The ports of ap030_top this core has no use for are kept //
// (tied off): the clock enable (the core runs on every clock), the Fast   //
// RAM port, the pipeline-model hooks (tm_*, fetch_*).  snoop_we/snoop_addr //
// (another master wrote the address) invalidates the line in both caches  //
// through the snooper.                                                    //
//--------------------------------------------------------------------------//
`include "ap030_defs.svh"
module ap68030
	import ap68060_pkg::*;
#(
	parameter bit         FPU_PIPE     = 0,
	parameter bit         LOOP_PRED    = 0,
	parameter bit         EARLY_RES    = 0,
	parameter bit         DC1_ATC      = 1,
	parameter bit         SKID         = 1,
	parameter logic [7:0] PCR_REV      = 8'h06,
	// (ap030_top's parameters, accepted: the bench sets them)
	parameter             PCREL_PROGRAM_SPACE = 1,
	parameter             FAST_PORT    = 0,
	// the caches: 1 = the MC68030's (256-byte instruction and data caches,
	// UM section 6, at the bus interface; the pipeline's caches off), 0 =
	// the pipeline's 8 KB caches with the 68030's CACR
	parameter bit         CACHE030     = 1,
	// the FPU: 1 = the on-chip FPU, 0 = none: F-line instructions with
	// CpID 1-7 run the coprocessor interface (UM section 10) on the bus
	parameter bit         FPU          = 0,
	parameter             USE_CE       = 0
)(
	input             clk,
	input             ce,
	// bus
	output     [31:0] a,
	output      [2:0] fc,
	output      [1:0] siz,
	output            rw,
	output            rmc_n,
	output            as_n,
	output            ds_n,
	output            dben_n,
	output            ecs_n,
	output            ocs_n,
	output            ciout_n,
	output            cbreq_n,
	output            bus_oe,
	output     [31:0] d_o,
	output            d_oe,
	input      [31:0] d_i,
	input             dsack0_n,
	input             dsack1_n,
	input             sterm_n,
	input             berr_n,
	input             halt_n,
	input             avec_n,
	input             ciin_n,
	input             cback_n,
	input             br_n,
	output            bg_n,
	input             bgack_n,
	// interrupts
	input       [2:0] ipl_n,
	output reg        ipend_n,
	// reset (open drain)
	input             reset_n_i,
	output            reset_n_oe,
	// emulator support
	input             cdis_n,
	input             mmudis_n,
	output            refill_n,
	output            status_n,
	// Fast RAM port (not used: tied off)
	output            fast_req,
	output     [31:0] fast_addr,
	output      [2:0] fast_fc,
	output            fast_rw, fast_ci, fast_burst,
	output      [3:0] fast_be,
	output     [31:0] fast_wdata,
	input             fast_match, fast_ready,
	input             fast_valid, fast_last,
	input       [1:0] fast_word,
	input      [31:0] fast_rdata,
	// observation
	output     [31:0] dbg_pc,
	output     [15:0] dbg_sr,
	output      [7:0] dbg_state,
	output            dbg_inst,
	output      [1:0] tm_pop,
	output      [1:0] tm_md,
	output     [95:0] tm_q,
	output      [2:0] tm_qn,
	output     [31:0] tm_scan,
	output            tm_flush,
	input             fetch_stop_v,
	input      [31:0] fetch_stop,
	input             fetch_scan_v,
	input      [31:0] fetch_scan_to,
	output            dbg_halted,
	output     [31:0] dbg_vbr,
	output     [31:0] dbg_cacr,
	output            dbg_cache_clear,
	input             snoop_we,
	input      [31:0] snoop_addr,
	input             nmi_vec_nocache,
	input             fetch_lazy
);

//--------------------------------------------------------------------------
// reset (UM 7.8): the pin is synchronized; while the RESET instruction
// drives it the processor ignores it, and an external assertion must
// outlast that by eight clocks
//--------------------------------------------------------------------------
logic        rsto_n;
logic  [1:0] rst_sync = 2'b00;
logic  [3:0] rst_cnt  = 4'd15;
logic        rst      = 1'b1;
wire         reset_drive = !rsto_n;
always_ff @(posedge clk) begin
	rst_sync <= {rst_sync[0], ~reset_n_i};
	if (reset_drive) rst_cnt <= 4'd0;
	else if (rst_sync[1]) begin if (rst_cnt != 4'd15) rst_cnt <= rst_cnt + 4'd1; end
	else rst_cnt <= 4'd0;
	rst <= rst_sync[1] && !reset_drive && (rst_cnt >= 4'd7 || rst);
end
assign reset_n_oe = reset_drive;
wire nreset = !rst;

// CDIS and MMUDIS, synchronized
logic cdis = 1'b0, mdis = 1'b0;
always_ff @(posedge clk) begin cdis <= !cdis_n; mdis <= !mmudis_n; end

// IPEND every clock
logic        irq_pending;
logic  [4:0] pst5;
always_ff @(posedge clk) ipend_n <= !(nreset && irq_pending);

// interrupt level synchronizer: a level is taken once two samples agree
// (the three pins may settle at different times)
logic [2:0] ipl_s1 = 3'd0, ipl_s2 = 3'd0, ipl_s3 = 3'd0, ipl_v = 3'd0;
always_ff @(posedge clk) begin
	ipl_s1 <= ~ipl_n;
	ipl_s2 <= ipl_s1;
	ipl_s3 <= ipl_s2;
	if (ipl_s2 == ipl_s3) ipl_v <= ipl_s3;
end

//--------------------------------------------------------------------------
// BIU: client 0 data (DMU), client 1 instruction fetch
//--------------------------------------------------------------------------
// (client 2: the coprocessor interface, FPU = 0)
localparam int NC = 3;
logic [NC-1:0] b_req, b_gnt, b_done, b_err;
logic          b_own;
logic          sn_ovf, sn_busy, sn_act;
logic          sb_empty, sb_err, sb_err_ack;
logic  [31:0]  sb_fslw;
busreq_t       b_breq [NC];
logic [127:0]  b_wdata [NC];
logic          b_rvalid;
logic  [1:0]   b_rclient;
logic [31:0]   b_rdata;
logic  [1:0]   b_rbeat;
logic          b_ravec, b_rtci;
logic  [1:0]   b_errbeat;
logic          b_idle;
logic          rsto_req, rsto_busy;
logic          be_fstop;
logic          b_drop, b_more;

logic dbg_retire;

//--------------------------------------------------------------------------
// bus snooper
//--------------------------------------------------------------------------
logic        sn_req, sn_look, sn_hit, sn_dirty, sn_inv, sn_wr;
logic [31:0] sn_pa;
logic [127:0] sn_line;
logic [31:0] sn_wword;
logic [15:0] sn_wbe;
logic        sn_ic, sn_ic_all, sn_ic_done, sn_ihit;
logic        sf_req, sf_more, sf_ack, dt_we, dv_we, dv_lv, pq0_v, pq1_v;
logic  [6:0] sf_set, dt_set, dv_set;
logic  [1:0] sf_way, dt_way;
logic [20:0] dt_tag;
logic  [3:0] dv_wm;
logic [27:0] pq0_l, pq1_l;
logic [31:0] sn_ic_pa;

// the snooped writes: another master's (the snoop port), and the
// coprocessor interface's own (it writes past the pipeline's caches); one
// of the latter that meets the former waits a clock
logic        c2w_v, cwp_v, xs_v;
logic [31:0] c2w_pa, cwp_pa, xs_pa;
always_ff @(posedge clk) begin
	if (!nreset) cwp_v <= 1'b0;
	else if (snoop_we && (c2w_v || cwp_v)) begin
		cwp_v  <= 1'b1;
		cwp_pa <= c2w_v ? c2w_pa : cwp_pa;
	end
	else cwp_v <= 1'b0;
end
assign xs_v  = snoop_we || c2w_v || cwp_v;
assign xs_pa = snoop_we ? snoop_addr : cwp_v ? cwp_pa : c2w_pa;
ap68060_snoop snoop (
	// (the snoop port: another master wrote snoop_addr -- presented to the
	// snooper as a snooped transfer of an alternate master)
	.clk(clk), .nreset(nreset), .bclk_en(1'b1), .own(1'b0),
	.a_i(xs_pa), .ts_n_i(!xs_v), .tt_i(2'b00), .snoop_n(!xs_v),
	.dc_req(sn_req), .dc_pa(sn_pa), .dc_look(sn_look), .dc_hit(sn_hit),
	.dc_inv(sn_inv), .dc_ovf(sn_ovf), .busy(sn_busy),
	.sf_req(sf_req), .sf_set(sf_set), .sf_way(sf_way), .sf_more(sf_more), .sf_ack(sf_ack),
	.dt_we(dt_we), .dt_set(dt_set), .dt_way(dt_way), .dt_tag(dt_tag),
	.dv_we(dv_we), .dv_set(dv_set), .dv_wm(dv_wm), .dv_lv(dv_lv),
	.pq0_v(pq0_v), .pq0_l(pq0_l), .pq1_v(pq1_v), .pq1_l(pq1_l),
	.ic_req(sn_ic), .ic_pa(sn_ic_pa), .ic_all(sn_ic_all), .ic_done(sn_ic_done)
);
assign sn_wr    = 1'b0;
assign sn_wbe   = '0;
assign sn_wword = '0;

//--------------------------------------------------------------------------
// back end and front end
//--------------------------------------------------------------------------
logic        redir_v, flush, fe_flush;
logic [31:0] redir_pc;
logic        exc_go;
logic  [3:0] exc_kind;
logic  [4:0] exc_ssp;
logic        ucond_v, ucond;
logic [15:0] sr;
logic [31:0] vbr, cacr, caar;
logic  [3:0] cclr30;
logic        nmi_rt;
logic [31:0] nmi_va;
// the PMMU (in the BIU)
logic        mm_req, mm_reg, mm_fd, mm_done, mm_cfgerr;
logic  [2:0] mm_sel, mm_kind, mm_level, mm_fc, mm_fcmask;
logic [31:0] mm_hi, mm_lo, mm_desc;
logic [31:0] mmu_tc, mmu_srp_hi, mmu_srp_lo, mmu_crp_hi, mmu_crp_lo, mmu_tt0, mmu_tt1;
logic [15:0] mmu_sr;
logic        bk_ov;
logic [31:0] bk_pc;
logic [15:0] bk_w;
logic        bc_clr;
logic  [2:0] sfc, dfc;
logic        kill_now, adv_ag, lk_abandon;
logic [31:0] dtt0, dtt1, itt0, itt1, tc;
logic        q_odd;
logic        dm_hold1, mt_v, mt_ng, mt_wr, mt_done;
logic  [2:0] mt_op, mt_fc;
logic  [1:0] mt_scope, mt_caches;
logic [31:0] mt_addr, mt_pa, mt_fslw, urp, srp, dm_st_faddr;
// 68030 bus fault frames: the failed write, the RTE's rerun and completion
logic [31:0] dm_st_dob, sb_fa, sb_dob, rr_pa, rr_wd, sub_a, sub_d;
logic        dm_st_lastc, rr_v, sub_arm, sub_clr, sb_wait, sb_fm, b_errm;
logic  [2:0] sb_fn, sb_ffc, rr_n, rr_fc;
logic  [1:0] sub_sz, rte_alt;
logic        mt_fault, dm_lkend, dm_iack_end;
logic  [2:0] iack_lvl;
logic  [1:0] dm_ack;

logic        dm_req, adv_dc1, adv_dc2, adv_ex, adv_wb;
logic [31:0] dm_va;
logic  [1:0] dm_mem, dm_msz;
logic  [2:0] dm_fc;
logic        dm_lock, dm_locke, dm_super, dm_noalloc, dm_older;
logic        bht_we, tw_busy;
logic  [7:0] bht_wa;
logic        bht_dis;
logic        lp_we, lp_tk, lp_mp;
logic [11:0] lp_wa;
logic        dm_dc2_rdy, dm_fault, dm_st_v, dm_st_rdy, dm_st_fault;
logic [31:0] dm_ldata, dm_faddr, dm_st_data, dm_ldata_al;
logic        dm_ldata_alv;
logic  [7:0] dm_fvec;
logic [31:0] dm_fslw, dm_st_fslw;

logic  [1:0] uq_n;
uop_t        uq0;
logic        uo_rdy;

// the 68030's CACR in the terms of the caches (the AP68060's bit
// positions): ED -> the data cache enable, FD -> no allocation, EI -> the
// instruction cache enable, FI -> no allocation; the branch cache always
// on (cleared with the instruction cache); no copyback
wire [31:0] cacr6;
// (ESB, bit 29, always: the 68030 posts every write -- its one-entry
// write buffer is the store buffer with one entry)
// (CACHE030=0: the 8 KB caches are tagged by the logical address alone and
// cannot keep function codes apart, so they are off while the PMMU
// translates -- TC.E without MMUDIS --; a PMOVE to TC clears them)
wire   pc_on = !CACHE030 && !(mmu_tc[31] && !mdis);
assign cacr6 = {cacr[8] && pc_on, cacr[9], 1'b1, 5'd0, 1'b1, 7'd0, cacr[0] && pc_on, cacr[1], 14'd0};

ap68060_backend #(.FPU_PIPE(FPU_PIPE), .EARLY_RES(EARLY_RES), .SKID(SKID),
                  .PCR_REV(PCR_REV), .CPIF(!FPU)) be (
	.clk(clk), .nreset(nreset),
	.in_v(uq_n != 2'd0), .in_u(uq0), .in_rdy(uo_rdy),
	.redir_v(redir_v), .redir_pc(redir_pc), .flush(flush), .fe_flush(fe_flush),
	.ras_rv(ras_rv), .ras_rtp(ras_rtp), .ras_rn(ras_rn),
	.exc_go(exc_go), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.ucond_v(ucond_v), .ucond(ucond),
	.sr(sr), .vbr(vbr), .cacr(cacr), .caar(caar), .cclr30(cclr30), .nmi_rt(nmi_rt), .nmi_va(nmi_va),
	.cp_req(b_req[2]), .cp_breq(b_breq[2]), .cp_wdata(b_wdata[2]), .cp_gnt(b_gnt[2]),
	.cp_done(b_done[2]), .cp_err(b_err[2]), .cp_rvalid(b_rvalid && b_rclient == 2'd2), .cp_rdata(b_rdata),
	.mm_req(mm_req), .mm_reg(mm_reg), .mm_sel(mm_sel), .mm_fd(mm_fd), .mm_hi(mm_hi), .mm_lo(mm_lo),
	.mm_kind(mm_kind), .mm_level(mm_level), .mm_fc(mm_fc), .mm_fcmask(mm_fcmask),
	.mm_done(mm_done), .mm_cfgerr(mm_cfgerr), .mm_desc(mm_desc),
	.mmu_tc(mmu_tc), .mmu_srp_hi(mmu_srp_hi), .mmu_srp_lo(mmu_srp_lo), .mmu_crp_hi(mmu_crp_hi),
	.mmu_crp_lo(mmu_crp_lo), .mmu_tt0(mmu_tt0), .mmu_tt1(mmu_tt1), .mmu_sr(mmu_sr), .bc_clr(bc_clr), .bk_ov(bk_ov), .bk_pc(bk_pc), .bk_w(bk_w), .sfc(sfc), .dfc(dfc),
	.dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem), .dm_msz(dm_msz),
	.dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke), .dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_ack(dm_ack), .dm_older(dm_older),
	.bht_we(bht_we), .bht_wa(bht_wa), .bht_dis(bht_dis),
	.lp_we(lp_we), .lp_wa(lp_wa), .lp_tk(lp_tk), .lp_mp(lp_mp), .sn_ihit(sn_ihit),
	.tw_busy(tw_busy), .pst5(pst5),
	.irq_pending(irq_pending),
	.adv_dc1(adv_dc1), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.dm_dc2_rdy(dm_dc2_rdy), .dm_ldata(dm_ldata), .dm_ldata_al(dm_ldata_al), .dm_ldata_alv(dm_ldata_alv), .dm_fault(dm_fault),
	.dm_fvec(dm_fvec), .dm_faddr(dm_faddr), .dm_fslw(dm_fslw), .if_cause(if_cause),
	.dm_st_v(dm_st_v), .dm_st_data(dm_st_data), .dm_st_rdy(dm_st_rdy),
	.dm_st_fault(dm_st_fault), .dm_st_fslw(dm_st_fslw), .dm_lkend(dm_lkend), .dm_iack_end(dm_iack_end),
	.sb_empty(sb_empty), .sb_err(sb_err), .sb_fslw(sb_fslw), .sb_err_ack(sb_err_ack),
	.dm_hold1(dm_hold1), .mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope),
	.mt_caches(mt_caches), .mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng),
	.mt_wr(mt_wr), .mt_done(mt_done), .mt_pa(mt_pa), .mt_fault(mt_fault), .mt_fslw(mt_fslw),
	.dm_st_faddr(dm_st_faddr),
	.dm_st_dob(dm_st_dob), .dm_st_lastc(dm_st_lastc),
	.sb_fa(sb_fa), .sb_dob(sb_dob), .sb_fn(sb_fn), .sb_ffc(sb_ffc), .dm_sbw(sb_wait), .sb_fm(sb_fm),
	.rr_v(rr_v), .rr_pa(rr_pa), .rr_n(rr_n), .rr_wd(rr_wd), .rr_fc(rr_fc),
	.sub_arm(sub_arm), .sub_clr(sub_clr), .sub_a(sub_a), .sub_d(sub_d), .sub_sz(sub_sz),
	.rte_alt(rte_alt),
	.urp(urp), .srp(srp),
	.ipl(ipl_v), .iack_lvl(iack_lvl), .rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.kill_now(kill_now), .lk_abandon(lk_abandon), .adv_ag(adv_ag),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1), .tc(tc),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .halted(dbg_halted), .fetch_stop(be_fstop)
);

ap68060_dmu #(.DC1_ATC(DC1_ATC), .NOCACHE(CACHE030 == 1)) dmu (
	.clk(clk), .nreset(nreset),
	.adv_ag(adv_ag), .dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem),
	.dm_msz(dm_msz), .dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke),
	.dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_ack(dm_ack), .iack_lvl(iack_lvl),
	.dm_older(dm_older),
	.adv_dc1(adv_dc1), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.kill_now(kill_now), .dm_hold1(dm_hold1),
	.dc2_rdy(dm_dc2_rdy), .ldata(dm_ldata), .ldata_al(dm_ldata_al), .ldata_alv(dm_ldata_alv), .fault(dm_fault), .fvec(dm_fvec),
	.faddr(dm_faddr), .fslw(dm_fslw),
	.st_v(dm_st_v), .st_data(dm_st_data), .st_rdy(dm_st_rdy),
	.st_fault(dm_st_fault), .st_fslw(dm_st_fslw), .st_faddr(dm_st_faddr), .lkend(dm_lkend), .iack_end(dm_iack_end),
	.st_dob(dm_st_dob), .st_lastc(dm_st_lastc),
	.mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope), .mt_caches(mt_caches),
	.mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng), .mt_wr(mt_wr),
	.mt_done(mt_done), .mt_pa(mt_pa), .mt_fault(mt_fault), .mt_fslw(mt_fslw),
	// (the 68060's translation is never on: the 68030's PMMU, in the BIU,
	// translates; constants here let synthesis drop the ATCs and the table
	// walker of the DMU and the fetch unit)
	.cacr(cacr6), .tc(32'd0), .urp(32'd0), .srp(32'd0),
	.dtt0(32'd0), .dtt1(32'd0), .itt0(32'd0), .itt1(32'd0),
	.cdis(cdis), .mdis(1'b1), .tw_busy(tw_busy),
	.iw_req(iw_req), .iw_va(iw_va), .iw_fc2(iw_fc2), .iw_done(iw_done), .iw_ent(iw_ent),
	.iw_cause(iw_cause),
	.ic_inv(ic_inv), .ic_inv_scope(ic_inv_scope), .ic_inv_pa(ic_inv_pa),
	.ic_inv_done(ic_inv_done),
	.iatc_flush_all(iatc_flush_all), .iatc_flush_page(iatc_flush_page),
	.iatc_flush_ng(iatc_flush_ng), .iatc_flush_la(iatc_flush_la),
	.iatc_flush_fc2(iatc_flush_fc2), .iatc_wr(iatc_wr), .iatc_wla(iatc_wla),
	.iatc_wfc2(iatc_wfc2), .iatc_went(iatc_went),
	.bi_req(b_req[0]), .bi_breq(b_breq[0]), .bi_wdata(b_wdata[0]),
	.bi_gnt(b_gnt[0]), .bi_done(b_done[0]), .bi_err(b_err[0]), .bi_drop(b_drop), .bi_more(b_more),
	.bi_rvalid(b_rvalid && b_rclient == 2'd0), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.sb_empty(sb_empty), .sb_err(sb_err), .sb_fslw(sb_fslw), .sb_err_ack(sb_err_ack),
	.sb_fa(sb_fa), .sb_dob(sb_dob), .sb_fn(sb_fn), .sb_ffc(sb_ffc), .sb_wait(sb_wait), .bi_errm(b_errm), .sb_fm(sb_fm),
	.rr_v(rr_v), .rr_pa(rr_pa), .rr_n(rr_n), .rr_wd(rr_wd), .rr_fc(rr_fc),
	.b_ravec(b_ravec), .b_rtci(b_rtci),
	.xs_v(xs_v), .xs_pa(xs_pa),
	.sn_req(sn_req), .sn_pa(sn_pa), .sn_look(sn_look), .sn_hit(sn_hit),
	.sn_dirty(sn_dirty), .sn_line(sn_line), .sn_inv(sn_inv), .sn_act(sn_act), .sn_wr(sn_wr),
	.sn_wbe(sn_wbe), .sn_wword(sn_wword),
	.sf_req(sf_req), .sf_set(sf_set), .sf_way(sf_way), .sf_more(sf_more), .sf_ack(sf_ack),
	.dt_we(dt_we), .dt_set(dt_set), .dt_way(dt_way), .dt_tag(dt_tag),
	.dv_we(dv_we), .dv_set(dv_set), .dv_wm(dv_wm), .dv_lv(dv_lv),
	.pq0_v(pq0_v), .pq0_l(pq0_l), .pq1_v(pq1_v), .pq1_l(pq1_l)
);

// instruction side <-> DMU
logic        iw_req, iw_fc2, iw_done;
atce_t       iw_ent;
logic [31:0] iw_cause, if_cause;
logic [31:0] iw_va;
logic        ic_inv, ic_inv_done;
logic  [1:0] ic_inv_scope;
logic [31:0] ic_inv_pa;
logic        iatc_flush_all, iatc_flush_page, iatc_flush_ng, iatc_flush_fc2;
logic [31:0] iatc_flush_la;
logic        iatc_wr, iatc_wfc2;
logic [31:0] iatc_wla;
atce_t       iatc_went;

// front end
logic [15:0] win [8];
logic  [7:0] win_flt, win_fdem, win_fatc, win_bt;
logic [31:0] bt_tgt;
logic        btb_we, btb_wv, btb_wslot;
logic [BTB_AW-1:0] btb_wi;
logic [BTB_TW-1:0] btb_wtag;
logic [30:0] btb_wtgt;
logic  [1:0] btb_wkind;
logic [30:0] d_ras;
logic        ras_rv;
logic  [2:0] ras_rtp;
logic  [3:0] ras_rn;
logic  [2:0] d_ras_tp;
logic  [3:0] d_ras_n;
pd_t         pd0, pd0_n;
logic  [3:0] qcnt;
logic [31:0] qpc;
logic  [2:0] consume;
logic        d_redir_v;
logic [31:0] d_redir_pc;
logic  [1:0] rq_n;
dinst_t      rq0, rq1;
logic        rq_pop, rq_pop_e;

ap68060_fetch #(.NOCACHE(CACHE030 == 1)) fetch (
	.clk(clk), .nreset(nreset),
	.redir_v(redir_v), .redir_pc(redir_pc),
	.d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	// no fetches while stopped or halted, nor while RSTO is asserted
	// (MC68060UM 7.8.2.1: no bus activity after the LPSTOP broadcast; 2.8.4:
	// all bus cycles are suspended before RSTO is asserted)
	.stop(be_fstop || rsto_busy), .smode(sr[13]),
	.cacr(cacr6), .tc(32'd0), .itt0(32'd0), .itt1(32'd0), .cdis(cdis), .mdis(1'b1),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .win_fatc(win_fatc),
	.win_bt(win_bt), .bt_tgt(bt_tgt),
	.pd0(pd0), .pd0_n(pd0_n), .qcnt(qcnt), .qpc(qpc), .consume(consume),
	.q_odd(q_odd),
	.bc_clr(bc_clr), .bk_ov(bk_ov), .bk_pc(bk_pc), .bk_w(bk_w),
	.btb_we(btb_we), .btb_wi(btb_wi), .btb_wv(btb_wv), .btb_wtag(btb_wtag),
	.btb_wslot(btb_wslot), .btb_wkind(btb_wkind), .btb_wtgt(btb_wtgt),
	.d_ras(d_ras), .d_ras_tp(d_ras_tp), .d_ras_n(d_ras_n),
	.iw_req(iw_req), .iw_va(iw_va), .iw_fc2(iw_fc2), .iw_done(iw_done),
	.iw_ent(iw_ent), .iw_cause(iw_cause), .if_cause(if_cause),
	.iatc_wr(iatc_wr), .iatc_wla(iatc_wla), .iatc_wfc2(iatc_wfc2), .iatc_went(iatc_went),
	.iatc_flush_all(iatc_flush_all), .iatc_flush_page(iatc_flush_page),
	.iatc_flush_ng(iatc_flush_ng), .iatc_flush_la(iatc_flush_la),
	.iatc_flush_fc2(iatc_flush_fc2),
	.ic_inv(ic_inv), .ic_inv_scope(ic_inv_scope), .ic_inv_pa(ic_inv_pa),
	.ic_inv_done(ic_inv_done),
	.sn_inv(sn_ic), .sn_inv_pa(sn_ic_pa), .sn_inv_all(sn_ic_all), .sn_inv_done(sn_ic_done),
	.sn_ihit(sn_ihit),
	.b_req(b_req[1]), .b_breq(b_breq[1]),
	.b_gnt(b_gnt[1]), .b_done(b_done[1]), .b_err(b_err[1]),
	.b_rvalid(b_rvalid && b_rclient == 2'd1), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.b_rtci(b_rtci)
);
assign b_wdata[1] = '0;

ap68060_decode #(.LOOP_PRED(LOOP_PRED), .CPIF(!FPU)) dec (
	.clk(clk), .nreset(nreset), .flush(fe_flush),
	.ras_rv(ras_rv), .ras_rtp(ras_rtp), .ras_rn(ras_rn),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .win_fatc(win_fatc),
	.win_bt(win_bt), .bt_tgt(bt_tgt),
	.pd0(pd0), .pd0_n(pd0_n), .qcnt(qcnt), .qpc(qpc), .q_odd(q_odd),
	.smode(sr[13]), .rte_alt(rte_alt),
	.consume(consume), .d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop), .rq_pop_e(rq_pop_e),
	.bht_we(bht_we), .bht_wa(bht_wa), .bht_dis(bht_dis),
	.lp_we(lp_we), .lp_wa(lp_wa), .lp_tk(lp_tk), .lp_mp(lp_mp),
	.btb_we(btb_we), .btb_wi(btb_wi), .btb_wv(btb_wv), .btb_wtag(btb_wtag),
	.btb_wslot(btb_wslot), .btb_wkind(btb_wkind), .btb_wtgt(btb_wtgt),
	.ras_o(d_ras), .ras_tp_o(d_ras_tp), .ras_n_o(d_ras_n)
);

ap68060_useq useq (
	.clk(clk), .nreset(nreset), .flush(fe_flush),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop), .rq_pop_e(rq_pop_e),
		.smode(sr[13]), .master(sr[12]),
	.exc_go(exc_go), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.uq_n(uq_n), .uq0(uq0), .uq_pop(uo_rdy && uq_n != 2'd0)
);

//--------------------------------------------------------------------------
// the BIU on the 68030 bus, and the outputs from the core
//--------------------------------------------------------------------------
ap68030_biu #(.NC(NC), .C030(CACHE030)) biu (
	.clk(clk), .nreset(nreset),
	.req(b_req & ~{1'b0, be_fstop || rsto_busy, 1'b0}), .breq(b_breq), .wdata(b_wdata),
	.gnt(b_gnt), .done(b_done), .err(b_err), .errm(b_errm), .c2w_v(c2w_v), .c2w_pa(c2w_pa),
	.rvalid(b_rvalid), .rclient(b_rclient), .rdata(b_rdata), .rbeat(b_rbeat),
	.ravec(b_ravec), .rtci(b_rtci), .errbeat(b_errbeat), .idle(b_idle), .own(b_own),
	.unlock(lk_abandon), .sn_busy(sn_busy || sn_act), .halt(dbg_halted),
	.rsto_req(rsto_req), .rsto_busy(rsto_busy), .rsto_n(rsto_n),
	.sub_arm(sub_arm), .sub_clr(sub_clr), .sub_a(sub_a), .sub_d(sub_d), .sub_sz(sub_sz),
	.cacr30(cacr), .cclr(cclr30), .caar_idx(caar[7:2]), .snoop_we(snoop_we), .snoop_addr(snoop_addr),
	.nc_v(nmi_rt && nmi_vec_nocache), .nc_a(nmi_va), .mmudis(mdis), .cdis(cdis),
	.mm_req(mm_req), .mm_reg(mm_reg), .mm_sel(mm_sel), .mm_fd(mm_fd), .mm_hi(mm_hi), .mm_lo(mm_lo),
	.mm_kind(mm_kind), .mm_level(mm_level), .mm_fc(mm_fc), .mm_fcmask(mm_fcmask),
	.mm_done(mm_done), .mm_cfgerr(mm_cfgerr), .mm_desc(mm_desc),
	.mmu_tc(mmu_tc), .mmu_srp_hi(mmu_srp_hi), .mmu_srp_lo(mmu_srp_lo), .mmu_crp_hi(mmu_crp_hi),
	.mmu_crp_lo(mmu_crp_lo), .mmu_tt0(mmu_tt0), .mmu_tt1(mmu_tt1), .mmu_sr(mmu_sr),
	.a_o(a), .fc_o(fc), .siz_o(siz), .rw_o(rw), .rmc_n_o(rmc_n),
	.as_n_o(as_n), .ds_n_o(ds_n), .dben_n_o(dben_n), .ecs_n_o(ecs_n), .ocs_n_o(ocs_n),
	.ciout_n_o(ciout_n), .cbreq_n_o(cbreq_n), .bus_oe(bus_oe),
	.d_o(d_o), .d_oe(d_oe), .d_i(d_i),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(sterm_n), .berr_n(berr_n),
	.halt_n(halt_n), .avec_n(avec_n), .ciin_n(ciin_n), .cback_n(cback_n),
	.br_n(br_n), .bgack_n(bgack_n), .bg_n_o(bg_n)
);

// the ports with no use here
assign refill_n = 1'b1;
assign status_n = 1'b1;
assign fast_req = 1'b0; assign fast_addr = 32'd0; assign fast_fc = 3'd0;
assign fast_rw = 1'b1; assign fast_ci = 1'b0; assign fast_burst = 1'b0;
assign fast_be = 4'd0; assign fast_wdata = 32'd0;
assign tm_pop = 2'd0; assign tm_md = 2'd0; assign tm_q = 96'd0; assign tm_qn = 3'd0;
assign tm_scan = 32'd0; assign tm_flush = 1'b0;
assign dbg_state = 8'd0;
assign dbg_sr    = sr;
assign dbg_vbr   = vbr;
assign dbg_cacr  = cacr;
assign dbg_cache_clear = 1'b0;
assign dbg_inst  = dbg_retire;

endmodule
