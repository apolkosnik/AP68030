//--------------------------------------------------------------------------//
// AP68030 - MC68030 compatible CPU                                         //
//                                                                          //
// ap030_cache.v - one 256-byte on-chip cache (UM Section 6)                //
//                                                                          //
// 16 lines of four longword entries, direct mapped by A7:A4, one valid     //
// bit per entry, tag = A31:A8 plus the function code bits (FC2 for the     //
// instruction cache, FC2-FC0 for the data cache).  Tags and valid bits     //
// live in flip-flops so a hit is known in the lookup clock; the data array  //
// is a byte-writable RAM read one clock later.                             //
//                                                                          //
// The write path implements UM 6.1.2.1: hits are always updated (even when  //
// frozen), a tag miss with WA set replaces the tag for an aligned longword  //
// write and validates only that entry, and any other miss with WA set      //
// clears the addressed entry's valid bit without altering the tag.         //
//--------------------------------------------------------------------------//

module ap030_cache
#(
	parameter FC_BITS = 3        // 1: compare FC2 only (instruction cache)
)
(
	input             clk,
	input             ce,        // clock enable: the core advances on enabled rising edges
	input             rst,       // clears all valid bits (UM 6.2)

	// lookup: combinational hit on the presented address
	input      [31:0] lk_la,
	input       [2:0] lk_fc,
	output            lk_tag_hit,   // tag matches (valid bits aside)
	output            lk_hit,       // tag matches and the addressed entry is valid
	output            lk_line_empty,// all four valid bits of the indexed line are clear
	output reg [31:0] lk_data,      // entry contents, one clock after the lookup

	// fill: a complete longword from the bus controller (single entry or burst)
	input             fi_we,
	input      [31:2] fi_addr,      // logical longword address
	input       [2:0] fi_fc,
	input      [31:0] fi_data,

	// write update (data cache): portion of an operand within one longword
	input             wr_we,
	input      [31:0] wr_la,
	input       [2:0] wr_fc,
	input       [3:0] wr_be,        // byte enables, bit 3 = D31-D24 lane (byte at A1:A0 = 00)
	input      [31:0] wr_data,      // lane-aligned longword image of the bytes
	input             wr_wa,        // CACR WA
	input             wr_allow_fill,// cache enabled and not frozen
	// invalidate the entry addressed by wr_la (write aborted by the MMU)
	input             inv_we,
	input      [31:0] inv_la,
	// invalidate the whole line inv_line_idx (UM 6.1.3.2: a bus error on the
	// first cycle of a burst marks the entire line invalid)
	input             inv_line,
	input       [7:4] inv_line_idx,
	// invalidate the entry at an externally written address (bus snoop; the
	// index alone selects it, whatever its tag)
	input             snp_we,
	input      [31:0] snp_la,
	// the bus accepted a read whose fills will be for line fw_line (with ce):
	// snoops from then on are applied again after those fills
	input             fw_start,
	input       [7:4] fw_line,

	// CACR clear controls
	input             clr_all,
	input             clr_entry,
	input       [7:2] clr_index     // CAAR bits 7:2
);

reg [23:0] tag_la [0:15];
reg  [2:0] tag_fc [0:15];
reg  [3:0] valid  [0:15];

wire [3:0] lk_idx = lk_la[7:4];
wire [1:0] lk_ent = lk_la[3:2];
wire [2:0] fc_mask = (FC_BITS == 1) ? 3'b100 : 3'b111;

assign lk_tag_hit    = (tag_la[lk_idx] == lk_la[31:8]) && (((tag_fc[lk_idx] ^ lk_fc) & fc_mask) == 3'd0);
assign lk_hit        = lk_tag_hit && valid[lk_idx][lk_ent];
assign lk_line_empty = (valid[lk_idx] == 4'd0);

// data array: 64 entries x 4 byte lanes, byte writable, registered read
reg [7:0] d0 [0:63];
reg [7:0] d1 [0:63];
reg [7:0] d2 [0:63];
reg [7:0] d3 [0:63];

wire [5:0] rd_a = lk_la[7:2];
always @(posedge clk) if (ce) begin
	lk_data <= {d0[rd_a], d1[rd_a], d2[rd_a], d3[rd_a]};
end

// write-side decisions
wire [3:0] wr_idx = wr_la[7:4];
wire [1:0] wr_ent = wr_la[3:2];
wire       wr_tag_hit = (tag_la[wr_idx] == wr_la[31:8]) && (((tag_fc[wr_idx] ^ wr_fc) & fc_mask) == 3'd0);
wire       wr_hit     = wr_tag_hit && valid[wr_idx][wr_ent];
wire       wr_long    = (wr_be == 4'b1111);
// data written into the array (hit, or WA allocation of an aligned longword,
// or a WA partial write into a tag-matching invalid entry)
wire       wr_store   = wr_we && (wr_hit || (wr_wa && wr_allow_fill && (wr_long || wr_tag_hit)));
wire       wr_alloc   = wr_we && !wr_hit && wr_wa && wr_allow_fill && wr_long;   // tag replaced/validated
wire       wr_kill    = wr_we && !wr_hit && wr_wa && wr_allow_fill && !wr_long;  // entry invalidated

wire [5:0] fi_a = fi_addr[7:2];
wire [5:0] wr_a = wr_la[7:2];

// a snoop on a clock without ce is applied at once and again at the next
// enabled clock, after a fill written there that was already under way
// (another bus master writes at most every third clock: one is enough)
reg       snp_late = 1'b0;
reg [7:2] snp_late_la;
always @(posedge clk)
	if (rst || ce) snp_late <= 1'b0;
	else if (snp_we) begin snp_late <= 1'b1; snp_late_la <= snp_la[7:2]; end

// a fill is written some clocks after the bus read its bytes (they are
// latched, the entry is reported, the memory subsystem registers it; a
// narrow port or a burst reads the line over several cycles), so a snoop in
// between may stand for a write the read did not see: the entries of the
// line being read that a snoop names, from the clock the read is accepted
// (that clock included) until its fills are written, are cleared again
// after their fill, as if the snoop had come after it
reg [7:4] fw_la = 4'd0;
reg [3:0] fw_snp = 4'd0;
wire      fw_new = ce && fw_start;
always @(posedge clk) begin
	if (rst || fw_new) fw_snp <= 4'd0;
	if (fw_new) fw_la <= fw_line;
	if (snp_we && (snp_la[7:4] == (fw_new ? fw_line : fw_la))) fw_snp[snp_la[3:2]] <= 1'b1;
end
wire      fi_snooped = fw_snp[fi_addr[3:2]] && (fi_addr[7:4] == fw_la);

integer i;
// (every clock: a snoop is a single-clock pulse from another bus master and
// must not be lost between enabled clocks; everything else advances with ce)
always @(posedge clk) begin
	if (rst || (ce && clr_all)) begin
		for (i = 0; i < 16; i = i + 1) valid[i] <= 4'd0;
	end else begin
		if (ce && clr_entry) valid[clr_index[7:4]][clr_index[3:2]] <= 1'b0;
		if (ce && fi_we) begin
			if ((tag_la[fi_addr[7:4]] == fi_addr[31:8]) && (((tag_fc[fi_addr[7:4]] ^ fi_fc) & fc_mask) == 3'd0)) begin
				valid[fi_addr[7:4]][fi_addr[3:2]] <= 1'b1;
			end else begin
				// new tag: only this entry is valid
				tag_la[fi_addr[7:4]] <= fi_addr[31:8];
				tag_fc[fi_addr[7:4]] <= fi_fc;
				valid[fi_addr[7:4]]  <= 4'b0001 << fi_addr[3:2];
			end
		end
		if (ce && fi_we && fi_snooped) valid[fi_addr[7:4]][fi_addr[3:2]] <= 1'b0;
		if (ce && wr_alloc) begin
			if (wr_tag_hit) valid[wr_idx][wr_ent] <= 1'b1;
			else begin
				tag_la[wr_idx] <= wr_la[31:8];
				tag_fc[wr_idx] <= wr_fc;
				valid[wr_idx]  <= 4'b0001 << wr_ent;
			end
		end
		if (ce && wr_kill) valid[wr_idx][wr_ent] <= 1'b0;
		if (ce && inv_we) valid[inv_la[7:4]][inv_la[3:2]] <= 1'b0;
		if (ce && inv_line) valid[inv_line_idx] <= 4'd0;
		if (ce && snp_late) valid[snp_late_la[7:4]][snp_late_la[3:2]] <= 1'b0;
		if (snp_we) valid[snp_la[7:4]][snp_la[3:2]] <= 1'b0;
	end
end

// the data array: fills write whole longwords, updates write byte lanes
always @(posedge clk) if (ce) begin
	if (fi_we) begin
		d0[fi_a] <= fi_data[31:24];
		d1[fi_a] <= fi_data[23:16];
		d2[fi_a] <= fi_data[15:8];
		d3[fi_a] <= fi_data[7:0];
	end else if (wr_store) begin
		if (wr_be[3]) d0[wr_a] <= wr_data[31:24];
		if (wr_be[2]) d1[wr_a] <= wr_data[23:16];
		if (wr_be[1]) d2[wr_a] <= wr_data[15:8];
		if (wr_be[0]) d3[wr_a] <= wr_data[7:0];
	end
end

integer k;
initial begin
	for (k = 0; k < 16; k = k + 1) begin tag_la[k] = 24'd0; tag_fc[k] = 3'd0; valid[k] = 4'd0; end
	for (k = 0; k < 64; k = k + 1) begin d0[k] = 8'd0; d1[k] = 8'd0; d2[k] = 8'd0; d3[k] = 8'd0; end
end

endmodule
