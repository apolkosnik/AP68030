// A minimal coprocessor at CpID 1 for the coprocessor interface tests
// (UM Section 10).  It implements: a command word that selects a scripted
// sequence of response primitives, the operand and register-select CIRs,
// the control CIR, and cpSAVE/cpRESTORE format words.
//   command $0001: null ($0902: CA=0, PF=1, done at once)
//   command $0002: evaluate EA and transfer 4 bytes to the coprocessor, then null
//   command $0003: transfer 4 bytes from the coprocessor to EA, then null
//   command $0004: transfer D3 to the coprocessor, then null
//   command $0005: transfer operand to D4 (from the operand CIR), then null
//   command $0006: take pre-instruction exception vector 48
//   command $0007: transfer from instruction stream 4 bytes, then null
//   command $0008: transfer status register (DR=0), then null
//   command $0009: write to previously evaluated EA (4 bytes), after an EA evaluation
//   command $000A: supervisor check, then null
//   command $000B: transfer main processor control register VBR to the coprocessor
//   command $000C: take post-instruction exception vector 49
//   command $000D: transfer to top of stack 4 bytes (DR=1), then null
//   command $000E: evaluate and transfer effective address, then null
//   command $000F: transfer multiple main processor registers D0,D1,A0 to the coprocessor
//   command $0010: busy once, then null
//   condition word: bit 0 = true/false returned in the null primitive
//   The save CIR returns cp_save_fmt (test register $F180); a "not ready"
//   value ($01xx) becomes format $F1 of the same length after one read.
//   The restore CIR echoes valid format words and answers $0200 to $02xx.
//   The last evaluated effective address is readable at $F17C, the last
//   control CIR value at $F184, the operands received at $F188/$F18C.
reg [15:0] cp_cmd = 0;
reg [15:0] cp_cond = 0;
reg [15:0] cp_resp = 16'h0902;
reg [15:0] cp_ctrl = 0;
reg [31:0] cp_operand [0:15];
integer    cp_ops = 0;          // operands received
integer    cp_step = 0;
reg [31:0] cp_rdata;
reg [15:0] cp_save_fmt = 16'h0000;  // format word for cpSAVE (empty/reset)
reg [15:0] cp_restore_resp = 0;
reg        cp_busy_done = 0;
reg [31:0] cp_last_ea = 0;
reg [15:0] cp_regsel = 16'h0803;    // for the control register test: D0,D1,A0 mask / VBR code
always @* begin
	case (a[4:0])
		5'h00: cp_rdata = {cp_resp, 16'd0};
		5'h04: cp_rdata = {cp_save_fmt, 16'd0};
		5'h06: cp_rdata = {16'd0, cp_restore_resp};   // offset 2 of its longword: D15-D0 (UM 7.2.1)
		5'h10: cp_rdata = cp_operand[0];
		5'h14: cp_rdata = {cp_regsel, 16'd0};
		default: cp_rdata = 32'd0;
	endcase
end
// the response after a command
task cp_start;
	input [15:0] cmd;
	begin
		cp_cmd = cmd; cp_step = 0; cp_ops = 0;
		case (cmd)
			16'h0001: cp_resp = 16'h0902;                 // null CA=0 PF=1
			16'h0002: cp_resp = 16'h9704;                 // CA, evaluate EA (any) and transfer 4 to cp
			16'h0003: cp_resp = 16'hB704;                 // CA, DR, transfer 4 from cp to EA
			16'h0004: cp_resp = 16'h8C03;                 // CA, transfer single register D3 to cp
			16'h0005: cp_resp = 16'hAC04;                 // CA, DR, operand CIR to D4
			16'h0006: cp_resp = 16'h1C30;                 // take pre-instruction exception, vector 48
			16'h0007: cp_resp = 16'h8F04;                 // CA, transfer 4 bytes from the instruction stream
			16'h0008: cp_resp = 16'h8200;                 // CA, transfer SR (DR=0, SP=0)
			16'h0009: cp_resp = 16'h8A00;                 // CA, evaluate and transfer EA first
			16'h000A: cp_resp = 16'h8400;                 // supervisor check
			16'h000B: cp_resp = 16'h8D00;                 // CA, transfer control register (select code in $14)
			16'h000C: cp_resp = 16'h1E31;                 // take post-instruction exception, vector 49
			16'h000D: cp_resp = 16'hAE04;                 // CA, DR, transfer 4 to top of stack
			16'h000E: cp_resp = 16'h8A00;                 // CA, evaluate and transfer EA
			16'h000F: cp_resp = 16'h8600;                 // CA, transfer multiple main processor registers
			16'h0010: begin cp_resp = cp_busy_done ? 16'h0902 : 16'hA400; cp_busy_done = 1; end   // busy once (UM Figure 10-23)
			default:  cp_resp = 16'h1C0B;                 // pre-instruction exception, F-line (11)
		endcase
	end
endtask
task cp_write;
	input [4:0] off; input [3:0] be; input [31:0] d;
	begin
		case (off)
			5'h02: begin
				cp_ctrl = d[31:16];
			end
			5'h06: begin
				// restore format word: echo valid ones, reject $02xx
				cp_restore_resp = (d[31:24] == 8'h02) ? 16'h0200 : d[31:16];
			end
			5'h0A: cp_start(d[31:16]);
			5'h0E: begin
				cp_cond = d[31:16];
				cp_resp = {15'd0, cp_cond[0]} | 16'h0902;   // null CA=0 PF=1 with TF
			end
			5'h10: begin
				cp_operand[cp_ops & 15] = d; cp_ops = cp_ops + 1;
				// operands delivered end the transfer for the scripted commands
				case (cp_cmd)
					16'h0002, 16'h0004, 16'h0007, 16'h0008, 16'h000B: cp_resp = 16'h0902;
					16'h000F: if (cp_ops == 3) cp_resp = 16'h0902;
					default: ;
				endcase
			end
			5'h18: ;                                        // instruction address
			5'h1C: begin
				cp_last_ea = d;
				if (cp_cmd == 16'h0009) begin cp_resp = 16'hA004; cp_operand[0] = 32'h0BADF00D; end   // write to previously evaluated EA
				else cp_resp = 16'h0902;
			end
			default: ;
		endcase
	end
endtask
// reads by the processor advance the script once the cycle is over (the
// data must not change while it is being sampled)
reg       cp_rd_seen = 0;
reg [4:0] cp_rd_off = 0;
always @(posedge clk) begin
	if (is_cp1 && as_asserted && rw && (!dsack0_n || !dsack1_n) && !cp_rd_seen) begin
		cp_rd_seen <= 1;
		cp_rd_off <= a[4:0];
	end
	if (cp_rd_seen && !as_asserted) begin
		cp_rd_seen <= 0;
		if (cp_rd_off == 5'h10) begin
			// operand delivered (DR=1 transfers)
			case (cp_cmd)
				16'h0003, 16'h0005, 16'h0009, 16'h000D: cp_resp <= 16'h0902;
				default: ;
			endcase
		end
		if (cp_rd_off == 5'h00 && cp_cmd == 16'h000A) begin
			// supervisor check passed: the processor reads the response again
			cp_resp <= 16'h0902;
		end
		if (cp_rd_off == 5'h04 && cp_save_fmt[15:8] == 8'h01) begin
			// not ready once, then a valid format of the same length
			cp_save_fmt <= {8'hF1, cp_save_fmt[7:0]};
		end
	end
end
