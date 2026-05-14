// =============================================================================
// Implementation of the DEQNA/DELQA Ethernet controller
// based on the M4 (LSI-11M) processor
// =============================================================================
// Module      : cmpmac
// Device      : Intel Cyclone IV E  –  EP4CE55F23C8
// Tool        : Quartus Prime
//
// Purpose:
//   Store up to USED_DEPTH (14) 48-bit MAC addresses in three 16-bit RAMs
//   (low / middle / high word).  Compare an incoming 48-bit Ethernet MAC
//   address against all stored entries.
//
// -----------------------------------------------------------------------------
// Domain WB  (wb_clk_i, rst_i active-high)
//   Active only when stpac = eth_pms_i[0] = 1.
//
//   Address map (wb_adr_i[2:0]):
//     3'b000  R/W : adr register (4 bits).
//                   Write wb_dat_i[3:0] → adr.  Write 0 to reset before
//                   RAM* population.
//                   Read  → {12'b0, adr}
//     3'b001  R/W : RAMl[adr]  MAC bits [15:0]
//     3'b010  R/W : RAMm[adr]  MAC bits [31:16]
//     3'b011  R/W : RAMh[adr]  MAC bits [47:32]
//
//   No byte-enable on RAM:
//     The WB bus writes full 16-bit words.  Byte-enable logic on the RAM
//     ports is unnecessary and removed.  wb_sel_i is only used to qualify
//     the adr register write (3'b000 uses wb_sel_i[0] per original code).
//
//   Read latency: 2 wb_clk_i cycles (M9K registered output + wb_dat register).
//   The 2-wait-state ack (ack[1]) aligns exactly with this latency.
//
// -----------------------------------------------------------------------------
// Domain Ethernet  (eth_clk_i, eth_rst_i active-high sync)
//   Active when stpac = 0.
//
//   eth_macr_i = 1 starts a comparison pass.
//   eth_macd_i must be stable for the full pass duration (guaranteed by the
//   upstream ethreceive FSM which holds CHK_MAC state until cmpdon_i is seen).
//
//   Compare mechanism:
//     adr_e increments every cycle while eth_macr_i=1 and cmp_done=0.
//     M9K Port-B has 1-cycle registered output latency: q_b* reflects the
//     address from the PREVIOUS cycle.  The compare therefore starts on the
//     second cycle after eth_macr_i goes high (first valid q_b* = RAM[0]).
//     The loop runs while adr_e <= USED_DEPTH-1.  When adr_e reaches
//     USED_DEPTH (14) the else branch fires and sets cmp_done.
//
//   Early exit: on first full 48-bit match, cmp_res=1 and cmp_done=1
//   are set immediately.  adr_e continues to increment but the
//   cmp_done=0 guard prevents any further compare or re-entry.
//
//   Self-reset: when eth_macr_i goes low and cmp_done=1, the module clears
//   adr_e, cmp_res, cmp_done — ready for the next frame.
//
//   eth_pms_i (stpac, promisc) are quasi-static; used combinatorially.
//   promisc forces cmp_res_o=1 and cmp_done_o=1 regardless of RAM contents.
//
// -----------------------------------------------------------------------------
// RAM:
//   Three altsyncram instances (RAMl, RAMm, RAMh).
//   Each: BIDIR_DUAL_PORT, M9K, RAM_DEPTH(16) words × 16 bits.
//   USED_DEPTH = 14 (addresses 0-13 valid; 14-15 unused).
//   No byteena on either port.
//   Port A: wb_clk_i,  R/W, gated by stpac=1.
//   Port B: eth_clk_i, read-only (wren_b=0), gated by stpac=0.
//   read_during_write_mode_mixed_ports = DONT_CARE — safe because stpac
//   enforces mutual exclusion at system level.
// =============================================================================
module cmpmac #(
   parameter integer RAM_DEPTH  = 16,   // physical M9K depth (power of 2)
   parameter integer USED_DEPTH = 14,   // number of valid MAC entries (0..13)
   parameter integer ADDR_W     = 4     // ceil(log2(RAM_DEPTH)) = 4
) (
// Domain WB
   input          wb_clk_i,   // clock
   input          rst_i,      // reset, active high
   input  [2:0]   wb_adr_i,   // module address
   input  [15:0]  wb_dat_i,   // input data
   output [15:0]  wb_dat_o,   // output data
   input          wb_cyc_i,   // cycle signal
   input          wb_we_i,    // [1]=write, [0]=read
   input          wb_stb_i,   // strobe  signal
   input  [1:0]   wb_sel_i,   // byte selector
   output         wb_ack_o,   // ack. signal
// Domain Ethernet
   input  [1:0]   eth_pms_i,  // [0]=stpac  [1]=promisc
   input          eth_clk_i,  // clock
   input          eth_rst_i,  // active-high, synchronised to eth_clk_i
   input          eth_macr_i, // MAC ready — level, held during CHK_MAC
   input  [47:0]  eth_macd_i, // MAC address to compare
   output         cmp_done_o, // Compare complete
   output         cmp_res_o   // [0]=not matched, [1]=matched
);

// stpac / promisc decode
wire stpac   = eth_pms_i[0];
wire promisc = eth_pms_i[1] | eth_pms_i[0];

// WB ack generator (2 wait states)
reg [1:0]   ack;

always @(posedge wb_clk_i, posedge rst_i) begin
   if (rst_i) begin
      ack <= 2'b00;
   end else begin
      ack[0] <= wb_cyc_i & wb_stb_i;
      ack[1] <= wb_cyc_i & ack[0];
   end
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

// Bus qualifier signals — all require stpac=1 for WB to own the RAM
wire bus_strobe    = wb_cyc_i & wb_stb_i & ~wb_ack_o & stpac;
wire bus_read_req  = bus_strobe & ~wb_we_i;
wire bus_write_req = bus_strobe &  wb_we_i;

// =========================================================================
// Internal address register (adr_wb)
// Load: write 3'b000 with wb_sel_i[0] (per original code)
// =========================================================================
reg [ADDR_W-1:0]  adr_wb;
wire adr_load = bus_write_req & wb_sel_i[0] & (wb_adr_i == 3'b000);

always @(posedge wb_clk_i, posedge rst_i) begin
   if (rst_i)
      adr_wb <= {ADDR_W{1'b0}};
   else if (adr_load)
      adr_wb <= wb_dat_i[ADDR_W-1:0];
end

// RAM write enables
wire we_l = bus_write_req & (wb_adr_i == 3'b001);
wire we_m = bus_write_req & (wb_adr_i == 3'b010);
wire we_h = bus_write_req & (wb_adr_i == 3'b011);

// Ethernet domain address counter
reg [ADDR_W-1:0] adr_e;

// =========================================================================
// Three altsyncram instances (RAMl, RAMm, RAMh)
// Port A: wb_clk_i,  R/W
// Port B: eth_clk_i, read-only
// width_byteena_a=1: no byte granularity (full word only)
// byteena_b not connected: read-only port, byteena has no effect on reads
// =========================================================================
wire [15:0] q_al, q_am, q_ah;   // Port A registered outputs
wire [15:0] q_bl, q_bm, q_bh;   // Port B registered outputs

// -------------------------------------------------------------------------
// RAMl  –  MAC [15:0]
// -------------------------------------------------------------------------
altsyncram #(
   .operation_mode                     ("BIDIR_DUAL_PORT"),
   .width_a                            (16),
   .widthad_a                          (ADDR_W),
   .numwords_a                         (RAM_DEPTH),
   .width_byteena_a                    (1),            // 1 = no byte enable
   .width_b                            (16),
   .widthad_b                          (ADDR_W),
   .numwords_b                         (RAM_DEPTH),
   .read_during_write_mode_port_a      ("NEW_DATA_NO_NBE_READ"),
   .read_during_write_mode_mixed_ports ("DONT_CARE"),
   .ram_block_type                     ("M9K"),
   .intended_device_family             ("Cyclone IV E"),
   .outdata_reg_a                      ("CLOCK0"),
   .outdata_reg_b                      ("CLOCK1"),
   .clock_enable_input_a               ("BYPASS"),
   .clock_enable_input_b               ("BYPASS"),
   .clock_enable_output_a              ("BYPASS"),
   .clock_enable_output_b              ("BYPASS"),
   .power_up_uninitialized             ("FALSE")
) RAMl (
   .clock0         (wb_clk_i),
   .address_a      (adr_wb),
   .data_a         (wb_dat_i),
   .wren_a         (we_l),
   .q_a            (q_al),
   .clock1         (eth_clk_i),
   .address_b      (adr_e),
   .wren_b         (1'b0),
   .data_b         (16'b0),
   .q_b            (q_bl),
   .aclr0          (1'b0), .aclr1          (1'b0),
   .addressstall_a (1'b0), .addressstall_b (1'b0),
   .byteena_a      (1'b1),
   .clocken0       (1'b1), .clocken1       (1'b1),
   .clocken2       (1'b1), .clocken3       (1'b1),
   .eccstatus      ()
);

// -------------------------------------------------------------------------
// RAMm  –  MAC [31:16]
// -------------------------------------------------------------------------
altsyncram #(
   .operation_mode                     ("BIDIR_DUAL_PORT"),
   .width_a                            (16),
   .widthad_a                          (ADDR_W),
   .numwords_a                         (RAM_DEPTH),
   .width_byteena_a                    (1),
   .width_b                            (16),
   .widthad_b                          (ADDR_W),
   .numwords_b                         (RAM_DEPTH),
   .read_during_write_mode_port_a      ("NEW_DATA_NO_NBE_READ"),
   .read_during_write_mode_mixed_ports ("DONT_CARE"),
   .ram_block_type                     ("M9K"),
   .intended_device_family             ("Cyclone IV E"),
   .outdata_reg_a                      ("CLOCK0"),
   .outdata_reg_b                      ("CLOCK1"),
   .clock_enable_input_a               ("BYPASS"),
   .clock_enable_input_b               ("BYPASS"),
   .clock_enable_output_a              ("BYPASS"),
   .clock_enable_output_b              ("BYPASS"),
   .power_up_uninitialized             ("FALSE")
) RAMm (
   .clock0         (wb_clk_i),
   .address_a      (adr_wb),
   .data_a         (wb_dat_i),
   .wren_a         (we_m),
   .q_a            (q_am),
   .clock1         (eth_clk_i),
   .address_b      (adr_e),
   .wren_b         (1'b0),
   .data_b         (16'b0),
   .q_b            (q_bm),
   .aclr0          (1'b0), .aclr1          (1'b0),
   .addressstall_a (1'b0), .addressstall_b (1'b0),
   .byteena_a      (1'b1),
   .clocken0       (1'b1), .clocken1       (1'b1),
   .clocken2       (1'b1), .clocken3       (1'b1),
   .eccstatus      ()
);

// -------------------------------------------------------------------------
// RAMh  –  MAC [47:32]
// -------------------------------------------------------------------------
altsyncram #(
   .operation_mode                     ("BIDIR_DUAL_PORT"),
   .width_a                            (16),
   .widthad_a                          (ADDR_W),
   .numwords_a                         (RAM_DEPTH),
   .width_byteena_a                    (1),
   .width_b                            (16),
   .widthad_b                          (ADDR_W),
   .numwords_b                         (RAM_DEPTH),
   .read_during_write_mode_port_a      ("NEW_DATA_NO_NBE_READ"),
   .read_during_write_mode_mixed_ports ("DONT_CARE"),
   .ram_block_type                     ("M9K"),
   .intended_device_family             ("Cyclone IV E"),
   .outdata_reg_a                      ("CLOCK0"),
   .outdata_reg_b                      ("CLOCK1"),
   .clock_enable_input_a               ("BYPASS"),
   .clock_enable_input_b               ("BYPASS"),
   .clock_enable_output_a              ("BYPASS"),
   .clock_enable_output_b              ("BYPASS"),
   .power_up_uninitialized             ("FALSE")
) RAMh (
   .clock0         (wb_clk_i),
   .address_a      (adr_wb),
   .data_a         (wb_dat_i),
   .wren_a         (we_h),
   .q_a            (q_ah),
   .clock1         (eth_clk_i),
   .address_b      (adr_e),
   .wren_b         (1'b0),
   .data_b         (16'b0),
   .q_b            (q_bh),
   .aclr0          (1'b0), .aclr1          (1'b0),
   .addressstall_a (1'b0), .addressstall_b (1'b0),
   .byteena_a      (1'b1),
   .clocken0       (1'b1), .clocken1       (1'b1),
   .clocken2       (1'b1), .clocken3       (1'b1),
   .eccstatus      ()
);

// =========================================================================
// Domain WB — read data register
//
//     Read timing (bus_read_req → wb_dat_o valid at wb_ack_o):
//       Cycle 0 : bus_read_req=1; adr_wb stable;
//                 M9K Port-A samples adr_wb → q_a* valid next cycle
//       Cycle 1 : q_a* valid; wb_dat captures selected word;
//                 ack[0] was set in cycle 0; ack[1] not yet
//       Cycle 2 : ack[1]=1 → wb_ack_o=1; wb_dat_o valid           
// =========================================================================
reg [15:0]  wb_dat;
assign wb_dat_o = wb_dat;

always @(posedge wb_clk_i, posedge rst_i) begin
   if (rst_i)
      wb_dat <= 16'b0;
   else if (bus_read_req) begin
      case (wb_adr_i[2:0])
         3'b000:  wb_dat <= {{(16-ADDR_W){1'b0}}, adr_wb};
         3'b001:  wb_dat <= q_al;
         3'b010:  wb_dat <= q_am;
         3'b011:  wb_dat <= q_ah;
         default: wb_dat <= 16'b0;
      endcase
   end
end

// =========================================================================
// Domain Ethernet — sequential MAC compare
// =========================================================================
reg         cmp_res;
reg         cmp_done;

// 48-bit match on the current Port-B registered output
wire entry_match = (q_bl == eth_macd_i[15:0])  &
                   (q_bm == eth_macd_i[31:16]) &
                   (q_bh == eth_macd_i[47:32]);

always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if (eth_rst_i) begin
      adr_e    <= {ADDR_W{1'b0}};
      cmp_res   <= 1'b0;
      cmp_done  <= 1'b0;
   end else begin
		if(eth_macr_i) begin
			if((adr_e <= USED_DEPTH[ADDR_W-1:0]-1) && ~cmp_done) begin
            if(entry_match) begin
					cmp_res <= 1'b1; cmp_done <= 1'b1;
				end
				adr_e <= adr_e + 1'b1;
			end
			else
				cmp_done <= 1'b1;
		end else begin
			if(cmp_done) begin
				adr_e <= {ADDR_W{1'b0}};
				cmp_res <= 1'b0;
				cmp_done <= 1'b0;
			end
		end
   end
end

assign cmp_res_o  = promisc ? 1'b1 : cmp_res;
assign cmp_done_o = promisc ? 1'b1 : cmp_done;

endmodule
