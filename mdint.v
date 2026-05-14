//=================================================================================
// Ethernet controller implementation based on RTL8211EG transceiver
//=================================================================================
// MDINT module (Management Data Interface)
// See "8.4 Register Table" in RTL8211E/RTL8211EGB datasheet
// !!! Operation bit ctl_i[5] must be cleared by the caller after
//     ready signal rdy is de-asserted -- otherwise an infinite loop occurs
//=================================================================================
module mdint(
   input          clk_i,		// Clock signal, minimum period 400 ns
   input          rst_i,      // Reset signal
   input          evt_i,      // Periodic status poll event signal
   inout          mdiol,		// Data link
   output         err_o,      // Error signal (not implemented)
   input  [6:0]   ctl_i,      // Control byte
   input  [15:0]  val_i,      // Write data to registers
   output [15:0]  val_o,      // Read data from registers
   output [7:0]   sts_o       // Status bits
);

wire [4:0]  phy = 5'b00001;   // PHY interface address
wire [15:0] datao;            // Output data bus
reg         start = 1'b0;     // Operation start signal
reg         rw = 1'b0;        // Operation code (0=read, 1=write)
wire        done;             // Operation complete signal
reg  [4:0]  md_addr;          // Address register
reg  [15:0] datout;           // Read data from registers
assign val_o[15:0] = datout[15:0];
assign err_o = 1'b0;

// Status regs
reg         rdy;              // Ready
reg  [1:0]  speed;            // Speed:   10=1000; 01=100; 00=10; 11=reserved
reg         duplex;           // Duplex:  1=full; 0=half
reg         mdi;              // Interface: 1=MDIX (crossover); 0=MDI
reg         lrec;             // Receiver: 1=ready; 0=not ready
reg         link;             // Link:    1=up; 0=down
assign sts_o = {rdy, speed, duplex, 1'b0, mdi, lrec, link};

//////////////MDC state machine///////////////
localparam	IDLE	= 3'b000;
localparam	READ	= 3'b001;
localparam	WRITE	= 3'b010;
localparam	WAIT	= 3'b011;
localparam	GETST	= 3'b100;
localparam	COPY	= 3'b101;
reg [2:0]   state = IDLE;
reg         proc_status = 1'b0;
reg         evt_sp = 1'b0;

always @(posedge clk_i or posedge rst_i) begin
   if(rst_i) begin
      start <= 1'b0; rw <= 1'b0; rdy <=1'b0;
      proc_status <= 1'b0; datout <= 16'b0;
      state <= IDLE; evt_sp <= 1'b0;
   end
	else begin
      if(evt_i) evt_sp <= 1'b1;                    // Set periodic signal pending flag
		case(state)
			IDLE: begin
				rdy <= 1'b1;									// Assert ready signal
				if(rdy & ~done & ~start) begin			// Module not busy?
					if(evt_sp) begin                    // Periodic poll event received?
						rdy <= 1'b0;							// Yes - de-assert ready signal ...
						proc_status <= 1'b1;					// ... set periodic processing flag ...
						md_addr <= 5'h11;                // ... status register address ...
						state <= READ;							// ... go to read operation
					end
					else begin
						if(ctl_i[5]) begin					// Operation bit set? 
							if(ctl_i[6]) state <= WRITE;	// Write operation ...
                     else         state <= READ;   // ... or read operation ...
                     md_addr <= ctl_i[4:0];        // ... get register number ...
                     rdy <= 1'b0;                  // ... de-assert ready signal
						end
					end
				end
			end
         READ: begin                               // Register read operation
            rw <= 1'b0;                            // Read operation code
            start <= 1'b1;                         // Assert start signal
            state <= WAIT;                         // Go to wait state
         end
         WRITE: begin                              // Register write operation
            rw <= 1'b1;                            // Write operation code
            start <= 1'b1;                         // Assert start signal
            state <= WAIT;                         // Go to wait state
         end
         WAIT: begin
            if(done) begin                         // Done signal received?
               start <= 1'b0;                      // De-assert start signal
               if(proc_status)                     // Periodic processing flag set?
                  state <= GETST;                  // Yes - go to status register update
               else
                  state <= COPY;                   // No - go to completion
				end
			end
         GETST: begin                              // Status register update
            speed[1:0] <= datao[15:14];            // Speed
            duplex <= datao[13];                   // Duplex
            link <= datao[10];                     // Link status
            mdi <= datao[6];                       // Interface type
            lrec <= datao[1];                      // Receiver ready
            proc_status <= 1'b0;                   // Clear periodic processing flag
            evt_sp <= 1'b0;                        // Clear periodic signal pending flag
            state <= IDLE;                         // Return to idle state
         end
         COPY: begin
            if(~ctl_i[6]) datout <= datao;         // Read operation - latch data to output
            state <= IDLE;                         // Return to idle state
			end
         default: begin
            state <= IDLE;
            rdy <= 1'b0;
         end
		endcase
	end
end
//////////////////////////////////////////////

mdio mdiom(
   .mdc_i(clk_i),
   .rst_i(rst_i),
   .phyadr_i(phy),
   .regadr_i(md_addr),
   .data_i(val_i),
   .data_o(datao),
   .start_i(start),
   .rw_i(rw),
   .mdiol(mdiol),
   .done_o(done)
);         
endmodule

//=================================================================================
// MDC clock generation module for MDINT block
// See "10.6.1 MDC/MDIO Timing" in RTL8211E/RTL8211EGB datasheet
//=================================================================================
module mdc(
   input       clk_i,      // 50 MHz clock
   input       rst_i,      // Reset signal
   output      mdcclk_o,   // MD clock (T=440ns)
   output      mdsevt_o    // MD periodic event (T~1.85sec)
);

reg        mdcclk;
reg        mdsevt;
reg [4:0]  delay = 5'o0;
reg [21:0] sdelay = 22'b0;
localparam limit = 5'd9;
assign mdcclk_o = mdcclk;
assign mdsevt_o = mdsevt;

always @(posedge clk_i or posedge rst_i) begin
   if(rst_i) begin
      delay <= 5'o0; mdcclk <= 1'b0;
   end
   else begin
      if(delay == limit) begin
         delay <= 5'o0;
         mdcclk <= ~mdcclk;         // Toggle MD clock
      end
      else delay <= delay + 1'b1;
   end
end

always @(posedge mdcclk or posedge rst_i) begin
   if(rst_i) begin
      sdelay <= 22'o0; mdsevt <= 1'b0;
   end
   else begin
      if(&sdelay) mdsevt <= 1'b1;   // All bits set - assert periodic event
      else			mdsevt <= 1'b0;   // Clear periodic event
      sdelay <= sdelay + 1'b1;
   end
end
endmodule


//=================================================================================
// MDIO module - read/write operation implementation
// See "7.10.4 Management Interface" in RTL8211E/RTL8211EGB datasheet
//=================================================================================
module mdio(
   input          mdc_i,         // Clock, Min perion 400 ns
   input          rst_i,         // Reset
   input  [4:0]   phyadr_i,      // Physical address
   input  [4:0]   regadr_i,      // Register address
   input  [15:0]  data_i,        // Transfer data (to the module)
   output [15:0]  data_o,        // Receive data (from the module)
   input          start_i,       // Start signal, active high. Goes low when done is high.
   input          rw_i,          // Operation type: 0=read, 1=write
   inout          mdiol,         // MDIO line
   output         done_o         // Transfer complete signal
);

reg  [5:0]  state;         // Cycle counter
reg         mdo;           // Output line
reg         wrdir;         // Transfer direction
reg  [15:0] dato, dati;    // Data buffers 

wire        mdii;          // Input line
assign mdiol = wrdir? 1'bz : mdo;   // Bidirection: - output
assign mdii = wrdir? mdiol : 1'b0;  // Bidirection: - input
assign done_o = xferd;
assign data_o = dato;

localparam OPCODE_READ  = 2'b10;    // IEEE 802.3 MDIO read  opcode
localparam OPCODE_WRITE = 2'b01;    // IEEE 802.3 MDIO write opcode

// Logic for transfer-complete-signal
wire wstate32 = (state == 6'd32)? 1'b1 : 1'b0;
wire xferd = start_i & wstate32;

// Cycle counter block
always@(posedge mdc_i or posedge rst_i)
	begin
		if(rst_i)
			state <= 6'b111111;
		else begin
			if(start_i == 1'b0) state <= 6'b111111;
			else begin
				if(state != 6'd32) state <= state + 1'b1;
			end
		end
	end

// Store input data
always@(posedge mdc_i or posedge rst_i)
	begin
		if(rst_i)
         dati <= 16'o0;
      else if (start_i && state == 6'b111111) dati <= data_i;
	end

// Bidirection control block
always@(*) begin
   if(start_i == 1'b1) begin
      if((rw_i == 1'b0) && (state < 6'd15)) wrdir = 1'b0;                             
      else if((rw_i == 1'b1) && (state < 6'd32)) wrdir = 1'b0;
      else wrdir = 1'b1;
   end
   else wrdir = 1'b1;
end

// Control block and write data block (looks like a state machine)
always@(*) begin
	mdo = 1'b0;
	casez(state)
		6'h0: mdo = 1'b0;				   // Preamble start bit
		6'h1: mdo = 1'b1;				   // Preamble start bit
		//---------------------------------------------------
      6'h2: mdo = rw_i ? OPCODE_WRITE[1] : OPCODE_READ[1];   // opcode MSB
      6'h3: mdo = rw_i ? OPCODE_WRITE[0] : OPCODE_READ[0];   // opcode LSB
		//---------------------------------------------------
		6'h4: mdo = phyadr_i[4];	   // PHY address MSB
		6'h5: mdo = phyadr_i[3];
		6'h6: mdo = phyadr_i[2];
		6'h7: mdo = phyadr_i[1];
		6'h8: mdo = phyadr_i[0];      // PHY address LSB
		//---------------------------------------------------
		6'h9: mdo = regadr_i[4];	   // Register address MSB
		6'ha: mdo = regadr_i[3];
		6'hb: mdo = regadr_i[2];
		6'hc: mdo = regadr_i[1];
		6'hd: mdo = regadr_i[0];      // Register address LSB
		//---------------------------------------------------
		6'he: mdo = 1'b1;				   // Turnaround bit 1
		6'hf: mdo = 1'b0;             // Turnaround bit 0
		//---------------------------------------------------
      6'h1?: mdo = (~rw_i) ? 1'b1 : dati[15 - (state - 6'd16)];   // Write data
		default: mdo = 1'b1;
	endcase
end

// Read data block
always @(posedge mdc_i or posedge rst_i) begin
	if(rst_i)
	   dato <= 16'h0;
	else if(rw_i == 1'b0) begin
		if(state == 6'd0) dato <= 16'h0;
		else if((state >= 6'd16) && (state <= 6'd31))
			dato <= {dato[14:0], mdii};
	end
end
endmodule
