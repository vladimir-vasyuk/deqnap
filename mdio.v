//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль MDC - реализация операций чтения/записи
// Раздел "7.10.4 Management interface" документа RTL8211E/RTL8211EGВ datasheet
//=================================================================================
module mdio(
   input          mdc_i,         // MDIO interface clock, Min perion 400 ns
   input          rst_i,         // Reset
   input  [4:0]   phyadr_i,      // Physical address
   input  [4:0]   regadr_i,      // Register address
   input  [15:0]  data_i,        // Transfer data (to the module)
   output [15:0]  data_o,        // Receive data (from the module)
   input          start_i,       // Start signal, active high/ Goes low if done signal is high.
   input          rw_i,          // Read/write (0/1) - operation type.
   inout          mdiol,         // MDIO line
   output         done_o         // Transfer completed signal
);         

reg  [5:0]  state;         // Cycle counter
reg         mdo;           // Output line
reg         wrdir;         // Transfer direction
reg  [15:0] dato, dati;    // Data buffers 
reg  [1:0]  opcode[1:0];   // Opcode (selected by "rw" signal)

wire        mdii;          // Input line
assign mdiol = wrdir? 1'bz : mdo;   // Bidirection: - output
assign mdii = wrdir? mdiol : 1'b0;  // Bidirection: - input
assign done_o = xferd;
assign data_o = dato;

initial begin
   opcode[0] <= 2'b01;
   opcode[1] <= 2'b10;
   wrdir <= 1'b1;
end

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
			dati = 16'o0;
		else if(start_i == 1'b1) dati = data_i;
	end

// Bidirection control block
always@(*) begin
	if(rst_i) wrdir = 1'b1;
	else begin
		if(start_i == 1'b1) begin
			if((rw_i == 1'b0) && (state < 6'd15))
				wrdir = 1'b0;
			else if((rw_i == 1'b1) && (state < 6'd32))
				wrdir = 1'b0;
			else wrdir = 1'b1;
		end
		else wrdir = 1'b1;
	end
end

// Control block and write data block (looks like a state machine)
always@(*) begin
	mdo = 1'b0;
	casez(state)
		6'h0: mdo = 1'b0;				// Start bit
		6'h1: mdo = 1'b1;				// Start bit
		//---------------------------------------------------
		6'h2: mdo = opcode[0][rw_i];	// OP code 10 = read
		6'h3: mdo = opcode[1][rw_i];	// OP code 01 = write
		//---------------------------------------------------
		6'h4: mdo = phyadr_i[4];	// Phys address
		6'h5: mdo = phyadr_i[3];
		6'h6: mdo = phyadr_i[2];
		6'h7: mdo = phyadr_i[1];
		6'h8: mdo = phyadr_i[0];
		//---------------------------------------------------
		6'h9: mdo = regadr_i[4];	// Reg address
		6'ha: mdo = regadr_i[3];
		6'hb: mdo = regadr_i[2];
		6'hc: mdo = regadr_i[1];
		6'hd: mdo = regadr_i[0];
		//---------------------------------------------------
		6'he: mdo = 1'b1;				// Turn around
		6'hf: mdo = 1'b0;
		//---------------------------------------------------
		6'h1?: mdo = (~rw_i)? 1'b1 : dati[31-state];	// write data
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
