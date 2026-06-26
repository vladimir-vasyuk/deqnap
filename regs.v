//=================================================================================
// Реализация контроллера Ethernet DEQNA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль внешних регистров и регистров Ethernet.
// Доступ к внешним регистрам как с внутренней так и с внешенй шины
// Доступ к регистрам Ethernet только с внутренней шины
//
// Внешние регистры
// BASE + 0
// BASE + 2
// BASE + 4  - RBDL low
// BASE + 6  - RBDL high
// BASE + 10 - TBDL low
// BASE + 12 - TBDL high
// BASE + 14 - VAR
// BASE + 16 - CSR
//
// Внутренние регистры :LB=24040
// LB+0 - ошибки приема/передачи и данные регистра режима работы
//  Чтение: {3'b0, e_crser, e_txerg, e_rxerg, e_mode[8:1], e_txdn, etprdy_i}
//  Запись: {9'bx, promis, mcast, skipb, stpac, txrdy, 1'bx}
// LB+2 - кол-во байт для передачи (только запись)
//  Запись:
//    15-12: не используются
//    10-0:  данные
// LB+4 - данные MD
// LB+6 - статус MD
//  Чтение: статус:
//		15: 	 1/0 - ready/busy
//		14,13: 10-1000Мб/с; 01-100Мб/с; 00-10Мб/с; 11-зарезервированно
//		12:	 1-полный дуплекс; 0-полудуплекс
//		11:    зарезервированно (0)
//		10:    1-MDI crossover; 0-MDI
//		09:    1-приемник готов; 0-приемник не готов
//		08:    1-связь есть; 0-связи нет
//    07:    0
//         управление:
//		6:    1/0 - запись/чтение
//		5:    1 - старт
//		4:0   адрес регистра
//  Запись:
//    15-7: не используются
//		6:    1/0 - запись/чтение
//		5:    1 - старт
//		4-0:  адрес регистра
// LB+10 - общее назначение
//   Чтение: {bdroms, 10'b0, stm_res, stm_ena, leds[2:0]}
//   Запись: {11'bx,  stm_res, stm_ena, leds[2:0]}
// LB+12 - не используется
// LB+14 - не используется
// LB+16 - не используется
// LB+20 - LB+26 - регистр физического адреса модуля и контрольная сумма
//  Чтение: LB+20: MAC address [15:0]
//          LB+22: MAC address [31:16]
//          LB+24: MAC address [47:32]
//          LB+26: Контрольная сумма
//=================================================================================
module extregs(
// Внутренняя шина
	input          lwb_clk_i,  // тактовая частота шины
	input  [3:0]   lwb_adr_i,  // адрес 
	input  [15:0]  lwb_dat_i,  // входные данные
	output [15:0]  lwb_dat_o,  // выходные данные
	input          lwb_cyc_i,  // начало цикла шины
	input          lwb_we_i,   // разрешение записи (0 - чтение)
	input  [1:0]   lwb_sel_i,  // выбор байтов для записи 
	input          lrg_stb_i,  // строб цикла шины
	output         lwb_ack_o,  // подтверждение выбора устройства
	output         combres_o,  // сигнал комбинированного сброса
// Внешняя шина
	input          ewb_rst_i,  // сброс
	input  [2:0]   ewb_adr_i,  // адрес 
	input  [15:0]  ewb_dat_i,  // входные данные
	output [15:0]  ewb_dat_o,  // выходные данные
	input          ewb_cyc_i,  // начало цикла шины
	input          ewb_we_i,   // разрешение записи (0 - чтение)
	input  [1:0]   ewb_sel_i,  // выбор байтов для записи 
	input          erg_stb_i,  // строб цикла шины
	output         ewb_ack_o,  // подтверждение выбора устройства
	input          iack_i,     // запрос обработки прерывания
	output         irq_o,      // подтверждение обработки прерывания
   input          w3_i,       // сигнал w3
// Ethernet
	input          let_stb_i,  // строб выбора регистров ethernet
	output [9:0]   e_mode_o,   // управляющие сигналы для модуля ethernet
	input  [4:0]   e_stse_i,   // состояние модуля ethernet
	output [10:0]  e_txcntb_o, // кол-во байт канала передачи
	output [15:0]  e_mdval_o,  // данные для блока MD
	input  [15:0]  e_mdval_i,  // данные из блока MD
	output [6:0]   e_mdctrl_o, // управляющие сигналы для блока MD
	input  [7:0]   e_mdstat_i, // состояние блока MD
   input          etprdy_i,   // признак наличия данных в FIFO канала приема
// Sanity timer
	output         santm_o,    // Сигнал генерации BDCOK
// Индикация
	output [2:0]   dev_ind_o
);

reg  [15:0]	ldat, edat, etdat;
reg  [1:0]	lack, eack;
assign ewb_dat_o = edat;

assign lwb_dat_o = lrg_stb_i ? ldat : etdat;
assign lwb_ack_o = lrg_ack | let_ack;

//************************************************
// Сигналы управления обменом по внутренней шине
wire lrg_ack;
wire lcyc = lwb_cyc_i & lrg_stb_i;
wire lstb = lcyc & ~lrg_ack;						// строб цикла шины
wire lread_req = lstb & bl & ~lwb_we_i;		// запрос чтения
wire lwrite_req = lstb & bl & lwb_we_i;		// запрос записи
// Сигналы управления обменом по внешней  шине
wire ecyc = ewb_cyc_i & erg_stb_i;
wire estb = ecyc & ~ewb_ack_o;					// строб цикла шины
wire eread_req = estb & be & ~ewb_we_i;		// запрос чтения
wire ewrite_req = estb & be & ewb_we_i;		// запрос записи

//************************************************
// Формирование сигналов подтверждения выбора и приоритезации доступа к данным
// Доступ к внешним регистрам
reg bl;						// разрешение работы внутренней шины
reg be;						// разрешение работы внешней шины
wire block = be | bl;	// сигнал блокировки данных

always @ (posedge lwb_clk_i, posedge comb_res) begin
	if(comb_res) begin
		be = 1'b0; bl = 1'b0;
	end
	else begin
		if(ecyc & ~block)
			be = 1'b1;
		else if(lcyc & ~block)
			bl = 1'b1;
		if(~lcyc & bl) bl = 1'b0;
		if(~ecyc & be) be = 1'b0;
	end
end

always @ (posedge lwb_clk_i) begin
	if(bl) begin
		lack[0] <= lcyc;
		lack[1] <= lwb_cyc_i & lack[0];
	end
	else
		lack <= 2'b0;
	if(be) begin
		eack[0] <= ecyc;
		eack[1] <= ewb_cyc_i & eack[0];
	end
	else
		eack <= 2'b0;
end

assign lrg_ack = lwb_cyc_i & lrg_stb_i & lack[1];
assign ewb_ack_o = ewb_cyc_i & erg_stb_i & eack[1];

//************************************************
// Формирование сигналов подтверждения выбора
// Доступ к регистрам ethernet
wire let_ack;
wire lestb = lwb_cyc_i & let_stb_i & ~let_ack;		// строб цикла шины
wire lerd_req = lestb & ~lwb_we_i;						// запрос чтения
wire lewr_req = lestb & lwb_we_i;						// запрос записи

reg [1:0] etack;
always @(posedge lwb_clk_i) begin
	etack[0] <= lwb_cyc_i & let_stb_i;
	etack[1] <= lwb_cyc_i & etack[0];
end
assign let_ack = lwb_cyc_i & let_stb_i & etack[1];

//************************************************
// Переключатели режима
//wire			s3_n, s4_n;
//assign s3_n = ~s3_i;
//assign s4_n = ~s4_i;

//************************************************
// Синхронизация сигналов модуля ethernet
wire e_crs, e_crser, e_txerg, e_rxerg, e_txdn;
syncheb synchb(
	.clk_i(lwb_clk_i),
	.ethsig_i(e_stse_i),
   .e_crs_o(e_crs),
   .e_crser_o(e_crser),
   .e_txerg_o(e_txerg),
   .e_rxerg_o(e_rxerg),
   .e_txdn_o(e_txdn)
);

//************************************************
// Регистр управления/состояния - csr - 174456
//
reg			csr_ri = 1'b0;		// 15	Receive Interrupt Request (RW1)
//reg			csr_pe = 1'b0;		// 14	Parity Error in Memory (RO)
wire			csr_ca;				// 13	Carrier from Receiver Enabled (RO)
//reg			csr_ok = 1'b1;		// 12	Ethernet Transceiver Power OK (RO) - replaced by link signal from MD
//reg			csr_rr = 1'b0;		// 11	reserved
reg			csr_se = 1'b0;		// 10	Sanity Timer Enable (RW)
reg			csr_el = 1'b0;		// 09	External  Loopback (RW)
reg			csr_il = 1'b0;		// 08	Internal Loopback (RW) active low
reg			csr_xi = 1'b0;		// 07	Transmit Interrupt Request (RW1)
reg			csr_ie = 1'b0;		// 06	Interrupt Enable (RW)
reg			csr_rl = 1'b1;		// 05	Receive List Invalid/Empty (RO)
reg			csr_xl = 1'b1;		// 04	Transmit List Invalid/Empty (RO)
reg			csr_bd = 1'b0;		// 03	Boot/Diagnostic ROM load (RW)
reg			csr_ni = 1'b0;		// 02	Nonexistance-memory timeout Interrupt (RO)
reg			csr_sr = 1'b0;		// 01	Software Reset (RW)
reg			csr_re = 1'b0;		// 00	Receiver Enable (RW)
wire [15:0] csr;
//assign csr_ca = (~csr_il)? 1'b0 : 1'b1;//(~e_crs);
assign csr_ca = (~csr_il)? 1'b0 : e_crs;
assign csr = {csr_ri,1'b0,csr_ca,e_mdstat_i[0],1'b0,csr_se,csr_el,csr_il,csr_xi,csr_ie,csr_rl,csr_xl,csr_bd,csr_ni,csr_sr,csr_re};

//************************************************
// Регистр адреса ветора - var - 174454
//
reg  [7:0]	var_iv;				// Interrupt vector
reg         blkreg = 1'b0;
wire [15:0]	vareg;
assign vareg = {6'b0,var_iv[7:0],2'b0};

//************************************************
// Регистр адреса блока приема (RBDL) - 174444, 174446
//
reg  [15:1]	rbdl_lwr;			// low address bits
reg  [5:0]	rbdl_hir;			// high address bits

//************************************************
// Регистр адреса блока передачи (TBDL) - 174450, 174452
//
reg  [15:1]	tbdl_lwr;			// low address bits
reg  [5:0]	tbdl_hir;			// high address bits

//************************************************
// Блок формирования комбинированного сброса
//
wire			res_soft;			// сигнал программного сброса
wire        comb_res = res_soft | ewb_rst_i;	// сигнал комбинированного сброса
wire	      blkbus;
wire		   allow_bus_ops = ~blkbus & ~blkreg;
assign combres_o = comb_res;

soft_reset sftresm(
	.clk_i(lwb_clk_i),
	.rst_i(ewb_rst_i),
	.csr_sr_i(csr_sr),
	.block_o(blkbus),
	.reset_o(res_soft)
);

//************************************************
// MAC address ROM
//
wire        sa_rom_chk;       // Checksum signal
assign sa_rom_chk = csr_el & (~csr_bd) & (~csr_re);
wire [63:0] macval;

small_rom sarom(
   .q(macval)
);

//************************************************
// Модуль обработки прерывания (внешняя шина)
//
reg	fint;		// выделение фронта сигнала
wire	wint_req = csr_ri | csr_xi;
wire	sint_req = ~fint & wint_req;

always @(posedge lwb_clk_i)
	fint <= wint_req;

bus_int inter(
	.clk_i(lwb_clk_i),
	.rst_i(comb_res),
	.ena_i(csr_ie),
	.req_i(sint_req),
	.ack_i(iack_i),
	.irq_o(irq_o)
);

//************************************************
// Регистры и сигналы управления модулем Ethernet
//************************************************
//reg			rxdon;		// Данные приняты
reg			txrdy;		// Данные готовы к передаче
reg			stpac;		// Конфигурационный пакет (setup)
reg			skipb;		// Пропуск байта
reg			mcast;		// Режим широковещания
reg			promis;		// Режим прослушивания
wire        intmode;		// Internal loopback
wire        intextmode;	// Internal extended loopback
wire        extmode;		// External loopback
wire        rxmode;		// Разрешение приема пакета
//(* keep = 1 *) wire			bdroms;		// Загрузка BDROM
wire			bdroms;		// Загрузка BDROM
assign intmode = (~csr_il) & (~csr_el);
assign intextmode = (~csr_il) & csr_el;
assign extmode = csr_il & csr_el & (~csr_re);
assign rxmode = csr_re;
assign bdroms = (~csr_il) & csr_el & (~csr_re) & csr_bd;

wire [9:0]	e_mode;			// Регистр режима работы
assign e_mode = {bdroms, extmode, intextmode, intmode, rxmode, promis, mcast, skipb, stpac, txrdy};
reg  [10:0]	e_txcntb;		// Регистры кол-ва байт передачи
reg  [15:0]	e_mdval;			// входные/выходные данные MD
reg  [6:0]	e_mdctrl;		// сигналы управления MD
reg			e_mdmux = 1'b0;// мультиплексер данных MD
// управление (e_mdctrl):
//		6:		1/0 - write/read
//		5:		1 - start
//		4:0	reg. address
// статус (e_mdstat_i)
//		7: 	1/0 - ready/busy
//		6,5:	10-1000Мб/с; 01-100Мб/с; 00-10Мб/с; 11-зарезервированно
//		4:		1-полный дуплекс; 0-полудуплекс
//		3:		зарезервированно (0)
//		2:		1-MDI crossover; 0-MDI
//		1:		1-приемник готов; 0-приемник не готов
//		0:		1-связь есть; 0-связи нет
assign e_txcntb_o = e_txcntb;
assign e_mode_o = e_mode[9:0];
assign e_mdval_o = e_mdval;
assign e_mdctrl_o = e_mdctrl;

reg			stm_res;          // генерация BDCOK
reg			stm_ena;				// регистр разрешения sanity timer
reg  [2:0]	leds;					// индикация
assign santm_o = stm_res & stm_ena;
assign dev_ind_o = leds;

`ifdef md_debug
reg  [6:0]  ext_mdctrl = 6'b0;
reg  [15:0] ext_mdval;
reg         e_md_clr = 1'b0;
reg         ext_md_ack = 1'b0;
`endif

//************************************************
// Работа с внешними регистрами
// Чтение регистров внешняя шина
always @(posedge lwb_clk_i) begin
	if (eread_req) begin
		case (ewb_adr_i[2:0])
			3'b000: begin	// Base + 00
				if (sa_rom_chk)
					edat <= {8'hFF, macval[55:48]};
				else
					edat <= e_mdmux? {8'h00, e_mdstat_i[7:0]} : {8'hFF, macval[7:0]};
			end
			3'b001: begin	// Base + 02
				if (sa_rom_chk)
					edat <= {8'hFF, macval[63:56]};
				else
               edat <= e_mdmux? {13'h00, e_txerg, e_rxerg, e_txdn} : {8'hFF, macval[15:8]};
			end
			3'b010: begin	// Base + 04
`ifdef md_debug
            edat <= e_mdmux? {e_mdval_i[7:0], macval[23:16]} : {8'hFF, macval[23:16]};
`else
				edat <= {8'hFF, macval[23:16]};
`endif
			end
			3'b011: begin	// Base + 06
`ifdef md_debug
            edat <= e_mdmux? {e_mdval_i[15:8], macval[31:24]} : {8'hFF, macval[31:24]};
`else
				edat <= {8'hFF, macval[31:24]};
`endif
			end
			3'b100: begin	// Base + 10
`ifdef md_debug
            edat <= e_mdmux? {1'b0, e_mdctrl[6:0], macval[39:32]} : {8'hFF, macval[39:32]};
`else
				edat <= {8'hFF, macval[39:32]};
`endif
			end
			3'b101: begin	// Base + 12
				edat <= {8'hFF, macval[47:40]};
			end
			3'b110: begin	// Base + 14 - VAR
				edat <= vareg;
			end
			3'b111: begin	// Base + 16 - CSR
				edat <= csr;
			end
		endcase 
	end
end
// Чтение регистров внутренняя шина
always @(posedge lwb_clk_i) begin
   if (lread_req) begin
      case (lwb_adr_i[2:0])
         3'b000:        // Base + 00
            ldat <= {15'b0, blkreg};
//         3'b001: begi    // Base + 02
//         end
         3'b010:        // Base + 04 - RBDL low
            ldat <= {rbdl_lwr[15:1], 1'B0};
         3'b011:        // Base + 06 - RBDL high
            ldat <= {10'b0, rbdl_hir[5:0]};
         3'b100:        // Base + 10 - TBDL low
            ldat <= {tbdl_lwr[15:1], 1'B0};
         3'b101:        // Base + 12 - TBDL high
            ldat <= {10'b0, tbdl_hir[5:0]};
         3'b110:        // Base + 14 - VAR
            ldat <= vareg;
         3'b111:        // Base + 16 - CSR
            ldat <= csr;
      endcase
   end
end

// Сброс регистров и запись регистров 
always @(posedge lwb_clk_i) begin
	// Сброс регистров
	if(comb_res) begin
		// Сброс регистра управления
		csr_ri <= 1'b0; csr_el <= 1'b0; csr_se <= 1'b0;
		csr_il <= 1'b0; csr_xi <= 1'b0; csr_ie <= 1'b0; csr_rl <= 1'b1; csr_xl <= 1'b1;
		csr_bd <= 1'b0; csr_ni <= 1'b0; csr_sr <= 1'b0; csr_re <= 1'b0;

		// Сброс регистра вектора
		if(~res_soft) var_iv <= 8'o0;

		// Сброс регистра MD
		e_mdmux <= 1'b0;
`ifdef md_debug
      ext_md_ack <= 1'b0;
`endif
   end
   else begin
      // Запись регистров внешняя шина
      if (ewrite_req) begin
         if (ewb_sel_i[0]) begin    // Запись младшего байта
            case (ewb_adr_i[2:0])
               3'b000: begin		// Base + 00 - MD
                  if(allow_bus_ops) e_mdmux <= ewb_dat_i[7];
`ifdef md_debug
                  if(allow_bus_ops && e_mdmux) ext_mdctrl[6:0] <= ewb_dat_i[6:0];
`endif
               end
`ifdef md_debug
               3'b001: begin      // Base + 02
                  if(allow_bus_ops) ext_mdval[7:0] <= ewb_dat_i[7:0];
               end
`endif
               3'b010:        // Base + 04 - RBDL low
                  if(allow_bus_ops) rbdl_lwr[7:1] <= ewb_dat_i[7:1];
               3'b011:        // Base + 06 - RBDL high
                  if(allow_bus_ops) rbdl_hir[5:0] <= ewb_dat_i[5:0];
               3'b100:        // Base + 10 - TBDL low
                  if(allow_bus_ops) tbdl_lwr[7:1] <= ewb_dat_i[7:1];
               3'b101:        // Base + 12 - TBDL high
                  if(allow_bus_ops) tbdl_hir[5:0] <= ewb_dat_i[5:0];
               3'b110:        // Base + 14 - VAR
                  var_iv[5:0] <= ewb_dat_i[7:2];
               3'b111: begin  // Base + 16 - CSR
                  if(allow_bus_ops) begin
                     csr_ie <= ewb_dat_i[6];
                     if(ewb_dat_i[7] == 1'b1) begin
                        csr_xi <= 1'b0;
                        csr_ni <= 1'b0;
                     end
                     csr_re <= ewb_dat_i[0];
                     // Только для PDP-11. Для алгоритма смотри доку
                     csr_bd <= ewb_dat_i[3];
                  end
                  if(~blkreg)
                     csr_sr <= ewb_dat_i[1]; // 1 - 0 => программный сброс
               end
            endcase
         end
         if(ewb_sel_i[1]) begin  // Запись старшего байта
            case (ewb_adr_i[2:0])
//               3'b000:
`ifdef md_debug
               3'b001:
                  if(allow_bus_ops) ext_mdval[15:8] <= ewb_dat_i[15:8];
`endif
               3'b010:        // Base + 04 - RBDL low
                  if(allow_bus_ops) rbdl_lwr[15:8] <= ewb_dat_i[15:8];
               3'b011:        // Base + 06 - RBDL high
                  if(allow_bus_ops) csr_rl <= 1'b0;
               3'b100:        // Base + 10 - TBDL low
                  if(allow_bus_ops) tbdl_lwr[15:8] <= ewb_dat_i[15:8];
               3'b101:        // Base + 12 - TBDL high
                  if(allow_bus_ops) csr_xl <= 1'b0;
               3'b110:        // Base + 14 - VAR
                  var_iv[7:6] <= ewb_dat_i[9:8];
               3'b111: begin  // Base + 16 - CSR
                  if(allow_bus_ops) begin
                     csr_il <= ewb_dat_i[8];
                     csr_el <= ewb_dat_i[9];
                     csr_se <= ewb_dat_i[10];
                     if(ewb_dat_i[15] == 1'b1)
                        csr_ri <= 1'b0;
                  end
               end
            endcase
         end
      end
`ifdef md_debug
      if (e_md_clr) begin
         ext_mdctrl[5] <= 1'b0;
         ext_md_ack    <= 1'b1;
      end
      else
         ext_md_ack <= 1'b0;
`endif
      // Запись регистров внутренняя шина
      if (lwrite_req) begin
         if (lwb_sel_i[0]) begin   // Запись младшего байта
            case (lwb_adr_i[2:0])
               3'b000:     // Base + 00
                  blkreg <= lwb_dat_i[0];
               3'b111: begin  // Base + 16 - CSR
                  csr_ni <= lwb_dat_i[2];
                  csr_xl <= lwb_dat_i[4];
                  csr_rl <= lwb_dat_i[5];
                  csr_xi <= lwb_dat_i[7];
               end
            endcase
         end
         if(lwb_sel_i[1]) begin    // Запись старшего байта
            case (lwb_adr_i[2:0])
               3'b111:        // Base + 16 - CSR
                  csr_ri <= lwb_dat_i[15];
            endcase
         end
      end
   end
end

//************************************************
// Работа с регистрами ethernet и регистром общего назначения
// Чтение регистров
always @(posedge lwb_clk_i) begin
	if (lerd_req == 1'b1) begin
		case (lwb_adr_i[3:0])
			4'b0000:	// 24040 - ошибки приема/передачи и данные регистра режима работы
            etdat <= {3'b0, e_crser, e_txerg, e_rxerg, e_mode[8:1], e_txdn, etprdy_i};
			4'b0010:	// 24044 - данные MD
				etdat <= e_mdval_i;
			4'b0011:	// 24046 - статус MD
				etdat <= {e_mdstat_i[7:0], 1'b0, e_mdctrl[6:0]};
			4'b0100:	// 24050 - данные регистра общего назначения
				etdat <= {bdroms, 10'b0, stm_res, stm_ena, leds[2:0]};
//			4'b0110:	// 24054
//			4'b0111:	// 24056
			4'b1000: // 24060 - MAC-адрес
				etdat <= {macval[15:8], macval[7:0]};
			4'b1001: // 24062 - MAC-адрес
				etdat <= {macval[31:24], macval[23:16]};
			4'b1010: // 24064 - MAC-адрес
				etdat <= {macval[47:40], macval[39:32]};
			4'b1011: // 24066 - Контрольная сумма
				etdat <= {macval[55:48], macval[63:56]};
		endcase
	end
end
// Сброс и запись регистров
always @(posedge lwb_clk_i, posedge comb_res) begin
   if(comb_res) begin 
      e_mdctrl <= 7'b0;    // регистр управления MD
      txrdy <= 1'b0;       // флаг готовности данных передачи
      skipb <= 1'b0;       // флаг пропуска байта
      stpac <= 1'b0;       // флаг setup-пакета
      mcast <= 1'b0;       // флаг широковещания
      promis <= 1'b0;      // флаг прослушивания
      leds <= 3'b0;        // регистр индикации
      stm_res <= 1'b0;     // регистр генерации BDCOK
      e_txcntb <= 11'b0;   // регистр кол-ва байт передачи
      e_mdval <= 16'b0;    // регистр данных MD
      stm_ena <= 1'b0;
`ifdef md_debug
      e_md_clr <= 1'b0;
`endif
   end
   else if (lewr_req == 1'b1) begin
      if (lwb_sel_i[0] == 1'b1) begin  // Запись младшего байта
         case (lwb_adr_i[3:0])
            4'b0000:			// 24040 - установка контрольных сигналов
               {promis, mcast, skipb, stpac, txrdy} <= lwb_dat_i[5:1];
            4'b0001:			// 24042 - запись кол-ва байт для передачи
               e_txcntb[7:0] <= lwb_dat_i[7:0];
            4'b0010:			// 24044 - данные для MD
               e_mdval[7:0] <= lwb_dat_i[7:0];
            4'b0011:			// 24046 - управляющие сигналы для MD
               e_mdctrl[6:0] <= lwb_dat_i[6:0];
            4'b0100: begin	// 24050 - управляющие сигналы РОНЕ
               stm_res <= lwb_dat_i[4];
               stm_ena <= lwb_dat_i[3];
               leds[2:0] <= lwb_dat_i[2:0];
            end
//            4'b0110: begin     // 24054
//            end
//            4'b0111: begin     // 24056
//            end
         endcase
      end
      if(lwb_sel_i[1] == 1'b1) begin   // Запись старшего байта
         case (lwb_adr_i[3:0])
            4'b0001:	// 24042 - запись кол-ва байт для передачи
               e_txcntb[10:8] <= lwb_dat_i[10:8];
            4'b0010:	// 24044 - данные для MD
               e_mdval[15:8] <= lwb_dat_i[15:8];
//            4'b0100: // 24050 - управляющие сигналы РОНЕ
         endcase
      end
   end
   else begin
`ifdef md_debug
      if((ext_mdctrl[5] == 1'b1) && (e_mdctrl[5] == 1'b0) && (e_mdstat_i[7] == 1'b1)) begin
         e_mdctrl[4:0] <= ext_mdctrl[4:0];
         e_mdctrl[6] <= ext_mdctrl[6];
         e_mdval[15:0] <= ext_mdval[15:0];
         e_mdctrl[5] <= 1'b1;
         e_md_clr <= 1'b1;
      end
      if (ext_md_ack)
         e_md_clr   <= 1'b0;
`endif
      if(~e_mdstat_i[7] & e_mdctrl[5]) // Сброс сигнала старта MDC
         e_mdctrl[5] <= 1'b0;
   end
end

endmodule

//============= Модуль синхронизации =================//
module syncheb(
	input				clk_i,
	input  [4:0]	ethsig_i,
   output			e_crs_o,
   output			e_crser_o,
   output			e_txerg_o,
   output			e_rxerg_o,
   output			e_txdn_o
);

reg  [1:0] e_crs, e_crser, e_txerg, e_rxerg, e_txdn;
assign e_crs_o = e_crs[1];
assign e_crser_o = e_crser[1];
assign e_txerg_o = e_txerg[1];
assign e_rxerg_o = e_rxerg[1];
assign e_txdn_o = e_txdn[1];

always @(posedge clk_i) begin
   e_txdn[0]  <= ethsig_i[0]; e_txdn[1]  <= e_txdn[0];
   e_rxerg[0] <= ethsig_i[1]; e_rxerg[1] <= e_rxerg[0];
   e_txerg[0] <= ethsig_i[2]; e_txerg[1] <= e_txerg[0];
   e_crser[0] <= ethsig_i[3]; e_crser[1] <= e_crser[0];
   e_crs[0]   <= ethsig_i[4]; e_crs[1]   <= e_crs[0];
end

endmodule
