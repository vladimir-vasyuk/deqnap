//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
// Основной модуль Ethernet
//=================================================================================
module ether(
	input				rst_i,		// Сброс
   input          rxfifoe_i,  // Разрешение работы с FIFO канала приема
   input          txfifoe_i,  // Разрешение работы с FIFO канала передачи
	input  [10:0]	txcntb_i,	// Счетчик данных передачи (байт)
	input  [15:0]	etxdbus_i,	// Шина данных блока передачи (память -> ether модуль)
	input  [9:0]	ethmode_i,	// Режим работы модуля Ethernet
	output [15:0]	erxdbus_o,	// Шина данных блока приема (ether модуль -> память)
	output [15:0]	rxfrsts_o,  // Статус принятого кадра (ошибки и кол-во байт)
	output			erdatwrn_o,	// Сигнал записи данных
   output			ercnfwrn_o,	// Сигнал записи данных флагов
   output         etxinca_o,  // Сигнал инкремента адреса FIFO
	output			rxclkb_o,	// Синхросигнал канала приема (запись в буферную память)
   output         txclkb_o,   // Синхросигнал канала передачи (чтение из буферной памяти)
	output [4:0]	stserrs_o,  // статус и ошибки приема/передачи
   output         enocsr_o,

	input				e_rxc,		// Синхросигнал канала приема
	input				e_rxdv,		// Сигнал готовности данных канала приема
	input				e_rxer,		// Ошибка канала приема
	input  [7:0]	e_rxd,		// Данные канала приема
	input				e_crs,		// Сигнал наличия несущей
	input				e_txc,		// Синхросигнал канала передачи
	output			e_txen,		// Сигнал разрешения передачи
	output			e_txer,		// Сигнал ошибки канала передачи
	output [7:0]	e_txd,		// Данные канала передачи
	output			e_rst,		// Сброс, активный - низкий
	output			e_gtxc,		// Опорный синхросигнал для 1Gb
	inout				e_mdio,		// Блок управления - линия данных
	input				md_clk_i,   // Блок управления - синхросигнал
	input				md_evt_i,   // Блок управления - сигнал опроса состояния
	input  [6:0]	md_ctr_i,   // Блок управления - сигналы управления от компьютера
	input  [15:0]	md_val_i,   // Блок управления - данные записи
	output [15:0]	md_out_o,   // Блок управления - данные чтения
	output [7:0]	md_sts_o,   // Блок управления - данные состояния

	output [1:0]	prmstp_o,	// Режим прослушивания/установки
	output			macrdy_o,   // MAC адрес сформирован
	output [47:0]	macdat_o,	// MAC адрес принятого кадра
	input				cmpdon_i,	// Операция сравнения завершена
	input				cmpres_i		// Результат операции сравнения
);

assign e_rst = ~rst_i;
assign e_gtxc = e_rxc;
//
//======================= CRC=======================//
wire [31:0] crctx;		// CRC канала передачи
wire [31:0] crcrx;		// CRC канала приема 
wire			crcretx;		// Сигнал сброса CRC канала передачи
wire			crcentx;		// Сигнал разрешения CRC канала передачи
wire			crcrerx;		// Сигнал сброса CRC канала приема
wire			crcenrx;		// Сигнал разрешения CRC канала приема

//======================= MDC ======================//
wire			mdc_err;		// Сигнал ошибки (!!! пока не используется)

//================== Прием/передача =================//
wire			skipb;		// Пропуск байта (DescriptorBits[6])
wire			rxena;		// Разрешение приема
wire			rx_errg;		// Ошибка канала приема с учетом блока MD
wire			tx_errg;		// Ошибка канала передачи с учетом блока MD
wire			crs_err;		// Отсутствие несущей
wire			txdone;		// Передача завершена 
assign crs_err = (e_rxer & (~e_rxdv))? 1'b1 : 1'b0;
assign rx_errg = loop? 1'b0 : ( ~md_sts_o[0] | ~md_sts_o[1]);
assign stserrs_o = {e_crs, crs_err, tx_errg, rx_errg, txdone};

//================ Синхронизация ====================//
wire			loop;				// Сигнал работы петли
wire			mcast;			// Режим широковещания разрешен (пока не используется)
wire			txrdyl;			// Сигнал готовности данных передачи
wire			rx_enable;		// Разрешение приема
wire        nocrc;         // Не обрабатывать CRC

synchonize synch(
	.clk_i(e_rxc),
	.ethmode_i(ethmode_i),
	.rx_ena_o(rx_enable),
	.skipb_o(skipb),
	.mcast_o(mcast),
	.prmstp_o(prmstp_o),
	.txrdy_o(txrdyl),
   .nocrc_o(nocrc),
	.loop_o(loop)
);
assign enocsr_o = nocrc;

// ===== Gigabit mode - speed=1000 and link=OK ======//
wire        gbmode;
assign gbmode	= ((md_sts_o[6:5] == 2'b10) & (md_sts_o[0] == 1'b1))? 1'b1 : 1'b0;

//===== Мультиплексор 4->8 входной шины данных ======//
wire			rxdv;				// Сигнал готовности данных блока приема
wire			rxer;				// Сигнал ошибки данных канала приема
wire			rxerm;			// Выходной сигнал ошибки данных канала приема
wire [7:0]	rxdb;				// Шина данных канала приема
wire [7:0]	ddinm;			// Мультиплексированные данные для 10Mb-100Mb
wire			rxclkm;			// Новый синхросигнал для 10Mb-100Mb
wire			rxdvm;			// Выходной сигнал готовности данных блока приема
wire			rxclk;			// Синхросигнал канала приема
ddin dd_in(
   .rxclk_i(e_rxc),
   .rst(rst_i),
   .rxdv(e_rxdv),
   .dat_i(e_rxd[3:0]),
   .dat_o(ddinm),
   .rxclk_o(rxclkm),
   .rxdv_o(rxdvm),
   .rxer_o(rxerm)
);
assign rxclk = gbmode? e_rxc : rxclkm;
assign rxdb = gbmode? e_rxd : ddinm;                                       // Мультиплексированные данные
assign {rxdv, rxer} = gbmode? {e_rxdv, e_rxer} : {rxdvm, rxerm | e_rxer};  // Сигналы управления

//===== Демультиплексор 8->4 выходной шины данны =====//
wire [3:0]	ddoutm;			// Выходные данные для 10Mb-100Mb
wire			txens, txers;	// Управляющие сигналы канала передачи
wire			txeno, txero;	// Управляющие сигналы канала передачи
wire [7:0]	txdb;				// Входные данные канала передачи
wire			txclk;			// Синхросигнал канала передачи
wire			txclkm;			// Новый синхросигнал для 10Mb-100Mb

ddout dd_out(
   .txclk_i(e_txc),
   .rst(rst_i),
   .txen_i(txens),
   .dat_i(txdb),
   .dat_o(ddoutm),
   .txclk_o(txclkm),
   .txen_o(txeno),
   .txerr_o(txero)
);
assign txclk = gbmode? e_gtxc : txclkm;							// Синхросигнал канала передачи
assign e_txd[7:0] = gbmode? txdb[7:0] : {4'o0,ddoutm[3:0]};	// Демультиплексированные данные
assign {e_txen, e_txer} = loop? {1'b0, 1'b0} : (gbmode? {txens, txers} : {txeno, txero | txers});  // Сигналы управления
assign tx_errg = loop? 1'b0 : (e_txer | ~md_sts_o[0]);

//======== Обработка данных канала передачи ========//
wire        txclkl;			// Синхросигнал канала передачи с учетом петли
assign txclkl	= loop? e_rxc : txclk;
assign txclkb_o = txclkl;

ethsend ethsendm(
   .clk_i(txclkl),
   .clr_i(rst_i),
   .txena_i(txrdyl),
   .txdatv_i(txfifoe_i),
   .nocrc_i(nocrc),
   .txdone_o(txdone),
   .txend_o(txens),
   .dataout_o(txdb),
   .crc_i(crctx),
   .txbdata_i(etxdbus_i),
   .txinca_o(etxinca_o),
   .txcntb_i(txcntb_i),
   .skipb_i(skipb),
   .crcen_o(crcentx),
   .crcre_o(crcretx),
   .errgen_o(txers)
);

//========= Обработка данных канала приема =========//
wire [7:0]	rxdbl;			// Шина данных канала приема с учетом петли
wire			rxclkl;			// Синхросигнал канала приема с учетом петли
wire			rxdvl;			// Сигнал готовности данных блока приема с учетом петли
wire			rxerl;			// Сигнал ошибки данных канала приема с учетом петли

assign rxdbl = loop? txdb : rxdb;
assign rxdvl = loop? txens : rxdv;
assign rxclkl = loop? ~e_rxc : rxclk;
assign rxerl = loop? txers : rxer;
assign rxclkb_o = rxclkl;
assign rxena = (loop? loop : rx_enable) & rxfifoe_i;

ethreceive ethrcvm(
   .clk_i(rxclkl),
   .clr_i(rst_i),
   .rxena_i(rxena),
   .data_i(rxdbl),
   .rxdv_i(rxdvl),
   .rxer_i(rxerl),
   .nocrc_i(nocrc),
   .rxfrsts_o(rxfrsts_o),
   .data_o(erxdbus_o),
   .datwe_o(erdatwrn_o),
   .cnfwe_o(ercnfwrn_o),
   .crc_i(crcrx),
   .crcen_o(crcenrx),
   .crcre_o(crcrerx),
	.macdat_o(macdat_o),
	.macrdy_o(macrdy_o),
	.cmpdon_i(cmpdon_i),
	.cmpres_i(cmpres_i)
);

//==== Контрольная сумма данных канала передачи ====//
crc_n crc_tx(
   .clk_i(txclkl),
   .rst_i(crcretx),
   .data_i(txdb),
   .crcen_i(crcentx),
   .crc_o(crctx)
);

//===== Контрольная сумма данных канала приема =====//
crc_n crc_rx(
   .clk_i(rxclkl),
   .rst_i(crcrerx),
   .data_i(rxdbl),
   .crcen_i(crcenrx),
   .crc_o(crcrx)
);

//================ Блок управления =================//
mdint mdintm(
   .clk_i(md_clk_i),
   .rst_i(rst_i),
   .evt_i(md_evt_i),
   .mdiol(e_mdio),
   .err_o(mdc_err),
   .ctl_i(md_ctr_i),
   .val_i(md_val_i),
   .val_o(md_out_o),
   .sts_o(md_sts_o)
);

endmodule

//============= Блок синхронизации =================//
module synchonize(
	input				clk_i,
	input  [9:0]	ethmode_i,
	output			rx_ena_o,
	output			skipb_o,
	output			txrdy_o,
	output			mcast_o,
	output [1:0]	prmstp_o,
   output			nocrc_o,
	output			loop_o
);

reg  [1:0]	rx_ena_r, iloop_r, ieloop_r, eloop_r, mcast_r;
reg  [1:0]	skipb_r, setup_r, txrdy_r, promis_r, nocrc_r;
wire			int_loop_o, inte_loop_o, ext_loop_o, setup_o;
assign rx_ena_o = rx_ena_r[1];
assign int_loop_o = iloop_r[1];
assign inte_loop_o = ieloop_r[1];
assign ext_loop_o = eloop_r[1];
assign skipb_o = skipb_r[1];
assign setup_o = setup_r[1];
assign txrdy_o = txrdy_r[1];
assign mcast_o = mcast_r[1];
assign prmstp_o[1] = promis_r[1] | inte_loop_o | setup_o | ext_loop_o;
assign prmstp_o[0] = setup_o;
assign nocrc_o = nocrc_r[1];
assign loop_o = int_loop_o | inte_loop_o | setup_o;

always @(posedge clk_i) begin
	rx_ena_r[0] <= ethmode_i[0]; rx_ena_r[1] <= rx_ena_r[0];
	iloop_r[0] <= ethmode_i[1]; iloop_r[1] <= iloop_r[0];
	ieloop_r[0] <= ethmode_i[2]; ieloop_r[1] <= ieloop_r[0];
	eloop_r[0] <= ethmode_i[3]; eloop_r[1] <= eloop_r[0];
	setup_r[0] <= ethmode_i[4]; setup_r[1] <= setup_r[0];
	skipb_r[0] <= ethmode_i[5]; skipb_r[1] <= skipb_r[0];
	txrdy_r[0] <= ethmode_i[6]; txrdy_r[1] <= txrdy_r[0];
	mcast_r[0] <= ethmode_i[7]; mcast_r[1] <= mcast_r[0];
	promis_r[0] <= ethmode_i[8]; promis_r[1] <= promis_r[0];
   nocrc_r[0] <= ethmode_i[9]; nocrc_r[1] <= nocrc_r[0];
end

endmodule

