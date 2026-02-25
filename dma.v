//=================================================================================
// Реализация контроллера Ethernet DEQNA/DELQA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль DMA
//=================================================================================
module dma (
	input					clk_i,		// тактовая частота шины
	input					rst_i,		// сброс
// Внутренняя шина (доступ к регистрам)
	input  [2:0]		wb_adr_i,	// адрес
	input  [15:0]		wb_dat_i,	// входные данные
	output [15:0]		wb_dat_o,	// выходные данные
	input					wb_cyc_i,	// начало цикла шины
	input					wb_we_i,		// разрешение записи (0 - чтение)
	input  [1:0]		wb_sel_i,	// выбор байтов для записи 
	input					wb_stb_i,	// строб цикла шины
	output				wb_ack_o,	// подтверждение выбора устройства
// Шина ПДП/DMA (передача данных)
	output				dma_req_o,	// запрос DMA
	input					dma_gnt_i,	// подтверждение DMA
	output [21:0]		dma_adr_o,	// выходной адрес при DMA-обмене
	input  [15:0]		dma_dat_i,	// входная шина данных DMA
	output [15:0]		dma_dat_o,	// выходная шина данных DMA
	output				dma_stb_o,	// строб цикла шины DMA
	output				dma_we_o,	// направление передачи DMA (0 - память->модуль, 1 - модуль->память) 
	input					dma_ack_i,	// Ответ от устройства, с которым идет DMA-обмен
	input  [15:0]		lbdata_i,	// входная шина данных
	output [15:0]		lbdata_o,	// выходная шина данных
	output				dma_txb_o,  // Сигнал мультиплексера адреса ПДП/DMA буфера передачи
	output				dma_rxb_o,  // Сигнал мультиплексера адреса ПДП/DMA буфера приема
	output				dma_bdl_o,  // Обший сигнал работы в редиме ПДП/DMA
   output            dma_inca_o, // Сигнал инкремента адреса
   input             dma_rerr_i, // Ошибка чтения данных
   input             dma_werr_i  // Ошибка записи данных
);

// регистры контроллера DMA
reg  [1:0]	dma_op;			// регистры операции                         -- 24020
reg  [15:0]	dma_wcount;		// кол-во слов передачи                      -- 24022
reg  [1:0]	dma_ldm;       // устройство обмена (rx, tx, bdl)           -- 24024
reg  [21:1]	dma_haddr;		// физический адрес памяти                   -- 24026/24030 
reg  [7:0]	ibus_wait;		// таймер ожидания ответа при DMA-обмене     -- 24032
reg  [7:0]	bus_wait;		// текущее значение таймера ожидания
reg  [15:0] data_index;		// указатель текущего слова
reg			nxm;				// признак таймаута шины
reg			iocomplete;		// признак завершения работы DMA-контроллера
reg  [15:0]	dma_dat, wb_dat, lbdata;
reg			dma_req, dma_stb, dma_we;
reg         dma_inc_ladr;  //

assign dma_adr_o = {dma_haddr, 1'b0};
assign dma_dat_o = dma_dat;
assign dma_req_o = dma_req;
assign dma_stb_o = dma_stb;
assign dma_we_o = dma_we;
assign lbdata_o = lbdata;
assign wb_dat_o = wb_dat;
assign dma_txb_o = ~dma_ldm[0] & dma_ldm[1];    // 2'b10
assign dma_rxb_o = dma_ldm[0] & ~dma_ldm[1];    // 2'b01
assign dma_bdl_o = dma_ldm[0] & dma_ldm[1];     // 2'b11
assign dma_inca_o = dma_inc_ladr;

wire			wstart, rstart;
assign wstart = dma_op[1];
assign rstart = dma_op[0];

// конечный автомат контроллера DMA
localparam[3:0] dma_idle = 0; 
localparam[3:0] dma_read_prep = 1;
localparam[3:0] dma_read = 2;
localparam[3:0] dma_read_next = 3;
localparam[3:0] dma_read_done = 4;
localparam[3:0] dma_write_prep = 5; 
localparam[3:0] dma_write = 6;
localparam[3:0] dma_write_next = 7;
localparam[3:0] dma_write_done = 8;
reg  [3:0]	dma_state;

// Сигналы упраления обменом с шиной
wire			bus_strobe, bus_read_req, bus_write_req;
assign bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
assign bus_read_req = bus_strobe & ~wb_we_i;				// запрос чтения
assign bus_write_req = bus_strobe & wb_we_i;				// запрос записи

reg  [1:0]	ack;
always @(posedge clk_i) begin
	ack[0] <= wb_cyc_i & wb_stb_i;
	ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

always @(posedge clk_i, posedge rst_i)  begin
   if (rst_i) begin
   // сброс
      dma_state <= dma_idle; 
      dma_req <= 1'b0; 
      dma_we <= 1'b0; 
      dma_stb <= 1'b0; 
      nxm <= 1'b0; 
      iocomplete <= 1'b0;
      dma_haddr <= 21'b0;
		dma_wcount <= 16'hFFFF;
		dma_op <= 2'b0;
      dma_ldm <= 2'b0;
		ibus_wait<= 8'b11111111;
      dma_inc_ladr <= 1'b0;
   end

   // рабочие состояния
   else  begin
		// Работа с внутренней шиной
		if (bus_read_req == 1'b1) begin
			case (wb_adr_i[2:0])
				3'b000:	//24020
					wb_dat <= {8'b0, iocomplete, nxm, 4'b0, dma_op[1:0]};
				3'b001:	//24022
					wb_dat <= dma_wcount;
				3'b010:	//24024
					wb_dat <= {14'b0, dma_ldm[1:0]};
				3'b011:	//24026
					wb_dat <= {dma_haddr[15:1], 1'b0};
				3'b100:	//24030
					wb_dat <= {10'b0, dma_haddr[21:16]};
				3'b101:	//24032
					wb_dat <= {10'b0, ibus_wait[5:0]};
			endcase
		end
		else if (bus_write_req == 1'b1) begin
			if (wb_sel_i[0] == 1'b1) begin   // Запись младшего байта
				case (wb_adr_i[2:0])
					3'b000:	//24020
						dma_op <= wb_dat_i[1:0];
					3'b001:	//24022
						dma_wcount[7:0] <= wb_dat_i[7:0];
					3'b010:	//24024
						dma_ldm <= wb_dat_i[1:0];
					3'b011:	//24026
						dma_haddr[7:1] <= wb_dat_i[7:1];
					3'b100:	//24030
						dma_haddr[21:16] <= wb_dat_i[5:0];
					3'b101:	//24032
						ibus_wait <= wb_dat_i[7:0];
				endcase
			end
			if (wb_sel_i[1] == 1'b1) begin    // Запись старшего байта
				case (wb_adr_i[2:0])
					3'b001:	//24022
						dma_wcount[15:8] <= wb_dat_i[15:8];
					3'b011:	//24026
						dma_haddr[15:8] <= wb_dat_i[15:8];
				endcase
			end
		end
		// конечный автомат ПДП/DMA
      case (dma_state)
         // ожидание запроса
         dma_idle: begin
            nxm <= 1'b0;											//  снимаем флаг ошибки nxm
            dma_we <= 1'b0;
            dma_inc_ladr <= 1'b0;
            data_index <= dma_wcount;							// счетчик слов

            // старт процедуры записи
            if (wstart == 1'b1) begin
               dma_req <= 1'b1;									// поднимаем запрос DMA
               if (dma_gnt_i == 1'b1)							// ждем подтверждения 
                  dma_state <= dma_write_prep;				// и переходим к записи
            end

            // старт процедуры чтения
            else if (rstart == 1'b1) begin
               dma_req <= 1'b1;									// поднимаем запрос DMA
               if (dma_gnt_i == 1'b1)							// ждем подтверждения
                  dma_state <= dma_read_prep;				// и переходим к чтению
            end
            else iocomplete <= 1'b0;
         end

         // чтение данных - подготовка шины к DMA
         dma_read_prep: begin
            dma_we <= 1'b0;
            dma_stb <= 1'b0;
            bus_wait <= ibus_wait;								// взводим таймер ожидания шины
            dma_state <= dma_read;								// переходим к чтению
         end

         dma_read: begin
            dma_dat <= lbdata_i;									// выставляем данные
            dma_we <= 1'b1;										// режим записи
            dma_stb <= 1'b1;										// строб транзакции
            bus_wait <= bus_wait - 1'b1;						// таймер ожидания ответа
            if ((|bus_wait == 0 ) | dma_rerr_i) begin
               // таймаут шины
               nxm <= 1'b1;										// флаг ошибки DMA
               dma_we <= 1'b0;
               dma_stb <= 1'b0;									// снимаем строб транзакции
               dma_state <= dma_read_done;					// завершаем чтение
            end
            else if (dma_ack_i == 1'b1) begin
               dma_stb <= 1'b0;									// снимаем строб транзакции
               data_index <= data_index + 1'b1;				// уменьшаем счетчик слов для передачи
               dma_we <= 1'b0;
               dma_inc_ladr <= 1'b1;
               dma_state <= dma_read_next;
            end
         end

         dma_read_next: begin
				dma_haddr <=  dma_haddr +  1'b1;					// увеличиваем физический адрес
            dma_inc_ladr <= 1'b0;                        // увеличиваем адрес
            if (|data_index != 0)								// все записано?
               dma_state <= dma_read_prep;					// нет - продолжаем
            else
               dma_state <= dma_read_done;					// да - завершаем
         end

         // чтение данных - завершение
         dma_read_done: begin
            dma_req <= 1'b0;										// освобождаем шину
            if (rstart == 1'b0) begin
               dma_state <= dma_idle;							// переходим в состояние ожидания команды
               iocomplete <= 1'b0;								// снимаем подтверждение окончания обмена
            end
            else
               iocomplete <= 1'b1;								// подтверждаем окончание обмена
         end

         // запись данных - подготовка шины к DMA
         dma_write_prep: begin
            dma_we <= 1'b0;
            dma_stb <= 1'b1;										// строб транзакции
            bus_wait <= ibus_wait;								// взводим таймер ожидания шины
            dma_state <= dma_write;
         end

         // запись данных - обмен по шине
         dma_write: begin
            bus_wait <= bus_wait - 1'b1;						// таймер ожидания ответа
				lbdata <= dma_dat_i;
            if ((|bus_wait == 0) | dma_werr_i) begin
               nxm <= 1'b1;										// флаг ошибки DMA
               dma_we <= 1'b0;
               dma_stb <= 1'b0;									// снимаем строб транзакции
               dma_state <= dma_write_done;					// завершаем запись
            end
            else if (dma_ack_i == 1'b1) begin
               dma_we <= 1'b0;
               dma_stb <= 1'b0;									// снимаем строб транзакции
               data_index <= data_index + 1'b1;				// уменьшаем счетчик слов для передачи
               dma_inc_ladr <= 1'b1;
               dma_state <= dma_write_next;
            end
         end

         // запись данных - изменение адреса и проверка
         dma_write_next: begin
				dma_haddr <=  dma_haddr +  1'b1;					// увеличиваем физический адрес
            dma_inc_ladr <= 1'b0;                        // увеличиваем адрес
            if (|data_index != 0)								// все записано?
               dma_state <= dma_write_prep;					// нет - продолжаем
            else
               dma_state <= dma_write_done;					// да - завершаем
         end

         // запись данных - завершение
         dma_write_done: begin
            dma_req <= 1'b0;										// освобождаем шину
            if (wstart == 1'b0)  begin
               iocomplete <= 1'b0;								// снимаем подтверждение окончания обмена
               dma_state <= dma_idle;							// переходим в состояние ожидания команды
            end
            else iocomplete <= 1'b1;							// подтверждаем окончание обмена
         end
      endcase
   end
end

endmodule
