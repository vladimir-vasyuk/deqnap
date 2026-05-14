//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль передачи кадра данных
//=================================================================================
module ethsend(
   input         clk_i,       // Синхросигнал
   input         clr_i,       // Сигнал сброса
   input         txena_i,     // Сигнал разрешения передачи
   input         txdatv_i,    // Сигнал  наличия данных
   input         nocrc_i,     // Не обрабатывать CRC
   output        txdone_o,    // Сигнал завершения передачи
   output        txend_o,     // Сигнал достоверности данных
   output [7:0]  dataout_o,   // Шина выходных данных
   input  [31:0] crc_i,       // Шина CRC
   input  [15:0] txbdata_i,   // Шина входных данных
   output        txinca_o,    // Сигнал инкремента адреса FIFO
   input  [10:0] txcntb_i,    // Счетчик переданных байтов
   input         skipb_i,     // Пропуск байта (H-bit of the "Address Descriptor Bits")
   output        crcen_o,     // Cигнал разрешения вычисления CRC
   output        crcre_o,     // Cигнал сброса CRC
   output        errgen_o     // Сигнал общей ошибки (не задействован)
);

reg         txdone, txend, txinca, crcen, crcre, txer;
reg  [7:0]  dataout;

assign txdone_o = txdone;
assign txend_o = txend;
assign dataout_o = dataout;
assign txinca_o = txinca;
assign crcen_o = crcen;
assign crcre_o = crcre;
assign errgen_o = txer;

reg  [10:0] i;				// Внутренний счетчик
reg  [15:0] bufdat;		// Буфер данных передачи
reg  [1:0]  bc;			// Счетчик байтов

// Конечный автомат канала передачи
localparam IDLE		= 3'd0;
localparam SENDPRE	= 3'd1;
localparam SENDDATA	= 3'd2;
localparam SENDCRC	= 3'd3;
localparam TXDELAY	= 3'd4;
localparam WAITDONE	= 3'd5;
reg  [2:0]  tx_state;

// Инициализация
initial
   begin
      tx_state <= IDLE;
   end

// Основной блок
always@(negedge clk_i, posedge clr_i) begin
   if(clr_i) tx_state <= IDLE;
   else begin
      case(tx_state)
         IDLE: begin
            txer <= 1'b0;              // Сброс сигнала ошибки
            txend <= 1'b0;             // Сброс сигнала достоверности данных
            crcen <= 1'b0;             // Сброс сигнала разрешения вычисления CRC
            crcre <= 1'b1;             // Установка сигнала сброса CRC
            txinca <= 1'b0;
            i <= 11'b0;                // Сброс счетчика
            bc <= 2'o0;                // Сброс счетчика байтов
            txdone <= 1'b0;            // Сброс сигнала завершения передачи
            if(txena_i & txdatv_i) begin
               tx_state <= SENDPRE;
               if(skipb_i) begin       // Если установлен H-bit, ...
                  bc <= 2'o1;          // ... пропустить один байт.
               end
            end
         end
         SENDPRE: begin    // Сигналы сихронизации и начала кадра
            txend <= 1'b1;                // Установить сигнал достоверности данных
            crcre <= 1'b1;                // Сброс CRC
            if(i < 7) begin               // Передача 7 ...
               dataout[7:0] <= 8'h55;     // ... байтов ...
               i <= i + 1'b1;             // ... преамбулы
            end
            else begin
               dataout[7:0] <= 8'hD5;	   // Байт разделитель
               i <= txcntb_i;             // Число байтов для передачи
               if(txdatv_i) begin           // FIFO не пустое?
                  bufdat <= txbdata_i;    // Да - принять новые данные, ...
                  txinca <= 1'b1;         // ... сигнал инкремента адреса, ...
                  tx_state <= SENDDATA;	// ... переход к передачи данных
               end
               else begin                 // FIFO  опустошено раньше времени - ошибка, ...
                  txer <= 1'b1; txinca <= 1'b0; // Установка сигнала ошибки, сброс сигнала...
                  tx_state <= WAITDONE;   // ...  инкремента адреса, переход на завершение
               end
            end
         end
         SENDDATA: begin   // Передача данных
            crcen <= 1'b1;                // Сигнал разрешения вычисления CRC
            crcre <= 1'b0;                // Убрать сигнал сброса CRC
            if(i == 11'h7FF) begin        // Последний байт данных?
               i <= 11'h0;                // Да - обнулить счетчик ...
               txinca <= 1'b0;            // ... сброс сигнала инкремента адреса
               if(nocrc_i) begin
                  tx_state <= TXDELAY;    // На завершение
                  crcen <= 1'b0;
               end
               else
                  tx_state <= SENDCRC;    // На передачу CRC
            // Передача последнего байта данных
               if(bc == 2'o0) begin
                  dataout[7:0] <= bufdat[7:0];
               end
               else if(bc == 2'o1) begin
                  dataout[7:0] <= bufdat[15:8];
                  bc <= 2'o0;
               end
            end
            else begin									//  Не последний байт данных, ...
               i <= i + 1'b1;							// ... инкремент счетчика ...
               case(bc)									// Передача очередного байта данных
                  2'o0: begin
                     dataout[7:0] <= bufdat[7:0]; // Данные на шину передачи, ...
                     bc <= bc + 1'b1;				// ... инкремент счетчика байтов, ...
                     txinca <= 1'b0;            // ... сброс сигнала инкремента адреса
                  end
                  2'o1: begin
                     dataout[7:0] <= bufdat[15:8]; // Данные на шину передачи, ...
                     bc <= 2'b0;						// ... сброс счетчика байтов, ..
                     if(txdatv_i) begin           // FIFO не пустое?
                        bufdat <= txbdata_i;    // Да - принять новые данные и ...
                        txinca <= 1'b1;         // ... сброс сигнала инкремента адреса
                     end
                     else begin
                        txer <= 1'b1;           // FIFO опустошено раньше времени - ошибка, ...
                        txinca <= 1'b0;         // ... сброс сигнала инкремента адреса, ...
                        if(nocrc_i) begin
                           tx_state <= TXDELAY; // На завершение
                           crcen <= 1'b0;
                        end
                        else
                           tx_state <= SENDCRC; // На передачу CRC
                     end
                  end
               endcase
            end
         end
         SENDCRC: begin		// Передача контрольной сумм (CRC)
            crcen <= 1'b0;
            case(bc)
               2'o0: begin
                  dataout[7:0] <= {~crc_i[24],~crc_i[25],~crc_i[26],~crc_i[27],~crc_i[28],~crc_i[29],~crc_i[30],~crc_i[31]};
                  bc <= bc + 1'b1;
               end
               2'o1: begin
                  dataout[7:0] <= {~crc_i[16],~crc_i[17],~crc_i[18],~crc_i[19],~crc_i[20],~crc_i[21],~crc_i[22],~crc_i[23]};
                  bc <= bc + 1'b1;
               end
               2'o2: begin
                  dataout[7:0] <= {~crc_i[8],~crc_i[9],~crc_i[10],~crc_i[11],~crc_i[12],~crc_i[13],~crc_i[14],~crc_i[15]};
                  bc <= bc + 1'b1;
               end
               2'o3: begin
                  dataout[7:0] <= {~crc_i[0],~crc_i[1],~crc_i[2],~crc_i[3],~crc_i[4],~crc_i[5],~crc_i[6],~crc_i[7]};
                  bc <= bc + 1'b1;
                  tx_state <= TXDELAY;
               end
            endcase
         end
         TXDELAY: begin		// Задержка 12 байт и установка сигнала завершения передачи
            txend <= 1'b0;             // Сброс сигнала достоверности данных
            dataout <= 8'hFF;          // Заглушка
            if(i < 11'd12) i <= i + 1'b1;	// Таймаут 12 байтов
            else begin
               txdone <= 1'b1;			// Таймаут завершен, сигнал завершения передачи ...
               tx_state <= WAITDONE;	// ... и переход к ожиданию сигнала подтверждения
            end
         end
         WAITDONE: begin	// Возврат в состояние ожидания
            if(txena_i == 1'b0) begin		// Получен сигнал подтверждения?
               txdone <= 1'b0;			// Да - сброс сигнала завершения ...
               tx_state <= IDLE;			// ... и переход в состояние ожидания
            end
         end
      endcase
   end
end

endmodule
