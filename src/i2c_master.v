`timescale 1ns / 1ps

// ============================================================================
// Модуль: i2c_master
// Опис:   Синтезований ведучий пристрій (Master) послідовної шини I2C
//         із підтримкою 7-бітної адресації, операцій запису/читання,
//         контролю підтвердження ACK/NACK та виходів із відкритим стоком.
// Стандарт: Verilog-2001 (IEEE 1364-2001)
// ============================================================================
module i2c_master #(
    parameter integer CLK_DIV = 4 // Дільник системної частоти для 1/4 періоду SCL
)(
    input  wire       clk,        // Системний тактовий сигнал (50 МГц)
    input  wire       rst,        // Асинхронне скидання (активний рівень '1')
    input  wire       start,      // Імпульс запуску транзакції I2C
    input  wire       rw,         // Напрям обміну: '0' - запис (Write), '1' - читання (Read)
    input  wire [6:0] addr,       // 7-бітна адреса цільового веденого пристрою
    input  wire [7:0] data_in,    // Байт даних для передачі веденому при записі
    inout  wire       scl,        // Тактова лінія шини I2C (open-drain)
    inout  wire       sda,        // Двонаправлена лінія даних шини I2C (open-drain)
    output wire       busy,       // Ознака зайнятості модуля ('1' під час обміну)
    output wire       ack_error,  // Прапорець відсутності ACK від веденого (NACK)
    output wire       done,       // Імпульс завершення поточної транзакції
    output wire [7:0] data_out    // Прийнятий від веденого байт даних (при читанні)
);

    // Кодування станів скінченного автомата (FSM)
    localparam [3:0]
        ST_IDLE       = 4'd0, // Стан спокою, шина вільна (SCL = 1, SDA = 1)
        ST_START      = 4'd1, // Формування умови START (спад SDA при SCL = 1)
        ST_ADDR_TX    = 4'd2, // Передача 7 бітів адреси веденого та біта R/W
        ST_ADDR_ACK   = 4'd3, // Прийом підтвердження (ACK) адреси від веденого
        ST_DATA_TX    = 4'd4, // Передача 8 бітів даних веденому при записі
        ST_DATA_ACK   = 4'd5, // Прийом підтвердження (ACK) даних від веденого
        ST_DATA_RX    = 4'd6, // Прийом 8 бітів даних від веденого при читанні
        ST_MASTER_ACK = 4'd7, // Формування ведучим NACK після читання байта
        ST_STOP       = 4'd8; // Формування умови STOP (наростання SDA при SCL = 1)

    // Внутрішні регістри керування FSM та трактом даних
    reg [3:0]  state;     // Поточний стан скінченного автомата
    reg [1:0]  phase;     // Поточна чверть-фаза такту SCL (0..3)
    reg [15:0] clk_cnt;   // Лічильник дільника системної частоти
    reg [2:0]  bit_cnt;   // Лічильник переданих/прийнятих бітів (7..0)

    // Регістри вихідних ліній: '0' - притискання до землі, '1' - стан високого опору (Z)
    reg        scl_reg;
    reg        sda_reg;

    // Зсувні регістри та регістри прапорців
    reg [7:0]  tx_reg;    // Буфер передачі (адреса+R/W або байт даних)
    reg [7:0]  rx_reg;    // Буфер прийому байта від веденого
    reg        ack_err_i; // Внутрішній тригер помилки підтвердження
    reg        done_reg;  // Внутрішній тригер строба завершення
    reg        rw_reg;    // Зафіксований напрям поточної транзакції

    // Реалізація виходів із відкритим стоком (Open-Drain / Tri-State)
    assign scl = (scl_reg == 1'b0) ? 1'b0 : 1'bz;
    assign sda = (sda_reg == 1'b0) ? 1'b0 : 1'bz;

    // Вихідні статусні сигнали та шина прийнятих даних
    assign busy      = (state != ST_IDLE) ? 1'b1 : 1'b0;
    assign ack_error = ack_err_i;
    assign done      = done_reg;
    assign data_out  = rx_reg;

    // Основний синхронний процес скінченного автомата з асинхронним скиданням
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= ST_IDLE;
            phase     <= 2'd0;
            clk_cnt   <= 16'd0;
            bit_cnt   <= 3'd7;
            scl_reg   <= 1'b1;
            sda_reg   <= 1'b1;
            tx_reg    <= 8'd0;
            rx_reg    <= 8'd0;
            ack_err_i <= 1'b0;
            done_reg  <= 1'b0;
            rw_reg    <= 1'b0;
        end else begin
            done_reg <= 1'b0; // Строб завершення активний протягом 1 такту clk

            case (state)
                // ------------------------------------------------------------
                // Стан спокою: очікування імпульсу start
                // ------------------------------------------------------------
                ST_IDLE: begin
                    scl_reg <= 1'b1;
                    sda_reg <= 1'b1;
                    phase   <= 2'd0;
                    clk_cnt <= 16'd0;
                    bit_cnt <= 3'd7;
                    if (start) begin
                        rw_reg    <= rw;
                        tx_reg    <= {addr, rw}; // Формування адресного кадру (7 біт + R/W)
                        ack_err_i <= 1'b0;
                        state     <= ST_START;
                    end
                end

                // ------------------------------------------------------------
                // Формування умови START: спад SDA при високому рівні SCL
                // ------------------------------------------------------------
                ST_START: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b1;
                                sda_reg <= 1'b1;
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1;
                                sda_reg <= 1'b0; // Спад SDA при SCL = 1
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                sda_reg <= 1'b0;
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0; // Підготовка до першого біта
                                sda_reg <= 1'b0;
                                phase   <= 2'd0;
                                bit_cnt <= 3'd7;
                                state   <= ST_ADDR_TX;
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Передача 7-бітної адреси та біта напряму R/W (MSB першим)
                // ------------------------------------------------------------
                ST_ADDR_TX: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= tx_reg[bit_cnt]; // Встановлення біта при SCL = 0
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1; // Наростаючий фронт SCL
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1; // Утримання SCL = 1 для вибірки веденим
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0; // Спадний фронт SCL
                                phase   <= 2'd0;
                                if (bit_cnt > 3'd0) begin
                                    bit_cnt <= bit_cnt - 3'd1;
                                end else begin
                                    state <= ST_ADDR_ACK;
                                end
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Прийом біта підтвердження адреси (ACK/NACK) від веденого
                // ------------------------------------------------------------
                ST_ADDR_ACK: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b1; // Відпускання SDA у Z для відповіді Slave
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                if (sda != 1'b0) begin
                                    ack_err_i <= 1'b1; // Зафіксовано NACK
                                end
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0;
                                phase   <= 2'd0;
                                bit_cnt <= 3'd7;
                                if (ack_err_i || (sda != 1'b0)) begin
                                    sda_reg <= 1'b0;
                                    state   <= ST_STOP; // Аварійне завершення при NACK
                                end else if (rw_reg == 1'b0) begin
                                    tx_reg <= data_in;
                                    state  <= ST_DATA_TX;
                                end else begin
                                    state  <= ST_DATA_RX;
                                end
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Передача 8 бітів даних веденому (операція Write)
                // ------------------------------------------------------------
                ST_DATA_TX: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= tx_reg[bit_cnt];
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0;
                                phase   <= 2'd0;
                                if (bit_cnt > 3'd0) begin
                                    bit_cnt <= bit_cnt - 3'd1;
                                end else begin
                                    state <= ST_DATA_ACK;
                                end
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Прийом біта підтвердження даних (ACK/NACK) від веденого
                // ------------------------------------------------------------
                ST_DATA_ACK: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b1; // Відпускання SDA для ACK
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                if (sda != 1'b0) begin
                                    ack_err_i <= 1'b1;
                                end
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b0; // Підготовка низького рівня для STOP
                                phase   <= 2'd0;
                                state   <= ST_STOP;
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Прийом 8 бітів даних від веденого (операція Read)
                // ------------------------------------------------------------
                ST_DATA_RX: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b1; // Відпускання SDA для передачі веденим
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                rx_reg[bit_cnt] <= (sda != 1'b0) ? 1'b1 : 1'b0; // Стробування
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0;
                                phase   <= 2'd0;
                                if (bit_cnt > 3'd0) begin
                                    bit_cnt <= bit_cnt - 3'd1;
                                end else begin
                                    state <= ST_MASTER_ACK;
                                end
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Формування ведучим NACK ('1') після прийому останнього байта
                // ------------------------------------------------------------
                ST_MASTER_ACK: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b1; // Видача NACK для завершення читання
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b0; // Підготовка SDA = 0 для умови STOP
                                phase   <= 2'd0;
                                state   <= ST_STOP;
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                // ------------------------------------------------------------
                // Формування умови STOP: наростання SDA при високому рівні SCL
                // ------------------------------------------------------------
                ST_STOP: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        case (phase)
                            2'd0: begin
                                scl_reg <= 1'b0;
                                sda_reg <= 1'b0;
                                phase   <= 2'd1;
                            end
                            2'd1: begin
                                scl_reg <= 1'b1; // SCL наростає в '1' при SDA = 0
                                sda_reg <= 1'b0;
                                phase   <= 2'd2;
                            end
                            2'd2: begin
                                scl_reg <= 1'b1;
                                sda_reg <= 1'b1; // Наростання SDA при SCL = 1 (STOP)
                                phase   <= 2'd3;
                            end
                            2'd3: begin
                                scl_reg  <= 1'b1;
                                sda_reg  <= 1'b1;
                                done_reg <= 1'b1; // Імпульс завершення операції
                                phase    <= 2'd0;
                                state    <= ST_IDLE;
                            end
                        endcase
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
