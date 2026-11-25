`timescale 1ns / 1ps

// ============================================================================
// Модуль: i2c_master_tb
// Опис:   Тестове оточення (Testbench) для функціональної верифікації модуля
//         i2c_master на мові Verilog. Включає підтягуючі резистори шини,
//         поведінкову модель I2C Slave (адреса 0x52) та сценарій запису/читання.
// ============================================================================
module i2c_master_tb;

    localparam integer CLK_PERIOD = 20; // Період тактового сигналу 20 нс (50 МГц)

    reg        clk;
    reg        rst;
    reg        start;
    reg        rw;
    reg  [6:0] addr;
    reg  [7:0] data_in;
    wire       scl;
    wire       sda;
    wire       busy;
    wire       ack_error;
    wire       done;
    wire [7:0] data_out;

    // Сигнал керування лінією SDA з боку поведінкової моделі веденого пристрою
    reg        slave_sda;
    integer    i;
    reg  [7:0] slave_rx_byte;
    reg  [7:0] slave_tx_data;

    // Моделювання підтягуючих резисторів шини I2C (Pull-up до логічної '1')
    pullup(scl);
    pullup(sda);
    assign sda = slave_sda;

    // Підключення тестованого модуля (Unit Under Test)
    i2c_master #(
        .CLK_DIV(4)
    ) uut (
        .clk       (clk),
        .rst       (rst),
        .start     (start),
        .rw        (rw),
        .addr      (addr),
        .data_in   (data_in),
        .scl       (scl),
        .sda       (sda),
        .busy      (busy),
        .ack_error (ack_error),
        .done      (done),
        .data_out  (data_out)
    );

    // Генерація системного тактового сигналу 50 МГц
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Поведінкова модель веденого пристрою (I2C Slave, адреса 0x52 = 7'b1010010)
    initial begin
        slave_sda     = 1'bz;
        slave_tx_data = 8'hD7; // Байт, який ведений повертає при операції читання

        forever begin
            slave_sda = 1'bz;
            // Очікування умови START (спадний фронт SDA при високому рівні SCL)
            @(negedge sda);
            if (scl === 1'b1) begin
                // Прийом 8 бітів адресного кадру (7 бітів адреси + 1 біт R/W)
                for (i = 7; i >= 0; i = i - 1) begin
                    @(posedge scl);
                    slave_rx_byte[i] = sda;
                end

                // Перевірка збігу адреси веденого (0x52)
                if (slave_rx_byte[7:1] == 7'h52) begin
                    // Формування підтвердження ACK на адресний кадр (9-й такт)
                    @(negedge scl);
                    slave_sda = 1'b0;
                    @(negedge scl);
                    slave_sda = 1'bz;

                    if (slave_rx_byte[0] == 1'b0) begin
                        // Транзакція запису: прийом 8 бітів даних від Master
                        for (i = 7; i >= 0; i = i - 1) begin
                            @(posedge scl);
                            slave_rx_byte[i] = sda;
                        end
                        // Формування ACK на прийнятий байт даних
                        @(negedge scl);
                        slave_sda = 1'b0;
                        @(negedge scl);
                        slave_sda = 1'bz;
                    end else begin
                        // Транзакція читання: послідовна видача 8 бітів даних ведучому
                        for (i = 7; i >= 0; i = i - 1) begin
                            slave_sda = slave_tx_data[i];
                            @(negedge scl);
                        end
                        slave_sda = 1'bz; // Звільнення SDA для прийому NACK від Master
                        @(negedge scl);
                    end
                end
            end
        end
    end

    // Процес формування тестових впливів (Stimulus)
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, i2c_master_tb);

        // 1. Початкове асинхронне скидання
        rst     = 1'b1;
        start   = 1'b0;
        rw      = 1'b0;
        addr    = 7'd0;
        data_in = 8'd0;
        #100;
        rst = 1'b0;
        #40;

        // 2. Тест 1: Операція запису (Write) байта 0x5A за адресою 0x52
        // Адресний кадр: {7'h52, 1'b0} = 8'hA4
        addr    = 7'h52;
        rw      = 1'b0;
        data_in = 8'h5A;
        start   = 1'b1;
        #CLK_PERIOD;
        start   = 1'b0;

        // Очікування завершення транзакції запису
        @(posedge done);
        #200;

        // 3. Тест 2: Операція читання (Read) байта з адреси 0x52
        // Адресний кадр: {7'h52, 1'b1} = 8'hA5; очікувані дані: 0xD7
        addr  = 7'h52;
        rw    = 1'b1;
        start = 1'b1;
        #CLK_PERIOD;
        start = 1'b0;

        // Очікування завершення транзакції читання
        @(posedge done);
        #200;

        // Перевірка коректності прийнятих даних
        if (data_out !== 8'hD7 || ack_error !== 1'b0) begin
            $display("ERROR: Expected data_out = 0xD7, got 0x%02X (ack_error=%b)", data_out, ack_error);
        end else begin
            $display("Simulation completed successfully! data_out = 0x%02X", data_out);
        end

        $finish;
    end

endmodule
