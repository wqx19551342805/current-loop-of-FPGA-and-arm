`timescale 1ns / 1ps
module encoder #(
    parameter integer CLK_FREQ    = 50_000_000,
    parameter integer BAUD_RATE   = 2500000,    // 编码器标准波特率2.5Mbps
    parameter integer SEND_MS     = 100,          // 周期性读取间隔（默认1ms）
    parameter integer POS_WIDTH   = 32          // 单圈绝对位置位数
)(
    input  wire                 clk,            // 50MHz系统时钟
    input  wire                 rst_n,          // 低电平复位
    input  wire                 rs485_r,        // RS485芯片接收输出 R/RO
    output reg                  rs485_d,        // RS485芯片发送输入 D/DI
    output reg                  rs485_de,       // RS485芯片方向控制 DE
    output reg                  tx_led,         // 发送状态指示
    output reg                  rx_led,         // 接收活动指示
    output reg [32-1:0]  abs_pos,        // 17位单圈绝对位置输出
    output reg                  pos_valid,      // 位置数据有效脉冲（1时钟周期）
    output reg [7:0]            status_flag,    // 编码器状态字SF
    output reg                  crc_err_flag,    // CRC校验错误标志
	 
	 
	 output wire                 beep_disable
);
assign beep_disable = 1'b1;
// =====================================================
// 波特率与时序参数计算
// =====================================================
localparam integer CLKS_PER_BIT   = CLK_FREQ / BAUD_RATE;      // 每比特时钟数（50MHz下为20）
localparam integer HALF_BIT_CNT   = CLKS_PER_BIT / 2;          // 半比特计数（采样中点对齐）
localparam integer PERIOD_CLKS    = (CLK_FREQ / 1000_000) * SEND_MS; // 读取周期时钟数
localparam integer PRE_DELAY_CNT  = CLK_FREQ / 500_000;                        // DE提前开启延时（2us）
localparam integer POST_DELAY_CNT = CLK_FREQ / 12_500_000;                        // 发送结束保持DE延时
localparam integer RX_TIMEOUT_CNT = CLK_FREQ / 20_000;                   // 接收超时（100us）

// =====================================================
// 状态定义
// =====================================================
localparam ST_WAIT     = 3'd0;  // 等待下一次读取周期
localparam ST_PRE      = 3'd1;  // 提前开启485驱动器
localparam ST_TX       = 3'd2;  // 发送命令字节
localparam ST_POST     = 3'd3;  // 发送结束保持驱动器
localparam ST_RX_WAIT  = 3'd4;  // 等待编码器返回数据
localparam ST_RX_PROC  = 3'd5;  // 解析接收数据

localparam RX_IDLE     = 1'b0;  // 接收空闲
localparam RX_RECV     = 1'b1;  // 接收中

// =====================================================
// 内部寄存器定义
// =====================================================
reg [2:0]  main_state;       // 主状态机
reg [31:0] period_cnt;       // 读取周期计数器
reg [15:0] delay_cnt;        // 通用延时计数器
reg [15:0] rx_timeout_cnt;   // 接收超时计数器

// 发送相关寄存器
reg [3:0]  tx_bit_cnt;       // 发送比特计数
reg [15:0] tx_baud_cnt;      // 发送波特计数器
reg [9:0]  tx_shift_reg;     // 发送移位寄存器：{停止位, 8位数据, 起始位}

// 接收同步与边沿检测
reg rx_sync1, rx_sync2, rx_sync3;
wire rx_fall = rx_sync3 & ~rx_sync2;  // 接收下降沿（起始位检测）

// UART字节接收
reg        rx_state;         // 接收字节状态机
reg [15:0] rx_baud_cnt;      // 接收波特计数器
reg [3:0]  rx_bit_cnt;       // 接收比特计数
reg [7:0]  rx_shift_reg;     // 接收移位寄存器
reg        rx_byte_valid;    // 单字节接收有效脉冲
reg [7:0]  rx_byte_data;     // 接收到的字节数据

// 帧接收缓冲与CRC
reg [2:0]  rx_byte_cnt;      // 已接收字节计数
reg [7:0]  rx_buf[5:0];      // 接收缓冲区：0:命令回显, 1:状态SF, 2:ABS0, 3:ABS1, 4:ABS2, 5:CRC
// CRC计算：对应手册G(X)=X^8+1，即所有字节异或和
wire [7:0] crc_calc = rx_buf[0] ^ rx_buf[1] ^ rx_buf[2] ^ rx_buf[3] ^ rx_buf[4];

// =====================================================
// 接收信号三同步 + 下降沿检测（消除亚稳态）
// =====================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_sync1 <= 1'b1;
        rx_sync2 <= 1'b1;
        rx_sync3 <= 1'b1;
    end else begin
        rx_sync1 <= rs485_r;
        rx_sync2 <= rx_sync1;
        rx_sync3 <= rx_sync2;
    end
end

// 接收活动LED：检测到下降沿翻转
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_led <= 1'b0;
    end else if (rx_fall) begin
        rx_led <= ~rx_led;
    end
end

// =====================================================
// UART字节接收逻辑（8N1，低位先发，中点采样）
// =====================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state      <= RX_IDLE;
        rx_baud_cnt   <= 16'd0;
        rx_bit_cnt    <= 4'd0;
        rx_shift_reg  <= 8'd0;
        rx_byte_valid <= 1'b0;
        rx_byte_data  <= 8'd0;
    end else begin
        rx_byte_valid <= 1'b0;
        case (rx_state)
            RX_IDLE: begin
                if (rx_fall) begin
                    // 检测到下降沿，进入起始位检测
                    rx_state    <= RX_RECV;
                    rx_baud_cnt <= 16'd0;
                    rx_bit_cnt  <= 4'd0;
                end
            end
            RX_RECV: begin
                rx_baud_cnt <= rx_baud_cnt + 1'b1;
                // 到达比特中点时采样
                if (rx_baud_cnt == HALF_BIT_CNT - 1) begin
                    if (rx_bit_cnt == 4'd0) begin
                        // 校验起始位，低电平有效
                        if (rx_sync3 != 1'b0) begin
                            rx_state <= RX_IDLE; // 起始位错误，退出
                        end
                    end else if (rx_bit_cnt <= 4'd8) begin
                        // 采样数据位，移入移位寄存器（高位进、右移）
                        rx_shift_reg <= {rx_sync3, rx_shift_reg[7:1]};
                    end else begin
                        // 采样停止位，高电平有效
                        if (rx_sync3 == 1'b1) begin
                            rx_byte_valid <= 1'b1;
                            rx_byte_data  <= rx_shift_reg;
                        end
                        rx_state <= RX_IDLE;
                    end
                end
                // 满1比特周期，比特计数+1
                if (rx_baud_cnt >= CLKS_PER_BIT - 1) begin
                    rx_baud_cnt <= 16'd0;
                    rx_bit_cnt  <= rx_bit_cnt + 1'b1;
                end
            end
        endcase
    end
end

// =====================================================
// 主状态机：周期发送命令 + 接收解析
// =====================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        main_state    <= ST_WAIT;
        rs485_de      <= 1'b0;
        rs485_d       <= 1'b1;
        tx_led        <= 1'b0;
        period_cnt    <= 32'd0;
        delay_cnt     <= 16'd0;
        rx_timeout_cnt<= 16'd0;
        tx_bit_cnt    <= 4'd0;
        tx_baud_cnt   <= 16'd0;
        tx_shift_reg  <= 10'h3FF;
        rx_byte_cnt   <= 3'd0;
        abs_pos       <= {POS_WIDTH{1'b0}};
        pos_valid     <= 1'b0;
        status_flag   <= 8'd0;
        crc_err_flag  <= 1'b0;
        rx_buf[0] <= 8'd0;
        rx_buf[1] <= 8'd0;
        rx_buf[2] <= 8'd0;
        rx_buf[3] <= 8'd0;
        rx_buf[4] <= 8'd0;
        rx_buf[5] <= 8'd0;
    end else begin
        pos_valid <= 1'b0;
        case (main_state)
            // 等待下一次读取周期
            ST_WAIT: begin
                rs485_de      <= 1'b0;
                rs485_d       <= 1'b1;
                tx_led        <= 1'b0;
                rx_byte_cnt   <= 3'd0;
                rx_timeout_cnt<= 16'd0;
                if (period_cnt >= PERIOD_CLKS - 1) begin
                    period_cnt <= 32'd0;
                    delay_cnt  <= 16'd0;
                    main_state <= ST_PRE;
                end else begin
                    period_cnt <= period_cnt + 1'b1;
                end
            end
            // 提前开启485驱动器，等待电平稳定
            ST_PRE: begin
                rs485_de <= 1'b1;
                rs485_d  <= 1'b1;
                tx_led   <= 1'b1;
                if (delay_cnt >= PRE_DELAY_CNT - 1) begin
                    delay_cnt    <= 16'd0;
                    // 加载读单圈命令 0x02
                    tx_shift_reg <= {1'b1, 8'h02, 1'b0}; // 停止位 + 数据 + 起始位
                    tx_bit_cnt   <= 4'd0;
                    tx_baud_cnt  <= 16'd0;
                    main_state   <= ST_TX;
                end else begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end
            // 发送命令字节（8N1格式）
            ST_TX: begin
                rs485_de <= 1'b1;
                rs485_d  <= tx_shift_reg[0]; // 输出最低位（起始位先发）
                tx_baud_cnt <= tx_baud_cnt + 1'b1;
                if (tx_baud_cnt >= CLKS_PER_BIT - 1) begin
                    tx_baud_cnt <= 16'd0;
                    tx_bit_cnt  <= tx_bit_cnt + 1'b1;
                    tx_shift_reg <= {1'b1, tx_shift_reg[9:1]}; // 右移，高位补1
                    if (tx_bit_cnt == 4'd9) begin
                        // 10位发送完成（起始+8数据+停止）
                        delay_cnt  <= 16'd0;
                        main_state <= ST_POST;
                    end
                end
            end
            // 发送结束，保持驱动器确保停止位完整
            ST_POST: begin
                rs485_de <= 1'b1;
                rs485_d  <= 1'b1;
                if (delay_cnt >= POST_DELAY_CNT - 1) begin
                    delay_cnt     <= 16'd0;
                    rs485_de      <= 1'b0; // 切换为接收模式
                    tx_led        <= 1'b0;
                    rx_byte_cnt   <= 3'd0;
                    rx_timeout_cnt<= 16'd0;
                    main_state    <= ST_RX_WAIT;
                end else begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end
            // 等待编码器返回数据
            ST_RX_WAIT: begin
                rs485_de <= 1'b0;
                rx_timeout_cnt <= rx_timeout_cnt + 1'b1;
                if (rx_byte_valid) begin
                    rx_buf[rx_byte_cnt] <= rx_byte_data;
                    rx_byte_cnt <= rx_byte_cnt + 1'b1;
                    rx_timeout_cnt <= 16'd0; // 收到字节，重置超时
                    // 收满6字节进入解析
                    if (rx_byte_cnt == 3'd5) begin
                        main_state <= ST_RX_PROC;
                    end
                end
                // 接收超时，退出等待
                else if (rx_timeout_cnt >= RX_TIMEOUT_CNT) begin
                    main_state <= ST_WAIT;
                end
            end
            // 解析数据：CRC校验 + 提取绝对位置
            ST_RX_PROC: begin
                status_flag <= rx_buf[1]; // 输出状态字SF
                // CRC校验：计算值与接收CRC对比
                if (crc_calc == rx_buf[5]) begin
                    // 17位位置拼接：ABS2[0]是最高位，ABS1是中8位，ABS0是低8位
                    abs_pos    <= {rx_buf[4][0], rx_buf[3], rx_buf[2]};
                    pos_valid  <= 1'b1;
                    crc_err_flag <= 1'b0;
                end else begin
                    crc_err_flag <= 1'b1;
                end
                main_state <= ST_WAIT;
            end
            default: begin
                main_state <= ST_WAIT;
                rs485_de   <= 1'b0;
            end
        endcase
    end
end

endmodule
