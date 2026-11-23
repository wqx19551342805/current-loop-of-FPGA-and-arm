`timescale 1ns / 1ps

module foc_closedloop_top (
    input  wire               CLK_50M,     // 25MHz 系统主频

    // ================= ADC AD7606C 硬件接口 ================= //
    inout  wire [15:0]        DB0_15,
    input  wire               BUSY,
    input  wire               FRSTDATA,
    output wire               SER_SEL,
    output wire               RESET,
    output wire               CS,
    output wire               RD_SCLK,
    output wire               WR,
    output wire               CONVST,
    output wire [ 2:0]        OSC2_0,
    output wire               STBY,

    // ================= SVPWM 功率输出 ================= //
    output wire               pwm_uh_out, pwm_ul_out,
    output wire               pwm_vh_out, pwm_vl_out,
    output wire               pwm_wh_out, pwm_wl_out,

    // ================= LED 状态指示 ================= //
    output wire               FPGA_LEDR,   // 上电常亮
    output wire               FPGA_LEDG,   // 虚拟电角度慢闪指示

    // ================= 17位编码器 RS485 接口 ================= //
    output wire               rs485_d,     // 连接到 RS485 芯片的 DI (发送数据) 引脚
    output wire               rs485_de,    // 连接到你刚割断并飞线出来的 DE (发送使能) 引脚
    input  wire               rs485_r      // RS485 芯片 RO → FPGA（编码器回传）
);

// ============================================================
// 0. PLL：50MHz → 25MHz
// ============================================================
(* preserve, keep, noprune *) wire CLK_25M;

my_pll u_pll (
    .areset  (1'b0),       // 高电平复位，这里直接接 0（永久不复位）
    .inclk0  (CLK_50M),    // 输入 50MHz
    .c0      (CLK_25M)     // 输出 25MHz
//  .locked  (pll_locked)  // 可选
);

// ============================================================
// 1. 系统延时复位
// ============================================================
reg [5:0] rst_cnt = 6'd0;
reg       rst_n   = 1'b0;

always @(posedge CLK_25M) begin
    if (rst_cnt == 6'd50) rst_n <= 1'b1;
    else                  rst_cnt <= rst_cnt + 1'b1;
end

assign FPGA_LEDR = rst_n;   // 上电常亮

// ============================================================
// 2. ADC 数据采集（10kHz 采样率）
// ============================================================
wire [15:0] data_ch1, data_ch2, data_ch3, data_ch4;
wire [15:0] data_ch5, data_ch6, data_ch7, data_ch8;
wire        adc_valid;     // ADC 转换并读取完成的全局脉冲
wire        adc_sync;      // 用于同步 SVPWM 和 ADC 的脉冲线

AD7606C_MODE1 #(
    .CH1_RANGE  ( 4'b0001 ),   // 通道 1 为 ±5V（Ia）
    .CH2_RANGE  ( 4'b0001 ),    // 通道 2 为 ±5V（Ib）
	 .CH3_RANGE  ( 4'b0001 ),    // 通道 3 为 ±5V（IC）
	 .CH4_RANGE  ( 4'b0001 )    // 通道 3 为 ±5V（IC）
) u_adc (
    .sys_clk    (CLK_25M),
    .sys_rst    (rst_n),
    .DB0_15     (DB0_15),
    .ext_trig   (adc_sync),
    .busy       (BUSY),
    .Frstdata   (FRSTDATA),
    .reset      (RESET),
    .cs_n       (CS),
    .rd_n       (RD_SCLK),
    .wr_n       (WR),
    .convst     (CONVST),
    .ser_sel    (SER_SEL),
    .OSC2_0     (OSC2_0),
    .data_ch1   (data_ch1),   // Ia绿色
    .data_ch2   (data_ch2),   // Ib黄色
    .data_ch3   (data_ch3),//Ic
    .data_ch4   (data_ch4),
    .data_ch5   (data_ch5),
    .data_ch6   (data_ch6),
    .data_ch7   (data_ch7),
    .data_ch8   (data_ch8),
    .sample_out (adc_valid)
);

assign STBY = 1'b1;


// ============================================================
// 3. ADC 零偏校准（改进版：等稳定 → 丢前16点 → 采1024点）
// ============================================================
localparam integer CALIB_WAIT    = 2_500_000;   // 上电后等 100ms @25MHz
localparam integer CALIB_SKIP    = 16;          // 丢弃前 16 个样本（前端尚未稳定）
localparam integer CALIB_SAMPLES = 1024;        // 1024 点，噪声降 2 倍
localparam integer CALIB_TOTAL   = CALIB_SKIP + CALIB_SAMPLES;

reg [23:0]        wait_cnt;
reg               calib_start;
reg [10:0]        calib_cnt;
reg signed [31:0] sum_ia, sum_ic;
reg               calib_done;
reg               calib_bad;       // 1 = 标定结果不合理（可点红灯报警）
reg signed [15:0] offset_ia_reg;   // ← 补回：A 相零点
reg signed [15:0] offset_ic_reg;   // ← 补回：C 相零点


// 标定完成后检查：偏移应在合理范围（±5V 量程 → |offset| < 20000）
wire signed [31:0] off_ia_tmp = sum_ia >>> 10;
wire signed [31:0] off_ic_tmp = sum_ic >>> 10;

always @(posedge CLK_25M or negedge rst_n) begin
    if (!rst_n) begin
        wait_cnt      <= 24'd0;
        calib_start   <= 1'b0;
        calib_cnt     <= 11'd0;
        sum_ia        <= 32'sd0;
        sum_ic        <= 32'sd0;
        calib_done    <= 1'b0;
        calib_bad     <= 1'b0;
        offset_ia_reg <= 16'sd0;
        offset_ic_reg <= 16'sd0;
    end
    else if (!calib_start) begin
        if (wait_cnt == CALIB_WAIT - 1) calib_start <= 1'b1;
        else                            wait_cnt <= wait_cnt + 1'b1;
    end
    else if (!calib_done && adc_valid) begin
        if (calib_cnt < CALIB_SKIP) begin
            calib_cnt <= calib_cnt + 1'b1;                 // 前 16 点直接丢
        end
        else if (calib_cnt < CALIB_TOTAL) begin
            sum_ia    <= sum_ia + $signed(data_ch1);
            sum_ic    <= sum_ic + $signed(data_ch3);
            calib_cnt <= calib_cnt + 1'b1;
        end
        else begin
            offset_ia_reg <= sum_ia >>> 10;                // /1024
            offset_ic_reg <= sum_ic >>> 10;
            calib_bad     <= (off_ia_tmp > 32'sd20000)  || (off_ia_tmp < -32'sd20000) ||
                             (off_ic_tmp > 32'sd20000)  || (off_ic_tmp < -32'sd20000);
            calib_done    <= 1'b1;
        end
    end
end



















//// ============================================================
//// 3. ADC 零偏校准（256 次采样）
//// ============================================================
//localparam CALIB_TARGET = 8'd255;
//
//reg [7:0]         calib_cnt;
//reg signed [31:0] sum_ia;
//reg signed [31:0] sum_ib;
//reg signed [31:0] sum_ic;
//reg signed [15:0] offset_ia_reg;
//reg signed [15:0] offset_ib_reg;
//reg signed [15:0] offset_ic_reg;
//reg               calib_done;
//
//always @(posedge CLK_25M or negedge rst_n) begin
//    if (!rst_n) begin
//        calib_cnt     <= 8'd0;
//        sum_ia        <= 32'sd0;
//        sum_ib        <= 32'sd0;
//        sum_ic        <= 32'sd0;
//        offset_ia_reg <= 16'sd0;
//        offset_ib_reg <= 16'sd0;
//        offset_ic_reg <= 16'sd0;
//        calib_done    <= 1'b0;
//    end
//    else if (!calib_done) begin
//        if (adc_valid) begin
//            if (calib_cnt < CALIB_TARGET) begin
//                sum_ia    <= sum_ia + $signed(data_ch1);
//                sum_ib    <= sum_ib + $signed(data_ch2);
//                sum_ic    <= sum_ic + $signed(data_ch3);
//                calib_cnt <= calib_cnt + 1'b1;
//            end
//            else begin
//                // 第 256 次：加上最后一次数据，右移 8 位等效 /256
//                offset_ia_reg <= (sum_ia + $signed(data_ch1)) >>> 8;
//                offset_ib_reg <= (sum_ib + $signed(data_ch2)) >>> 8;
//                offset_ic_reg <= (sum_ic + $signed(data_ch3)) >>> 8;
//                calib_done    <= 1'b1;
//            end
//        end
//    end
//end

// 去掉零偏的相电流
//(* keep *) wire signed [15:0] i_a = ($signed(data_ch1) - offset_ia_reg);
//(* keep *) wire signed [15:0] i_b = ($signed(data_ch2) - offset_ib_reg);
//(* keep *) wire signed [15:0] i_c = -i_a - i_b;   // 三相对称自动重构
(* keep *) wire signed [15:0] i_a = ($signed(data_ch1) - offset_ia_reg);
(* keep *) wire signed [15:0] i_b = -i_a - i_c;
(* keep *) wire signed [15:0] i_c = ($signed(data_ch3) - offset_ic_reg);
// ============================================================
// 4. 预定位（Alignment）：ADC 校准完成后，对齐 0.5s 再起转
// ===== 两段预定位：先猛吸，再稳住 =====
localparam [24:0] ALIGN1_CYCLES = 25'd12_500_000;    // 第一段 0.3s（大电流猛吸）
localparam [24:0] ALIGN2_CYCLES = 25'd25_000_000;  // 总时间 1.0s（到这里结束）

localparam signed [15:0] ID_ALIGN_BIG   = 16'sd30000;  // 第一段：大电流拉过去
localparam signed [15:0] ID_ALIGN_SMALL = 16'sd10000;  // 第二段：小电流稳住
localparam signed [15:0] IQ_ALIGN       = 16'sd0;

localparam signed [15:0] ID_RUN   = 16'sd0;
localparam signed [15:0] IQ_RUN   = 16'sd0;

reg [24:0] align_cnt     = 25'd0;
reg        align_done    = 1'b0;
reg        align_phase2 = 1'b0;   // 1 = 进入第二段（小电流）

always @(posedge CLK_25M or negedge rst_n) begin
    if (!rst_n) begin
        align_cnt     <= 25'd0;
        align_done    <= 1'b0;
        align_phase2 <= 1'b0;
    end
    else if (!align_done && calib_done) begin
        if (align_cnt == ALIGN2_CYCLES - 1)
            align_done <= 1'b1;                      // 1.0s 到 → 结束预定位
        else begin
            align_cnt <= align_cnt + 1'b1;
            if (align_cnt == ALIGN1_CYCLES - 1)
                align_phase2 <= 1'b1;                // 0.3s 到 → 切小电流
        end
    end
end


reg align_done_d1;
always @(posedge CLK_25M or negedge rst_n) begin
    if (!rst_n) align_done_d1 <= 1'b0;
    else        align_done_d1 <= align_done;
end

wire align_done_pulse = align_done & ~align_done_d1;

reg [31:0] enc_zero_offset;                 // 对齐完成时编码器零位锁存
always @(posedge CLK_25M or negedge rst_n) begin
    if (!rst_n)            enc_zero_offset <= 32'd0;
    else if (align_done_pulse) enc_zero_offset <= enc_abs_pos; // 转子在电角度0处的机械位置
end

// θ_elec = (机械角 − 零位) × 5，5对极；取低20位即 mod 2^20（自动环绕）
wire [31:0] enc_theta_raw = (enc_abs_pos - enc_zero_offset) * 5; // 32位中间结果，防溢出
wire [19:0] theta_calc    = enc_theta_raw[19:0];                 // 取低20位 = mod 2^20
 wire [19:0] theta = align_done ? theta_calc : 20'd0; // 对齐阶段锁0，运行用编码器

assign FPGA_LEDG = enc_ok;               // 绿灯：亮=编码器健康；灭=报警/未就绪


// ============================================================
// 6. Clarke 变换（并行触发）
// ============================================================
wire signed [15:0] i_alpha, i_beta;
wire               clarke_done;

clarke_transform u_clarke (
    .clk        (CLK_25M),
    .rst_n      (rst_n),
    .en         (adc_valid),
    .ia         (i_a),
    .ib         (i_b),
    .i_alpha    (i_alpha),
    .i_beta     (i_beta),
    .out_valid  (clarke_done)
);

// ============================================================
// 7. CORDIC 解算（并行触发）
// ============================================================
wire signed [17:0] sin_th, cos_th;
wire               cordic_done;

cordic_sincos_full u_cordic (
    .clk        (CLK_25M),
    .rstn       (rst_n),
    .i_en       (adc_valid),
    .i_theta    (theta[16:0]),
    .o_en       (cordic_done),
    .o_sin      (sin_th),
    .o_cos      (cos_th)
);

// ============================================================
// 8. Park 变换
// ============================================================
(* keep *) wire signed [15:0] i_d, i_q;
wire                          park_done;

park_transform u_park (
    .clk        (CLK_25M),
    .rst_n      (rst_n),
    .en         (cordic_done),   // CORDIC 最慢，用它的完成信号抓 Clarke 结果
    .i_alpha    (i_alpha),
    .i_beta     (i_beta),
    .sin_th     (sin_th),
    .cos_th     (cos_th),
    .id_out     (i_d),
    .iq_out     (i_q),
    .out_valid  (park_done)
);

// ============================================================
// 9. PI 闭环控制器（D 轴 / Q 轴并行）
// ============================================================
(* keep *) wire signed [15:0] v_d, v_q;
wire                          pi_d_done, pi_q_done;

// 🚨 电流给定随对齐状态切换
//wire signed [15:0] id_ref = align_done ? ID_RUN : ID_ALIGN;
//// 预定位电流随阶段切换：大电流→小电流→运行值
(* keep *)wire signed [15:0] id_ref = align_done
                          ? ID_RUN
                          : (align_phase2 ? ID_ALIGN_SMALL : ID_ALIGN_BIG);

(* keep *)wire signed [15:0] iq_ref = align_done ? IQ_RUN : IQ_ALIGN;

hw_pi_controller u_pi_d (
    .clk        (CLK_25M),
    .rst_n      (rst_n),
    .en         (park_done),
	 .clear_i    (align_done_pulse), // 积分器清零脉冲
    .target_val (id_ref),
    .actual_val (i_d),
    .kp         (16'sd1024),   // Q11 格式，1024 ≈ 0.5
    .ki         (16'sd50),     // Q11 格式，积分系数要小
    .pi_out     (v_d),
    .out_valid  (pi_d_done)
);

hw_pi_controller u_pi_q (
    .clk        (CLK_25M),
    .rst_n      (rst_n),
    .en         (park_done),
	 .clear_i    (align_done_pulse), // 积分器清零脉冲
    .target_val (iq_ref),
    .actual_val (i_q),
    .kp         (16'sd1524),
    .ki         (16'sd50),
    .pi_out     (v_q),
    .out_valid  (pi_q_done)
);

// ============================================================
// 10. 逆 Park 变换
// ============================================================
wire signed [15:0] v_alpha, v_beta;
wire               inv_park_done;

inv_park_transform u_inv_park (
    .clk        (CLK_25M),
    .rst_n      (rst_n),
    .en         (pi_q_done),   // PI_D / PI_Q 节拍一致，任取其一
    .vd         (v_d),
    .vq         (v_q),
    .sin_th     (sin_th),
    .cos_th     (cos_th),
    .v_alpha    (v_alpha),
    .v_beta     (v_beta),
    .out_valid  (inv_park_done)
);

// ============================================================
// 11. SVPWM 波形发生器
// ============================================================
(* keep *)wire [15:0] dbg_cmp_u, dbg_cmp_v, dbg_cmp_w;

svpwm_generator #(
    .SYS_CLK_FREQ  (25_000_000),
    .PWM_FREQ      (16_000),      // 10kHz 载波
    .DEAD_TIME_NS  (1_000),       // 1us 死区
    .MAX_AMP       (16'd32767),
    .MAX_AMP_SHIFT (15)
) u_svpwm (
    .clk        (CLK_25M),
    .rst_n      (rst_n),
    .en         (svpwm_en),       // 校准完成 + 运行阶段编码器健康才允许发波
    .v_alpha    (v_alpha),
    .v_beta     (v_beta),
    .adc_trig   (adc_sync),
    .pwm_uh     (pwm_uh_out),
    .pwm_ul     (pwm_ul_out),
    .pwm_vh     (pwm_vh_out),
    .pwm_vl     (pwm_vl_out),
    .pwm_wh     (pwm_wh_out),
    .pwm_wl     (pwm_wl_out),
    .dbg_cmp_u  (dbg_cmp_u),
    .dbg_cmp_v  (dbg_cmp_v),
    .dbg_cmp_w  (dbg_cmp_w)
);

wire [31:0] enc_abs_pos;
wire        enc_pos_valid, enc_crc_err;
wire        enc_tx, enc_de;

encoder #(
    .CLK_FREQ  (25_000_000),   // ⚠ 必须与实际时钟匹配！默认 50M 会算错波特率
    .BAUD_RATE (2_500_000),
    .SEND_MS   (70),
    .POS_WIDTH (32)
) u_encoder (
    .clk          (CLK_25M),
    .rst_n        (rst_n),
    .rs485_r      (rs485_r),
    .rs485_d      (enc_tx),
    .rs485_de     (enc_de),
    .tx_led       (),
    .rx_led       (),
    .abs_pos      (enc_abs_pos),
    .pos_valid    (enc_pos_valid),
    .status_flag  (),
    .crc_err_flag (enc_crc_err),
    .beep_disable ()
);

// 顶层 RS485 输出连线（必须，否则端口悬空）
assign rs485_d  = enc_tx;
assign rs485_de = enc_de;

// ============================================================
// 13. 编码器有效性监控（失效 → 报警 + 停车）
// ============================================================
localparam [24:0] ENC_TIMEOUT_CYCLES = 25'd2_500_000; // 100ms@25MHz 内无有效帧即判失效（可按需调整）

reg [24:0] enc_fail_cnt;
reg        enc_ok;      // 1 = 编码器健康（最近 100ms 内收到过有效位置）
(* keep *) reg enc_alarm;   // 1 = 曾发生编码器失效（锁存报警，复位后清除）

always @(posedge CLK_25M or negedge rst_n) begin
    if (!rst_n) begin
        enc_fail_cnt <= 25'd0;
        enc_ok       <= 1'b0;
        enc_alarm    <= 1'b0;
    end else if (enc_pos_valid) begin
        // 收到有效帧：复位超时计数，恢复健康
        enc_fail_cnt <= 25'd0;
        enc_ok       <= 1'b1;
    end else if (enc_fail_cnt >= ENC_TIMEOUT_CYCLES - 1) begin
        // 连续超时未收到有效帧：判失效并锁存报警
        enc_fail_cnt <= ENC_TIMEOUT_CYCLES - 1;  // 停在最大值，防止计数翻转
        enc_ok       <= 1'b0;
        enc_alarm    <= 1'b1;
    end else begin
        enc_fail_cnt <= enc_fail_cnt + 1'b1;
    end
end

// 停车：预定位阶段角度锁 0，不依赖编码器；运行阶段编码器失效立即封锁 6 路 PWM
wire svpwm_en = calib_done && (enc_ok || !align_done);

endmodule