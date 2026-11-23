`timescale 1ns / 1ps

module foc_openloop_top (
    input  wire               CLK_25M,

    output wire               pwm_uh_out, pwm_ul_out,
    output wire               pwm_vh_out, pwm_vl_out,
    output wire               pwm_wh_out, pwm_wl_out,

    output               	LEDR,   // 上电运行指示（常亮）
    output wire               FPGA_LEDG,   // 相位累加器慢闪（~3Hz，肉眼可视）
    output wire               FPGA_LEDB    // 常灭
);
//always@(posedge CLK_25M)
//	begin
//		LEDR <= CLK_25M;
//end
//wire CLK_25M;
//assign LEDR = CLK_25M;
//    // ------------------------- 0. 延时复位 ------------------------- //
    reg [5:0] rst_cnt = 6'd0;
    reg rst_n = 1'd0;
    always @(posedge CLK_25M) begin
        if (rst_cnt == 6'd50) rst_n <= 1'b1;
        else rst_cnt <= rst_cnt + 1'b1;
    end

    // ------------------------- 1. 虚拟电角度（NCO 相位累加器） ------------------------- //
    // 32 位累加器，高 20 位 phase_acc[31:12] 即 Q19 电角度，范围 0 ~ 2^20 对应 0~360°
    // 电频率 f_elec = PHASE_INC × 25e6 / 2^32
    localparam [31:0] PHASE_INC = 32'd17180;   // 100Hz 电频率（10对极 → 600 rpm）
    reg [31:0] phase_acc;
    always @(posedge CLK_25M or negedge rst_n) begin
        if (!rst_n) phase_acc <= 32'd0;
        else phase_acc <= phase_acc + PHASE_INC;
    end
    // 位模式直接接 CORDIC: 无符号 0~2^20 按补码重解释为 -180°~+180°，循环连续无跳变
    wire [19:0] theta = phase_acc[31:12];

    // ------------------------- 2. CORDIC 触发（每 64 拍采样一次角度） ------------------------- //
    // CORDIC 在 i_en 拍只采样一次 i_theta，之后锁存计算 21 拍，期间相位变化不影响结果。
    // 64 拍 = 2.56us 一算，对 100Hz 电频率 = 每电周期约 3900 个角度台阶，足够平滑。
    reg [5:0] trig_cnt;
    reg trig_pulse;
    always @(posedge CLK_25M or negedge rst_n) begin
        if (!rst_n) begin
            trig_cnt   <= 6'd0;
            trig_pulse <= 1'b0;
        end else begin
            trig_pulse <= 1'b0;
            if (trig_cnt == 6'd63) begin
                trig_cnt   <= 6'd0;
                trig_pulse <= 1'b1;
            end else begin
                trig_cnt <= trig_cnt + 1'b1;
            end
        end
    end

    // ------------------------- 3. CORDIC: θ → sin/cos ------------------------- //
    wire signed [19:0] cordic_sin, cordic_cos;
    wire cordic_done;
    cordic_sincos_full #(.WIDTH(16)) u_cordic (
        .clk     (CLK_25M),
        .rstn    (rst_n),
        .i_en    (trig_pulse),
        .i_theta (theta),          // 20 位补码角度，直接接入
        .o_en    (cordic_done),
        .o_sin   (cordic_sin),
        .o_cos   (cordic_cos)
    );

    // ------------------------- 4. 开环电压指令 ------------------------- //
    // 表贴式电机: Vd 恒为 0，Vq 决定相电压幅值（Q15）。
    localparam signed [15:0] VQ_DEFAULT = 16'sd30000;  // 相电压幅值 ≈ Q15 的 24.4%
    wire signed [15:0] vd = 16'sd0;
    wire signed [15:0] vq = VQ_DEFAULT;

    // ------------------------- 5. 逆 Park: (Vd,Vq,θ) → (Vα,Vβ) ------------------------- //
    wire signed [15:0] v_alpha, v_beta;
    wire inv_park_valid;
    inv_park_transform u_inv_park (
        .clk       (CLK_25M),
        .rst_n     (rst_n),
        .en        (cordic_done),
        .vd        (vd),
        .vq        (vq),
        .sin_th    (cordic_sin),
        .cos_th    (cordic_cos),
        .v_alpha   (v_alpha),
        .v_beta    (v_beta),
        .out_valid (inv_park_valid)
    );
	 
//my_pll u_pll (
//    .areset  (1'b0),       // 高电平复位，这里直接接地（永久不复位）
//    .inclk0  (CLK_50M),    // 输入 25MHz
//    .c0      (CLK_25M),    // 输出 50MHz
////    .locked  (pll_locked)  // 可选：接一个 LED 指示 PLL 是否锁定
//);
    // ------------------------- 6. SVPWM + 死区 → 6 路 PWM ------------------------- //
    wire [15:0] dbg_cmp_u, dbg_cmp_v, dbg_cmp_w;
    svpwm_generator #(
        .SYS_CLK_FREQ  (25_000_000),
        .PWM_FREQ      (10_000),
        .DEAD_TIME_NS  (1_000),
        .MAX_AMP       (16'd32767),
        .MAX_AMP_SHIFT (15)
    ) u_svpwm (
        .clk        (CLK_25M),
        .rst_n      (rst_n),
        .en         (1'b1),          // 开环测试常开发波（急停直接断电）
        .v_alpha    (v_alpha),
        .v_beta     (v_beta),
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

    // ------------------------- 7. LED 指示 ------------------------- //
//    assign FPGA_LEDR = rst_n;              // 上电后常亮
    assign FPGA_LEDG = phase_acc[22];      // 25MHz/2^23 ≈ 3Hz 慢闪
    assign FPGA_LEDB = 1'b0;               // 常灭

endmodule
