`timescale 1ns / 1ps

module svpwm_generator #(
    parameter SYS_CLK_FREQ = 25_000_000, // 系统时钟频率25MHz
    parameter PWM_FREQ     = 20_000,     // PWM 载波频率 10kHz
    parameter DEAD_TIME_NS = 1_000,      // 死区时间 1us (1000ns)
    // 归一化最大幅值，必须严格等于 2 的幂次方 (这里用 8192 即 2^13)
//    parameter MAX_AMP      = 16'd8192,   
//    parameter MAX_AMP_SHIFT= 13          // 对应 MAX_AMP 的右移位数
parameter MAX_AMP      = 16'd32767,  // Q15 的最大正数！
    parameter MAX_AMP_SHIFT= 15          // 右移 15 位
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                en,        // 使能信号 (为 0 时封锁所有输出)
    input  wire signed [15:0]  v_alpha,   // 逆 Park 变换输出的 V_alpha
    input  wire signed [15:0]  v_beta,    // 逆 Park 变换输出的 V_beta

    // 6路带死区的 PWM 输出 (高电平导通)
    output wire                pwm_uh, pwm_ul,
    output wire                pwm_vh, pwm_vl,
    output wire                pwm_wh, pwm_wl,
	 output wire adc_trig, // 输出给 ADC 的同步触发脉冲
	 
	 output wire [15:0] dbg_cmp_u,
    output wire [15:0] dbg_cmp_v,
    output wire [15:0] dbg_cmp_w
);

    // ==========================================================
    // 1. 参数与定时器常数计算
    // ==========================================================
    // 中心对齐 PWM 的计数器周期 = (系统时钟 / PWM频率) / 2
    localparam PERIOD = SYS_CLK_FREQ / PWM_FREQ / 2;     
    // 死区时钟周期数 (50MHz 下 1us = 50 ticks)
    localparam DT_TICKS = DEAD_TIME_NS / (1_000_000_000 / SYS_CLK_FREQ); 

    // ==========================================================
    // 2. 数学预处理：计算 U1, U2, U3 
    // ==========================================================
    // sqrt(3)/2 在 Q15 下为 28378
    wire signed [31:0] valpha_sq3_32 = v_alpha * $signed(16'd28378);
    wire signed [15:0] valpha_sq3    = valpha_sq3_32[30:15]; 
    wire signed [15:0] vbeta_half    = v_beta >>> 1; 

    wire signed [15:0] u1 = v_beta;
    wire signed [15:0] u2 = valpha_sq3 - vbeta_half;
    wire signed [15:0] u3 = -valpha_sq3 - vbeta_half;

    // ==========================================================
    // 3. 扇区判断 (Sector Calculation)
    // ==========================================================
    wire a = (u1 > 0) ? 1'b1 : 1'b0;
    wire b = (u2 > 0) ? 1'b1 : 1'b0;
    wire c = (u3 > 0) ? 1'b1 : 1'b0;
    wire [2:0] N = {c, b, a}; 

    // ==========================================================
    // 4. 计算非零矢量作用时间 T1 和 T2 (已彻底修复偶数扇区反向 Bug)
    // ==========================================================
    wire signed [15:0] X = v_beta;
    wire signed [15:0] Y = valpha_sq3 + vbeta_half;
    wire signed [15:0] Z = -valpha_sq3 + vbeta_half;

    reg signed [15:0] t1, t2;
    always @(*) begin
        case (N)
            3'd3: begin t1 = -Z; t2 = X;  end // 扇区 1
            3'd1: begin t1 = Y;  t2 = Z;  end // 扇区 2
            3'd5: begin t1 = X;  t2 = -Y; end // 扇区 3
            3'd4: begin t1 = Z;  t2 = -X; end // 扇区 4
            3'd6: begin t1 = -Y; t2 = -Z; end // 扇区 5
            3'd2: begin t1 = -X; t2 = Y;  end // 扇区 6
            default: begin t1 = 0; t2 = 0; end
        endcase
    end

    // ==========================================================
    // 5. 过调制处理与占空比比较值转换 (安全装甲版：防负数下溢崩溃)
    // ==========================================================
    // 第一道防线：强行滤除一切负数时间，避免后续算术崩溃
//    wire signed [15:0] t1_pos = (t1 > 0) ? t1 : 16'd0;
//    wire signed [15:0] t2_pos = (t2 > 0) ? t2 : 16'd0;
	 
	 wire signed [15:0] t1_pos = (t1 > 0) ? t1 : 16'sd0; // 🚨 必须加 s!
    wire signed [15:0] t2_pos = (t2 > 0) ? t2 : 16'sd0; // 🚨 必须加 s!
    wire signed [16:0] t_sum  = t1_pos + t2_pos; 

    reg signed [15:0] t1_sat, t2_sat;

    always @(*) begin
        // 第二道防线：利用拼接符打破无符号传染，安全比较限幅
        if (t_sum > $signed({1'b0, MAX_AMP})) begin
            t1_sat = (t1_pos > $signed({1'b0, MAX_AMP})) ? MAX_AMP : t1_pos; 
            t2_sat = (t2_pos > $signed({1'b0, MAX_AMP})) ? MAX_AMP : t2_pos;
            if ((t1_sat + t2_sat) > $signed({1'b0, MAX_AMP})) begin
                t1_sat = t1_sat >>> 1; // 极速等比例缩小
                t2_sat = t2_sat >>> 1;
            end
        end else begin
            t1_sat = t1_pos;
            t2_sat = t2_pos;
        end
    end

//    // 跨越领域：将数学比例转换为物理定时器节拍
//    wire [15:0] t1_scaled = (t1_sat * PERIOD) >> MAX_AMP_SHIFT;
//    wire [15:0] t2_scaled = (t2_sat * PERIOD) >> MAX_AMP_SHIFT;
//    wire [15:0] ta_base   = (PERIOD - t1_scaled - t2_scaled) >> 1;
// 先用 32 位的线接住乘法和移位的结果，明确告诉 Quartus 我们知道它有多大
    wire [31:0] t1_scaled_32 = (t1_sat * PERIOD) >> MAX_AMP_SHIFT;
    wire [31:0] t2_scaled_32 = (t2_sat * PERIOD) >> MAX_AMP_SHIFT;
    wire [31:0] ta_base_32   = (PERIOD - t1_scaled_32[15:0] - t2_scaled_32[15:0]) >> 1;

    // 然后显式地截取低 16 位，Quartus 就明白这是你有意为之的物理降维，不再警告！
    wire [15:0] t1_scaled = t1_scaled_32[15:0];
    wire [15:0] t2_scaled = t2_scaled_32[15:0];
    wire [15:0] ta_base   = ta_base_32[15:0];

    // 为了消除 PERIOD/2 带来的警告，提前准备好 16 位的常量
    localparam [15:0] HALF_PERIOD = PERIOD / 2;
    // 比较寄存器赋值（时序化：切断 v_alpha→SVPWM 全组合长路径，满足 25MHz 时序）
    // 占空比更新因此延迟 1 拍（40ns），对 10kHz 载波无影响
    reg [15:0] cmp_u, cmp_v, cmp_w;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmp_u <= HALF_PERIOD; cmp_v <= HALF_PERIOD; cmp_w <= HALF_PERIOD;
        end else begin
            case (N)
                3'd3: begin cmp_u <= ta_base; cmp_v <= ta_base + t1_scaled; cmp_w <= ta_base + t1_scaled + t2_scaled; end 
                3'd1: begin cmp_u <= ta_base + t2_scaled; cmp_v <= ta_base; cmp_w <= ta_base + t1_scaled + t2_scaled; end 
                3'd5: begin cmp_u <= ta_base + t1_scaled + t2_scaled; cmp_v <= ta_base; cmp_w <= ta_base + t1_scaled; end 
                3'd4: begin cmp_u <= ta_base + t1_scaled + t2_scaled; cmp_v <= ta_base + t2_scaled; cmp_w <= ta_base; end 
                3'd6: begin cmp_u <= ta_base + t1_scaled; cmp_v <= ta_base + t1_scaled + t2_scaled; cmp_w <= ta_base; end 
                3'd2: begin cmp_u <= ta_base; cmp_v <= ta_base + t1_scaled + t2_scaled; cmp_w <= ta_base + t2_scaled; end 
                default: begin cmp_u <= HALF_PERIOD; cmp_v <= HALF_PERIOD; cmp_w <= HALF_PERIOD; end
            endcase
        end
    end

    // ==========================================================
    // 6. 中心对齐载波生成 (无平顶完美三角波版)
    // ==========================================================
    (* preserve *)reg [15:0] cnt;
    reg        dir; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 16'd0;
            dir <= 1'b0;
        end else begin
            if (dir == 1'b0) begin // 向上计数
                if (cnt == PERIOD - 1) begin
                    dir <= 1'b1; // 撞顶反向
                end
                cnt <= cnt + 16'd1;
            end else begin // 向下计数
                if (cnt == 16'd1) begin
                    dir <= 1'b0; // 撞底反向
                end
                cnt <= cnt - 16'd1;
            end
        end
    end
	 // 在向上计数撞顶的瞬间产生一个时钟周期的脉冲
    assign adc_trig = (dir == 1'b1 && cnt == 16'd2) ? 1'b1 : 1'b0;
	 
	 assign dbg_cmp_u = cmp_u;
    assign dbg_cmp_v = cmp_v;
    assign dbg_cmp_w = cmp_w;
    // 理想 PWM 比较输出
    (* keep *)wire u_ideal = (cnt > cmp_u) ? 1'b1 : 1'b0;
    wire v_ideal = (cnt > cmp_v) ? 1'b1 : 1'b0;
    wire w_ideal = (cnt > cmp_w) ? 1'b1 : 1'b0;

    // ==========================================================
    // 7. 死区发生器模块实例化
    // ==========================================================
    dead_time_gen #(.DT_TICKS(DT_TICKS)) dt_u (.clk(clk), .rst_n(rst_n), .en(en), .in(u_ideal), .out_h(pwm_uh), .out_l(pwm_ul));
    dead_time_gen #(.DT_TICKS(DT_TICKS)) dt_v (.clk(clk), .rst_n(rst_n), .en(en), .in(v_ideal), .out_h(pwm_vh), .out_l(pwm_vl));
    dead_time_gen #(.DT_TICKS(DT_TICKS)) dt_w (.clk(clk), .rst_n(rst_n), .en(en), .in(w_ideal), .out_h(pwm_wh), .out_l(pwm_wl));

endmodule


// ==============================================================
// 子模块：绝对防御版死区发生器 (与上文放在同一 .v 文件即可)
// ==============================================================
module dead_time_gen #(
    parameter DT_TICKS = 50
)(
    input  wire clk,
    input  wire rst_n,
    input  wire en,
    input  wire in,
    output reg  out_h,
    output reg  out_l
);
    reg [15:0] cnt_h, cnt_l;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_h <= 1'b0; out_l <= 1'b0;
            cnt_h <= 16'd0; cnt_l <= 16'd0;
        end else if (!en) begin // 使能被拉低时，立刻切断所有输出，保命
            out_h <= 1'b0; out_l <= 1'b0;
            cnt_h <= 16'd0; cnt_l <= 16'd0;
        end else begin
            // ----------------------------------------------------
            // 上桥臂逻辑：遇到下降沿立刻关断；遇到上升沿开始倒计时，计满才开
            // ----------------------------------------------------
            if (!in) begin
                out_h <= 1'b0;
                cnt_h <= 16'd0;
            end else begin
                if (cnt_h < DT_TICKS) begin
                    cnt_h <= cnt_h + 16'd1;
                end else begin
                    out_h <= 1'b1;
                end
            end

            // ----------------------------------------------------
            // 下桥臂逻辑：与上桥臂互补，遇到上升沿立刻关断，遇到下降沿倒计时
            // ----------------------------------------------------
            if (in) begin
                out_l <= 1'b0;
                cnt_l <= 16'd0;
            end else begin
                if (cnt_l < DT_TICKS) begin
                    cnt_l <= cnt_l + 16'd1;
                end else begin
                    out_l <= 1'b1;
                end
            end
        end
    end
endmodule
