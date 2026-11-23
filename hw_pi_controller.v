`timescale 1ns / 1ps

// ============================================================
// 硬件 PI 控制器 (带跨时钟域独立清零功能)
// ============================================================
module hw_pi_controller (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 en,          // 接收上一级的有效脉冲
    
    input  wire signed [15:0]   target_val,  // 目标给定
    input  wire signed [15:0]   actual_val,  // 实际反馈
    
    input  wire signed [15:0]   kp,          // 比例系数 (Q11格式)
    input  wire signed [15:0]   ki,          // 积分系数 (Q11格式)

    output reg  signed [15:0]   pi_out,      // 输出指令
    output reg                  out_valid,   // 传给下一级的有效脉冲
    
    input  wire                 clear_i      // 🚨 高优先级积分器清零信号
);

    // 1. 计算误差 Error
    wire signed [16:0] error = target_val - actual_val;

    // 2. 乘法项计算
    wire signed [31:0] p_term_wire = error * kp;
    wire signed [31:0] i_term_wire = error * ki;

    // 内部流水线寄存器
    reg signed [31:0] p_term;
    reg signed [31:0] integral_sum;
    reg en_d1;
    
    // 积分与输出限幅
    localparam signed [31:0] INTEGRAL_MAX =  32'sd65536000;
    localparam signed [31:0] INTEGRAL_MIN = -32'sd65536000;
    localparam signed [15:0] OUT_MAX      =  16'sd32000;
    localparam signed [15:0] OUT_MIN      = -16'sd32000;

    // ==========================================
    // 第一级流水线：积分独立清零判定与正常累加
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_term       <= 0;
            integral_sum <= 0;
            en_d1        <= 0;
        end else if (clear_i) begin  
            // 🚨 独立判断极高优先级：只要收到清零脉冲，无视 en 状态立刻清零积分器
            integral_sum <= 0;
            en_d1        <= 1'b0;    
        end else if (en) begin
            p_term <= p_term_wire;
            en_d1  <= 1'b1;
            
            // 正常的积分累加与抗饱和 (Anti-Windup) 钳位
            if (integral_sum + i_term_wire > INTEGRAL_MAX)
                integral_sum <= INTEGRAL_MAX;
            else if (integral_sum + i_term_wire < INTEGRAL_MIN)
                integral_sum <= INTEGRAL_MIN;
            else
                integral_sum <= integral_sum + i_term_wire;
        end else begin
            en_d1 <= 1'b0;
        end
    end
    
    // ==========================================
    // 第二级流水线：P + I 求和，恢复比例及输出限幅
    // ==========================================
    wire signed [31:0] pi_sum_32 = p_term + integral_sum;
    wire signed [15:0] pi_sum_16 = pi_sum_32[26:11]; // Q11格式移位恢复

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pi_out    <= 0;
            out_valid <= 0;
        end else if (en_d1) begin
            // 最终的输出钳位保护
            if (pi_sum_16 > OUT_MAX)
                pi_out <= OUT_MAX;
            else if (pi_sum_16 < OUT_MIN)
                pi_out <= OUT_MIN;
            else
                pi_out <= pi_sum_16;
                
            out_valid <= 1'b1;
        end else begin
            out_valid <= 1'b0;
        end
    end

endmodule