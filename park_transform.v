//`timescale 1ns / 1ps
//module park_transform (
//    input  wire                   clk,
//    input  wire                   rst_n,
//    input  wire                   en,       
//    input  wire signed [15:0]     i_alpha,
//    input  wire signed [15:0]     i_beta,
//    input  wire signed [19:0]     sin_th,  // 🚨 接收 20 位 sin
//    input  wire signed [19:0]     cos_th,  // 🚨 接收 20 位 cos
//    output reg  signed [15:0]     id_out,
//    output reg  signed [15:0]     iq_out,
//    output reg                    out_valid
//);
//    // 🚨 物理级：强制符号位扩展到 36 位，阻断溢出 bug！
//    wire signed [35:0] alpha_36 = {{20{i_alpha[15]}}, i_alpha};
//    wire signed [35:0] beta_36  = {{20{i_beta[15]}}, i_beta};
//    wire signed [35:0] sin_36   = {{16{sin_th[19]}}, sin_th};
//    wire signed [35:0] cos_36   = {{16{cos_th[19]}}, cos_th};
//
//    // 36位乘法寄存器
//    reg signed [35:0] mul_id1, mul_id2;
//    reg signed [35:0] mul_iq1, mul_iq2;
//    reg en_d1; 
//
//    wire signed [35:0] sum_id = mul_id1 + mul_id2;
//    wire signed [35:0] sum_iq = mul_iq1 - mul_iq2;
//
//    always @(posedge clk or negedge rst_n) begin
//        if (!rst_n) begin
//            mul_id1 <= 0; mul_id2 <= 0;
//            mul_iq1 <= 0; mul_iq2 <= 0;
//            en_d1   <= 0;
//        end else if (en) begin
//            mul_id1 <= alpha_36 * cos_36;
//            mul_id2 <= beta_36  * sin_36;
//            mul_iq1 <= beta_36  * cos_36;
//            mul_iq2 <= alpha_36 * sin_36;
//            en_d1   <= 1'b1;
//        end else begin
//            en_d1   <= 1'b0;
//        end
//    end
//
//    always @(posedge clk or negedge rst_n) begin
//        if (!rst_n) begin
//            id_out <= 0; iq_out <= 0;
//            out_valid <= 0;
//        end else if (en_d1) begin
//            // 🚨 Q19 截断：抛弃 >>> 19，物理抽取 [34:19] 还原回 16 位 Q15
//            id_out <= sum_id[34:19];
//            iq_out <= sum_iq[34:19];
//            out_valid <= 1'b1;
//        end else begin
//            out_valid <= 1'b0;
//        end
//    end
//endmodule
`timescale 1ns / 1ps
module park_transform (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   en,       
    input  wire signed [15:0]     i_alpha,
    input  wire signed [15:0]     i_beta,
    input  wire signed [17:0]     sin_th,  // 🚨 接收 20 位 sin
    input  wire signed [17:0]     cos_th,  // 🚨 接收 20 位 cos
    output reg  signed [15:0]     id_out,
    output reg  signed [15:0]     iq_out,
    output reg                    out_valid
);
    // 🚨 物理级：强制符号位扩展到 36 位，阻断溢出 bug！
    wire signed [35:0] alpha_36 = {{20{i_alpha[15]}}, i_alpha};
    wire signed [35:0] beta_36  = {{20{i_beta[15]}}, i_beta};
    wire signed [35:0] sin_36   = {{18{sin_th[17]}}, sin_th};
    wire signed [35:0] cos_36   = {{18{cos_th[17]}}, cos_th};

    // 36位乘法寄存器
    reg signed [35:0] mul_id1, mul_id2;
    reg signed [35:0] mul_iq1, mul_iq2;
    reg en_d1; 

    wire signed [35:0] sum_id = mul_id1 + mul_id2;
    wire signed [35:0] sum_iq = mul_iq1 - mul_iq2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_id1 <= 0; mul_id2 <= 0;
            mul_iq1 <= 0; mul_iq2 <= 0;
            en_d1   <= 0;
        end else if (en) begin
            mul_id1 <= alpha_36 * cos_36;
            mul_id2 <= beta_36  * sin_36;
            mul_iq1 <= beta_36  * cos_36;
            mul_iq2 <= alpha_36 * sin_36;
            en_d1   <= 1'b1;
        end else begin
            en_d1   <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_out <= 0; iq_out <= 0;
            out_valid <= 0;
        end else if (en_d1) begin
            // 🚨 Q19 截断：抛弃 >>> 19，物理抽取 [34:19] 还原回 16 位 Q15
            id_out <= sum_id[33:16];
            iq_out <= sum_iq[33:16];
            out_valid <= 1'b1;
        end else begin
            out_valid <= 1'b0;
        end
    end
endmodule
