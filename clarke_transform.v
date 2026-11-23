`timescale 1ns / 1ps

module clarke_transform (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   en,        // ADC 采样完成触发信号
    
    // 输入：ADC 采集到的实际物理相电流 (带符号的 16 位定点数)
    input  wire signed [15:0]     ia,
    input  wire signed [15:0]     ib,
    
    // 输出：静止正交坐标系电流 (送给后续的 Park 变换)
    output reg  signed [15:0]     i_alpha,
    output reg  signed [15:0]     i_beta,
    output reg                    out_valid  // 变换完成标志
);

    // Q15 格式的常数: 1/sqrt(3) ≈ 0.57735 -> 18919
    localparam signed [15:0] INV_SQRT3 = 16'sd18919; 

    // 第一级流水线寄存器：计算 (Ia + 2*Ib)   因为iB=1/sqrt(3)(ia+2ib)
    reg signed [16:0] ia_plus_2ib;
    reg en_d1;
    reg signed [15:0] ia_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ia_plus_2ib <= 0;
            en_d1       <= 0;
            ia_d1       <= 0;
        end else if (en) begin
            // Ib 左移 1 位就等于乘以 2
            ia_plus_2ib <= ia + (ib <<< 1); 
            ia_d1       <= ia;   // I_alpha 直接等于 Ia，打拍对齐
            en_d1       <= 1'b1;
        end else begin
            en_d1       <= 1'b0;
        end
    end

    // 第二级流水线寄存器：乘法与截位
    wire signed [32:0] ibeta_mul = ia_plus_2ib * INV_SQRT3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_alpha   <= 0;
            i_beta    <= 0;
            out_valid <= 0;
        end else if (en_d1) begin
            i_alpha   <= ia_d1;
            // 乘法结果右移 15 位，恢复到原来的 Q15 比例
            i_beta    <= ibeta_mul[30:15]; 
            out_valid <= 1'b1;
        end else begin
            out_valid <= 1'b0;
        end
    end

endmodule