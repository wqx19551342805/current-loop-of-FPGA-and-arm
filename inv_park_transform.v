`timescale 1ns / 1ps
module inv_park_transform (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   en,          
    input  wire signed [15:0]     vd,          
    input  wire signed [15:0]     vq,          
    input  wire signed [17:0]     sin_th,      // 🚨 20位输入
    input  wire signed [17:0]     cos_th,      // 🚨 20位输入
    output reg  signed [15:0]     v_alpha,     
    output reg  signed [15:0]     v_beta,      
    output reg                    out_valid    
);
    wire signed [35:0] vd_36 = {{20{vd[15]}}, vd};
    wire signed [35:0] vq_36 = {{20{vq[15]}}, vq};
    wire signed [35:0] sin_36 = {{18{sin_th[17]}}, sin_th};
    wire signed [35:0] cos_36 = {{18{cos_th[17]}}, cos_th};

    wire signed [35:0] vd_cos = vd_36 * cos_36;
    wire signed [35:0] vq_sin = vq_36 * sin_36;
    wire signed [35:0] vd_sin = vd_36 * sin_36;
    wire signed [35:0] vq_cos = vq_36 * cos_36;

    wire signed [35:0] sum_valpha = vd_cos - vq_sin;
    wire signed [35:0] sum_vbeta  = vd_sin + vq_cos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_alpha   <= 16'd0;
            v_beta    <= 16'd0;
            out_valid <= 1'b0;
        end else if (en) begin
            // 🚨 Q19 还原截断
            v_alpha   <= sum_valpha[33:16]; 
            v_beta    <= sum_vbeta[33:16];
            out_valid <= 1'b1;
        end else begin
            out_valid <= 1'b0;
        end
    end
endmodule