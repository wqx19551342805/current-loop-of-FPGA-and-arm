//`timescale 1ns / 1ps
//
//module cordic_sincos_full #(
//    parameter WIDTH = 20
//)(
//    input  wire                   clk,
//    input  wire                   rstn,
//    input  wire                   i_en,
//    input  wire signed [19:0]     i_theta,  // 🚨 升级为 20 位输入
//    output reg                    o_en,
//    output reg  signed [19:0]     o_sin,    // 🚨 升级为 20 位输出 (Q19格式)
//    output reg  signed [19:0]     o_cos
//);
//    // 绝对安全的查表函数 (Q19 格式：180度 = 524288)
//    function signed [19:0] get_atan;
//        input [4:0] idx;
//        begin
//            case (idx)
//                5'd0:  get_atan = 20'sd131072; // 45 度
//                5'd1:  get_atan = 20'sd77376;
//                5'd2:  get_atan = 20'sd40884;
//                5'd3:  get_atan = 20'sd20753;
//                5'd4:  get_atan = 20'sd10416;
//                5'd5:  get_atan = 20'sd5214;
//                5'd6:  get_atan = 20'sd2608;
//                5'd7:  get_atan = 20'sd1304;
//                5'd8:  get_atan = 20'sd652;
//                5'd9:  get_atan = 20'sd326;
//                5'd10: get_atan = 20'sd163;
//                5'd11: get_atan = 20'sd82;
//                5'd12: get_atan = 20'sd41;
//                5'd13: get_atan = 20'sd20;
//                5'd14: get_atan = 20'sd10;
//                5'd15: get_atan = 20'sd5;
//                5'd16: get_atan = 20'sd3;
//                5'd17: get_atan = 20'sd1;
//                5'd18: get_atan = 20'sd1;
//                5'd19: get_atan = 20'sd0;
//                default: get_atan = 20'sd0;
//            endcase
//        end
//    endfunction
//
//    // K=0.607252935 的放大补偿，Q19 下为 318375
//    localparam signed [19:0] K_FACTOR = 20'sd318375;
//
//    
//    // 状态机寄存器
//    reg [5:0] state;
//    // 🚨 修复1：将内部 X 和 Y 寄存器扩展为 22 位有符号数，增加保护位防溢出
//    reg signed [21:0] cur_x, cur_y; 
//    reg signed [19:0] cur_z; // Z 代表角度，不会触及溢出边界，保持 20 位即可
//    wire [4:0] step = state - 6'd1;
//
//    always @(posedge clk or negedge rstn) begin
//        if (~rstn) begin
//            state <= 6'd0;
//            o_en  <= 1'b0;
//            o_sin <= 20'd0;
//            o_cos <= 20'd0;
//            cur_x <= 22'd0;
//            cur_y <= 22'd0;
//            cur_z <= 20'd0;
//        end else begin
//            o_en <= 1'b0;
//            if (state == 6'd0) begin
//                if (i_en) begin
//                    if (i_theta > 20'sd262144 || i_theta < -20'sd262144) begin
//                        // 20位的 -K_FACTOR 赋给 22位寄存器，Verilog 会自动进行符号扩展
//                        cur_x <= -K_FACTOR; 
//                        cur_y <= 22'd0;
//                        cur_z <= {~i_theta[19], i_theta[18:0]};
//                    end else begin
//                        cur_x <= K_FACTOR;
//                        cur_y <= 22'd0;
//                        cur_z <= i_theta;
//                    end
//                    state <= 6'd1;
//                end
//            end else if (state >= 6'd1 && state <= 6'd20) begin
//                if (cur_z >= 0) begin
//                    cur_x <= cur_x - (cur_y >>> step);
//                    cur_y <= cur_y + (cur_x >>> step);
//                    cur_z <= cur_z - get_atan(step);
//                end else begin
//                    cur_x <= cur_x + (cur_y >>> step);
//                    cur_y <= cur_y - (cur_x >>> step);
//                    cur_z <= cur_z + get_atan(step);
//                end
//                state <= state + 1'b1;
//            end else if (state == 6'd21) begin
//                // 🚨 修复2：输出前进行 20位饱和截断 (Saturation)
//                // 处理余弦输出
//                if (cur_x > 22'sd524287) 
//                    o_cos <= 20'sd524287;
//                else if (cur_x < -22'sd524288) 
//                    o_cos <= -20'sd524288;
//                else 
//                    o_cos <= cur_x[19:0];
//                
//                // 处理正弦输出
//                if (cur_y > 22'sd524287) 
//                    o_sin <= 20'sd524287;
//                else if (cur_y < -22'sd524288) 
//                    o_sin <= -20'sd524288;
//                else 
//                    o_sin <= cur_y[19:0];
//
//                o_en  <= 1'b1; 
//                state <= 6'd0;
//            end
//        end
//    end
//endmodule
`timescale 1ns / 1ps

module cordic_sincos_full (
    input  wire                   clk,
    input  wire                   rstn,
    input  wire                   i_en,
    input  wire [16:0]            i_theta,  // 17位绝对编码器输入 [0,131071] 对应 [0°,360°)
    output reg                    o_en,
    output reg  signed [17:0]     o_sin,    // Q16格式 (180°=65536)
    output reg  signed [17:0]     o_cos
);

    // 查表函数 (Q16 格式：180度 = 65536)
    function signed [17:0] get_atan;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  get_atan = 18'sd16384; // atan(1)    45.000°
                5'd1:  get_atan = 18'sd9672;  // atan(1/2)  26.565°
                5'd2:  get_atan = 18'sd5111;  // atan(1/4)  14.036°
                5'd3:  get_atan = 18'sd2594;  // atan(1/8)   7.125°
                5'd4:  get_atan = 18'sd1302;
                5'd5:  get_atan = 18'sd652;
                5'd6:  get_atan = 18'sd326;
                5'd7:  get_atan = 18'sd163;
                5'd8:  get_atan = 18'sd82;
                5'd9:  get_atan = 18'sd41;
                5'd10: get_atan = 18'sd20;
                5'd11: get_atan = 18'sd10;
                5'd12: get_atan = 18'sd5;
                5'd13: get_atan = 18'sd3;
                5'd14: get_atan = 18'sd1;
                5'd15: get_atan = 18'sd1;
                5'd16: get_atan = 18'sd0;
                default: get_atan = 18'sd0;
            endcase
        end
    endfunction

    // K = 0.60725293500888，在 Q16 下为 39797
    localparam signed [17:0] K_FACTOR = 18'sd39797;

    // 状态机寄存器
    reg [5:0] state;
    reg signed [19:0] cur_x, cur_y;   // 多2位保护位防溢出
    reg signed [17:0] cur_z;          // Z 代表剩余角度
    wire [4:0] step = state[4:0] - 5'd1;

    // ---------------------------------------------------------------
    // 17位无符号编码器 -> 有符号角度 [-180°, 180°) 映射
    // 若 i_theta >= 65536 (即 >= 180°)，减去 131072 得到负角度
    // ---------------------------------------------------------------
    wire [17:0]        theta_ext = {1'b0, i_theta};  // 零扩展到18位
    wire signed [17:0] theta_signed = theta_ext[16]
                                    ? (theta_ext - 18'd131072)
                                    :  theta_ext;

    always @(posedge clk or negedge rstn) begin
        if (~rstn) begin
            state <= 6'd0;
            o_en  <= 1'b0;
            o_sin <= 18'sd0;
            o_cos <= 18'sd0;
            cur_x <= 20'sd0;
            cur_y <= 20'sd0;
            cur_z <= 18'sd0;
        end else begin
            o_en <= 1'b0;
            if (state == 6'd0) begin
                if (i_en) begin
                    // ---------- 象限预处理 ----------
                    if (theta_signed > 18'sd32768) begin
                        // 第二象限 (90°, 180°)：旋转 -180°
                        cur_x <= -K_FACTOR;
                        cur_y <= 20'sd0;
                        cur_z <= theta_signed - 18'sd65536;
                    end else if (theta_signed < -18'sd32768) begin
                        // 第三象限 (-180°, -90°)：旋转 +180°
                        cur_x <= -K_FACTOR;
                        cur_y <= 20'sd0;
                        cur_z <= theta_signed + 18'sd65536;
                    end else begin
                        // 第一 / 第四象限 [-90°, 90°]
                        cur_x <= K_FACTOR;
                        cur_y <= 20'sd0;
                        cur_z <= theta_signed;
                    end
                    state <= 6'd1;
                end
            end else if (state >= 6'd1 && state <= 6'd17) begin
                // ---------- CORDIC 迭代 17 次 ----------
                if (cur_z >= 0) begin
                    cur_x <= cur_x - (cur_y >>> step);
                    cur_y <= cur_y + (cur_x >>> step);
                    cur_z <= cur_z - get_atan(step);
                end else begin
                    cur_x <= cur_x + (cur_y >>> step);
                    cur_y <= cur_y - (cur_x >>> step);
                    cur_z <= cur_z + get_atan(step);
                end
                state <= state + 1'b1;
            end else if (state == 6'd18) begin
                // ---------- 输出饱和截断 ----------
                if (cur_x > 20'sd65535)
                    o_cos <= 18'sd65535;
                else if (cur_x < -20'sd65536)
                    o_cos <= -18'sd65536;
                else
                    o_cos <= cur_x[17:0];

                if (cur_y > 20'sd65535)
                    o_sin <= 18'sd65535;
                else if (cur_y < -20'sd65536)
                    o_sin <= -18'sd65536;
                else
                    o_sin <= cur_y[17:0];

                o_en  <= 1'b1;
                state <= 6'd0;
            end
        end
    end
endmodule