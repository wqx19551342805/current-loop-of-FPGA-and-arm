module AD7606C_MODE1 #(
    parameter  SYS_CLK_FREQ = 25_000_000, // 系统时钟频率(Hz)，必须与实际一致
    parameter  CH1_RANGE  = 4'b1011,//+-12.5V
    parameter  CH2_RANGE  = 4'b1011,
    parameter  CH3_RANGE  = 4'b1011,
    parameter  CH4_RANGE  = 4'b1011,
    parameter  CH5_RANGE  = 4'b1011,
    parameter  CH6_RANGE  = 4'b1011,
    parameter  CH7_RANGE  = 4'b1011,
    parameter  CH8_RANGE  = 4'b1011
)(
    input   wire          sys_clk   ,
    input   wire          sys_rst   ,
    input   wire          ext_trig  , // 🚨 新增：外部同步触发信号
    inout   wire  [15:0]  DB0_15    ,
    input   wire          busy      ,
    input   wire          Frstdata  ,
    output  wire          reset     ,
    output  wire          cs_n      ,
    output  wire          rd_n      ,
    output  wire          wr_n      ,
    output  wire          convst    ,
    output  wire          ser_sel   ,
    output  wire  [ 2:0]  OSC2_0    ,
    output  wire  [15:0]  data_ch1  ,
    output  wire  [15:0]  data_ch2  ,
    output  wire  [15:0]  data_ch3  ,
    output  wire  [15:0]  data_ch4  ,
    output  wire  [15:0]  data_ch5  ,
    output  wire  [15:0]  data_ch6  ,
    output  wire  [15:0]  data_ch7  ,
    output  wire  [15:0]  data_ch8  ,
    output  wire          sample_out
);

localparam delay_200ms = ( SYS_CLK_FREQ / 5   ) - 1;
localparam delay_100ms = ( SYS_CLK_FREQ / 10  ) - 1;
localparam delay_10ms  = ( SYS_CLK_FREQ / 100 ) - 1;
localparam delay_40ns  = ( SYS_CLK_FREQ / 25_000_000 ) - 1;
localparam delay_1us   = ( SYS_CLK_FREQ / 1_000_000  ) - 1;
localparam cycle_cnt   = ( 2 * 8 ) - 1;

reg          santai_sel;
wire  [15:0] in_data   ;

assign in_data = DB0_15;
assign DB0_15  = santai_sel ? SEND_DATA : 16'dz;

reg          cs       ;
reg          rd       ;
reg          rst      ;
reg          conv     ;
reg          wr       ;
reg  [ 3:0]  state    ;
reg  [ 2:0]  DATA_CNT ;
reg  [ 2:0]  DATA_SEL ;
reg  [23:0]  delay_cnt;
reg  [15:0]  ch1_data ;
reg  [15:0]  ch2_data ;
reg  [15:0]  ch3_data ;
reg  [15:0]  ch4_data ;
reg  [15:0]  ch5_data ;
reg  [15:0]  ch6_data ;
reg  [15:0]  ch7_data ;
reg  [15:0]  ch8_data ;
reg  [15:0]  SEND_DATA;

reg          sample_sig;             
reg          busy1;
reg          busy2;
wire         busy_nege;

reg          rd_reg ;
wire         rd_nege;

localparam RANGE_CH2_CH1  = 7'h03;
localparam RANGE_CH4_CH3  = 7'h04;
localparam RANGE_CH6_CH5  = 7'h05;
localparam RANGE_CH8_CH7  = 7'h06;
localparam BANDWIDTH      = 7'h07;
localparam OPEN_DETECTED  = 7'h24;

always@(*)begin
    case(DATA_CNT)
        3'd0:SEND_DATA = {1'b1,OPEN_DETECTED,8'hff};
        3'd1:SEND_DATA = {1'b0,RANGE_CH2_CH1,CH2_RANGE,CH1_RANGE};
        3'd2:SEND_DATA = {1'b0,RANGE_CH4_CH3,CH4_RANGE,CH3_RANGE};
        3'd3:SEND_DATA = {1'b0,RANGE_CH6_CH5,CH6_RANGE,CH5_RANGE};
        3'd4:SEND_DATA = {1'b0,RANGE_CH8_CH7,CH8_RANGE,CH7_RANGE};
        3'd5:SEND_DATA = {1'b0,BANDWIDTH,8'b11111111}; 
        3'd6:SEND_DATA = 16'h0000; 
        default:SEND_DATA = 16'd0;
    endcase
end

always@(posedge sys_clk or negedge sys_rst)
begin
    if(!sys_rst)begin
        busy1 <= 1'b0;
        busy2 <= 1'b0;
    end
    else begin
        busy1 <= busy;
        busy2 <= busy1;
    end
end
assign busy_nege = (~busy1) & busy2;

always@(posedge sys_clk or negedge sys_rst)
begin
    if(!sys_rst)begin
        rst  <= 1'b0;
        cs   <= 1'b1;
        rd   <= 1'b1; 
        wr   <= 1'b1;
        conv <= 1'b1;
        santai_sel <= 1'b1;
        DATA_CNT   <= 3'd0;
        delay_cnt  <= 24'd0;
        state      <= 4'd0;
    end
    else begin
        case(state)
            4'd0:begin  
                rst  <= 1'b0;
                cs   <= 1'b1;
                rd   <= 1'b1;
                wr   <= 1'b1;
                conv <= 1'b1;
                DATA_CNT   <= 3'd0;
                santai_sel <= 1'b1;
                if(delay_cnt >= delay_100ms)begin
                    delay_cnt <= 24'd0;
                    state <= 4'd1;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd0;
                end
            end
            4'd1:begin  
                if(delay_cnt >= delay_100ms)begin
                    delay_cnt <= 24'd0;
                    state <= 4'd2;
                end
                else if(delay_cnt >= delay_10ms)begin
                    rst   <= 1'b0;
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd1;
                end
                else begin
                    rst   <= 1'b1;
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd1;
                end
            end
            4'd2:begin
                cs    <= 1'b0;
                wr    <= 1'b1;
                state <= 4'd3;
            end
            4'd3:begin
                wr <= 1'b0;
                if(delay_cnt >= delay_40ns)begin
                    delay_cnt <= 24'd0;
                    state <= 4'd4;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd3;
                end
            end
            4'd4:begin
                wr <= 1'b1; 
                state <= 4'd5;
            end
            4'd5:begin
                rd <= 1'b0;
                if(delay_cnt >= delay_40ns)begin
                    delay_cnt <= 24'd0;
                    state <= 4'd6;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd5;
                end
            end
            4'd6:begin
                rd <= 1'b1;
                state   <= 4'd7;
            end
            4'd7:begin
                wr <= 1'b0;
                if(delay_cnt >= delay_40ns)begin
                    delay_cnt <= 24'd0;
                    state <= 4'd8;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd7;
                end
            end
            4'd8:begin
                wr <= 1'b1;
                if(DATA_CNT >= 3'd6)begin
                    DATA_CNT <= DATA_CNT;
                    state <= 4'd9;
                end
                else begin
                    DATA_CNT <= DATA_CNT + 1'b1;
                    state <= 4'd2;
                end
            end
            4'd9:begin  
                cs <= 1'b1;
                santai_sel <= 1'b0;
                state <= 4'd10;
            end
            4'd10:begin
                conv  <= 1'b0;
                state <= 4'd11;
            end
            4'd11:begin
                conv <= 1'b1;
                if(busy_nege == 1'b1)begin
                    cs <= 1'b0; 
                    state  <= 4'd12;
                end
                else begin
                    cs <= 1'b1; 
                    state  <= 4'd11;
                end
            end
            4'd12:begin
                rd <= ~rd;
                if(delay_cnt >= cycle_cnt)begin
                    delay_cnt <= 24'd0;
                    state <= 4'd13;
                end
                else begin
                    delay_cnt <= delay_cnt + 1'b1;
                    state <= 4'd12;
                end
            end
            4'd13:begin
                cs <= 1'b1;
                rd <= 1'b1;
                // 🚨 依赖于 ext_trig 信号进行外同步触发
                if(sample_sig == 1'b1)begin
                    conv  <= 1'b0;
                    state <= 4'd11;  
                end
                else begin
                    conv  <= 1'b1;
                    state <= 4'd13;  
                end
            end
            default:state <= 4'd0;
        endcase
    end
end

// 🚨 核心改动：用外部 ext_trig 替换内部计时器
always@(posedge sys_clk or negedge sys_rst)
begin
    if(!sys_rst)begin
        sample_sig <= 1'b0;
    end
    else if(ext_trig == 1'b1 && state == 4'd13)begin
        sample_sig <= 1'b1;
    end
    else begin
        sample_sig <= 1'b0;
    end
end

always@(posedge sys_clk or negedge sys_rst)
begin
    if(!sys_rst)begin
        rd_reg <= 1'b0;
    end
    else begin
        rd_reg <= rd;
    end
end
assign rd_nege = (~rd) & rd_reg;

always@(posedge sys_clk or negedge sys_rst)
begin
    if(!sys_rst)begin
        DATA_SEL <= 3'd0;
        ch1_data <= 16'd0;
        ch2_data <= 16'd0;
        ch3_data <= 16'd0;
        ch4_data <= 16'd0;
        ch5_data <= 16'd0;
        ch6_data <= 16'd0;
        ch7_data <= 16'd0;
        ch8_data <= 16'd0;
    end
    else if((rd_nege == 1'b1) && (state >= 4'd11))begin
        case(DATA_SEL)
            3'd0:begin
               if(Frstdata == 1'b1)begin
                    ch1_data <= in_data;
                    DATA_SEL <= 3'd1;
               end
               else begin
                   ch1_data <= ch1_data;
                   DATA_SEL <= 3'd0;
               end
            end
            3'd1:begin
                ch2_data <= in_data;
                DATA_SEL <= 3'd2;
            end
            3'd2:begin
                ch3_data <= in_data;
                DATA_SEL <= 3'd3;
            end
            3'd3:begin
                ch4_data <= in_data;
                DATA_SEL <= 3'd4;
            end
            3'd4:begin
                ch5_data <= in_data;
                DATA_SEL <= 3'd5;
            end
            3'd5:begin
                ch6_data <= in_data;
                DATA_SEL <= 3'd6;
            end
            3'd6:begin
                ch7_data <= in_data;
                DATA_SEL <= 3'd7;
            end
            3'd7:begin
                ch8_data <= in_data;
                DATA_SEL <= 3'd0;
            end
            default:DATA_SEL <= 3'd0;
        endcase
    end
    else begin
        ch1_data <= ch1_data;
        ch2_data <= ch2_data;
        ch3_data <= ch3_data;
        ch4_data <= ch4_data;
        ch5_data <= ch5_data;
        ch6_data <= ch6_data;
        ch7_data <= ch7_data;
        ch8_data <= ch8_data;
    end
end

assign data_ch1 = ch1_data ;
assign data_ch2 = ch2_data ;
assign data_ch3 = ch3_data ;
assign data_ch4 = ch4_data ;
assign data_ch5 = ch5_data ;
assign data_ch6 = ch6_data ;
assign data_ch7 = ch7_data ;
assign data_ch8 = ch8_data ;

// sample_out 可继续留给外界 (比如顶层的 phase_acc) 作为数据更新有效的标志
assign sample_out = (state == 4'd12 && delay_cnt >= cycle_cnt && DATA_SEL == 3'd7) ? 1'b1 : 1'b0; 

assign OSC2_0  = 3'b111; 
assign reset   = rst  ;
assign cs_n    = cs   ;
assign rd_n    = rd   ;
assign wr_n    = wr   ;
assign convst  = conv ;
assign ser_sel = 1'b0 ;  

endmodule