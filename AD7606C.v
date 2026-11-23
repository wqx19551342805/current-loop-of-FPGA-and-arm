module AD7606C(
  input   wire          SYS_CLK ,
  input   wire          SYS_RST ,
  inout   wire  [15:0]  DB0_15  ,
  input   wire          BUSY    ,
  input   wire          FRSTDATA,
  output  wire          SER_SEL ,
  output  wire          RESET   ,
  output  wire          CS      ,
  output  wire          RD_SCLK ,
  output  wire          WR      ,
  output  wire          CONVST  ,
  output  wire  [ 2:0]  OSC2_0  ,
  output  wire          STBY    
);

// 通道输入范围配置
localparam RANGE_0  = 4'b0000; // ±2.5    V，单端输入范围
localparam RANGE_1  = 4'b0001; // ±5      V，单端输入范围
localparam RANGE_2  = 4'b0010; // ±6.25   V，单端输入范围
localparam RANGE_3  = 4'b0011; // ±10     V，单端输入范围
localparam RANGE_4  = 4'b0100; // ±12.5   V，单端输入范围
localparam RANGE_5  = 4'b0101; // 0 ~ 5   V，单端输入范围
localparam RANGE_6  = 4'b0110; // 0 ~ 10  V，单端输入范围
localparam RANGE_7  = 4'b0111; // 0 ~ 12.5V，单端输入范围
localparam RANGE_8  = 4'b1000; // ±5   V，差分输入范围
localparam RANGE_9  = 4'b1001; // ±10  V，差分输入范围
localparam RANGE_10 = 4'b1010; // ±12.5V，差分输入范围
localparam RANGE_11 = 4'b1011; // ±20  V，差分输入范围

// 采样率设置
localparam SAMPLE_CLK_1MHz   = 20'd1_000_000 ;
localparam SAMPLE_CLK_500kHz = 20'd500_000   ;
localparam SAMPLE_CLK_200kHz = 20'd200_000   ;
localparam SAMPLE_CLK_100kHz = 20'd100_000   ;
localparam SAMPLE_CLK_50kHz  = 20'd50_000    ;
localparam SAMPLE_CLK_20kHz  = 20'd20_000    ;
localparam SAMPLE_CLK_10kHz  = 20'd10_000    ;
localparam SAMPLE_CLK_5kHz   = 20'd5_000     ;
localparam SAMPLE_CLK_2kHz   = 20'd2_000     ;
localparam SAMPLE_CLK_1kHz   = 20'd1_000     ;
localparam SAMPLE_CLK_500Hz  = 20'd500       ;
localparam SAMPLE_CLK_200Hz  = 20'd200       ;
localparam SAMPLE_CLK_100Hz  = 20'd100       ;

AD7606C_MODE1 #(  // 软件模式并行
    .CH1_RANGE  ( RANGE_1         ), // 通道1默认配置±5 V输入
    .CH2_RANGE  ( RANGE_1         ), // 通道2默认配置±5 V输入
    .CH3_RANGE  ( RANGE_1         ), // 通道3默认配置±5V输入
    .CH4_RANGE  ( RANGE_3         ), // 通道4默认配置±10  V输入
    .CH5_RANGE  ( RANGE_8         ), // 通道5默认配置±5   V输入
    .CH6_RANGE  ( RANGE_9         ), // 通道6默认配置±10  V输入
    .CH7_RANGE  ( RANGE_10        ), // 通道7默认配置±12.5V输入
    .CH8_RANGE  ( RANGE_11        ), // 通道8默认配置±20  V输入
    .SAMPLE_CLK ( SAMPLE_CLK_1MHz )
)u_AD7606C_MODE1( // 软件模式
    .sys_clk    ( SYS_CLK         ),
    .sys_rst    ( SYS_RST         ),
    .DB0_15     ( DB0_15          ),
    .busy       ( BUSY            ),
    .Frstdata   ( FRSTDATA        ),
    .reset      ( RESET           ),
    .cs_n       ( CS              ),
    .rd_n       ( RD_SCLK         ),
    .wr_n       ( WR              ),
    .convst     ( CONVST          ),
    .ser_sel    ( SER_SEL         ),
    .OSC2_0     ( OSC2_0          ),
    .data_ch1   (                 ),
    .data_ch2   (                 ),
    .data_ch3   (                 ),
    .data_ch4   (                 ),
    .data_ch5   (                 ),
    .data_ch6   (                 ),
    .data_ch7   (                 ),
    .data_ch8   (                 ),
    .sample_out (                 )
); 


assign STBY = 1'b1;

endmodule

