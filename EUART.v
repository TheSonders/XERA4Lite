`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// MIT License
// Copyright (c) 2022 Antonio Sánchez (@TheSonders)
//Serial TX/RX module for Z80 Processor
//XERA4 Project
//
//___LINEAS____
//Transmitted Data		TD
//Received Data			RD	
//Data Terminal Ready	DTR	
//Data Set Ready		DSR	
//Request To Send		RTS	
//Clear To Send			CTS	
//Carrier Detect		DCD
//Ring Indicator		RI
//
//___REGISTROS___
//00	Baudrate Low
//01	Baudrate High
//02	Input Lines (Read Only)
//		[0] RD
//		[1] CTS
//		[2] DSR
//		[3] DCD
//		[4] RI
//03	Output Lines
//      [0] TD
//      [1] DTR
//      [2] RTS
//04	RX Data		(Read Only)
//05	TX Data
//06	Config
//		[0]Flow Control auto
//		[1]Polarity invert
//		[2]9 bits communication
//		[3]9th bit transmision
//07	Status		(Read Only)
//		[0]Data in available
//		[1]Data in overflow
//		[2]Data buffer out available
//		[3]9th bit reception
//		[4]Data in frame error
////////////////////////////////////
`define	BaudLow		3'h0
`define	BaudHigh	3'h1
`define	InputLines	3'h2
`define	OutputLines	3'h3
    `define bitRTS         register[3][2]
`define	RXData		3'h4
`define	TXData		3'h5
`define	Config		3'h6
    `define bitFlowAuto    register[6][0]
    `define bitPolarity    register[6][1]
    `define bitComm9b      register[6][2]
    `define bitTX9         register[6][3]
    `define bitEnable      register[6][4]
`define	Status		3'h7
    `define bitRXAvailable  register[7][0]
    `define bitRXOverflow   register[7][1]
    `define bitTXAvailable  register[7][2]
    `define bitRX9          register[7][3]
    `define bitFrameError   register[7][4]
    `define bitTXBusy       register[7][5]
    `define bitRXBusy       register[7][6]
`define	ST_IDLE		6'h00
`define	ST_LAST8b   6'h27
`define	ST_LAST9b   6'h2B

module EUART(
	input wire 		clk,_cs,_we,_re,_reset,
	input wire		[2:0] add,
	inout tri		[7:0] databus,
	input wire		rd,cts,dsr,dcd,ri,
	output wire		td,rts,dtr);
    
reg [7:0] register  [0:7];
integer x;
initial for (x=0;x<8;x=x+1) register[x]<=0;
reg [15:0] prescaler=0;
wire RX=rd^`bitPolarity;
assign rts=(`bitFlowAuto)?
        rRTS^`bitPolarity:
        `bitRTS;
wire CTS=cts^`bitPolarity;
reg prev_RX=0;
reg [5:0] rxm=`ST_IDLE;
reg [5:0] txm=`ST_IDLE;
reg [10:0] rxbuff=0;
reg [9:0] txbuff=0;
reg rRTS=0;

//Buffers
reg RXfifo_re=0;
reg RXfifo_we=0;
wire [9:0] RXfifo_dataout;
wire RXfifo_empty,RXfifo_full;
reg TXfifo_re=0;
reg TXfifo_we=0;
wire [9:0] TXfifo_dataout;
wire TXfifo_empty;
wire [9:0]TXfifo_datain=(`bitComm9b)?
        {1'b0,register[`TXData],`bitTX9}:
        {1'b0,register[`TXData],1'b1};
assign databus= (_cs || _re || ~_we)? 8'HZZ:register[add];
assign td=(`bitTXBusy)?txbuff[9]^`bitPolarity:
            ~`bitPolarity;

always @(posedge clk) begin
    //RESET
    if (~_reset) begin
        for (x=0;x<8;x=x+1)register[x]<=0;
    end
    //REGISTERS READ/WRITE
	if (~_cs) begin
		if (~_we) begin
			if (add!=`InputLines && add!=`RXData && add!=`Status)
                register[add]<=databus;
            if (add==`TXData && ~TXfifo_full) TXfifo_we<=1;
		end
        else if (~_re)begin
            if (add==`RXData) begin 
                RXfifo_re<=1;
                register[`RXData]<=RXfifo_dataout[9:2];
                `bitRX9<=RXfifo_dataout[1];
                `bitFrameError<=~RXfifo_dataout[0];
                `bitRXOverflow<=0;
            end
        end
	end
    //READ-ONLY REGISTERS UPDATE
    register[`InputLines]<={3'h0,ri,dcd,dsr,cts,rd};
    `bitRXAvailable<=~RXfifo_empty;
    `bitTXAvailable<=~TXfifo_full;
    if (TXfifo_we)TXfifo_we<=0;
    if (TXfifo_re)TXfifo_re<=0;
    //RX / TX
    if (`bitEnable) begin
        if (prescaler) prescaler<=prescaler-1;
        else begin
            prescaler<={register[`BaudHigh],register[`BaudLow]};
            prev_RX<=RX;
            //RECEPTION
            if (rxm==`ST_IDLE) begin
                if (prev_RX & ~RX) begin
                    RXfifo_we<=0;
                    `bitRXBusy<=1'h1;
                end       
            end
            rxm<=rxm+`bitRXBusy;
            if (rxm[1:0]==2'h1 & `bitRXBusy) rxbuff<={rxbuff[9:0],RX};
            if ((rxm==`ST_LAST9b && `bitComm9b)||
                (rxm==`ST_LAST8b && ~`bitComm9b))begin
                `bitRXBusy<=1'h0;
                rxm<=`ST_IDLE;
                if (RXfifo_full) `bitRXOverflow<=1;
                else RXfifo_we<=1;
            end 
            //TRANSMISION
            if (~`bitTXBusy) begin
                if (`bitFlowAuto)rRTS<=~TXfifo_empty;
                else rRTS<=1'b0;
            end          
            if (~`bitTXBusy && ~TXfifo_empty) begin
                if (~`bitFlowAuto || CTS) begin
                    txbuff<=TXfifo_dataout;
                    TXfifo_re<=1;
                    `bitTXBusy<=1'h1;
                end
            end
            txm<=txm+`bitTXBusy;
            if (txm[1:0]==2'h3 & `bitTXBusy) txbuff<={txbuff[9:0],1'h1};
            if ((txm==`ST_LAST9b && `bitComm9b)||
                (txm==`ST_LAST8b && ~`bitComm9b))begin
                `bitTXBusy<=1'h0;
                txm<=`ST_IDLE;
            end       
        end  //prescaler
    end //(`bitEnable)
end

fifo_pointer 
    #(.WIDTH(10),.DEPTH(8))
    RXfifo(
    .datain(rxbuff),
    .dataout(RXfifo_dataout),
    .we(RXfifo_we),
    .re(RXfifo_re),
    .clk(clk),
    ._reset(_reset),
    .full(RXfifo_full), 
    .empty(RXfifo_empty)) ;
    
fifo_pointer 
    #(.WIDTH(10),.DEPTH(8))
    TXfifo(
    .datain(TXfifo_datain),
    .dataout(TXfifo_dataout),
    .we(TXfifo_we),
    .re(TXfifo_re),
    .clk(clk),
    ._reset(_reset),
    .full(TXfifo_full),
    .empty(TXfifo_empty)) ;
endmodule


module fifo_pointer 
    #(parameter WIDTH=9,
	parameter DEPTH= 8)(
  input wire [WIDTH-1:0] datain,
  output wire [WIDTH-1:0] dataout,
  input wire we, re, clk, _reset,
  output wire full, empty) ;

  reg [WIDTH-1:0] buff [0:DEPTH-1];
  reg [$clog2(DEPTH):0] counter=0;
  reg [$clog2(DEPTH)-1:0] w_pointer=0;
  reg [$clog2(DEPTH)-1:0] r_pointer=0;
  reg pwe=0,pre=0;
  
  assign dataout=buff[r_pointer];
  assign empty=(counter==0);
  assign full=(counter[$clog2(DEPTH)]);
  
  always @(posedge clk) begin
    pwe<=we;pre<=re;
    if (~_reset) begin
        counter<=0;
        w_pointer<=0;
        r_pointer<=0;
    end
    else begin
        if (we && ~pwe && ~full) begin
            buff[w_pointer] <= datain;
            w_pointer<=w_pointer+1;
            if (~re)counter<=counter+1;
        end
        if(re && ~pre && ~empty) begin
            r_pointer<=r_pointer+1;
            if (~we)counter<=counter-1;
        end
    end
  end
endmodule

