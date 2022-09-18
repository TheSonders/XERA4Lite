`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// MIT License
// Copyright (c) 2022 Antonio Sánchez (@TheSonders)
//
// Part of the XERA4Lite Project
// Sept/2022
//
// SPI Controller for a Z80 Processor
//////////////////////////////////////////////////////////////////////////////////
module X4LSPI(
	input wire CLK,
	input wire ADD,
	input wire nRD,
	input wire nWR,
	inout wire [7:0]DATA,
	input wire nCS,
	output reg nWAIT=0,
	input wire SPI_MISO,
	output reg SPI_MOSI=0,
	output reg SPI_CLK=0,
	output reg SPI_CS=0);
	
	localparam Idle=0;
	localparam Last=17;
	localparam End=18;
	
	assign DATA=(nCS || nRD)?8'hZZ:Buffer;
	
	reg [$clog2(End):0]STM=0;
	reg prev_nRD=0;
	reg prev_nWR=0;
	reg [7:0]Buffer=0;
	
	always @(posedge CLK)begin
		prev_nRD<=nRD;
		prev_nWR<=nWR;
		case (STM)
			Idle:begin
				SPI_MOSI<=1;
				if (~nCS)begin
					if (prev_nRD && ~nRD)begin
						STM<=STM+1;
						SPI_CS<=0;
						nWAIT<=0;
					end
					else if (prev_nWR && ~nWR)begin
						STM<=STM+1;
						SPI_CS<=0;
						nWAIT<=0;
						Buffer<=DATA;
					end
				end
				else begin
					nWAIT<=1;
				end
			end
			Last:begin
				STM<=STM+1;
				SPI_CLK<=0;				
			end
			End:begin
				STM<=Idle;
				Buffer<={Buffer,SPI_MISO};
				SPI_CS<=ADD;
				nWAIT<=1;
			end
			default:begin
				STM<=STM+1;
				nWAIT<=0;
				if (STM[0])begin
					SPI_CLK<=0;
					SPI_MOSI<=Buffer[7];					
				end
				else begin
					SPI_CLK<=1;
					Buffer<={Buffer,SPI_MISO};
				end
			end
		endcase
	end
endmodule
