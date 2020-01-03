`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, branch, bne;
  wire        pcsrc, zero;
  wire        alusrc, regwrite;
  wire [1:0]  regdst, memtoreg, jump;
  wire [2:0]  alucontrol;

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
		.funct      (instr[5:0]), 
		.zero       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,					//1 if we signextend
                  output       shiftl16,					//1 if we shift left 16
                  output       memwrite,					//1 if we write to mem
                  output       pcsrc, alusrc,			//1 if we branch					//1 if we use immediate (0 if we use from reg file)
                  output       regwrite,					//1 if we write to register
						output [1:0] regdst,	memtoreg,		//10 if ra 01 if rd 00 if rt  //10 if we use PC+4, 01 if we use read data, 00 if we use alu operation result
                  output [1:0] jump,						//10 ir jr, 01 if jal or jump, 00 if no jump
                  output [2:0] alucontrol);

  wire [1:0] aluop;
  wire       branch;
  wire		 bne;
  wire 		 bne_true;
  wire		 branch_true;

  maindec md(
    .op       (op),
	 .funct	  (funct),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
	 .bne		  (bne),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol));

  
  assign bne_true = ~zero & bne;
  assign branch_true = branch & zero;
  assign pcsrc = bne_true | branch_true;
  
endmodule


module maindec(input  [5:0] op,
					input  [5:0] funct,
               output       signext,
               output       shiftl16,
               output       memwrite,
               output       branch, bne,
					output		 alusrc,
               output [1:0] regdst, memtoreg,
					output		 regwrite,
               output [1:0] jump,
               output [1:0] aluop);

  reg [14:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, bne, memwrite,
          memtoreg, jump, aluop} = controls;
			 
			 //aluop decides what to do in the alu
			 //alusrc is 1 if we use immediate and 0 if we use data from reg file
			 

  always @(*)
    case(op)
      6'b000000: case(funct)
			6'b001000: controls <= #`mydelay 15'b000000000001000; // JR
			default: controls   <= #`mydelay 15'b001010000000011; // Rtype
			endcase
      6'b100011: controls    <= #`mydelay 15'b101001000010000; // LW
      6'b101011: controls    <= #`mydelay 15'b100001001000000; // SW
      6'b000100: controls    <= #`mydelay 15'b100000100000001; // BEQ - signext and branch
		6'b000101: controls    <= #`mydelay 15'b100000010000001; // BNE - signext and bne
      6'b001000, 
      6'b001001: controls    <= #`mydelay 15'b101001000000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls    <= #`mydelay 15'b001001000000010; // ORI
      6'b001111: controls    <= #`mydelay 15'b011001000000000; // LUI
      6'b000010: controls    <= #`mydelay 15'b000000000000100; // J
		
		//the next thing to add is jal, jr
		6'b000011: controls    <= #`mydelay 15'b001100000100100; // jal - regwrite, regdst, memtoreg, jump
		
      default:   controls    <= #`mydelay 15'bxxxxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [2:0] alucontrol);

  always @(*)
    case(aluop)
      2'b00: alucontrol <= #`mydelay 3'b010;  // add
      2'b01: alucontrol <= #`mydelay 3'b110;  // sub
      2'b10: alucontrol <= #`mydelay 3'b001;  // or
      default: case(funct)          // RTYPE
          6'b100100: alucontrol <= #`mydelay 3'b000; // AND
          6'b100101: alucontrol <= #`mydelay 3'b001; // OR
			 6'b100000,
          6'b100001: alucontrol <= #`mydelay 3'b010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 3'b110; // SUB, SUBU: only difference is exception
          
          6'b101010,
			 6'b101011: alucontrol <= #`mydelay 3'b111; // SLT, SLTU
			 
          default:   alucontrol <= #`mydelay 3'bxxx; // ???
      endcase
    endcase
    
endmodule


module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         pcsrc,
                input         alusrc,
					 input  [1:0]  regdst, memtoreg,
                input         regwrite,
					 input  [1:0]  jump,
                input  [2:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb/*, jrloc*/;
  wire [31:0] result;
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(					//PC + 4 is calculated here
    .a (pc),
    .b (32'b100),
    .y (pcplus4));				//result is PC + 4

  sl2 immsh(						//immediate shift -> goes directly to pcbrmux
    .a (signimm),					//what we want to shift left 2
    .y (signimmsh));				//result of shift left 2
				 
  adder pcadd2(					//shift left 2 for branch
    .a (pcplus4),					//PC + 4
    .b (signimmsh),				//result of shift left 2
    .y (pcbranch));				//branch address

  mux2 #(32) pcbrmux(			//PC + 4 vs branch
    .d0  (pcplus4),				//PC + 4
    .d1  (pcbranch),				//
    .s   (pcsrc),					//
    .y   (pcnextbr));			//decide between PC + 4 and branch

  mux4 #(32) pcmux(				//10 jr, 01 jump, 00 branch
    .d0   (pcnextbr),			//Branch address
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),	//jump address
	 .d2   (srca),				   //PC counter saved in ra
	 .d3   (32'b0),				//no instruction yet
    .s    (jump),					//10 jr, 01 jump, 00 branch
    .y    (pcnext));				//next instruction to run

  // register file logic
  regfile rf(
    .clk     (clk),					//input clock
    .we      (regwrite),			//if 1 write to register
    .ra1     (instr[25:21]),		//input register 1
    .ra2     (instr[20:16]),		//input register 2
    .wa      (writereg),			//which register to write in (5 bits)
    .wd      (result),				//output result
    .rd1     (srca),					
    .rd2     (writedata));			
	 
	 //added ra as d2
  mux4 #(5) wrmux(					//determine where to write
    .d0  (instr[20:16]),			//rt
    .d1  (instr[15:11]),			//rd
	 .d2	(5'b11111),					//ra - used in jal and jr
	 .d3	(5'bxxxxx),					//null
    .s   (regdst),					//
    .y   (writereg));				//where to write
	 

  mux4 #(32) resmux(					//
    .d0 (aluout),						//result of alu
    .d1 (readdata),					//data read from memory
	 .d2 (pcplus4),					//PC + 4
	 .d3 (32'b0),
    .s  (memtoreg),					//00 if we save alu result, 01 if we save data read from memory, 10 if we save PC+4
    .y  (result));					//what we will save
	 
	 
	 
  sign_zero_ext sze(						//zero extend if 0, sign extend if 1
    .a       (instr[15:0]),			//immediate from instruction
    .signext (signext),					//zero extend if 0, sign extend if 1
    .y       (signimm[31:0]));		//result of extension

  shift_left_16 sl16(					
    .a         (signimm[31:0]),		//the input
    .shiftl16  (shiftl16),				//do nothing if 0, shift left 16 if 1
    .y         (shiftedimm[31:0]));	//result

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));

  alu alu(
    .a       (srca),						//thing to do sth with 1
    .b       (srcb),						//thing to do sth with 2
    .alucont (alucontrol),
    .result  (aluout),					//result
    .zero    (zero));					//1 if the result is zero
    
endmodule