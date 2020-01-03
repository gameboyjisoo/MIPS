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

  wire        pcsrc, zero;
  // changed so that control signals are 1-bit to avoid confusion
  wire        signext, shiftl16, memtoreg, memread;
  wire        alusrc, regdst, regwrite, jump, jr;
  //I changed the control signals from M3 because they lacked intuition.
  //I could use a single control signal instead of extending both jump and memtoreg
  
  wire [3:0]  alucontrol;           // extended to 4 bits because I realized sltu was different from slt
  wire [31:0] ifid_instr;				// used in datapath, controller
  wire        idex_memread;			
  wire        exmem_regwrite;
  wire        memwr_regwrite;
  wire        stall;  // hazard detection unit control

  //##### Jisoo Kim: Start #####
  wire        memwr_jump, memwr_jr;	// jump and jr are needed in the writeback stage so that we check where and whether to jump or not
  // used in hazard detection unit to clear inputs whenever we branch or jump
  // we "kill" the instructions 
  wire        flush;
  //##### Jisoo Kim: End #####
  
  // Instantiate Controller
  controller c(
		.zero         (zero),
		.signext      (signext),
		.shiftl16     (shiftl16),
		.memtoreg     (memtoreg),
		.memwrite     (memwrite),
		.pcsrc        (pcsrc),
		.alusrc       (alusrc),
		.regdst       (regdst),
		.regwrite     (regwrite),
		.jump         (jump),
		.jr           (jr),
		.alucontrol   (alucontrol),
		.clk            (clk),            
      .reset          (reset),
		.stall          (stall),
      .op             (ifid_instr[31:26]), //op and function must come from the flipflop
		.funct          (ifid_instr[5:0]), 	 //op and function must come from the flipflop
      .memread        (memread),
      .exmem_regwrite (exmem_regwrite),     
      .memwr_regwrite (memwr_regwrite),     
      .idex_memread   (idex_memread),
		
		//##### Jisoo Kim: Start #####
      .memwr_jump     (memwr_jump),			 //determines what we write to registers and which register we write to
      .memwr_jr       (memwr_jr),			 //determines what we write to registers and which register we write to
		.flush          (flush));				 //sent to controller from hazard detection unit in the datapath
		//##### Jisoo Kim: End #####

  // Instantiate Datapath
  datapath dp(
    .clk             (clk),
    .reset           (reset),
    .signext         (signext),
    .shiftl16        (shiftl16),
    .memtoreg        (memtoreg),
    .pcsrc           (pcsrc),
    .alusrc          (alusrc),
    .regdst          (regdst),
    .regwrite        (regwrite),
    .jump            (jump),
    .alucontrol      (alucontrol),
    .zero            (zero),
    .pc              (pc),
    .instr           (instr),
    .aluout          (memaddr), 
    .writedata       (memwritedata),
    .readdata        (memreaddata),
    .ifid_instr      (ifid_instr),			 //output to controller, needed for pipelining
	 .idex_memread    (idex_memread),
    .exmem_regwrite  (exmem_regwrite),
    .memread         (memread),
	 .memwr_regwrite  (memwr_regwrite),
	 .jr              (jr),
    .stall           (stall),
	 
	 //##### Jisoo Kim: Start #####
	 .memwr_jump      (memwr_jump),
    .memwr_jr        (memwr_jr),
	 .flush           (flush));
	 //##### Jisoo Kim: End #####

endmodule


module controller(input  [5:0] op, funct, // these now come from the ifid stage
                  input        zero,
                  output       signext,   // 1 if we signextend
                  output       shiftl16,  // 1 if we shift left 16
                  output       pcsrc, alusrc,
						output		 regwrite,  // 1 if we write to register
                  output       regdst,		// 1 if rd, 0 if rt
                  // need control signals and clk for pipelining
                  input        clk,
                  input        reset,
						input        stall,
						output       memtoreg, memread, memwrite,
						output       jump, jr,
                  output [3:0] alucontrol,
                  output       idex_memread,
                  output       exmem_regwrite,
                  output       memwr_regwrite,
						
						// #####    Jisoo Kim: Start    #####						
						input        flush,			// control signal created in hazard detection unit
						output       memwr_jump,   // control signal used in hazard detection unit
						output       memwr_jr);    // control signal used in hazard detection unit
                  // #####    Jisoo Kim: End    #####

  // #####    Jisoo Kim: Start    #####
  wire [12:0] ifid_control;			// control signals initially created
  wire [12:0] ifid_control_output;	// control signals after stall check + flush check
  // #####    Jisoo Kim: End    #####
  
  // ifid control signals used in execution stage
  wire [2:0] ifid_aluop;      
  wire       ifid_alusrc, ifid_regdst, ifid_shiftl16;
  // #####    Jisoo Kim: Start    #####
  wire       ifid_jump;
  // #####    Jisoo Kim: End    #####
  
  // ifid control signals sent to memory access stage
  wire       ifid_branch, ifid_bnebeq;
  wire       ifid_memread, ifid_memwrite;
  
  // ifid control signals sent to writeback stage
  wire       ifid_memtoreg, ifid_regwrite;
  
  // idex control signals
  wire [5:0] idex_funct;
  wire [2:0] idex_aluop;
  wire       idex_alusrc, idex_jr, idex_regdst, idex_shiftl16;
  // #####    Jisoo Kim: Start    #####
  wire		 idex_jump;						// control signal used in hazard detection unit
  wire		 idex_branch, idex_bnebeq /*, idex_beq, idex_bne*/;	//signals used to determine branch
  // #####    Jisoo Kim: End    #####
  
  // exmem control signals
  wire       exmem_memread, exmem_memwrite;	// used in current stage
  // #####    Jisoo Kim: Start    #####
  //wire       exmem_branch, exmem_bnebeq;						// used in current stage
  wire       /*exmem_zero,*/ exmem_jump, exmem_jr;				// used in current stage - exmem_jump added
  wire [1:0] exmem_control;	// control signals sent to memory access stage - updated in flipflop
  // fewer control signals are sent because we decide to flush or not at the execution stage
  // #####    Jisoo Kim: End    #####
  wire [1:0] exwr_control;		// control signals sent to writeback stage (through memory access stage) - updated in flipflop
  
  // memwr control signals
  wire       memwr_memtoreg;
  wire [1:0] memwr_control;		// control signals sent to writeback stage - updated in flipflop
  
  // constantly update all the control signals
  // #####    Jisoo Kim: Start    #####
  // ifid stage updates
  // added jump because 
  assign ifid_control = {ifid_regdst, ifid_aluop, ifid_alusrc, ifid_shiftl16, ifid_jump,
                         ifid_branch, ifid_bnebeq, ifid_memread, ifid_memwrite,
								 ifid_memtoreg, ifid_regwrite};
  // #####    Jisoo Kim: End    #####
  // execution stage updates
  assign regdst = idex_regdst;
  assign alusrc = idex_alusrc;
  assign shiftl16 = idex_shiftl16;
  // #####    Jisoo Kim: Start    #####
  //erroneously tried to simplify logic in M4, only didn't show because M4 didn't have any branching instructions
  assign pcsrc = (idex_bnebeq & ~zero) | (idex_branch & zero);
  assign jump = idex_jump;
  assign jr = /*exmem_jr*/idex_jr;				// we check the control signal at the execution stage now
  // #####    Jisoo Kim: End    #####
  
  // memory stage updates
  assign memread = exmem_memread;
  assign memwrite = exmem_memwrite;
  assign idex_memread = exmem_control[1];
  
  // writeback stage updates
  assign memtoreg = memwr_memtoreg;
  assign regwrite = memwr_regwrite;
  assign exmem_regwrite = memwr_control[0];
  
  maindec md(
    .op         (op),				
    .signext    (signext), 		// because we don't need to use this control in our pipelines - the only one?
	 // #####    Jisoo Kim: Start    #####
	 .jump       (/*jump*/ifid_jump),	// jump is pipelined as well now
	 // #####    Jisoo Kim: End    #####
    .alusrc     (ifid_alusrc),	
    .branch     (ifid_branch),	
    .bnebeq     (ifid_bnebeq),	
    .memtoreg   (ifid_memtoreg),	
    .memread    (ifid_memread),	
    .memwrite   (ifid_memwrite),	
    .regdst     (ifid_regdst),	
    .regwrite   (ifid_regwrite),	
	 .shiftl16   (ifid_shiftl16),	
    .aluop      (ifid_aluop));	
	 
  aludec ad( 
    .funct      (idex_funct),
    .aluop      (idex_aluop), 
    .alucontrol (alucontrol),
    .jr         (idex_jr));
	   
// unless we stall, we pass the control signals and other necessary bits to the next stage
  
  // #####    Jisoo Kim: Start    #####
  // because we now need to pipeline the jump control as well, we increase the hazardmux to 13 bits
  mux2 #(13) hazardmux (
    .d0 (ifid_control),
    .d1 (13'b0),
    .s  (stall | flush),			// must include flush as well
	 // #####    Jisoo Kim: Start    #####
    .y  (ifid_control_output));
// hazard detection signal is used in ifid because we need to implement nop at this stage (no branch, jump hazard yet)

// flipflips using the clock signal
// the length of the input decreases beccause we use fewer control signals as time progresses
  // #####    Jisoo Kim: Start    #####
  flopr #(19) idexflop (
  // #####    Jisoo Kim: End    #####
    .clk     (clk),
    .reset   (reset),
    .d       ({ifid_control_output, funct}),
	 // #####    Jisoo Kim: Start    #####
    .q       ({idex_regdst, idex_aluop, idex_alusrc, idex_shiftl16, idex_jump, idex_branch, idex_bnebeq, exmem_control, exwr_control, idex_funct}));
	 // included the jump control because we need to pass it now
	 // #####    Jisoo Kim: End    #####
// the ones used here are not passed to the next stage of the pipeline

// idex_funct and aluop are used for aludec, which outputs alu control signals
// shiftleft16 is used to shift the sign/zero extended immediate
// alusrc is used to determine what the second input to the alu is
// the rest are passed forward
  
  // #####    Jisoo Kim: Start    #####
  flopr #(6) exmemflop (
  // #####    Jisoo Kim: End    #####
    .clk     (clk),
    .reset   (reset),
	 // #####    Jisoo Kim: Start    #####
    .d       ({exmem_control, exwr_control, /*zero*/idex_jump, idex_jr}),
    .q       ({/*exmem_branch, exmem_bnebeq, */exmem_memread, exmem_memwrite, memwr_control, /*exmem_zero*/ exmem_jump, exmem_jr}));
	 // less signals are sent forward because we determine branching in the execution signal.
	 // jumping is needed
	 // #####    Jisoo Kim: End    #####
	 
// branch, bnebeq, zero are needed for branch and jump decisions.
// jr is needed for jr instructions (choosing which register to use)
// memread and memwrite are needed to access memory
// the rest are passed forward

  // #####    Jisoo Kim: Start    #####
  flopr #(4) memwrflop (
  // #####    Jisoo Kim: End    #####
    .clk     (clk),
    .reset   (reset),
	 // #####    Jisoo Kim: Start    #####
    .d       ({memwr_control, exmem_jump, exmem_jr}), //added exmem_jump, exmem_jr
    .q       ({memwr_memtoreg, memwr_regwrite, memwr_jump, memwr_jr}));
	 // jump and jr are needed to determine the final register we write to
	 // #####    Jisoo Kim: End    #####
	 
// regwrite determines whether we write to reg or not
// memtoreg determines what the writedata becomes
endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
					output       regdst, regwrite,
               output       jump,
               output       memtoreg, memread, memwrite,
               output       branch, bnebeq, alusrc,
					output [2:0] aluop);

  reg [13:0] controls;
  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, bnebeq, memwrite, memread,
          memtoreg, jump, aluop} = controls;
			 
			 //aluop decides what to do in the alu
			 //alusrc is 1 if we use immediate and 0 if we use data from reg file

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 14'b00110000000111; // Rtype
      6'b100011: controls <= #`mydelay 14'b10101000110000; // LW
      6'b101011: controls <= #`mydelay 14'b10001001000000; // SW
      6'b000100: controls <= #`mydelay 14'b10000100000001; // BEQ
      6'b000101: controls <= #`mydelay 14'b10000010000001; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 14'b10101000000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 14'b00101000000010; // ORI
      6'b001111: controls <= #`mydelay 14'b01101000000000; // LUI
      6'b000010: controls <= #`mydelay 14'b00000000001000; // J
      6'b000011: controls <= #`mydelay 14'b00100000001000; // JAL
		// #####    Jisoo Kim: Start    #####
		// regwrite because we write result of slt to a register, alusrc because we use the immediate 
		6'b001010: controls <= #`mydelay 14'b10101000000011; // SLTI		// signext, regwrite, alusrc, aluop 011
		// #####    Jisoo Kim: End    #####
      default:   controls <= #`mydelay 14'bxxxxxxxxxxxxxx; // ???
      // removed case statement for the r-type and moved the jr control signal to be decided in aludec.
		// this reduces the number of bits transferred from the id/ex flipflop, increasing efficiency
    endcase

endmodule

module aludec(input      [5:0] funct,	//received at idex stage
              input      [2:0] aluop,	//received at idex stage
              output reg [3:0] alucontrol,
              output reg       jr);
  always @(*)
    begin
      jr <= #`mydelay 1'b0;
      case(aluop)
        3'b000: alucontrol <= #`mydelay 4'b0010;  // add
        3'b001: alucontrol <= #`mydelay 4'b0110;  // sub
        3'b010: alucontrol <= #`mydelay 4'b0001;  // or
		  // #####    Jisoo Kim: Start    #####
		  // we need to add these instructions because slti is not an r-type instruction (some code redundancy)
		  3'b011: alucontrol <= #`mydelay 4'b0111;  // slti
		  // #####    Jisoo Kim: End    #####
        default: case(funct)                      // RTYPE
            6'b001000:
              begin
                jr <= #`mydelay 1'b1;
                alucontrol <= #`mydelay 4'b0010;        // JR
              end
            6'b100000,
            6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
            6'b100010,
            6'b100011: alucontrol <= #`mydelay 4'b0110; // SUB, SUBU: only difference is exception
            6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
            6'b100101: alucontrol <= #`mydelay 4'b0001; // OR
            6'b101010: alucontrol <= #`mydelay 4'b0111; // SLT
            6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU
            default:   alucontrol <= #`mydelay 4'bxxxx; // ???
          endcase
      endcase
    end
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,	
					 input  [31:0] instr,				
                input  [31:0] readdata,			
					 input			jr,					
                input  [3:0]  alucontrol,			
					 input         idex_memread,
					 input         exmem_regwrite,
					 input         memwr_regwrite,
					 input         memread,
					 output        zero,					
                output [31:0] pc,					
                output [31:0] aluout, writedata,
					 output        stall,
                output [31:0] ifid_instr,
					 
					 // #####    Jisoo Kim: Start    #####
                input         memwr_jump,
                input         memwr_jr,
					 output        flush);

  wire [31:0] if_instr;					// when flush is true, we stop the next instruction
  wire [31:0] pcnext, /*pcnextbr*/ idex_pcnextbr, pcplus4, /*pcbranch*/ idex_pcbranch;
  wire [31:0] final_pcnext;
  // #####    Jisoo Kim: End    #####
  
  wire [31:0] pcnext_candidate;		// for branch and jump
  
  wire [4:0]  ifid_rd;
  wire [4:0]  ifid_rs, ifid_rt;		// for forwarding unit
  wire [31:0] ifid_signimm, ifid_writedata, ifid_srca;
  // #####    Jisoo Kim: Start    #####
  wire [31:0] ifid_pcplus4;
  wire [31:0] ifid_instr_output;
  // #####    Jisoo Kim: End    #####
  
  wire [4:0]  idex_rs, idex_rt;		// for forwarding unit
  wire [31:0] idex_srca, idex_srcb;
  wire [31:0] idex_writedata, idex_aluout;
  wire [31:0] idex_signimm, idex_signimmsh, idex_shiftedimm;	// for alu srcb
  wire [4:0]  idex_rd, idex_writereg;
  // #####    Jisoo Kim: Start    #####
  wire [31:0] idex_pcplus4;
  wire [31:0] idex_instr;
  // #####    Jisoo Kim: End    #####
  
  wire [31:0] exmem_aluout, exmem_writedata;
  // #####    Jisoo Kim: Start    #####
  wire [31:0] exmem_pcplus4;
  wire [4:0]  /*exmem_rt, */exmem_writereg;	// for forwarding unit - now we need to check rt at idex stage
  // #####    Jisoo Kim: End    #####
  
  wire [31:0] memwr_readdata, memwr_aluout;
  wire [4:0]  memwr_writereg;					// for forwarding unit
  // #####    Jisoo Kim: Start    #####
  wire [31:0] memwr_pcplus4;
  wire        memwr_write;
  // #####    Jisoo Kim: End    #####
  
  wire [31:0] result;
  wire [31:0] final_wd;												// final data to write
  wire [4:0]  final_wa;												// final address to write at
  wire [31:0] ifid_forwardA, ifid_forwardB;	// srca and writedata potential candidates (compared with data from writeback stage)
  wire [31:0] forwardA_candidate, forwardB_candidate;   // forwardA at ifid stage (potentially used in alu)
  wire [31:0] final_forwardA, final_forwardB;				// final results of the forwarding unit muxes (used in alu)
  wire [2:0]  forwardcontrol_a, forwardcontrol_b; // forwarding unit control
  
  // continuously update these values
  assign ifid_rs = ifid_instr[25:21];
  assign ifid_rt = ifid_instr[20:16];
  assign ifid_rd = ifid_instr[15:11];
  assign aluout = exmem_aluout;
  assign writedata = exmem_writedata;
  // #####    Jisoo Kim: Start    #####
  assign memwr_write = memwr_jump & ~memwr_jr;	// this determines what we write to registers and which register we write to
  // #####    Jisoo Kim: End    #####

  forwarding_unit forward (
    .ifid_rs           (ifid_rs),
    .ifid_rt           (ifid_rt),
    .idex_rs           (idex_rs),
    .idex_rt           (idex_rt),
    .exmem_writereg    (exmem_writereg),
	 .exmem_regwrite    (exmem_regwrite),
    .memwr_writereg    (memwr_writereg),
	 .memwr_regwrite    (memwr_regwrite),
    .forwardcontrol_a  (forwardcontrol_a),
    .forwardcontrol_b  (forwardcontrol_b));
	
  hazard_detector hazard (
    .reset         (reset),			
	 .ifid_rs       (ifid_rs),			
    .ifid_rt       (ifid_rt),			
	 .idex_rt       (idex_rt),			
    .idex_memread  (idex_memread),	
	 //.exmem_memread (memread),
	 //.exmem_rt		 (exmem_rt),
	 .stall         (stall),
	 // #####    Jisoo Kim: Start    #####
	 .idex_pcsrc    (pcsrc),
    .idex_jump     (jump),
    .idex_jr       (jr),
    .flush         (flush));			
	 // #####    Jisoo Kim: End    #####
	
  // next PC logic - changed from flopr to flopenr because it needs to be stalled at times
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .en    (~stall),				// if stall is true, then we must disable it
	 // #####    Jisoo Kim: Start    #####
    .d     (final_pcnext),
	 // #####    Jisoo Kim: End    #####
    .q     (pc));

  adder pcadd1(				//PC + 4 is calculated here
    .a (pc),
    .b (32'b100),
    .y (pcplus4));			//result is PC + 4

  sl2 immsh(					// shift left 2 for branch
    .a (idex_signimm),
    .y (idex_signimmsh));

  adder pcadd2(				// calculate branch address
    // #####    Jisoo Kim: Start    #####
    .a (idex_pcplus4),
    // #####    Jisoo Kim: End    #####
    .b (idex_signimmsh),	// result of shift left 2
    // #####    Jisoo Kim: Start    #####
    .y (idex_pcbranch));	// branch address
	 // #####    Jisoo Kim: End    #####
	 //pcbranch is determined based on the signimmsh that is defined in the idex flopenr

  // even though it seems none of the values have changed, the inputs into pcbranch and pcsrc have changed according to the pipeline
  // 
  mux2 #(32) pcbrmux(		// PC + 4 vs branch
  // #####    Jisoo Kim: Start    #####
    .d0  (idex_pcplus4),			// PC + 4
    .d1  (idex_pcbranch),			// branch address
	 // #####    Jisoo Kim: End    #####
    .s   (pcsrc),
	 // #####    Jisoo Kim: Start    #####
    .y   (idex_pcnextbr));		// decide between PC + 4 and branch
	 // #####    Jisoo Kim: End    #####

  // mux4 used in M3 was removed because it had 1 unused input
  mux2 #(32) pcmux(
    // #####    Jisoo Kim: Start    #####
	 .d0   (idex_pcnextbr),		// Branch address (or just pcplus4)
    .d1   ({idex_pcplus4[31:28], idex_instr[25:0], 2'b00}),//jump address
	 // #####    Jisoo Kim: End    #####
    .s    (jump),
    .y    (pcnext_candidate));
    
  // because the mux4 was removed, I used another mux2 that receives the result from pcmux.
  // thus, even though I use 2 mux2s, there are only 3 inputs rather than 4
  mux2 #(32) jrmux(
    .d0  (pcnext_candidate),
	 // #####    Jisoo Kim: Start    #####
    .d1  (idex_aluout),		// because the jr is determined at the execution stage, we use aluout from that stage as well
	 // #####    Jisoo Kim: End    #####
    .s   (jr),
    .y   (pcnext));			
  
  // #####    Jisoo Kim: Start    #####
  mux2 #(32) pcflushmux (
    .d0 (pcplus4),
    .d1 (pcnext),
    .s  (flush),				// flush is true when we branch or jump, thus we use pcnext instead of pcplus4 when flush is true
    .y  (final_pcnext));	// next instruction to run
	 // #####    Jisoo Kim: End    #####

  // register file logic
  regfile rf(
    .clk     (clk),				// input clock
    .we      (regwrite),		// regwrite. if 1 write to register
    .ra1     (ifid_rs),			// input register 1
    .ra2     (ifid_rt),			// input register 2
    .wa      (final_wa),		// which register to write in (5 bits)
    .wd      (final_wd),		// data we want to write
    .rd1     (ifid_srca),
    .rd2     (ifid_writedata));

  // as aforementioned, mux4 was disabled because of the unnecessary and nonexistent 4th input.
  mux2 #(5) wrmux(				// determine where to write
	 .d0  (idex_rt),				// rt
    .d1  (idex_rd),				// rd
    .s   (regdst),
    .y   (idex_writereg));		// reg we want to write to at this stage
  
  // another reason why the mux4 could not be used is because the writereg needed is different.
  // wrmux requires the writereg from idex, while the final write address requires memwr_writereg
  mux2 #(5) regmux(
    .d0  (memwr_writereg),
    .d1  (5'b11111),
	 // #####    Jisoo Kim: Start    #####
    .s   (memwr_write),					// need both signals because jr is an r-type command with control signal regwrite
												// without it, the jr command writes to the register
	 // #####    Jisoo Kim: End    #####
    .y   (final_wa));
  
  // the next two muxes are also from my original mux4 resmux
  mux2 #(32) resmux(
    .d0 (memwr_aluout),
    .d1 (memwr_readdata),
    .s  (memtoreg),
    .y  (result));

  mux2 #(32) writedatamux(
    .d0 (result),
	 // #####    Jisoo Kim: Start    #####	 
    .d1 (memwr_pcplus4),				// we use memwr_pcplus4 at this stage because this pipelined pcplus4 is what we finally write
    .s  (memwr_write),					// the control signal for regmux and writedatamux are the same, because if we write
												// to register 31, this is a jal, so we have to write memwr_pcplus4.
	 // #####    Jisoo Kim: End    #####
    .y  (final_wd));

  sign_zero_ext sze(						// zero extend if 0, sign extend if 1
    .a       (ifid_instr[15:0]),		// immediate from instruction
    .signext (signext),			
    .y       (ifid_signimm[31:0]));	// result of extension

  shift_left_16 sl16(
    .a         (idex_signimm),				// the input
    .shiftl16  (shiftl16),						// do nothing if 0, shift left 16 if 1
    .y         (idex_shiftedimm[31:0]));	// result

// the following muxes are used to determine what will be used in the alu
  // in this mux, we check if the input to the alu is precalculated at the memory access stage
  mux2 #(32) aluresultmem_forwarda (
    .d0 (idex_srca),
    .d1 (exmem_aluout),
    .s  (forwardcontrol_a[1]),
    .y  (forwardA_candidate));

  //in this mux, we check if the input to the alu is precalculated at the writeback stage
  mux2 #(32) aluresultwr_forwarda (
    .d0 (forwardA_candidate),
    .d1 (result),
    .s  (forwardcontrol_a[0]),
    .y  (final_forwardA));
	
  // in this mux, we check if the input to the alu is precalculated at the memory access stage
  mux2 #(32) aluresultmem_forwardb (
    .d0 (idex_writedata),
    .d1 (exmem_aluout),
    .s  (forwardcontrol_b[1]),
    .y  (forwardB_candidate));
	 
  //in this mux, we check if the input to the alu is precalculated at the writeback stage
  mux2 #(32) aluresultwr_forwardb (
    .d0 (forwardB_candidate),
    .d1 (result),
    .s  (forwardcontrol_b[0]),
    .y  (final_forwardB));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (final_forwardB),						// the final result of the forwarding process
    .d1 (idex_shiftedimm[31:0]),				// the "new input"
    .s  (alusrc),
    .y  (idex_srcb));
	 
  alu alu(
    .a       (final_forwardA),				// the final result of the forwarding process
    .b       (idex_srcb),
    .alucont (alucontrol),
    .result  (idex_aluout),
    .zero    (zero));
  
  // in this mux, we check if we plan to write somthing to the register we are getting information from
  mux2 #(32) srca_forwarda (
    .d0 (ifid_srca),
    .d1 (result),
    .s  (forwardcontrol_a[2]),
    .y  (ifid_forwardA));		//saved to idex_srca

  // in this mux, we check if we plan to write somthing to the register we are getting information from
  mux2 #(32) writereg_forwardb (
    .d0 (ifid_writedata),
    .d1 (result),
    .s  (forwardcontrol_b[2]),
    .y  (ifid_forwardB));		//saved to idex_writedata
  
  // pipeline logic is implemented using floprs, but if_id is made using a flopenr because we need to stall for load word.
  
  // #####    Jisoo Kim: Start    #####
  // 2 muxes were added so that we check flushing. One is before the pipelining process, and one is for the idix pipeline.
  // These muxes are used to stop the feeding of instructions to the next stage.
  mux2 #(32) ifmux (
    .d0 (instr),
    .d1 (32'b0),
    .s  (flush),
    .y  (if_instr));

  mux2 #(32) idmux (
    .d0 (ifid_instr),
    .d1 (32'b0),
    .s  (flush),
    .y  (ifid_instr_output));
  // #####    Jisoo Kim: End    #####
  
  // #####    Jisoo Kim: Start    #####
  // we now input instructions and pcplus4 to the pipeline, and pcplus4 is moved through the whole pipeline.
  // outside of that, adjustments to the number of control signals passed at each stage was also made
  flopenr #(64) ifid_pipeline (
    .clk    (clk),
    .reset  (reset),
    .en     (~stall),				// if stall is true, then we must disable it
    .d      ({if_instr, pcplus4}),
    .q      ({ifid_instr, ifid_pcplus4}));

  flopr #(175) idex_pipeline (
    .clk    (clk),
    .reset  (reset),
    .d      ({ifid_instr_output, ifid_pcplus4, ifid_forwardA, ifid_forwardB, ifid_signimm, ifid_rs, ifid_rt, ifid_rd}),
    .q      ({idex_instr,        idex_pcplus4, idex_srca, idex_writedata, idex_signimm, idex_rs, idex_rt, idex_rd}));
	 // added instructions and pcplus4
  
  flopr #(101) exmem_pipeline (
    .clk    (clk),
    .reset  (reset),
    .d      ({idex_pcplus4, idex_aluout, final_forwardB, idex_writereg}),
    .q      ({exmem_pcplus4, exmem_aluout, exmem_writedata, exmem_writereg}));
	 // added pcplus4, but removed exmem_rt because we now determine hazards at the execution stage.
	 // thus, the hazard detection unit no longer receives exmem_rt as an input

  flopr #(101) memwr_pipeline (
    .clk    (clk),
    .reset  (reset),
    .d      ({exmem_pcplus4, readdata, exmem_aluout, exmem_writereg}),
    .q      ({memwr_pcplus4, memwr_readdata, memwr_aluout, memwr_writereg}));
	 // pcplus4 was added
  // #####    Jisoo Kim: End    #####
endmodule

module forwarding_unit(input      [4:0] ifid_rs, ifid_rt, 
							  input      [4:0] idex_rs, idex_rt, 
							  input      [4:0] exmem_writereg, 
							  input            exmem_regwrite,
							  input      [4:0] memwr_writereg,
							  input            memwr_regwrite,
							  output reg [2:0] forwardcontrol_a,
							  output reg [2:0] forwardcontrol_b);
// is mydelay needed?
  always @ (*)
  begin
    if (memwr_regwrite & (ifid_rs != 0) & (memwr_writereg == ifid_rs))
	   forwardcontrol_a[2] <= #`mydelay 1'b1;
    else
	   forwardcontrol_a[2] <= #`mydelay 1'b0;
		
    if (memwr_regwrite & (ifid_rt != 0) & (memwr_writereg == ifid_rt))
	   forwardcontrol_b[2] <= #`mydelay 1'b1;
	 else 
	   forwardcontrol_b[2] <= #`mydelay 1'b0;
	  
	 if (exmem_regwrite & (idex_rs != 0) & (exmem_writereg == idex_rs))
	   forwardcontrol_a[1:0] <= #`mydelay 2'b10;
	 else if (memwr_regwrite & (idex_rs != 0) & (memwr_writereg == idex_rs))
	   forwardcontrol_a[1:0] <= #`mydelay 2'b01;
	 else
	   forwardcontrol_a[1:0] <= #`mydelay 2'b00;
	 
	 if (exmem_regwrite & (idex_rt != 0) & (exmem_writereg == idex_rt))
	   forwardcontrol_b[1:0] <= #`mydelay 2'b10;
	 else if (memwr_regwrite & (idex_rt != 0) & (memwr_writereg == idex_rt))
	   forwardcontrol_b[1:0] <= #`mydelay 2'b01;
	 else
	   forwardcontrol_b[1:0] <= #`mydelay 2'b00;
  end

  endmodule


// hazard detection checks lw(data access to register we load), branch, jump hazard 
module hazard_detector (input         reset,					
                        input         idex_memread, /*exmem_memread,*/
                        input   [4:0] ifid_rs, ifid_rt,	
								input   [4:0] idex_rt, 			
								//input   [4:0] exmem_rt,
                        output reg    stall,
								
								// #####    Jisoo Kim: Start    #####
								input         idex_jump, idex_jr,
								input         idex_pcsrc,
								output reg    flush);
								// #####    Jisoo Kim: End    #####

  always @ (*)
  begin
    if (reset)
	   stall <= #`mydelay 1'b0;
    else if (idex_memread & ((idex_rt == ifid_rs) | (idex_rt == ifid_rt)))		// hazard at ex stage
	   stall <= #`mydelay 1'b1;
	 /*
	 else if (exmem_memread & ((exmem_rt == ifid_rs) | (exmem_rt == ifid_rt)))	// hazard at mem stage
	   stall <= #`mydelay 1'b1;
	 */ // no more hazard at this stage
    else
	   stall <= #`mydelay 1'b0;
		
	 // #####    Jisoo Kim: Start    #####
	 if (idex_jump | idex_jr | idex_pcsrc)		// jump or branch at execution stage
	   flush <= #`mydelay 1'b1;
    else
	   flush <= #`mydelay 1'b0;
	 // #####    Jisoo Kim: End    #####
  end
endmodule
