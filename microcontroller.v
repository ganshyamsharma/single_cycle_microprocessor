`timescale 1us / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:     
// Engineer: Ganshyam
// 
// Create Date: 14.11.2024 17:02:32
// Design Name: 
// Module Name: mic_controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module mic_controller( 
    input clk,
    input[7:0]field_in,
    output [7:0]field_out
    );
        wire datamemwrite, iop;
        wire [7:0] pcop, data, memdata, datamemadd, memdata0, memdata1, data0, data1;
        wire [15:0] ins;
        ins_memory im1(clk, pcop, ins);
        mic_proc mp3(clk, ins, data, pcop, datamemadd, memdata, datamemwrite, iop);
        mux21 mu1(data0, data1, iop, data);
        demux12 mu2(memdata, iop, memdata0, memdata1);
        data_memory dm1(clk, datamemwrite, datamemadd, memdata0, data0);
        io_memory iom1(clk, datamemwrite, memdata1, field_in, data1, field_out);
endmodule

/////////////////////////////////////////////////////////////////////////   Microprocessor

module mic_proc(
    input clk,
    input [15:0] ins,
    input [7:0] data,
    output [7:0] pcop, datamemadd, datamemdata,
    output datamemwrite, iop
    );
        wire rgaddsel, src2sel, jumpcon1, jumpcon2, rgwrite, flagwrite;
        wire [2:0] aluctrl;
        wire [1:0] rgwritesel; 
        control_unit c1(ins[15:11], rgaddsel, src2sel, jumpcon1, jumpcon2, rgwrite, flagwrite, datamemwrite, iop, aluctrl, rgwritesel);
        datapath d1(clk, ins, data, aluctrl, rgwritesel, rgaddsel, src2sel, jumpcon1, jumpcon2, rgwrite, flagwrite, pcop, datamemadd, datamemdata);
endmodule

/////////////////////////////////////////////////////////////////////////   Control Unit

module control_unit(
    input [4:0] opc,
    output rgaddsel, src2sel, jumpcon1, jumpcon2, rgwrite, flagwrite, datawrite, iop,
    output [2:0] aluctrl,
    output [1:0] rgwritesel
    );
    reg [12:0]control;
    assign {rgaddsel, rgwritesel, src2sel, jumpcon1, jumpcon2, rgwrite, aluctrl, flagwrite, datawrite, iop} = control;
    always @(*) begin
        case(opc)
            5'd0 : control = 13'bx00x001xxx000;
            5'd1 : control = 13'bx11x001xxx000;
            5'd2 : control = 13'bx01x001xxx000;
            5'd3 : control = 13'b0xxx000xxx010;
            5'd4 : control = 13'b1100001011100;
            5'd5 : control = 13'b1100001100100;      
            5'd6 : control = 13'b010x001110000;           
            5'd7 : control = 13'b010x001111000;            
            5'd8 : control = 13'b0101001011100;           
            5'd9 : control = 13'b0101001100100;           
            5'd10 : control = 13'b1100001000000;           
            5'd11 : control = 13'b1100001001000;           
            5'd12 : control = 13'b1100001010000;            
            5'd13 : control = 13'b0100001101000;           
            5'd14 : control = 13'bxxxx110xxx000;          
            5'd15 : control = 13'bxxxx100xxx000;
            5'd16 : control = 13'bx01x001xxx001;
            5'd17 : control = 13'b0xxx000xxx011;
            default : control = 13'bxxxxxxxxxxxxx;
        endcase
    end
endmodule                        

/////////////////////////////////////////////////////////////////////////   Datapath

module datapath(
    input clk,
    input [15:0]ins,
    input [7:0]data,
    input [2:0]aluctrl,
    input [1:0]rgwritesel,
    input rgaddsel, src2sel, jumpcon1, jumpcon2, rgwrite, flagwrite,
    output [7:0]pcop, datamemadd, datamemdata
    );  
        wire [7:0]pcin, pcout, pcin1, reg_data1, reg_data2, regmem_write, aluout, alusrc2;
        wire pcsel, pcjmpctrl, zerowire, carrywire, zeroalu, carryalu;
        wire [2:0]reg_add1;
        assign pcop = pcout;
        assign datamemadd = ins[7:0];
        assign datamemdata = reg_data1;
        assign pcsel = jumpcon1 & pcjmpctrl;
        bit8_fulladder fa2(8'b1, pcout, 1'b0, pcin1);
        mux21 m1(pcin1, ins[10:3], pcsel, pcin);
        basic_mux21 m2(zerowire, ~zerowire, jumpcon2, pcjmpctrl);
        mux41 m3(ins[7:0], data, aluout, reg_data2, rgwritesel, regmem_write);
        mux21_3ip m4(ins[10:8], ins[4:2], rgaddsel, reg_add1);
        mux21 m5(reg_data2, 8'b1, src2sel, alusrc2);
        program_counter pc1(clk, pcin, pcout);
        register_memory r1(clk, rgwrite, reg_add1, ins[7:5], ins[10:8], regmem_write, reg_data1, reg_data2);
        ALU a1(aluctrl, reg_data1, alusrc2, carryalu, zeroalu, aluout);
        flag_register fr1(clk, flagwrite, carryalu, zeroalu, carrywire, zerowire);
endmodule

/////////////////////////////////////////////////////////////////////////   ALU      
   
module ALU(
    input [2:0] aluctrl,
    input [7:0] src1, src2,
    output carry, zero,
    output reg [7:0] result
    );
        wire [7:0] srcb;
        wire [7:0] addresult;
        wire [7:0] leftshifted;
        wire [7:0] rightshifted;
        mux21 m1(src2, ~src2, aluctrl[2], srcb);
        bit8_fulladder fa1(src1, srcb, aluctrl[2], addresult, carry, zero);
        left_shift ls1(src1, leftshifted);
        right_shift rs1(src1, rightshifted);
        always @(*) begin
            case(aluctrl)
                3'b000 : result = src1 & src2;
                3'b001 : result = src1 | src2;
                3'b010 : result = src1 ^ src2;
                3'b011 : result = addresult;
                3'b100 : result = addresult;
                3'b101 : result = ~src1;
                3'b110 : result = leftshifted;
                3'b111 : result = rightshifted;
                default : result = 8'b0;
            endcase
        end
endmodule    

/////////////////////////////////////////////////////////////////////////   Register Memory

module register_memory(
    input clk, rgwrite,
    input [2:0] ra1, ra2, wa3,
    input [7:0] wd, 
    output [7:0] rd1, rd2
    );
        reg [7:0] regmem[7:0];
        always @(negedge clk) begin
            if (rgwrite) begin
                 regmem[wa3] <= wd;
            end
            else if (!rgwrite) begin
                regmem[wa3] <= regmem[wa3];
            end
        end
        assign rd1 = regmem[ra1];
        assign rd2 = regmem[ra2];
endmodule

/////////////////////////////////////////////////////////////////////////   Memories (Instruction, Data & IO)

module ins_memory(
    input clk,
    input [7:0] a,
    output [15:0] ins
    );
        reg [15:0] insmem[99:0];
        
//////////////////////////////   Program Start        
        /*initial begin
            insmem[0] = 16'b0000011111111111;
            insmem[1] = 16'b10000000xxxxxxxx;
            insmem[2] = 16'b00001001000xxxxx;
            insmem[3] = 16'b00110001xxxxxxxx;
            insmem[4] = 16'b01010010000001xx;
            insmem[5] = 16'b1000101011111111;
            insmem[6] = 16'b0111000000000xxx;
        end*/
/////////////////////////////   Program End     
         
        assign ins = insmem[a];       
endmodule            


module data_memory(
    input clk, datawrite,
    input [7:0] a, wd,
    output [7:0] rd
    );
        reg [7:0] datamem[255:100];
        always @(negedge clk) begin
            if (datawrite) begin
                datamem[a] <= wd;
            end
        end
        assign rd = datamem[a];
endmodule


module io_memory(
    input clk, datawrite,
    input [7:0] wd, field_in,
    output [7:0] rd, field_out
    );
        reg [7:0]iomem[1:0];
        
        always @(negedge clk) begin
                if (datawrite) begin
                    iomem[1] <= wd;
                end
        end
        assign rd = iomem[0];
        always @(posedge clk) iomem[0] = field_in;
        assign field_out = iomem[1];
endmodule
       
/////////////////////////////////////////////////////////////////////////   PC and Flag Register

module program_counter(
    input clk,
    input [7:0]ip,
    output reg [7:0]out
    );  
        reg [7:0]pcnt;
        initial pcnt = 8'b0;
        always @(posedge clk) out <= pcnt;
        always @(negedge clk) pcnt <= ip;       
endmodule


module flag_register(
    input clk, flagwrite, carry, zero,
    output reg carryflag, zeroflag
    );
        initial begin
            carryflag = 1'b0;
            zeroflag = 1'b0;
        end
        always @(negedge clk) begin
            if(flagwrite) begin
                carryflag <= carry;
                zeroflag <= zero;
            end
        end 
endmodule

/////////////////////////////////////////////////////////////////////////   Basic Building Blocks

module basic_mux21(
    input a, b, sel,
    output out
    );
        assign out = sel ? b : a;    
endmodule


module mux21_3ip(
    input [2:0]a, b,
    input sel,
    output [2:0]out
    );
        assign out = sel ? b : a;
endmodule


module mux21(
    input [7:0]a, [7:0]b,
    input sel,
    output [7:0]out
    );
        assign out = sel ? b : a;
endmodule


module mux41(
    input [7:0]a, [7:0]b, [7:0]c, [7:0]d,
    input [1:0]sel,
    output reg [7:0]out
    );
        always @(a, b, c, d, sel) begin
            case(sel)
                2'b00 : out = a;
                2'b01 : out = b;
                2'b10 : out = c;
                2'b11 : out = d;
                default : out = 0;
             endcase
         end      
endmodule


module demux12(
    input [7:0]a,
    input sel,
    output [7:0]op0, op1
    );
        assign {op1, op0} = sel ? {a, 8'b0} : {8'b0, a};
endmodule


module bit8_fulladder(
    input [7:0] a, b,
    input cin,
    output [7:0] sum,
    output carry, zero
    );
        assign {carry, sum} = a + b + cin;
        assign zero = ~|sum;
endmodule


module left_shift(
    input [7:0] ip,
    output [7:0] op
    );
        assign op = {ip[6:0], 1'b0};
endmodule


module right_shift(
    input [7:0] ip,
    output [7:0] op
    );
        assign op = {1'b0, ip[7:1]};
endmodule  
        
        
        
    


        

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        