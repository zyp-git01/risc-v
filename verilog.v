`timescale 1ns / 1ps

`define idle 1'b0
`define exec 1'b1
//Data Transfer & Arithmetic
`define NOP 5'b00000
`define HALT 5'b00001
`define LOAD 5'b00010
`define STORE 5'b00011
`define LDIH 5'b10000
`define ADD 5'b01000
`define ADDI 5'b01001
`define ADDC 5'b10001
`define SUB 5'b01010
`define SUBI 5'b01011
`define SUBC 5'b10010
`define CMP 5'b01100
//Logical/Shift
`define AND 5'b01101
`define OR 5'b01110
`define XOR 5'b01111
`define SLL 5'b00100
`define SRL 5'b00110
`define SLA 5'b00101
`define SRA 5'b00111
//Control
`define JUMP 5'b11000
`define JMPR 5'b11001
`define BZ 5'b11010
`define BNZ 5'b11011
`define BN 5'b11100
`define BNN 5'b11101
`define BC 5'b11110
`define BNC 5'b11111
//************************************************//
//* You need to complete the design below        *//
//* by yourself according to your operation set  *//
//************************************************//
module CPU(
input [15:0]i_datain,
input [15:0]d_datain,
input clock,
input reset,
input enable,
input start,
//input [3:0]select_y,
//output reg [15:0]y,
output [7:0]i_addr,
output [7:0]d_addr,
output [15:0]d_dataout,
output d_we
);

reg [15:0]gr[7:0];

reg state;
reg next_state;

reg [7:0]pc;

reg [15:0]id_ir;
reg [15:0]ex_ir;
reg [15:0]mem_ir;
reg [15:0]wb_ir;

reg [15:0]reg_A;
reg [15:0]reg_B;
reg [15:0]reg_C;
reg [15:0]reg_C1;

reg [15:0]smdr;
reg [15:0]smdr1;

wire [15:0]ALUo;

reg nf,zf,dw;
wire cf;    

assign i_addr = pc;
assign d_addr = reg_C[7:0];
assign d_we = dw;
assign d_dataout = smdr1;

//************* CPU control *************//
always @(posedge clock)
    begin
        if (!reset)
            state <= `idle;
        else
            state <= next_state;
    end

always @(*)
    begin
        case (state)
            `idle : 
                if ((enable == 1'b1) 
                && (start == 1'b1))
                    next_state <= `exec;
                else    
                    next_state <= `idle;
            `exec :
                if ((enable == 1'b0) 
                || (wb_ir[15:11] == `HALT))
                    next_state <= `idle;
                else
                    next_state <= `exec;
        endcase
    end

//************* IF *************//
always @(posedge clock or negedge reset)
    begin
        if (!reset)
            begin
                id_ir <= 0;
                pc <= 0;
            end

        else if (state ==`exec)
            begin
                id_ir <= i_datain;

                if((mem_ir[15:11] == `JUMP)
                || (mem_ir[15:11] == `JMPR)
                ||  ((mem_ir[15:11] == `BZ)
                    && (zf == 1'b1)) 
                || ((mem_ir[15:11] == `BNZ)
                    && (zf == 1'b0)) 
                || ((mem_ir[15:11] == `BN)
                    && (nf == 1'b1))
                || ((mem_ir[15:11] == `BNN)
                    && (nf == 1'b0))
                || ((mem_ir[15:11] == `BC)
                    && (cf == 1'b1))
                || ((mem_ir[15:11] == `BNC)
                    && (cf == 1'b0)))
                    pc <= reg_C[7:0];//if jumped
                else
                    pc <= pc + 1;//if not jumped
            end
    end

//************* ID *************//
always @(posedge clock or negedge reset)
    begin
        if (!reset)
            begin
                ex_ir<=0;
                reg_A<=0;
                reg_B<=0;
                smdr<=0;
            end

        else if (state == `exec)
            begin
                ex_ir <= id_ir;

                if ((id_ir[15:11] == `LDIH)
                || (id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI)
                || (id_ir[15:11] == `JMPR)
                || (id_ir[15:11] == `BZ) || (id_ir[15:11] == `BNZ)
                || (id_ir[15:11] == `BN) || (id_ir[15:11] == `BNN)
                || (id_ir[15:11] == `BC) || (id_ir[15:11] == `BNC))
                    reg_A <= gr[(id_ir[10:8])];//r1

                else if(id_ir[15:11] == `JUMP)
                    reg_A <= gr[(id_ir[7:0])];//val2+val3

                else
                    reg_A <= gr[id_ir[6:4]];//r2
                ///

                if ((id_ir[15:11] == `LOAD) || (id_ir[15:11] == `STORE) 
                || (id_ir[15:11] == `SLL) || (id_ir[15:11] == `SRL) 
                || (id_ir[15:11] == `SLA) || (id_ir[15:11] == `SRA))
                    reg_B <= {12'b0000_0000_0000, id_ir[3:0]};//val3

                else if (id_ir[15:11] == `STORE)
                    begin
                        reg_B <= {12'b0000_0000_0000, id_ir[3:0]};//val3
                        smdr <= gr[id_ir[10:8]];
                    end

                else if ((id_ir[15:11] == `ADDI) || (id_ir[15:11] == `SUBI) 
                || (id_ir[15:11] == `BZ) || (id_ir[15:11] == `BNZ) 
                || (id_ir[15:11] == `BN) || (id_ir[15:11] == `BNN) 
                || (id_ir[15:11] == `BC) || (id_ir[15:11] == `BNC) 
                || (id_ir[15:11] == `JMPR))
                    reg_B <= {8'b0000_0000,id_ir[7:0]};//{val2,val3}

                else if (id_ir[15:11] == `JUMP) 
                    reg_B <= 0;//0

                else if ((id_ir[15:11] == `ADDC) || (id_ir[15:11] == `SUBC))
                    reg_B <= gr[id_ir[2:0]]+cf;//r3+cf

                else if (id_ir[15:11] == `LDIH)
                   reg_B <= {id_ir[7:0],8'b0000_0000};//{val2,val3,8'b0000_0000}

                else
                   reg_B <= gr[id_ir[2:0]];
            end
    end

ALU alu(
    .a(reg_A),
    .b(reg_B),
    .ir(ex_ir[15:11]),
    .ALUo(ALUo),
    .cf(cf)
);

//************* EX *************//  
always @(posedge clock or negedge reset)
    begin
        if (!reset)
            begin
                reg_C<=0;
                mem_ir<=0;
                zf<=0;
                nf<=0;
                dw<=0;
                smdr1<=0;
            end

        else if (state == `exec)
            begin
                mem_ir <= ex_ir;
                reg_C <= ALUo;

                if ((ex_ir[15:11] == `ADD) || (ex_ir[15:11] == `ADDI) || (ex_ir[15:11] == `ADDC)
                || (ex_ir[15:11] == `SUB) || (ex_ir[15:11] == `SUBI) || (ex_ir[15:11] == `SUBC)
                || (ex_ir[15:11] == `CMP))
                    begin
                        if (ALUo == 16'b0000_0000_0000_0000)
                            zf <= 1'b1;
                        else
                            zf <= 1'b0;
                        begin
                        if (ALUo[15] == 1'b1)
                            nf <= 1'b1;
                        else
                            nf <= 1'b0;
                        end
                    end
                if (ex_ir[15:11] == `STORE)
                    begin
                        dw <= 1'b1;
                        smdr1 <= smdr;
                    end
                else
                    dw <= 1'b0;
            end
    end

//************* MEM *************//
always @(posedge clock or negedge reset)
    begin
        if (!reset)
            begin
                reg_C1<=0;
                wb_ir<=0;
            end

        else if (state == `exec)
            begin
                wb_ir <= mem_ir;

                if (mem_ir[15:11] == `LOAD)
                    reg_C1 <= d_datain;
                else
                    reg_C1 <= reg_C;
            end
    end     

//************* WB *************//
always @(posedge clock or negedge reset)
    begin
        if (!reset)
            begin
                gr[0] <= 16'b0000_0000_0000_0000;
                gr[1] <= 16'b0000_0000_0000_0000;
                gr[2] <= 16'b0000_0000_0000_0000;
                gr[3] <= 16'b0000_0000_0000_0000;
                gr[4] <= 16'b0000_0000_0000_0000;
                gr[5] <= 16'b0000_0000_0000_0000;
                gr[6] <= 16'b0000_0000_0000_0000;
                gr[7] <= 16'b0000_0000_0000_0000;
            end

        else if (state == `exec)
            begin
                if ((wb_ir[15:11] == `LOAD) || (wb_ir[15:11] == `LDIH)
                || (wb_ir[15:11] == `ADD)|| (wb_ir[15:11] == `ADDI) || (wb_ir[15:11] == `ADDC)
                || (wb_ir[15:11] == `SUB)|| (wb_ir[15:11] == `SUBI) || (wb_ir[15:11] == `SUBC)
                || (wb_ir[15:11] == `AND)|| (wb_ir[15:11] == `OR) || (wb_ir[15:11] == `XOR)
                || (wb_ir[15:11] == `SLL)|| (wb_ir[15:11] == `SRL) 
                || (wb_ir[15:11] == `SLA)|| (wb_ir[15:11] == `SRA))
                    gr[wb_ir[10:8]] <= reg_C1;
            end
    end

endmodule

module ALU(
input [15:0]a,
input [15:0]b,
input [4:0]ir,
output reg [15:0]ALUo,
output reg cf
);
reg eat;
always@(a or b or ir)begin
    if(ir==`LOAD || ir==`STORE
    || ir==`ADD || ir==`ADDI || ir==`ADDC
    || ir==`JUMP || ir==`JMPR 
    || ir==`BZ || ir==`BNZ || ir==`BN || ir==`BNN || ir==`BC || ir==`BNC)
        {cf,ALUo} <= a+b;
    else if(ir==`SUB || ir==`SUBI || ir==`SUBC
    || ir==`CMP)
        ALUo <= $signed(a)-b;
    else if(ir==`AND)
        ALUo <= a&b;
    else if(ir==`OR)
        ALUo <= a|b;
    else if(ir==`XOR)
        ALUo <= a^b;
    else if(ir==`SLL)
        ALUo <= a<<b;
    else if(ir==`SRL)
        ALUo <= a>>b;
    else if(ir==`SLA)
        ALUo <= $signed(a)<<<b;
    else if(ir==`SRA)
        ALUo <= $signed(a)>>>b;
    else
        ALUo <= 16'bXXXXXXXXXXXXXXXX;
end
endmodule
