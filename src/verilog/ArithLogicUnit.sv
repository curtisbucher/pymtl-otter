`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: J. Callenes
//
// Create Date: 06/07/2018 05:03:50 PM
// Design Name:
// Module Name: ArithLogicUnit
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

module OTTER_ALU(op, a, b, out);
        input [3:0] op;  //func7[5],func3
        input [31:0] a,b;
        output logic [31:0] out;

        always_comb
        begin //reevaluate If these change
            case(op)
                0:  out = a + b;     //add
                8:  out = a - b;     //sub
                6:  out = a | b;     //or
                7:  out = a & b;     //and
                4:  out = a ^ b;     //xor
                5:  out =  a >> b[4:0];    //srl
                1:  out =  a << b[4:0];    //sll
               13:  out =  $signed(a) >>> b[4:0];    //sra
                2:  out = $signed(a) < $signed(b) ? 1: 0;       //slt
                3:  out = a < b ? 1: 0;      //sltu
                9:  out = a; //copy op1 (lui)
                10: out = a * b;
                default: out = 0;
            endcase
        end
    endmodule

