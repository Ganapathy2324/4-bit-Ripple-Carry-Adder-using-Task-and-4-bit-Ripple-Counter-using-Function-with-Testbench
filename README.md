# 4-bit-Ripple-Carry-Adder-using-Task-and-4-bit-Ripple-Counter-using-Function-with-Testbench
## Ganapathy Shriram V B
## 212223060064
Aim:
To design and simulate a 4-bit Ripple Carry Adder using Verilog HDL with a task to implement the full adder functionality and verify its output using a testbench.
To design and simulate a 4-bit Ripple Counter using Verilog HDL with a function to calculate the next state and verify its functionality using a testbench.
Apparatus Required:
Computer with Vivado or any Verilog simulation software.
Verilog HDL compiler.
## Verilog Code
```
`timescale 1ns / 1ps
module ripple(input [3:0] A,
input [3:0] B,
input Cin,
output [3:0] Sum,
output Cout 
);

reg [3:0] sum_temp;
reg cout_temp;
task full_adder;
    input a, b, cin;
    output sum, cout;
    begin
        sum = a ^ b ^ cin;
        cout = (a & b) | (b & cin) | (cin & a);
    end
endtask
always @(*) begin
    full_adder(A[0], B[0], Cin, sum_temp[0], cout_temp);
    full_adder(A[1], B[1], cout_temp, sum_temp[1], cout_temp);
    full_adder(A[2], B[2], cout_temp, sum_temp[2], cout_temp);
    full_adder(A[3], B[3], cout_temp, sum_temp[3],cout_temp);
end

assign Sum = sum_temp;
assign Cout=cout_temp;
endmodule
```
## OUTPUT
![Screenshot 2025-04-19 143240](https://github.com/user-attachments/assets/d8a4139a-cb58-4dda-a698-f458ca43da54)

## Test bench for Ripple carry adder
```
`timescale 1ns / 1ps
module ripple_tb;

reg [3:0] A, B;
reg Cin;
wire [3:0] Sum;
wire Cout;
ripple uut (
    .A(A),
    .B(B),
    .Cin(Cin),
    .Sum(Sum),
    .Cout(Cout)
);

initial begin
    // Test cases
    A = 4'b0001; B = 4'b0010; Cin = 0;
    #10;
    
    A = 4'b0110; B = 4'b0101; Cin = 0;
    #10;
    
    A = 4'b1111; B = 4'b0001; Cin = 0;
    #10;
    
    A = 4'b1010; B = 4'b1101; Cin = 1;
    #10;
    
    A = 4'b1111; B = 4'b1111; Cin = 1;
    #10;
end
endmodule
```

## Verilog Code ripple counter
```
`timescale 1ns / 1ps
module counter_4bit( input clk,
input rst,
output reg [3:0]Q
);
function [3:0] next_state;
    input [3:0] curr_state;
    begin
        next_state = curr_state + 1;
    end
endfunction
    
always @(posedge clk)
 begin
    if (rst)
        Q<=4'b0000;       
    else
        Q <= next_state(Q);
end
endmodule
```
## OUTPUT
![Screenshot 2025-04-26 085534](https://github.com/user-attachments/assets/e7b87662-0f2c-4f19-91f4-e361280b23b8)

## Conclusion:
The 4-bit Ripple Carry Adder was successfully designed and implemented using Verilog HDL with the help of a task for the full adder logic. The testbench verified that the ripple carry adder correctly computes the 4-bit sum and carry-out for various input combinations. The simulation results matched the expected outputs.
The 4-bit Ripple Counter was successfully designed and implemented using Verilog HDL. A function was used to calculate the next state of the counter.

