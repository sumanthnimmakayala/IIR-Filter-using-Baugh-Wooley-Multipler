// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // IO
    assign io_out = count;
    assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

//IIR dut(.clk(clk),.reset(rst),.a(rdata[3:0]),.x(rdata[7:3]),.y(count[3:0]));
IIR dut(clk,rst,a,x,y);

endmodule


module IIR(clk,rst,a,x,y);
input clk,rst;
input [3:0]a,x;
output [3:0]y; 
reg [3:0]y_val;
wire [7:0] baugh_prod_actual; 
baugh_mult bm1(.a(a), .b(y_val), .p(baugh_prod_actual)); 


always@*  //(posedge clk,rst,x,a)

begin
if (rst) begin 
    y_val<= x;
end
else begin
y_val<= x + booth_prod_actual[3:0];
end
end
assign y = y_val;
endmodule




module baugh_mult(a, b, p);
input [3:0] a, b;
output [7:0] p; 
supply1 one;
wire w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23;

assign p[0] = a[0]&b[0];

half_adder HA1(a[1]&b[0], a[0]&b[1], p[1], w1);
half_adder HA2(a[2]&b[0], a[1]&b[1], w2, w3); 
half_adder HA3(~(a[3]&b[0]), a[2]&b[1], w4, w5);

full_adder FA1(w2, w1, a[0]&b[2], p[2],w6);
full_adder FA2(w4, w3, a[1]&b[2], w7, w8);
full_adder FA3(w5, a[2]&b[2], ~(a[3]&b[1]), w9, w10);

full_adder FA4(w6, w7, ~(a[0]&b[3]), p[3], w11); 
full_adder FA5(w8, w9, ~(a[1]&b[3]), w12, w13);
full_adder FA6(w10, ~(a[2]&b[3]) , ~(a[3]&b[2]), w14,w15);

full_adder FA7(one, w11, w12, p[4], w16); 
half_adder HA4(w13, w14, w17, w18);
half_adder HA5(a[3]&b[3], w15, w19, w20);

half_adder HA6(w16,w17, p[5], w21);
half_adder HA7(w18, w19, w22,w23);

half_adder HA8(w21, w22,p[7], p[6]);

endmodule



module half_adder (x, y, s, cout);
input x, y;
output s, cout;
assign s = x^y^cin;
assign cout = (x&y) | (y&cin) | (x&cin);
endmodule


module full_adder (x, y, cin,s,cout);
input x, y, cin;
output s, cout;
assign s = x^y;
assign cout = x&y; 
endmodule
`default_nettype wire
