/*  This file is part of JTCORES.
    JTCORES program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JTCORES program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JTCORES.  If not, see <http://www.gnu.org/licenses/>.

    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: 19-3-2023 */

module jtngp_snd(
    input                rst,
    input                clk,

    input         [11:0] main_addr,
    input         [15:0] main_dout,
    input         [15:0] main_din,
    input         [ 1:0] main_we,
    output               main_irqn,
    input         [ 7:0] comm,      // where do we store these 8 bits?
    input                int_n,

    output               sample,
    output signed [11:0] snd
);

wire [15:0] cpu_addr;
wire [ 1:0] ram_bwe;
reg  [ 7:0] cpu_din;
wire [ 7:0] ram_lsb, ram_msb;
reg         ram_cs, psg_cs, latch_cs, intset_cs;
wire        wr_n, mreq_n, cen_snd;

assign sample  = 0;
assign snd     = 0;
assign ram_bwe = {2{ram_cs&~wr_n}} & { cpu_addr[0], ~cpu_addr[0] };

jtframe_cen3p57 u_cen(
    .clk        ( clk       ),
    .cen_3p57   ( cen_snd   ),
    .cen_1p78   (           )
);

always @* begin
    ram_cs    = !mreq_n && cpu_addr[15:14]==0;
    psg_cs    = !mreq_n && cpu_addr[15:14]==1;
    latch_cs  = !mreq_n && cpu_addr[15:14]==2; // part of main's IO map
    intset_cs = !mreq_n && cpu_addr[15:14]==3;
end

always @(posedge clk) begin
    cpu_din <=  latch_cs ? comm :
                ram_cs   ? (cpu_addr[0] ? ram_msb : ram_lsb );
end

jtframe_edge #(.QSET(0)) u_mainint(
    .rst    ( rst       ),
    .clk    ( clk       ),
    .edgeof ( intset_cs ),
    .clr    ( ~iorq_n   ),
    .q      ( main_irqn )
);

jtframe_dual_ram #(.AW(11)) u_ramlow(
    // Port 0
    .clk0   ( clk             ),
    .data0  ( main_dout[7:0]  ),
    .addr0  ( main_addr[11:1] ),
    .we0    ( main_we[0]      ),
    .q0     ( main_din[7:0]   ),
    // Port 1
    .clk1   ( clk             ),
    .data1  ( cpu_dout        ),
    .addr1  ( cpu_addr[11:1]  ),
    .we1    ( ram_bwe[0]      ),
    .q1     ( ram_lsb         )
);

jtframe_dual_ram #(.AW(11)) u_ramhi(
    // Port 0
    .clk0   ( clk             ),
    .data0  ( main_dout[15:8] ),
    .addr0  ( main_addr[11:1] ),
    .we0    ( main_we[1]      ),
    .q0     ( main_din[15:8]  ),
    // Port 1
    .clk1   ( clk             ),
    .data1  ( cpu_dout        ),
    .addr1  ( cpu_addr[11:1]  ),
    .we1    ( ram_bwe[1]      ),
    .q1     ( ram_msb         )
);

jtframe_z80_romwait #(.CLR_INT(1)) u_cpu(
    .rst_n      ( ~rst      ),
    .clk        ( clk       ),
    .cen        ( cen_snd   ),
    .cpu_cen    (           ),
    .int_n      ( int_n     ),
    .nmi_n      ( 1'b1      ),
    .busrq_n    ( 1'b1      ),
    .m1_n       (           ),
    .mreq_n     ( mreq_n    ),
    .iorq_n     ( iorq_n    ),
    .rd_n       (           ),
    .wr_n       ( wr_n      ),
    .rfsh_n     (           ),
    .halt_n     (           ),
    .busak_n    (           ),
    .A          ( cpu_addr  ),
    .cpu_din    ( cpu_din   ),
    .cpu_dout   ( cpu_dout  ),
    .ram_dout   ( ram_dout  ),
    // ROM access
    .rom_cs     ( 1'b0      ),
    .rom_ok     ( 1'b1      )
);

endmodule