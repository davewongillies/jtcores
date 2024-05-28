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
    Date: 29-4-2024 */

module jts18_colmix(
    input              rst,
    input              clk,
    input              vdp_en,
    input              pxl_cen,
    input              pxl2_cen,  // pixel clock enable (2x)
    input              LHBL,
    input              LVBL,
    output             LHBL_dly,
    output             LVBL_dly,
    input        [3:0] gfx_en,
    input        [5:0] s16_r, s16_g, s16_b,
    input        [7:0] vdp_r, vdp_g, vdp_b,
    output       [7:0] red,   green, blue
);

wire [7:0] ex_r, ex_g, ex_b;
reg  [7:0] pr, pg, pb;
wire       s16_blank;

assign ex_r = {s16_r,s16_r[5:4]};
assign ex_g = {s16_g,s16_g[5:4]};
assign ex_b = {s16_b,s16_b[5:4]};
assign s16_blank = {ex_r,ex_g,ex_b}==0;

always @(posedge clk) begin
    pr <= s16_blank ? vdp_r : ex_r;
    pg <= s16_blank ? vdp_g : ex_g;
    pb <= s16_blank ? vdp_b : ex_b;
    if(!vdp_en) {pr,pg,pb} <= {ex_r,ex_g,ex_b};
end

jtframe_blank #(.DLY(4),.DW(24)) u_blank(
    .clk        ( clk       ),
    .pxl_cen    ( pxl_cen   ),
    .preLHBL    ( LHBL      ),
    .preLVBL    ( LVBL      ),
    .LHBL       ( LHBL_dly  ),
    .LVBL       ( LVBL_dly  ),
    .preLBL     (           ),
    .rgb_in     ( { pr,   pg,  pb} ),
    .rgb_out    ( {red,green,blue} )
);

endmodule