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
    Date: 13-7-2024 */

module jtriders_prot(
    input                rst,
    input                clk,
    input                cen_16,
    input                cen_8,
    input                tmnt2,

    input                cs,
    input         [19:1] addr,
    input         [ 1:0] dsn,
    input         [15:0] din, // cpu dout
    output reg    [15:0] dout,
    input                cpu_we,
    input                ram_we,

    // DMA
    output        [23:1] prot_addr,
    input         [15:0] prot_din,
    output         [1:0] prot_dsn,
    output               prot_we,
    input                bus_busy,

    output        [13:1] oram_addr,

    output               irqn,
    output               BRn,
    input                BGn,
    output               BGACKn,

    input         [ 7:0] debug_bus
);

localparam [13:1] DATA = 13'hd05, // 5a0a-4000>>1
                  CMD  = 13'hc7e, // 58fc-4000>>1
                  V0   = 13'hc0c, // 5818-4000>>1
                  V1   = 13'he58, // 5cb0-4000>>1
                  V2   = 13'h064; // 40c8-4000>>1

reg        [15:0] cmd, odma;
reg signed [15:0] v0,v1,v2,vx0,vx1,vx2,vsum,c,vcalc,calc;

assign irqn = 1; // always high on the PCB
// DMA
reg [13:1] dma_addr;
reg [ 7:0] hw_prio,logic_prio;
reg [ 6:0] scan_addr;
reg [ 1:0] st;
reg        owr;

reg        BRn_riders, BGACKn_riders;
reg        BRn_tmnt2, BGACKn_tmnt2;
assign     BRn = BRn_riders & BRn_tmnt2;
assign     BGACKn = BGACKn_riders & BGACKn_tmnt2;

// signal order expected at object chip pins
// function [13:1] conv( input [6:0] a);
// begin
//     reg [8:0] b;
//     b = {a,2'd0};
//     conv={b[8:2],2'd0,b[2:0],1'b0};
// end
// endfunction

function [13:1] conv13( input[13:1] a);
begin
    conv13 = { a[6:5], a[1], a[13:7], a[4:2] };
end
endfunction

assign prot_addr = !BGACKn_tmnt2 ? tmnt2_dma_addr[23:1] : {8'h18, 2'b00, dma_addr};
assign prot_we   = !BGACKn_tmnt2 ? tmnt2_dma_write : !BGACKn_riders ? owr : 1'b0;
assign prot_dsn  = !BGACKn_tmnt2 ? {2{tmnt2_mem_wait}} : !BGACKn_riders ? (owr ? 2'b10 : 2'b00) : 2'b11;

assign oram_addr = !BGACKn_tmnt2 ? conv13(tmnt2_dma_addr[13:1]) : !BGACKn_riders ? dma_addr     : conv13(addr);

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        BRn_riders    <= 1;
        BGACKn_riders <= 1;
        logic_prio <= 0;
        hw_prio    <= 0;
        scan_addr  <= 0;
        dma_addr   <= 0;
        st         <= 0;
    end else if (!tmnt2) begin
        if( cs && cpu_we && addr[7:1]==1 ) begin
            BRn_riders <= 0;
            logic_prio <= 1;
            hw_prio    <= 1;
            scan_addr  <= 0;
            st         <= 0;
        end
        if( !BGn && !BRn_riders) begin
            $display("DMA in progress");
            {BRn_riders, BGACKn_riders} <= 2'b10;
        end
        if( !BGACKn_riders && cen_8 ) begin
            st <= st+2'd1;
            case( st )
            0: begin
                dma_addr <= conv13({ scan_addr, 6'd3 });
                owr      <= 0;
            end
            2: begin
                if( prot_din[15:8]==logic_prio) begin
                    dma_addr <= {3'd0,scan_addr,3'd0};
                    owr      <= 1;
                end
            end
            3: begin
                owr <= 0;
                scan_addr <= scan_addr+7'd1;
                if( owr ) hw_prio <= hw_prio+8'd1;
                if( &scan_addr ) begin
                    logic_prio <= logic_prio<<1;
                    scan_addr  <= 0;
                    if( logic_prio[7] ) BGACKn_riders <= 1;
                end
            end
            endcase
        end
    end
end

// Data read
// MAME does a convoluted operation that seems to rely on the upper 16 bits
// being zero and getting set to 1 when v0 is negative. That changes the /8
// result by 1, which then creates a 0x40 difference in the final value
// This implementation does not rely on phantom bits. In any case, we need
// more information to accurately implement this chip
always @* begin
    c     = 0;
    vx0   = (v0+16'd32)>>3;
    vx1   = -vx0;
    vx2   = {5'd0,vx1[4:0],6'd0};
    vsum  = v1+v2-16'd6;
    c[0]  = vsum[15] & (|vsum[2:0]);
    vcalc = vx2+(((vsum>>3)+c+16'd12)&16'h3f);
end

`define WR16(a) begin if(!dsn[0]) a[7:0]<=din[7:0]; if(!dsn[1]) a[15:8]<=din[15:8]; end
always @(posedge clk, posedge rst) begin
    if( rst ) begin
        { cmd, odma, v0, v1, v2 } <= 0;
    end else if(ram_we) begin
        case(addr[13:1])
            DATA: `WR16( odma )
            CMD:  `WR16( cmd  )
            V0:   `WR16( v0   )
            V1:   `WR16( v1   )
            V2:   `WR16( v2   )
            default:;
        endcase
    end
end

always @(posedge clk) begin
    if (!BGACKn_tmnt2)
        dout <= tmnt2_dma_write_data;
    else if (!BGACKn_riders)
        dout <= {2{hw_prio}};
    else begin
        calc <= vcalc;
        case(cmd)
            16'h100b: dout <= 16'h64;
            16'h6003: dout <= {12'd0,odma[3:0]};
            16'h6004: dout <= {11'd0,odma[4:0]};
            16'h6000: dout <= {15'd0,odma[  0]};
            16'h0000: dout <= { 8'd0,odma[7:0]};
            16'h6007: dout <= { 8'd0,odma[7:0]};
            16'h8abc: dout <= calc;
            default:  dout <= 16'hffff;
        endcase
    end
end

// TMNT2 MCU
reg  [15:0] mcu[16], src[4], mod[24];
wire [23:0] tmnt2_src_addr = {mcu[1][7:0], mcu[0]};
wire [23:0] tmnt2_dst_addr = {mcu[3][7:0], mcu[2]};
wire [23:0] tmnt2_mod_addr = {mcu[5][7:0], mcu[4]};
wire        tmnt2_zlock = mcu[8][7:0] == 8'h01;

reg  [23:1] tmnt2_dma_addr;
reg  [ 6:0] tmnt2_scan_addr;
reg  [ 1:0] tmnt2_st;
wire        tmnt2_data_ok = ~bus_busy;
wire [15:0] tmnt2_dma_data = prot_din;
reg  [15:0] tmnt2_dma_data_r;
reg         tmnt2_dma_write;
reg  [15:0] tmnt2_dma_write_data;
reg         tmnt2_mem_wait;

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        BRn_tmnt2    <= 1;
        BGACKn_tmnt2 <= 1;
        tmnt2_st <= 0;
        tmnt2_mem_wait <= 1;
        tmnt2_dma_write <= 0;
    end else if (tmnt2) begin
        if (cs && cpu_we) begin
            `WR16( mcu[addr[4:1]] )
            if(addr[4:1] == 4'hc && mcu[8][15:8] == 8'h82 && !dsn[1]) begin
                tmnt2_mem_wait <= 1;
                BRn_tmnt2 <= 0;
                tmnt2_st <= 0;
            end
        end

        if (tmnt2_data_ok && tmnt2_st != 3) begin
            tmnt2_mem_wait <= 1;
            if (!tmnt2_mem_wait) tmnt2_dma_data_r <= tmnt2_dma_data;
        end

        tmnt2_dma_write <= tmnt2_st == 3;

        if( !BGn && !BRn_tmnt2 && cen_16 ) begin
            $display("DMA in progress");
            {BRn_tmnt2, BGACKn_tmnt2} <= 2'b10;
            tmnt2_dma_addr <= tmnt2_src_addr[23:1];
            tmnt2_scan_addr <= 0;
            tmnt2_st <= 0;
            tmnt2_mem_wait <= 0;
        end
        if( !BGACKn_tmnt2 && cen_16 ) begin
            case (tmnt2_st)
                // read source
                0: if (tmnt2_mem_wait) begin
                    tmnt2_mem_wait <= 0;
                    tmnt2_dma_addr <= tmnt2_dma_addr + 1'd1;
                    tmnt2_scan_addr <= tmnt2_scan_addr + 1'd1;
                    src[tmnt2_scan_addr] <= tmnt2_dma_data_r;
                    if (tmnt2_scan_addr == 3) begin
                        tmnt2_scan_addr <= 0;
                        tmnt2_st <= 1;
                        tmnt2_dma_addr <= tmnt2_mod_addr[23:1];
                    end
                end
                // read mod
                1: if (tmnt2_mem_wait) begin
                    tmnt2_mem_wait <= 0;
                    tmnt2_dma_addr <= tmnt2_dma_addr + 1'd1;
                    tmnt2_scan_addr <= tmnt2_scan_addr + 1'd1;
                    mod[tmnt2_scan_addr] <= tmnt2_dma_data_r;
                    if (tmnt2_scan_addr == 23) begin
                        tmnt2_st <= 2;
                        tmnt2_mem_wait <= 1;
                    end
                end
                // calculate
                2: begin
                    tmnt2_dma_addr <= tmnt2_dst_addr[23:1];
                    tmnt2_scan_addr <= 0;
                    tmnt2_st <= 3;
                    tmnt2_mem_wait <= 0;
                end
                // write sprite data
                3: begin
                    tmnt2_mem_wait <= 0;
                    tmnt2_dma_addr <= tmnt2_dma_addr + 1'd1;
                    tmnt2_scan_addr <= tmnt2_scan_addr + 1'd1;
                    if (tmnt2_scan_addr == 3) begin
                        tmnt2_dma_addr <= tmnt2_dma_addr + 2'd3;
                    end else if (tmnt2_scan_addr == 4) begin
                        tmnt2_st <= 0;
                        tmnt2_mem_wait <= 1;
                        BGACKn_tmnt2 <= 1;
                    end
                end
            endcase
        end
    end
end

`undef WR16

/*
<------>code = src[0];          // code

<------>i = src[1];
<------>attr1 = i >> 2 & 0x3f00;    // flip y, flip x and sprite size
<------>attr2 = i & 0x380;      // mirror y, mirror x, shadow
<------>cbase = i & 0x01f;      // base color
<------>cmod  = mod[0x2a / 2] >> 8;
<------>color = (cbase != 0x0f && cmod <= 0x1f && !zlock) ? cmod : cbase;

<------>xoffs = (int16_t)src[2];  // local x
<------>yoffs = (int16_t)src[3];  // local y

<------>i = mod[0];
<------>attr2 |= i & 0x0060;    // priority
<------>keepaspect = (i & 0x0014) == 0x0014;
<------>if (i & 0x8000) { attr1 |= 0x8000; }    // active
<------>if (keepaspect) { attr1 |= 0x4000; }    // keep aspect
//  if (i & 0x????) { attr1 ^= 0x2000; yoffs = -yoffs; }    // flip y (not used?)
<------>if (i & 0x4000) { attr1 ^= 0x1000; xoffs = -xoffs; }    // flip x

<------>xmod = (int16_t)mod[6];   // global x
<------>ymod = (int16_t)mod[7];   // global y
<------>zmod = (int16_t)mod[8];   // global z
<------>xzoom = mod[0x1c / 2];
<------>yzoom = (keepaspect) ? xzoom : mod[0x1e / 2];

<------>ylock = xlock = (i & 0x0020 && (!xzoom || xzoom == 0x100));
*/
reg  [15:0] code, i, attr1, attr2, xoffs, yoffs, xmod, ymod, zmod, xzoom, yzoom /* synthesis keep */;
reg  [ 7:0] cbase, cmod, color;
reg         keepaspect, xlock, ylock, zlock;

always @(*) begin
    code = src[0];
    i = src[1];
    keepaspect = mod[0][2] & mod[0][4];
    attr1 = {mod[0][15], keepaspect, i[13:8], 8'd0};
    attr2 = {7'd0, i[9:7], mod[0][6:5], 5'd0};
    cbase = {3'd0, i[4:0]};
    cmod = mod[21][15:8];
    color = (cbase != 8'h0f && cmod <= 8'h1f && !tmnt2_zlock) ? cmod : cbase;
    xoffs = src[2];
    yoffs = src[3];
    xmod = mod[6];
    ymod = mod[7];
    zmod = mod[8];

    if (mod[0][14]) begin
        attr1 = attr1 ^ 16'h1000;
        xoffs = -xoffs;
    end
    xzoom = mod[14];
    yzoom = keepaspect ? xzoom : mod[15];

    xlock = (mod[0][5] && (xzoom == 0 || xzoom == 16'h100));
    ylock = xlock;

    if (!tmnt2_zlock) yoffs = yoffs + zmod;
    xoffs = xoffs + xmod;
    yoffs = yoffs + ymod;
end

always @(*) begin
    if (tmnt2_scan_addr == 0)      tmnt2_dma_write_data = attr1;
    else if (tmnt2_scan_addr == 1) tmnt2_dma_write_data = code;
    else if (tmnt2_scan_addr == 2) tmnt2_dma_write_data = yoffs;
    else if (tmnt2_scan_addr == 3) tmnt2_dma_write_data = xoffs;
    else if (tmnt2_scan_addr == 4) tmnt2_dma_write_data = {attr2[15:5], color[4:0]};
    else tmnt2_dma_write_data = 0;
end

endmodule