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
    Date: 1-2-2023 */

module jttmnt_game(
    `include "jtframe_game_ports.inc" // see $JTFRAME/hdl/inc/jtframe_game_ports.inc
);

/* verilator tracing_off */
wire [ 7:0] snd_latch;
wire        snd_irq, rmrd, rst8;
wire        pal_we, cpu_we, tilesys_cs, objsys_cs;
wire        cpu_rnw, odtac, vdtac;
wire [ 7:0] tilesys_dout, objsys_dout,
            obj_dout, pal_dout, cpu_d8,
            st_main, st_video, st_snd;
wire [ 1:0] prio;
reg  [ 7:0] debug_mux;
// reg  [ 1:0] cpu_cfg;

assign debug_view = debug_mux;
assign ram_addr   = main_addr[13:1];
assign ram_we     = cpu_we;

always @(posedge clk) begin
    case( debug_bus[7:6] )
        0: debug_mux <= st_main;
        1: debug_mux <= st_video;
        2: debug_mux <= st_snd;
        3: debug_mux <= { 2'd0, prio, 3'd0, rmrd };
    endcase
end

// always @(posedge clk) begin
//     if( prog_addr==0 && prog_we && header )
//         cpu_cfg <= prog_data[2:1];
// end

// always @(*) begin
//     post_addr = prog_addr;
//     if( prog_ba[1] ) begin
//         post_addr[]
//     end
// end

/* verilator tracing_off */
jttmnt_main u_main(
    .rst            ( rst           ),
    .clk            ( clk           ),
    .LVBL           ( LVBL          ),

    // .cfg            ( cpu_cfg       ),
    .cpu_d8         ( cpu_d8        ),
    .cpu_we         ( cpu_we        ),
    .cpu_dout       ( ram_din       ),
    .odtac          ( odtac         ),
    .vdtac          ( vdtac         ),

    .main_addr      ( main_addr     ),
    .rom_data       ( main_data     ),
    .rom_cs         ( main_cs       ),
    .rom_ok         ( main_ok       ),
    // RAM
    .ram_dsn        ( ram_dsn       ),
    .ram_dout       ( ram_data      ),
    .ram_cs         ( ram_cs        ),
    .ram_ok         ( ram_ok        ),
    // cabinet I/O
    .start_button   ( start_button  ),
    .coin_input     ( coin_input    ),
    .joystick1      ( joystick1     ),
    .joystick2      ( joystick2     ),
    .joystick3      ( joystick3     ),
    .joystick4      ( joystick4     ),
    .service        ( service       ),

    .vram_dout      ( tilesys_dout  ),
    .oram_dout      ( objsys_dout   ),
    .pal_dout       ( pal_dout      ),
    // To video
    .prio           ( prio          ),
    .rmrd           ( rmrd          ),
    .obj_cs         ( objsys_cs     ),
    .vram_cs        ( tilesys_cs    ),
    .pal_we         ( pal_we        ),
    // To sound
    .snd_latch      ( snd_latch     ),
    .sndon          ( snd_irq       ),
    // DIP switches
    .dip_pause      ( dip_pause     ),
    .dipsw          ( dipsw[19:0]   ),
    // Debug
    .st_dout        ( st_main       )
);

/* verilator tracing_off */
jttmnt_sound u_sound(
    .rst        ( rst           ),
    .clk        ( clk           ),
    .cen_fm     ( cen_fm        ),
    .cen_fm2    ( cen_fm2       ),
    .cen_640    ( cen_640       ),
    .cen_20     ( cen_20        ),
    // communication with main CPU
    .snd_irq    ( snd_irq       ),
    .snd_latch  ( snd_latch     ),
    // ROM
    .rom_addr   ( snd_addr      ),
    .rom_cs     ( snd_cs        ),
    .rom_data   ( snd_data      ),
    .rom_ok     ( snd_ok        ),
    // ADPCM ROM
    .pcma_addr  ( pcma_addr     ),
    .pcma_dout  ( pcma_data     ),
    .pcma_cs    ( pcma_cs       ),
    .pcma_ok    ( pcma_ok       ),

    .pcmb_addr  ( pcmb_addr     ),
    .pcmb_dout  ( pcmb_data     ),
    .pcmb_cs    ( pcmb_cs       ),
    .pcmb_ok    ( pcmb_ok       ),

    .upd_addr   ( upd_addr      ),
    .upd_cs     ( upd_cs        ),
    .upd_data   ( upd_data      ),
    .upd_ok     ( upd_ok        ),
    // Title music
    .title_data ( title_data    ),
    .title_cs   ( title_cs      ),
    .title_addr ( title_addr    ),
    .title_ok   ( title_ok      ),
    // Sound output
    .snd        ( snd           ),
    .sample     ( sample        ),
    .peak       ( game_led      ),
    // Debug
    .debug_bus  ( debug_bus     ),
    .st_dout    ( st_snd        )
);

/* verilator tracing_on */
jttmnt_video u_video (
    .rst            ( rst           ),
    .rst8           ( rst8          ),
    .clk            ( clk           ),
    .pxl_cen        ( pxl_cen       ),
    .pxl2_cen       ( pxl2_cen      ),
    // .cfg            ( cpu_cfg       ),
    .cpu_prio       ( prio          ),

    .lhbl           ( LHBL          ),
    .lvbl           ( LVBL          ),
    .hs             ( HS            ),
    .vs             ( VS            ),
    .flip           ( dip_flip      ),
    // PROMs
    .prom_we        ( prom_we       ),
    .prog_addr      ( prog_addr[8:0]),
    .prog_data      ( prog_data[2:0]),
    // GFX - CPU interface
    .cpu_we         ( cpu_we        ),
    .objsys_cs      ( objsys_cs     ),
    .tilesys_cs     ( tilesys_cs    ),
    .pal_we         ( pal_we        ),
    .cpu_addr       (main_addr[16:1]),
    .cpu_dsn        ( ram_dsn       ),
    .cpu_dout       ( cpu_d8        ),
    .odtac          ( odtac         ),
    .vdtac          ( vdtac         ),
    .tilesys_dout   ( tilesys_dout  ),
    .objsys_dout    ( objsys_dout   ),
    .pal_dout       ( pal_dout      ),
    .rmrd           ( rmrd          ),
    // SDRAM
    .lyra_addr      ( lyra_addr     ),
    .lyrb_addr      ( lyrb_addr     ),
    .lyrf_addr      ( lyrf_addr     ),
    .lyro_addr      ( lyro_addr     ),
    .lyra_data      ( lyra_data     ),
    .lyrb_data      ( lyrb_data     ),
    .lyro_data      ( lyro_data     ),
    .lyrf_data      ( lyrf_data     ),
    .lyrf_cs        ( lyrf_cs       ),
    .lyra_cs        ( lyra_cs       ),
    .lyrb_cs        ( lyrb_cs       ),
    .lyro_cs        ( lyro_cs       ),
    .lyra_ok        ( lyra_ok       ),
    .lyro_ok        ( lyro_ok       ),
    // pixels
    .red            ( red           ),
    .green          ( green         ),
    .blue           ( blue          ),
    // Debug
    .debug_bus      ( debug_bus     ),
    .ioctl_addr     (ioctl_addr[14:0]),
    .ioctl_din      ( ioctl_din     ),
    .ioctl_ram      ( ioctl_ram     ),
    .gfx_en         ( gfx_en        ),
    .st_dout        ( st_video      )
);

endmodule