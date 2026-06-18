//============================================================================
// bldc_pwm_icb_top.v
//----------------------------------------------------------------------------
// Top level of the BLDC / PMSM PWM Custom IP, with a Hummingbird E203 ICB
// (Internal Chip Bus) slave interface so it can be mounted on the E203
// peripheral ICB fabric.
//
// ICB is a simple command / response bus with valid-ready handshakes:
//     command : icb_cmd_valid / icb_cmd_ready / icb_cmd_addr / icb_cmd_read /
//               icb_cmd_wdata / icb_cmd_wmask
//     response: icb_rsp_valid / icb_rsp_ready / icb_rsp_rdata / icb_rsp_err
// This slave keeps a single outstanding transaction (registers respond in the
// cycle after the command handshake), which matches the E203 peripheral ICB.
//
// 32-bit data bus, word-aligned registers. See README.md for the register map.
//============================================================================
module bldc_pwm_icb_top #(
    parameter AW = 12,    // peripheral address width (4 KB region)
    parameter DW = 32,
    parameter CW = 16
)(
    input  wire            clk,
    input  wire            rst_n,

    // ---------------- ICB slave ----------------
    input  wire            icb_cmd_valid,
    output wire            icb_cmd_ready,
    input  wire [AW-1:0]   icb_cmd_addr,
    input  wire            icb_cmd_read,    // 1 = read, 0 = write
    input  wire [DW-1:0]   icb_cmd_wdata,
    input  wire [DW/8-1:0] icb_cmd_wmask,
    output reg             icb_rsp_valid,
    input  wire            icb_rsp_ready,
    output reg  [DW-1:0]   icb_rsp_rdata,
    output wire            icb_rsp_err,

    // ---------------- IO ----------------
    input  wire            brk_in,          // external break / fault (async)
    output wire [5:0]      pwm_out,         // {Wl,Wh,Vl,Vh,Ul,Uh}
    output wire            irq              // level interrupt to PLIC
);

    //------------------------------------------------------------------
    // Register map (word index = addr[7:2])
    //------------------------------------------------------------------
    localparam [5:0]
        A_CTRL   = 6'h00, // 0x00 control
        A_STATUS = 6'h01, // 0x04 status (W1C flags)
        A_PSC    = 6'h02, // 0x08 prescaler-1
        A_ARR    = 6'h03, // 0x0C auto-reload (period)
        A_OCR0   = 6'h04, // 0x10
        A_OCR1   = 6'h05, // 0x14
        A_OCR2   = 6'h06, // 0x18
        A_OCR3   = 6'h07, // 0x1C
        A_OCR4   = 6'h08, // 0x20
        A_OCR5   = 6'h09, // 0x24
        A_DTG    = 6'h0A, // 0x28 dead-time {.. ,W,V,U}
        A_CNT    = 6'h0B, // 0x2C counter (RO)
        A_SFREQ  = 6'h0C, // 0x30 sine freq step (speed)
        A_SAMP   = 6'h0D, // 0x34 sine amplitude (drive strength)
        A_SCTRL  = 6'h0E, // 0x38 sine control
        A_ID     = 6'h0F; // 0x3C ID / version (RO)

    localparam [31:0] ID_VALUE = 32'hB1DC_0001;

    //------------------------------------------------------------------
    // ICB handshake (single outstanding)
    //------------------------------------------------------------------
    wire cmd_hsk = icb_cmd_valid & icb_cmd_ready;
    wire rsp_hsk = icb_rsp_valid & icb_rsp_ready;
    assign icb_cmd_ready = ~icb_rsp_valid;        // accept when no pending rsp
    assign icb_rsp_err   = 1'b0;

    wire        is_wr = cmd_hsk & ~icb_cmd_read;
    wire        is_rd = cmd_hsk &  icb_cmd_read;
    wire [5:0]  widx  = icb_cmd_addr[7:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) icb_rsp_valid <= 1'b0;
        else if (cmd_hsk) icb_rsp_valid <= 1'b1;
        else if (rsp_hsk) icb_rsp_valid <= 1'b0;
    end

    // byte-masked write helper
    function [31:0] wmsk;
        input [31:0] old;
        input [31:0] nw;
        input [3:0]  m;
        begin
            wmsk[7:0]   = m[0] ? nw[7:0]   : old[7:0];
            wmsk[15:8]  = m[1] ? nw[15:8]  : old[15:8];
            wmsk[23:16] = m[2] ? nw[23:16] : old[23:16];
            wmsk[31:24] = m[3] ? nw[31:24] : old[31:24];
        end
    endfunction

    //------------------------------------------------------------------
    // Registers
    //------------------------------------------------------------------
    reg [31:0] ctrl;     // [0]EN [1]MOE [2]CMS [3]AUTO [4]PRELOAD
                         // [5]UIE [6]BIE [7]BRK_POL [8]BRK_SW [9]INDEP [10]SVPWM
    reg [31:0] psc_r;
    reg [31:0] arr_r;
    reg [31:0] ocr_r [0:5];
    reg [31:0] dtg_r;
    reg [31:0] sfreq_r;
    reg [31:0] samp_r;
    reg [31:0] sctrl_r;  // reserved for future use

    wire en       = ctrl[0];
    wire moe      = ctrl[1];
    wire cms      = ctrl[2];
    wire auto_inj = ctrl[3];
    wire preload  = ctrl[4];
    wire uie      = ctrl[5];
    wire bie      = ctrl[6];
    wire brk_pol  = ctrl[7];
    wire brk_sw   = ctrl[8];
    wire indep    = ctrl[9];
    wire svpwm    = ctrl[10];

    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl    <= 32'h0;
            psc_r   <= 32'h0;
            arr_r   <= 32'd1023;          // default period
            for (k = 0; k < 6; k = k + 1) ocr_r[k] <= 32'h0;
            dtg_r   <= 32'h0;
            sfreq_r <= 32'h0;
            samp_r  <= 32'h0;
            sctrl_r <= 32'h0;
        end else if (is_wr) begin
            case (widx)
                A_CTRL : ctrl    <= wmsk(ctrl,    icb_cmd_wdata, icb_cmd_wmask);
                A_PSC  : psc_r   <= wmsk(psc_r,   icb_cmd_wdata, icb_cmd_wmask);
                A_ARR  : arr_r   <= wmsk(arr_r,   icb_cmd_wdata, icb_cmd_wmask);
                A_OCR0 : ocr_r[0]<= wmsk(ocr_r[0],icb_cmd_wdata, icb_cmd_wmask);
                A_OCR1 : ocr_r[1]<= wmsk(ocr_r[1],icb_cmd_wdata, icb_cmd_wmask);
                A_OCR2 : ocr_r[2]<= wmsk(ocr_r[2],icb_cmd_wdata, icb_cmd_wmask);
                A_OCR3 : ocr_r[3]<= wmsk(ocr_r[3],icb_cmd_wdata, icb_cmd_wmask);
                A_OCR4 : ocr_r[4]<= wmsk(ocr_r[4],icb_cmd_wdata, icb_cmd_wmask);
                A_OCR5 : ocr_r[5]<= wmsk(ocr_r[5],icb_cmd_wdata, icb_cmd_wmask);
                A_DTG  : dtg_r   <= wmsk(dtg_r,   icb_cmd_wdata, icb_cmd_wmask);
                A_SFREQ: sfreq_r <= wmsk(sfreq_r, icb_cmd_wdata, icb_cmd_wmask);
                A_SAMP : samp_r  <= wmsk(samp_r,  icb_cmd_wdata, icb_cmd_wmask);
                A_SCTRL: sctrl_r <= wmsk(sctrl_r, icb_cmd_wdata, icb_cmd_wmask);
                default: ; // CNT, STATUS, ID handled elsewhere / read-only
            endcase
        end
    end

    //------------------------------------------------------------------
    // Break synchronizer + W1C status flags (UIF / BIF)
    //------------------------------------------------------------------
    reg  brk_s1, brk_s2, brk_s3;
    wire brk_lvl = brk_in ^ brk_pol;          // normalize to active-high
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {brk_s1, brk_s2, brk_s3} <= 3'b0;
        else        {brk_s1, brk_s2, brk_s3} <= {brk_lvl, brk_s1, brk_s2};
    end
    wire brk_sync = brk_s2 | brk_sw;          // synchronized HW break OR sw break
    wire brk_rise = brk_s2 & ~brk_s3;         // rising edge for flag latch

    wire uev;
    reg  uif, bif;
    wire clr_uif = is_wr & (widx == A_STATUS) & icb_cmd_wmask[0] & icb_cmd_wdata[0];
    wire clr_bif = is_wr & (widx == A_STATUS) & icb_cmd_wmask[0] & icb_cmd_wdata[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uif <= 1'b0;
            bif <= 1'b0;
        end else begin
            if (uev)            uif <= 1'b1;     // set wins over clear
            else if (clr_uif)   uif <= 1'b0;

            if (brk_rise | brk_sw) bif <= 1'b1;
            else if (clr_bif)      bif <= 1'b0;
        end
    end

    assign irq = (uif & uie) | (bif & bie);

    //------------------------------------------------------------------
    // OCR source mux : auto injection feeds U/V/W (OCR0/2/4)
    //------------------------------------------------------------------
    wire [CW-1:0] inj_u, inj_v, inj_w;

    // Full-word reads of the OCR array, then bit-select on the net
    // (portable to strict Verilog-2001; avoids array[idx][range] in one ref).
    wire [31:0] ocr0f = ocr_r[0];
    wire [31:0] ocr1f = ocr_r[1];
    wire [31:0] ocr2f = ocr_r[2];
    wire [31:0] ocr3f = ocr_r[3];
    wire [31:0] ocr4f = ocr_r[4];
    wire [31:0] ocr5f = ocr_r[5];

    wire [CW-1:0] ocr0_mux = auto_inj ? inj_u : ocr0f[CW-1:0];
    wire [CW-1:0] ocr2_mux = auto_inj ? inj_v : ocr2f[CW-1:0];
    wire [CW-1:0] ocr4_mux = auto_inj ? inj_w : ocr4f[CW-1:0];

    wire [CW-1:0] cnt_val;

    //------------------------------------------------------------------
    // Timer core
    //------------------------------------------------------------------
    bldc_pwm_timer #(.CW(CW)) u_timer (
        .clk(clk), .rst_n(rst_n),
        .en(en), .cms(cms), .moe(moe),
        .preload_en(preload), .indep_mode(indep),
        .psc(psc_r[CW-1:0]), .arr(arr_r[CW-1:0]),
        .ocr0(ocr0_mux),          .ocr1(ocr1f[CW-1:0]),
        .ocr2(ocr2_mux),          .ocr3(ocr3f[CW-1:0]),
        .ocr4(ocr4_mux),          .ocr5(ocr5f[CW-1:0]),
        .dtg_u(dtg_r[7:0]), .dtg_v(dtg_r[15:8]), .dtg_w(dtg_r[23:16]),
        .brk(brk_sync),
        .uev(uev), .dir(), .cnt(cnt_val), .pwm_out(pwm_out)
    );

    //------------------------------------------------------------------
    // Sine / SVPWM injector (extended)
    //------------------------------------------------------------------
    bldc_sine_injector #(.CW(CW), .PAW(32), .LAW(8), .SW(16)) u_inj (
        .clk(clk), .rst_n(rst_n),
        .en(auto_inj), .load(uev), .svpwm_sel(svpwm),
        .freq_step(sfreq_r), .amp(samp_r[CW-1:0]), .arr(arr_r[CW-1:0]),
        .ocr_u(inj_u), .ocr_v(inj_v), .ocr_w(inj_w)
    );

    //------------------------------------------------------------------
    // Read data mux (registered at command handshake)
    //------------------------------------------------------------------
    reg [31:0] rdata_n;
    always @(*) begin
        case (widx)
            A_CTRL  : rdata_n = ctrl;
            A_STATUS: rdata_n = {27'b0, brk_sync, /*[4]*/
                                 1'b0,  /*[3] dir not exported here*/
                                 1'b0,  /*[2] reserved*/
                                 bif, uif};
            A_PSC   : rdata_n = psc_r;
            A_ARR   : rdata_n = arr_r;
            A_OCR0  : rdata_n = ocr_r[0];
            A_OCR1  : rdata_n = ocr_r[1];
            A_OCR2  : rdata_n = ocr_r[2];
            A_OCR3  : rdata_n = ocr_r[3];
            A_OCR4  : rdata_n = ocr_r[4];
            A_OCR5  : rdata_n = ocr_r[5];
            A_DTG   : rdata_n = dtg_r;
            A_CNT   : rdata_n = {{(32-CW){1'b0}}, cnt_val};
            A_SFREQ : rdata_n = sfreq_r;
            A_SAMP  : rdata_n = samp_r;
            A_SCTRL : rdata_n = sctrl_r;
            A_ID    : rdata_n = ID_VALUE;
            default : rdata_n = 32'h0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      icb_rsp_rdata <= 32'h0;
        else if (is_rd)  icb_rsp_rdata <= rdata_n;
    end

endmodule
