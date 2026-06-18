//============================================================================
// bldc_pwm_timer.v
//----------------------------------------------------------------------------
// 16-bit advanced timer core for 3-phase BLDC / PMSM drive.
//
//   * 16-bit up / up-down counter with 16-bit prescaler.
//   * Edge-aligned (saw-tooth) or center-aligned (triangular) carrier.
//       - edge-aligned  : period = (ARR + 1) * (PSC + 1) clk cycles
//       - center-aligned : period = 2*ARR * (PSC + 1) clk cycles
//   * 6 Output-Compare values (OCR0..OCR5).
//   * Two output modes:
//       - Complementary mode (default, SAFE): phases U/V/W use OCR0/OCR2/OCR4.
//         Each phase reference is split into a complementary gate pair with a
//         hardware-inserted, per-phase dead-time (no shoot-through possible).
//       - Independent mode: OCR0..OCR5 drive the 6 outputs directly with NO
//         hardware complement / dead-time (software owns the gap). Flexible
//         but unsafe for a real power stage -- use for bench testing only.
//   * OCR preload (shadow) -- new compare values take effect only at the
//     Update Event (UEV), giving glitch-free periodic PWM data updating
//     (basic requirement #3).
//   * BREAK input forces ALL outputs to the safe (all-off) state combinatorially
//     for fastest possible shutdown (basic requirement #4).
//
// Output bit map (pwm_out):
//     [0] = U high   [1] = U low
//     [2] = V high   [3] = V low
//     [4] = W high   [5] = W low
//============================================================================
module bldc_pwm_timer #(
    parameter CW = 16                 // counter / compare width
)(
    input  wire           clk,
    input  wire           rst_n,
    // control
    input  wire           en,         // counter enable
    input  wire           cms,        // 1 = center-aligned, 0 = edge-aligned
    input  wire           moe,        // main output enable
    input  wire           preload_en, // 1 = OCR shadow updates only at UEV
    input  wire           indep_mode, // 1 = 6 independent outputs (no HW DT)
    // timing
    input  wire [CW-1:0]  psc,        // prescaler-1 (clk / (psc+1))
    input  wire [CW-1:0]  arr,        // auto-reload (period top)
    // compare values (already muxed manual/auto upstream)
    input  wire [CW-1:0]  ocr0,
    input  wire [CW-1:0]  ocr1,
    input  wire [CW-1:0]  ocr2,
    input  wire [CW-1:0]  ocr3,
    input  wire [CW-1:0]  ocr4,
    input  wire [CW-1:0]  ocr5,
    // per-phase dead-time
    input  wire [7:0]     dtg_u,
    input  wire [7:0]     dtg_v,
    input  wire [7:0]     dtg_w,
    // break (synchronized, active high)
    input  wire           brk,
    // status / outputs
    output reg            uev,        // update-event, 1 clk pulse
    output wire           dir,        // 0 = counting up, 1 = counting down
    output wire [CW-1:0]  cnt,        // current counter value
    output wire [5:0]     pwm_out     // see header for bit map
);

    //------------------------------------------------------------------
    // Prescaler
    //------------------------------------------------------------------
    reg [CW-1:0] psc_cnt;
    wire tick = (psc_cnt == psc);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        psc_cnt <= {CW{1'b0}};
        else if (!en)      psc_cnt <= {CW{1'b0}};
        else if (tick)     psc_cnt <= {CW{1'b0}};
        else               psc_cnt <= psc_cnt + 1'b1;
    end

    //------------------------------------------------------------------
    // Main counter (up or up/down) + Update Event
    //------------------------------------------------------------------
    reg [CW-1:0] cnt_r;
    reg          dir_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_r <= {CW{1'b0}};
            dir_r <= 1'b0;
            uev   <= 1'b0;
        end else if (!en) begin
            cnt_r <= {CW{1'b0}};
            dir_r <= 1'b0;
            uev   <= 1'b0;
        end else begin
            uev <= 1'b0;
            if (tick) begin
                if (!cms) begin
                    // Edge-aligned, count up and wrap.
                    if (cnt_r >= arr) begin
                        cnt_r <= {CW{1'b0}};
                        uev   <= 1'b1;
                    end else begin
                        cnt_r <= cnt_r + 1'b1;
                    end
                end else begin
                    // Center-aligned, count up to ARR then down to 0.
                    if (!dir_r) begin
                        if (cnt_r >= arr) begin
                            dir_r <= 1'b1;
                            cnt_r <= cnt_r - 1'b1;
                        end else begin
                            cnt_r <= cnt_r + 1'b1;
                        end
                    end else begin
                        if (cnt_r == {CW{1'b0}}) begin // reached bottom
                            dir_r <= 1'b0;
                            cnt_r <= cnt_r + 1'b1;
                            uev   <= 1'b1;          // UEV at the bottom
                        end else begin
                            cnt_r <= cnt_r - 1'b1;
                        end
                    end
                end
            end
        end
    end

    assign cnt = cnt_r;
    assign dir = dir_r;

    //------------------------------------------------------------------
    // OCR shadow (preload) registers
    //   preload_en = 0 : transparent (new value used after 1 clk)
    //   preload_en = 1 : value latched only at UEV  -> glitch-free
    //------------------------------------------------------------------
    reg [CW-1:0] s0, s1, s2, s3, s4, s5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0 <= {CW{1'b0}}; s1 <= {CW{1'b0}}; s2 <= {CW{1'b0}};
            s3 <= {CW{1'b0}}; s4 <= {CW{1'b0}}; s5 <= {CW{1'b0}};
        end else if (!preload_en || uev) begin
            s0 <= ocr0; s1 <= ocr1; s2 <= ocr2;
            s3 <= ocr3; s4 <= ocr4; s5 <= ocr5;
        end
    end

    //------------------------------------------------------------------
    // Compare: active-high while counter < compare value
    //------------------------------------------------------------------
    wire cmp0 = (cnt_r < s0);
    wire cmp1 = (cnt_r < s1);
    wire cmp2 = (cnt_r < s2);
    wire cmp3 = (cnt_r < s3);
    wire cmp4 = (cnt_r < s4);
    wire cmp5 = (cnt_r < s5);

    //------------------------------------------------------------------
    // Output enable -- combinational so BREAK shuts down ASAP
    //------------------------------------------------------------------
    wire out_en = moe & ~brk;

    //------------------------------------------------------------------
    // Complementary mode: 3 phases from OCR0/OCR2/OCR4 + dead-time
    //------------------------------------------------------------------
    wire u_h, u_l, v_h, v_l, w_h, w_l;

    bldc_deadtime_gen u_dt_u (
        .clk(clk), .rst_n(rst_n), .ena(out_en),
        .ref_in(cmp0), .dt(dtg_u), .oh(u_h), .ol(u_l)
    );
    bldc_deadtime_gen u_dt_v (
        .clk(clk), .rst_n(rst_n), .ena(out_en),
        .ref_in(cmp2), .dt(dtg_v), .oh(v_h), .ol(v_l)
    );
    bldc_deadtime_gen u_dt_w (
        .clk(clk), .rst_n(rst_n), .ena(out_en),
        .ref_in(cmp4), .dt(dtg_w), .oh(w_h), .ol(w_l)
    );

    wire [5:0] cmpl_out = {w_l, w_h, v_l, v_h, u_l, u_h};
    wire [5:0] indep_out = {cmp5, cmp4, cmp3, cmp2, cmp1, cmp0};

    // Final hard mask: any time outputs are disabled, force all gates OFF.
    assign pwm_out = {6{out_en}} & (indep_mode ? indep_out : cmpl_out);

endmodule
