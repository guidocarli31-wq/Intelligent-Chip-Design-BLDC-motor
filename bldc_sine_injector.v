//============================================================================
// bldc_sine_injector.v   (EXTENDED requirements 5, 6, 7)
//----------------------------------------------------------------------------
// Hardware automatic Output-Compare data injection for 3-phase SPWM / SVPWM.
//
//   * A 32-bit phase accumulator (NCO) advances by `freq_step` on every
//     Update Event from the timer. The top LAW bits index a sine look-up
//     table. Three samples 120 deg apart are read for phases U/V/W.
//        -> output electrical frequency  f_e = f_uev * freq_step / 2^32
//        -> SPEED CONTROL (req #7): change freq_step (or the UEV rate).
//
//   * `svpwm_sel` chooses between a pure sine table and a third-harmonic
//     injected table (classic min/max SVPWM shape) that extends the linear
//     modulation range by ~15%.
//
//   * Each signed sample is scaled by `amp` and centered around ARR/2:
//        OCR = ARR/2 + (amp * sample) / 2^15            (result clamped to [0,ARR])
//     -> DRIVING-STRENGTH / POWER CONTROL (req #6): change `amp`
//        (amp = modulation depth, 0 .. ARR/2 = 0..100%).
//
//   Results are registered at the Update Event so they line up with the
//   timer's OCR preload mechanism (glitch-free).
//
//   LUTs are signed Q15 (range -32768..32767) loaded from .hex files.
//============================================================================
module bldc_sine_injector #(
    parameter CW  = 16,    // compare width (matches timer)
    parameter PAW = 32,    // phase-accumulator width
    parameter LAW = 8,     // LUT address width (2^LAW entries)
    parameter SW  = 16     // signed sample width (Q15)
)(
    input  wire           clk,
    input  wire           rst_n,
    input  wire           en,         // 1 = auto-injection active
    input  wire           load,       // Update Event pulse from timer
    input  wire           svpwm_sel,  // 0 = sine, 1 = third-harmonic (SVPWM)
    input  wire [PAW-1:0] freq_step,  // phase increment per UEV  (speed)
    input  wire [CW-1:0]  amp,        // amplitude / drive strength (0..ARR/2)
    input  wire [CW-1:0]  arr,        // period top (for the ARR/2 midpoint)
    output reg  [CW-1:0]  ocr_u,
    output reg  [CW-1:0]  ocr_v,
    output reg  [CW-1:0]  ocr_w
);
    // 120 / 240 degree offsets in 32-bit accumulator units (2^32/3, 2*2^32/3)
    localparam [PAW-1:0] OFF120 = 32'h5555_5555;
    localparam [PAW-1:0] OFF240 = 32'hAAAA_AAAA;

    //------------------------------------------------------------------
    // Look-up tables (signed Q15), initialized from hex files
    //------------------------------------------------------------------
    reg signed [SW-1:0] sine_lut  [0:(1<<LAW)-1];
    reg signed [SW-1:0] svpwm_lut [0:(1<<LAW)-1];
    initial begin
        $readmemh("sine_lut.hex",  sine_lut);
        $readmemh("svpwm_lut.hex", svpwm_lut);
    end

    //------------------------------------------------------------------
    // Phase accumulator and table indices
    //------------------------------------------------------------------
    reg  [PAW-1:0] phase;
    wire [PAW-1:0] ph_u = phase;
    wire [PAW-1:0] ph_v = phase + OFF120;
    wire [PAW-1:0] ph_w = phase + OFF240;
    wire [LAW-1:0] idx_u = ph_u[PAW-1 -: LAW];
    wire [LAW-1:0] idx_v = ph_v[PAW-1 -: LAW];
    wire [LAW-1:0] idx_w = ph_w[PAW-1 -: LAW];

    wire signed [SW-1:0] s_u = svpwm_sel ? svpwm_lut[idx_u] : sine_lut[idx_u];
    wire signed [SW-1:0] s_v = svpwm_sel ? svpwm_lut[idx_v] : sine_lut[idx_v];
    wire signed [SW-1:0] s_w = svpwm_sel ? svpwm_lut[idx_w] : sine_lut[idx_w];

    //------------------------------------------------------------------
    // Scale + center :  OCR = ARR/2 + (amp * sample) >>> 15
    //------------------------------------------------------------------
    wire signed [SW:0]   amp_s = {1'b0, amp};            // amp as positive signed
    wire signed [SW+CW:0] prod_u = amp_s * s_u;          // signed product
    wire signed [SW+CW:0] prod_v = amp_s * s_v;
    wire signed [SW+CW:0] prod_w = amp_s * s_w;

    wire signed [CW+1:0] mid    = {2'b00, arr[CW-1:1]};  // ARR/2, non-negative
    wire signed [CW+1:0] calc_u = mid + (prod_u >>> 15);
    wire signed [CW+1:0] calc_v = mid + (prod_v >>> 15);
    wire signed [CW+1:0] calc_w = mid + (prod_w >>> 15);

    // Saturate a signed value into [0, ARR].
    // NOTE: test the MSB directly for "negative" -- a mixed signed/unsigned
    // comparison ( val < 0 ) would be evaluated unsigned and never fire.
    function [CW-1:0] clamp;
        input [CW+1:0] val;          // 2's-complement, MSB = sign
        input [CW-1:0] hi;
        begin
            if (val[CW+1])                       // negative -> 0
                clamp = {CW{1'b0}};
            else if (val[CW:0] > {1'b0, hi})     // > ARR  -> ARR
                clamp = hi;
            else
                clamp = val[CW-1:0];
        end
    endfunction

    //------------------------------------------------------------------
    // Register outputs and advance phase at the Update Event
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= {PAW{1'b0}};
            ocr_u <= {1'b0, arr[CW-1:1]};
            ocr_v <= {1'b0, arr[CW-1:1]};
            ocr_w <= {1'b0, arr[CW-1:1]};
        end else if (!en) begin
            // Idle: 50% duty (ARR/2) on all phases -> zero differential drive.
            phase <= {PAW{1'b0}};
            ocr_u <= {1'b0, arr[CW-1:1]};
            ocr_v <= {1'b0, arr[CW-1:1]};
            ocr_w <= {1'b0, arr[CW-1:1]};
        end else if (load) begin
            ocr_u <= clamp(calc_u, arr);
            ocr_v <= clamp(calc_v, arr);
            ocr_w <= clamp(calc_w, arr);
            phase <= phase + freq_step;
        end
    end
endmodule
