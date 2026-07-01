//============================================================================
// bldc_selftest_top.v  -- STANDALONE hardware self-test for the BLDC PWM IP.
//
// No E203, no firmware. A small hard-wired state machine performs the same
// register writes the CPU would do (PSC/ARR/DTG/SAMP/SFREQ/CTRL) over the ICB
// port, then the IP free-runs in hardware SVPWM mode. Use this to verify the
// IP synthesizes and produces correct PWM on the real FPGA.
//
// What you should see:
//   * pwm_out[5:0] on a scope: 3 complementary pairs with dead-time gaps.
//   * led[2:0] "breathe" (brightness follows each phase's sine duty), the
//     three phases 120 deg apart, cycling at ~2 Hz. That breathing IS the
//     hardware sine/SVPWM injection working. (No motor/driver needed for this.)
//
// Board: Sipeed Tang Primer 20K, 27 MHz oscillator -> clk_in.
//============================================================================
module bldc_selftest_top #(
    parameter CLK_HZ = 27000000    // Tang Primer 20K oscillator (for reference)
)(
    input  wire       clk_in,      // 27 MHz board clock
    output wire [5:0] pwm_out,     // to scope header / gate-driver inputs
    output wire [2:0] led          // 3 on-board LEDs = the 3 high-side phases
);
    wire clk = clk_in;

    //------------------------------------------------------------------
    // Power-on reset: hold reset for 65536 cycles after config, then release.
    // (No external reset pin -> avoids the dedicated SSPI config pad.)
    //------------------------------------------------------------------
    reg [15:0] por = 16'd0;
    reg        rst_n_r = 1'b0;
    always @(posedge clk) begin
        if (por != 16'hFFFF) begin
            por     <= por + 16'd1;
            rst_n_r <= 1'b0;
        end else begin
            rst_n_r <= 1'b1;
        end
    end
    wire rst_n = rst_n_r;

    //------------------------------------------------------------------
    // ICB master wires
    //------------------------------------------------------------------
    reg         cmd_valid;
    wire        cmd_ready;
    reg  [11:0] cmd_addr;
    reg  [31:0] cmd_wdata;
    wire        rsp_valid;
    wire [31:0] rsp_rdata;

    //------------------------------------------------------------------
    // Configuration sequence (address, data). CTRL written last.
    //   ARR=1350 @27MHz center-aligned -> ~10 kHz carrier / update rate.
    //   SFREQ=858993 -> ~2 Hz electrical (visible LED breathing).
    //   SAMP=405 (~60% of ARR/2). DTG=8 clk (~296 ns) per phase.
    //------------------------------------------------------------------
    localparam N = 6;
    function [43:0] cfg;             // {addr[11:0], data[31:0]}
        input [2:0] idx;
        begin
            case (idx)
                3'd0: cfg = {12'h008, 32'd0};          // PSC   = 0
                3'd1: cfg = {12'h00C, 32'd1350};       // ARR   = 1350
                3'd2: cfg = {12'h028, 32'h0008_0808};  // DTG   = 8/8/8
                3'd3: cfg = {12'h034, 32'd405};        // SAMP  = 405
                3'd4: cfg = {12'h030, 32'd858993};     // SFREQ = ~2 Hz
                3'd5: cfg = {12'h000, 32'h0000_041F};  // CTRL  = EN|MOE|CMS|AUTO|PRELOAD|SVPWM
                default: cfg = {12'h000, 32'h0};
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // Simple ICB write FSM: issue the 6 writes once, then idle.
    //------------------------------------------------------------------
    localparam S_SET = 2'd0, S_WAITC = 2'd1, S_WAITR = 2'd2, S_DONE = 2'd3;
    reg [1:0] st;
    reg [2:0] idx;
    wire [43:0] cur = cfg(idx);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_SET;
            idx       <= 3'd0;
            cmd_valid <= 1'b0;
            cmd_addr  <= 12'd0;
            cmd_wdata <= 32'd0;
        end else begin
            case (st)
                S_SET: begin
                    cmd_addr  <= cur[43:32];
                    cmd_wdata <= cur[31:0];
                    cmd_valid <= 1'b1;
                    st        <= S_WAITC;
                end
                S_WAITC: begin
                    if (cmd_valid && cmd_ready) begin   // command accepted
                        cmd_valid <= 1'b0;
                        st        <= S_WAITR;
                    end
                end
                S_WAITR: begin
                    if (rsp_valid) begin                // response received
                        if (idx == N-1) st <= S_DONE;
                        else begin idx <= idx + 3'd1; st <= S_SET; end
                    end
                end
                default: st <= S_DONE;                  // S_DONE: hold
            endcase
        end
    end

    //------------------------------------------------------------------
    // The IP under test
    //------------------------------------------------------------------
    bldc_pwm_icb_top #(.AW(12), .DW(32), .CW(16)) u_dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .icb_cmd_valid (cmd_valid),
        .icb_cmd_ready (cmd_ready),
        .icb_cmd_addr  (cmd_addr),
        .icb_cmd_read  (1'b0),
        .icb_cmd_wdata (cmd_wdata),
        .icb_cmd_wmask (4'hF),
        .icb_rsp_valid (rsp_valid),
        .icb_rsp_ready (1'b1),
        .icb_rsp_rdata (rsp_rdata),
        .icb_rsp_err   (),
        .brk_in        (1'b0),        // no break during self-test
        .pwm_out       (pwm_out),
        .irq           ()
    );

    //------------------------------------------------------------------
    // 3 LEDs show the three HIGH-SIDE phase signals -> they breathe 120 deg
    // apart. Tang Primer 20K dock LEDs are ACTIVE-LOW (lit = 0), so invert.
    // If yours are active-high, drop the '~'.
    //------------------------------------------------------------------
    assign led = ~{pwm_out[4], pwm_out[2], pwm_out[0]};   // {W_h, V_h, U_h}

endmodule
