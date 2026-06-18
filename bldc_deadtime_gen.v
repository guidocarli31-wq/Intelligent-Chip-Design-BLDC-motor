//============================================================================
// bldc_deadtime_gen.v
//----------------------------------------------------------------------------
// Complementary output + programmable dead-time generator for ONE half-bridge.
//
// Given a reference PWM level (ref_in = desired high-side ON level), this block
// produces a complementary high/low gate pair (oh, ol) and inserts a dead-time
// (blanking window) of `dt` clock cycles on EVERY edge so the two transistors
// are never on at the same time. This is the safety-critical block required by
// basic requirement #2 ("dead zone configuration for each phase").
//
//   ref_in : __----____----__
//   oh     : ___---_____---__   (rising edge delayed by dt)
//   ol     : --___----___----   (rising edge delayed by dt)
//            both LOW during the dt window  => no shoot-through
//
// `ena` low forces BOTH gates OFF immediately (used by BREAK / MOE).
// Effective dead-time is approximately (dt + 1) clk cycles; set dt=0 for the
// minimum 1-cycle blanking.
//============================================================================
module bldc_deadtime_gen (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ena,        // 1 = run, 0 = force safe state (both off)
    input  wire        ref_in,     // reference high-side level
    input  wire [7:0]  dt,         // dead-time in clk cycles
    output reg         oh,         // high-side gate
    output reg         ol          // low-side gate
);
    reg        ref_d;              // delayed reference (edge detect)
    reg [7:0]  dcnt;               // dead-time down-counter
    reg        in_dt;              // 1 = inside dead-time blanking window

    wire edge_det = (ref_in ^ ref_d);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ref_d <= 1'b0;
            dcnt  <= 8'd0;
            in_dt <= 1'b0;
            oh    <= 1'b0;
            ol    <= 1'b0;
        end else if (!ena) begin
            // Forced safe state: both transistors off (motor freewheels).
            ref_d <= ref_in;
            dcnt  <= 8'd0;
            in_dt <= 1'b0;
            oh    <= 1'b0;
            ol    <= 1'b0;
        end else begin
            ref_d <= ref_in;
            if (edge_det) begin
                // Edge on the reference: blank both outputs for dt cycles.
                oh <= 1'b0;
                ol <= 1'b0;
                if (dt != 8'd0) begin
                    in_dt <= 1'b1;
                    dcnt  <= dt;
                end else begin
                    in_dt <= 1'b0;
                    dcnt  <= 8'd0;
                end
            end else if (in_dt) begin
                oh <= 1'b0;
                ol <= 1'b0;
                if (dcnt > 8'd1)
                    dcnt <= dcnt - 8'd1;
                else begin
                    in_dt <= 1'b0;
                    dcnt  <= 8'd0;
                end
            end else begin
                // Steady state: drive the complementary pair.
                oh <= ref_in;
                ol <= ~ref_in;
            end
        end
    end
endmodule
