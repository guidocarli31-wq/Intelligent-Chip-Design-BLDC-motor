//============================================================================
// tb_bldc_pwm.v  -- smoke testbench for the BLDC PWM Custom IP
//
// Run (Icarus Verilog), from this sim/ directory so the .hex LUTs are found:
//     iverilog -g2012 -o sim.out ../rtl/*.v tb_bldc_pwm.v
//     vvp sim.out
//     gtkwave tb_bldc_pwm.vcd      (optional)
//
// Exercises: manual complementary PWM + dead-time, BREAK shutdown,
// and automatic sine/SVPWM injection.
//============================================================================
`timescale 1ns/1ps
module tb_bldc_pwm;

    localparam AW = 12, DW = 32, CW = 16;

    reg              clk = 0, rst_n = 0;
    reg              icb_cmd_valid = 0, icb_cmd_read = 0;
    reg  [AW-1:0]    icb_cmd_addr  = 0;
    reg  [DW-1:0]    icb_cmd_wdata = 0;
    reg  [DW/8-1:0]  icb_cmd_wmask = 0;
    reg              icb_rsp_ready = 0;
    wire             icb_cmd_ready, icb_rsp_valid, icb_rsp_err;
    wire [DW-1:0]    icb_rsp_rdata;

    reg              brk_in = 0;
    wire [5:0]       pwm_out;
    wire             irq;

    // register offsets
    localparam CTRL=12'h00, STATUS=12'h04, PSC=12'h08, ARR=12'h0C,
               OCR0=12'h10, OCR2=12'h18, OCR4=12'h20, DTG=12'h28,
               CNT=12'h2C, SFREQ=12'h30, SAMP=12'h34, ID=12'h3C;

    bldc_pwm_icb_top #(.AW(AW), .DW(DW), .CW(CW)) dut (
        .clk(clk), .rst_n(rst_n),
        .icb_cmd_valid(icb_cmd_valid), .icb_cmd_ready(icb_cmd_ready),
        .icb_cmd_addr(icb_cmd_addr), .icb_cmd_read(icb_cmd_read),
        .icb_cmd_wdata(icb_cmd_wdata), .icb_cmd_wmask(icb_cmd_wmask),
        .icb_rsp_valid(icb_rsp_valid), .icb_rsp_ready(icb_rsp_ready),
        .icb_rsp_rdata(icb_rsp_rdata), .icb_rsp_err(icb_rsp_err),
        .brk_in(brk_in), .pwm_out(pwm_out), .irq(irq)
    );

    always #5 clk = ~clk;   // 100 MHz

    reg [31:0] rdbk;

    task icb_write;
        input [AW-1:0] addr;
        input [31:0]   data;
        begin
            @(negedge clk);
            icb_cmd_valid = 1; icb_cmd_addr = addr; icb_cmd_read = 0;
            icb_cmd_wdata = data; icb_cmd_wmask = 4'hF; icb_rsp_ready = 1;
            @(posedge clk);
            while (!icb_cmd_ready) @(posedge clk);
            @(negedge clk);
            icb_cmd_valid = 0;
            @(posedge clk);
            while (!icb_rsp_valid) @(posedge clk);
        end
    endtask

    task icb_read;
        input  [AW-1:0] addr;
        output [31:0]   data;
        begin
            @(negedge clk);
            icb_cmd_valid = 1; icb_cmd_addr = addr; icb_cmd_read = 1;
            icb_cmd_wmask = 4'h0; icb_rsp_ready = 1;
            @(posedge clk);
            while (!icb_cmd_ready) @(posedge clk);
            @(negedge clk);
            icb_cmd_valid = 0;
            @(posedge clk);
            while (!icb_rsp_valid) @(posedge clk);
            data = icb_rsp_rdata;
        end
    endtask

    // continuous shoot-through check in complementary mode
    integer shoot = 0;
    always @(posedge clk) begin
        if (rst_n && !dut.indep) begin
            if (pwm_out[0] & pwm_out[1]) shoot = shoot + 1; // U hi&lo
            if (pwm_out[2] & pwm_out[3]) shoot = shoot + 1; // V hi&lo
            if (pwm_out[4] & pwm_out[5]) shoot = shoot + 1; // W hi&lo
        end
    end

    initial begin
        $dumpfile("tb_bldc_pwm.vcd");
        $dumpvars(0, tb_bldc_pwm);

        // reset
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // sanity: read ID
        icb_read(ID, rdbk);
        $display("[%0t] ID = %h (expect B1DC0001)", $time, rdbk);

        //----------------------------------------------------------------
        // TEST 1 : manual complementary PWM + dead-time
        //----------------------------------------------------------------
        icb_write(PSC,  32'd0);            // no prescale
        icb_write(ARR,  32'd199);          // 200-tick edge-aligned period
        icb_write(DTG,  32'h00_0A_0A_0A);  // 10-cycle dead-time U/V/W
        icb_write(OCR0, 32'd60);           // U ~30%
        icb_write(OCR2, 32'd100);          // V 50%
        icb_write(OCR4, 32'd140);          // W ~70%
        // CTRL: EN | MOE | PRELOAD  (edge-aligned, complementary)
        icb_write(CTRL, (1<<0)|(1<<1)|(1<<4));
        repeat (1200) @(posedge clk);      // a few PWM periods

        //----------------------------------------------------------------
        // TEST 2 : BREAK
        //----------------------------------------------------------------
        $display("[%0t] asserting BREAK", $time);
        brk_in = 1;
        repeat (5) @(posedge clk);
        if (pwm_out !== 6'b000000)
            $display("  ** ERROR: outputs not all-off during break: %b", pwm_out);
        else
            $display("  OK: all outputs off during break");
        brk_in = 0;
        // clear BIF and re-enable main output
        icb_write(STATUS, 32'h2);
        repeat (200) @(posedge clk);

        //----------------------------------------------------------------
        // TEST 3 : automatic sine / SVPWM injection
        //----------------------------------------------------------------
        icb_write(SAMP,  32'd90);          // amplitude (drive strength)
        icb_write(SFREQ, 32'd20000000);    // phase step (speed)
        // CTRL: EN|MOE|CMS|AUTO|PRELOAD|SVPWM
        icb_write(CTRL, (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<10));
        repeat (4000) @(posedge clk);
        icb_read(CNT, rdbk);
        $display("[%0t] running auto-inject, CNT=%0d", $time, rdbk);

        $display("[%0t] shoot-through violations = %0d (expect 0)", $time, shoot);
        $display("TEST DONE");
        $finish;
    end

endmodule
