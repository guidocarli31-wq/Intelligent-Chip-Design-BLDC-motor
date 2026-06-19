//============================================================================
// bldc_standalone_top.v
//----------------------------------------------------------------------------
// Wrapper STANDALONE (nessun bus ICB / CPU) per portare in hardware sulla
// Tang Primer 20K la IP di guidocarli31-wq/Intelligent-Chip-Design-BLDC-motor.
//
// Istanzia direttamente bldc_sine_injector + bldc_pwm_timer (che a sua volta
// usa bldc_deadtime_gen) con parametri FISSI in questo file, al posto dei
// registri scritti via firmware. Comodo per il primo bring-up su hardware
// reale; in seguito, se vuoi controllo dinamico, puoi integrare
// bldc_pwm_icb_top.v in un vero SoC E203.
//
// MODALITA' "SAFE": indep_mode = 0 -> complementare + dead-time hardware
// (mai shoot-through). NON cambiarlo per i test con il driver di potenza.
//============================================================================
module bldc_standalone_top (
    input  wire clk,            // 27 MHz onboard, pin FISSO H11 (core board)
    input  wire rst_n_pin,      // reset di sistema, pin FISSO T10 (core board)
    input  wire run,            // assegna a un pin libero PMOD + pulsante/switch esterno (pull-up; 1 = motore abilitato)
    input  wire estop_n,        // assegna a un pin libero PMOD + pulsante esterno (pull-up; premuto = 0 = STOP immediato)
    output wire [5:0] pwm_out,  // [0]=U_H [1]=U_L [2]=V_H [3]=V_L [4]=W_H [5]=W_L -> verso il driver 6PWM
    output wire heartbeat_led   // opzionale: LED lampeggiante, conferma che il bitstream gira
);

    // ------------------------------------------------------------------
    // Parametri di bring-up (assumono clk = 27 MHz)
    //   carrier (centro-allineato, PSC=0): f_uev = clk / (2*(ARR+1))
    //     ARR = 674  ->  f_uev = 27e6 / 1350 = 20.000 kHz esatti
    //   freq. elettrica: f_e = f_uev * SFREQ / 2^32
    //     SFREQ = 214748  ->  f_e ~= 1.0 Hz (rotazione lenta, facile da vedere/misurare)
    //   dead-time = (DTG+1) cicli di clk @ 27 MHz
    //     DTG = 53  ->  ~2.0 us  *** DA VERIFICARE col datasheet del tuo driver ***
    //   AMP = ampiezza/coppia (max = ARR/2 = 337): parti basso
    // ------------------------------------------------------------------
    localparam [15:0] PSC_VAL   = 16'd0;
    localparam [15:0] ARR_VAL   = 16'd674;
    localparam [7:0]  DTG_VAL   = 8'd53;
    localparam [31:0] SFREQ_VAL = 32'd214748;
    localparam [15:0] AMP_VAL   = 16'd80;

    // ------------------------------------------------------------------
    // Power-on-reset interno, in AND col pin di reset di scheda: il
    // design parte pulito anche se rst_n_pin dovesse restare flottante.
    // ------------------------------------------------------------------
    reg [3:0] por_cnt = 4'd0;
    wire por_done = &por_cnt;
    always @(posedge clk)
        if (!por_done) por_cnt <= por_cnt + 1'b1;
    wire rst_n = rst_n_pin & por_done;

    // ------------------------------------------------------------------
    // Sincronizzazione ingressi asincroni (pulsanti esterni)
    // ------------------------------------------------------------------
    reg [1:0] run_sync, estopn_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_sync    <= 2'b00;
            estopn_sync <= 2'b11;
        end else begin
            run_sync    <= {run_sync[0],    run};
            estopn_sync <= {estopn_sync[0], estop_n};
        end
    end
    wire run_s = run_sync[1];
    wire brk   = ~estopn_sync[1];   // pulsante premuto -> BREAK attivo (tutte le uscite OFF)

    // ------------------------------------------------------------------
    // Iniettore seno/SVPWM -> genera ocr_u/v/w in automatico
    // ------------------------------------------------------------------
    wire uev;
    wire [15:0] ocr_u, ocr_v, ocr_w;

    bldc_sine_injector #(
        .CW(16), .PAW(32), .LAW(8), .SW(16)
    ) u_injector (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (run_s),
        .load      (uev),
        .svpwm_sel (1'b1),       // tabella SVPWM (un po' piu' di range utile della sinusoide pura)
        .freq_step (SFREQ_VAL),
        .amp       (AMP_VAL),
        .arr       (ARR_VAL),
        .ocr_u     (ocr_u),
        .ocr_v     (ocr_v),
        .ocr_w     (ocr_w)
    );

    // ------------------------------------------------------------------
    // Timer PWM + dead-time hardware + BREAK -> 6 uscite complementari
    // ------------------------------------------------------------------
    wire dir;
    wire [15:0] cnt;

    bldc_pwm_timer #(
        .CW(16)
    ) u_timer (
        .clk        (clk),
        .rst_n      (rst_n),
        .en         (run_s),
        .cms        (1'b1),      // centro-allineato
        .moe        (run_s),
        .preload_en (1'b1),      // aggiornamenti OCR glitch-free
        .indep_mode (1'b0),      // SAFE: complementare + dead-time hardware
        .psc        (PSC_VAL),
        .arr        (ARR_VAL),
        .ocr0       (ocr_u), .ocr1 (16'd0),
        .ocr2       (ocr_v), .ocr3 (16'd0),
        .ocr4       (ocr_w), .ocr5 (16'd0),
        .dtg_u      (DTG_VAL),
        .dtg_v      (DTG_VAL),
        .dtg_w      (DTG_VAL),
        .brk        (brk),
        .uev        (uev),
        .dir        (dir),
        .cnt        (cnt),
        .pwm_out    (pwm_out)
    );

    // Heartbeat: conferma visiva che l'FPGA e' programmata e viva
    reg [23:0] hb;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) hb <= 24'd0; else hb <= hb + 1'b1;
    assign heartbeat_led = hb[23];

endmodule
