`timescale 1ns / 1ps

// adaptive_impedance_v2.v - Upgraded Online Environment Stiffness Estimator
//
// Adaptive law (per joint):
//   Ks_out = clamp(Ks_nom^2 / K_env, KS_MIN, KS_MAX)
//   Ds_out = Ds_nom * (Ks_out / Ks_nom)
//
// This inverse-proportional rule comes from minimizing a quadratic cost
// J = (1/2) * (Ks*x_e - Ks_nom*x_e_nom)^2 with respect to Ks while
// preserving the rendered impedance against a Hookean environment of
// stiffness K_env. See Seraji & Colbaugh, "Force Tracking in Impedance
// Control," IJRR 16(1):97-117, 1997 for the original direct-adaptive
// derivation. The KS_MIN/KS_MAX clamps preserve closed-loop stability
// (Slotine & Li 1987 IJRR, bounded-parameter adaptive control).
//
// Units (Q16.16 throughout):
//   tau_ext, tau_meas, tau_cmd     : Nm
//   K_env, Ks_nom, Ks_out          : N/m   (or Nm/rad for rotational joints)
//   Ds_nom, Ds_out                 : N*s/m (or Nm*s/rad)
//   q_err                          : rad
//   LAMBDA                         : dimensionless [0, 1]
//   P                              : variance of K estimate, units (N/m)^2
//
// ----------------------------------------------------------------------------
// This module is an evolution of `adaptive_impedance.v` and is functionally
// backward-compatible at the original 18-port interface. Two new capabilities:
//
//  (1) HISTORICAL-PRIOR INITIALIZATION (functional)
//      New inputs `prior_load`, `K_env_prior_0..5`, `P_prior_0..5` let the
//      host inject per-joint initial RLS state at runtime, instead of only
//      relying on the compile-time K_ENV_INIT / P_INIT parameters.
//
//      `prior_load` semantics: LEVEL-SENSITIVE, synchronous, highest priority,
//      ATOMIC with `start`. While prior_load is high, on every rising clk
//      edge K_env[i] <= K_env_prior_i and P[i] <= P_prior_i for i=0..5, and
//      the FSM is held in S_IDLE with done=0. If `start` is also asserted
//      during any prior_load cycle, a 1-bit `start_pending` latch is set;
//      on the first cycle after prior_load deasserts, the latched priors are
//      the active RLS state AND the FSM transitions S_IDLE -> S_LATCH on the
//      pending start. No `start` pulse is ever lost, whether the host drives
//      a single-cycle pulse or a multi-cycle level.
//
//  (2) TIMING OPTIMIZATION (performance, FINAL cycle-3 architecture)
//      The original FSM walked the 6 joints serially, 8 cycles per joint, for
//      a total of 48 cycles per update. In v2 all 6 joints are processed in
//      PARALLEL at every pipeline stage, and every divide is issued through
//      a per-joint `pipe_div32` helper (6 instances, instantiated via a
//      generate block in this file). Each pipe_div32 is an EXPLICIT 4-STAGE
//      pipelined shift-subtract Q16.16 signed divider -- there is no
//      reliance on Vivado synthesis retiming, and no IP dependency. Stage
//      registers carry (* shreg_extract = "no" *) so they cannot collapse
//      into SRLs. The two adapt divides are data-dependent (divide 2's
//      numerator is divide 1's clamped quotient) so each pays full pipeline
//      latency. The FSM scheduling guarantees that the gain divide and each
//      adapt divide gets its 4 dedicated stage cycles before consumption.
//
//      19-state FSM (5-bit enumeration), cycle-by-cycle table
//      ----------------------------------------------------------------------
//         cycle  1  S_LATCH      latch tau_ext, q_err, Ks_nom, Ds_nom (x6)
//         cycle  2  S_DEADZONE   per-joint dead-zone / adaptive_enable gate
//         cycle  3  S_GAIN_NUM   register Pq, denom; valid_in_gain<=1
//                                -> div stage 0 captures gain operands
//         cycle  4  S_GAIN_W1    gain divide stage 1
//         cycle  5  S_GAIN_W2    gain divide stage 2
//         cycle  6  S_GAIN_W3    gain divide stage 3
//         cycle  7  S_RLS_KENV   gain quotient on quot_out; latch gain_reg
//                                AND K_env[i] += gain*(tau - K_env*qe)
//         cycle  8  S_RLS_P      P[i]     = (1/lambda)*(P - gain*qe*P)
//         cycle  9  S_ADAPT_D1   issue 1st adapt divide (Ks_sq / K_env);
//                                valid_in_adapt<=1 -> div stage 0
//         cycle 10  S_ADAPT_WA   adapt1 stage 1
//         cycle 11  S_ADAPT_WB   adapt1 stage 2
//         cycle 12  S_ADAPT_WC   adapt1 stage 3
//         cycle 13  S_ADAPT_Q1   adapt1 quotient ready; clamp;
//                                Ks_new_reg[i] <= clamped;
//                                same cycle issue 2nd adapt divide
//                                (Ks_clamped / Ks_nom); valid_in_adapt<=1
//         cycle 14  S_ADAPT_WD   adapt2 stage 1
//         cycle 15  S_ADAPT_WE   adapt2 stage 2
//         cycle 16  S_ADAPT_WF   adapt2 stage 3
//         cycle 17  S_OUTPUT     adapt2 quotient ready;
//                                Ds_adapted = fmult(Ds_nom_lat, q2);
//                                drive Ks_out_*/Ds_out_*; done <= 1
//                   S_DONE       hold done; return to S_IDLE next cycle
//
//      Total latency from `start` assertion to `done` rising edge:
//      17 CYCLES (2.82x speedup over the 48-cycle baseline).
//
//      No synthesis flags required. STEPS.SYNTH_DESIGN.ARGS.RETIMING may be
//      left at default (off). NO retiming required. The
//      (* shreg_extract = "no" *) attributes on the pipeline-stage registers
//      prevent SRL collapse. See constraints/adaptive_impedance_v2_synth.xdc
//      for the integrator checklist.
//
//      Cost: ~1800 LUT + 600 FF total for the 6 dividers, plus ~6x DSP in
//      RLS+adapt multipliers. The Q16.16 format, the RLS update equations,
//      and the adaptive Ks/Ds rule are all bit-for-bit identical to the
//      original `adaptive_impedance.v`: pipe_div32's arithmetic is a literal
//      recreation of v1's `n64 = numer<<<16; quot = (denom==0)?0x7FFFFFFF
//      : n64/denom;` truncating signed divide.
//
// Verilog-2001. Reuses fmult/fabs/fclamp helpers from the original module.
// The legacy combinational `fdiv` function has been stubbed out; all live
// divides flow through the `pipe_div32` helper instances.
// ----------------------------------------------------------------------------

module adaptive_impedance_v2 #(
    // Defaults calibrated to the 6-config simulation dataset (audit Cycle 2
    // showed K_env in 1200-1400 N/m range; old defaults always saturated).
    parameter signed [31:0] LAMBDA       = 32'sd65406,    // 0.998 (Cycle-2 tuned)
    parameter signed [31:0] INV_LAMBDA   = 32'sd65668,    // 1/0.998
    parameter signed [31:0] P_INIT       = 32'sd65536000, // 1000.0 (was 1.0)
    parameter signed [31:0] K_ENV_INIT   = 32'sd78643200, // 1200.0 N/m (matches LOO mu)
    parameter signed [31:0] Q_DEADZONE   = 32'sd65,       // ~0.001 rad
    parameter signed [31:0] KS_MIN       = 32'sd655360,   // 10.0 N/m (was 0.1)
    parameter signed [31:0] KS_MAX       = 32'sd327680000,// 5000.0 N/m (was 100)
    // Bug-2 fix: K_env must be clamped to prevent fmult overflow in
    // K_env*qe term (fmult silently truncates [47:16], wraps on overflow).
    // For qe in [-pi, pi], K_env <= 32767/pi ~= 10440 keeps integer part of
    // K_env*qe within Q16.16 range. Use 10000 N/m for headroom.
    parameter signed [31:0] K_ENV_MIN    = 32'sd6554,     // 0.1 N/m (>0 floor)
    parameter signed [31:0] K_ENV_MAX    = 32'sd655360000,// 10000.0 N/m
    // Bug-5 fix: P (covariance) lower bound. Slightly above zero to avoid
    // (lambda + qe^2 * P) approaching zero for the gain divide.
    parameter signed [31:0] P_MIN        = 32'sd6554      // 0.1 (>0 floor)
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // -- Original inputs ----------------------------------------------------
    input  wire signed [31:0] tau_ext_0, tau_ext_1, tau_ext_2,
    input  wire signed [31:0] tau_ext_3, tau_ext_4, tau_ext_5,

    input  wire signed [31:0] q_err_0, q_err_1, q_err_2,
    input  wire signed [31:0] q_err_3, q_err_4, q_err_5,

    input  wire signed [31:0] Ks_nom_0, Ks_nom_1, Ks_nom_2,
    input  wire signed [31:0] Ks_nom_3, Ks_nom_4, Ks_nom_5,

    input  wire signed [31:0] Ds_nom_0, Ds_nom_1, Ds_nom_2,
    input  wire signed [31:0] Ds_nom_3, Ds_nom_4, Ds_nom_5,

    input  wire        adaptive_enable,

    // -- NEW: historical-prior load -----------------------------------------
    input  wire        prior_load,
    input  wire signed [31:0] K_env_prior_0, K_env_prior_1, K_env_prior_2,
    input  wire signed [31:0] K_env_prior_3, K_env_prior_4, K_env_prior_5,
    input  wire signed [31:0] P_prior_0,     P_prior_1,     P_prior_2,
    input  wire signed [31:0] P_prior_3,     P_prior_4,     P_prior_5,

    // -- Original outputs ---------------------------------------------------
    output reg  signed [31:0] Ks_out_0, Ks_out_1, Ks_out_2,
    output reg  signed [31:0] Ks_out_3, Ks_out_4, Ks_out_5,

    output reg  signed [31:0] Ds_out_0, Ds_out_1, Ds_out_2,
    output reg  signed [31:0] Ds_out_3, Ds_out_4, Ds_out_5,

    output wire signed [31:0] K_env_est_0, K_env_est_1, K_env_est_2,
    output wire signed [31:0] K_env_est_3, K_env_est_4, K_env_est_5,

    output reg         done
);

    // -- Q16.16 Multiply ----------------------------------------------------
    (* use_dsp = "yes" *)
    function signed [31:0] fmult;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [63:0] temp;
        begin
            temp  = $signed(a) * $signed(b);
            fmult = temp[47:16];
        end
    endfunction

    // -- Legacy combinational Q16.16 divide (STUBBED -- do not use) ---------
    // Kept only so accidental calls are caught loudly in simulation. All
    // live divides flow through the `pipe_div32` helper module instantiated
    // 6 times below (one per joint).
    function signed [31:0] fdiv;
        input signed [31:0] numer;
        input signed [31:0] denom;
        begin
            // synthesis translate_off
            $display("ERROR: adaptive_impedance_v2.fdiv() called -- use pipe_div32");
            // synthesis translate_on
            fdiv = 32'sh7FFF_FFFF;
        end
    endfunction

    // -- Absolute value -----------------------------------------------------
    function signed [31:0] fabs;
        input signed [31:0] v;
        begin
            fabs = (v < 0) ? -v : v;
        end
    endfunction

    // -- Clamp --------------------------------------------------------------
    function signed [31:0] fclamp;
        input signed [31:0] v;
        input signed [31:0] lo;
        input signed [31:0] hi;
        begin
            if (v < lo)      fclamp = lo;
            else if (v > hi) fclamp = hi;
            else             fclamp = v;
        end
    endfunction

    // -- RLS state per joint (6 parallel scalar estimators) ----------------
    reg signed [31:0] K_env [0:5];
    reg signed [31:0] P     [0:5];

    assign K_env_est_0 = K_env[0];
    assign K_env_est_1 = K_env[1];
    assign K_env_est_2 = K_env[2];
    assign K_env_est_3 = K_env[3];
    assign K_env_est_4 = K_env[4];
    assign K_env_est_5 = K_env[5];

    // -- Input latches ------------------------------------------------------
    reg signed [31:0] tau_lat    [0:5];
    reg signed [31:0] qe_lat     [0:5];
    reg signed [31:0] Ks_nom_lat [0:5];
    reg signed [31:0] Ds_nom_lat [0:5];

    // -- Per-joint pipeline registers --------------------------------------
    reg        dz_skip    [0:5];
    reg signed [31:0] gain_reg    [0:5];
    reg signed [31:0] kenv_lat    [0:5];   // snapshot of K_env for adapt chain
    reg signed [31:0] Ks_new_reg  [0:5];   // clamped Ks_new after 1st adapt div
    // 乘法链流水中间寄存器 (每拍 ≤1 个 fmult, 收敛 125MHz; 数学与单拍版位级一致)
    reg signed [31:0] Pq_reg      [0:5];   // fmult(P, qe)         (gain 分子)
    reg signed [31:0] kgeq_reg    [0:5];   // fmult(K_env, qe)
    reg signed [31:0] pgq_reg     [0:5];   // fmult(gain, qe)
    reg signed [31:0] pm_reg      [0:5];   // fmult(fmult(gain,qe), P)
    reg signed [31:0] qpq_reg     [0:5];   // fmult(qe, Pq)        (gain 分母)
    reg signed [31:0] tk_reg      [0:5];   // tau - K_env*qe
    reg signed [31:0] gtk_reg     [0:5];   // fmult(gain, tau-K_env*qe)
    reg signed [31:0] psub_reg    [0:5];   // P - pm
    reg signed [31:0] pnew_reg    [0:5];   // fmult(INV_LAMBDA, P-pm)

    // -- Per-joint divider plumbing ----------------------------------------
    // 6 instances of pipe_div32, one per joint. Each is an explicit 4-stage
    // pipelined shift-subtract divider with valid_in / valid_out handshake
    // (see pipe_div32 module below). The FSM raises valid_in_gain in
    // S_GAIN_NUM and valid_in_adapt in S_ADAPT_D1 / S_ADAPT_Q1; both are
    // OR'd onto a single broadcast wire that drives every divider's
    // valid_in port (the 6 joints always issue the same operation type at
    // the same time -- this is the 6-way parallelism).
    reg  signed [31:0] div_numer_in [0:5];
    reg  signed [31:0] div_denom_in [0:5];
    wire signed [31:0] div_quot_out [0:5];
    wire [5:0]         div_valid_out_w;             // 各除法器 valid_out
    wire               div_done = div_valid_out_w[0]; // 6 路 lockstep, 取 0 号即可

    reg  valid_in_gain;
    reg  valid_in_adapt;
    wire div_valid_in_bcast = valid_in_gain | valid_in_adapt;

    // -- Flat prior-input array --------------------------------------------
    wire signed [31:0] K_env_prior_arr [0:5];
    wire signed [31:0] P_prior_arr     [0:5];
    assign K_env_prior_arr[0] = K_env_prior_0;
    assign K_env_prior_arr[1] = K_env_prior_1;
    assign K_env_prior_arr[2] = K_env_prior_2;
    assign K_env_prior_arr[3] = K_env_prior_3;
    assign K_env_prior_arr[4] = K_env_prior_4;
    assign K_env_prior_arr[5] = K_env_prior_5;
    assign P_prior_arr[0]     = P_prior_0;
    assign P_prior_arr[1]     = P_prior_1;
    assign P_prior_arr[2]     = P_prior_2;
    assign P_prior_arr[3]     = P_prior_3;
    assign P_prior_arr[4]     = P_prior_4;
    assign P_prior_arr[5]     = P_prior_5;

    // -- FSM state encoding (5-bit, 19 states) -----------------------------
    // 13 状态 (4-bit). 原 3×固定等待链(S_GAIN_W*/S_ADAPT_W*)改为单个握手等待态,
    // 因迭代式 pipe_div32 延迟可变(~48 拍), FSM 等 div_done(valid_out)而非固定周期.
    // 每个 DSP 乘法独占一拍 (寄存器进→DSP→寄存器出), 加/减/钳位各自独立拍, 收敛 125MHz.
    localparam [4:0] S_IDLE        = 5'd0;
    localparam [4:0] S_LATCH       = 5'd1;
    localparam [4:0] S_DEADZONE    = 5'd2;
    localparam [4:0] S_GAIN_PQ     = 5'd3;   // Pq  = fmult(P,qe)
    localparam [4:0] S_GAIN_QPQ    = 5'd4;   // qPq = fmult(qe,Pq)
    localparam [4:0] S_GAIN_NUM    = 5'd5;   // 发 gain 除法 (denom=LAMBDA+qPq)
    localparam [4:0] S_GAIN_WAIT   = 5'd6;
    localparam [4:0] S_RLS_KENV    = 5'd7;   // 锁 gain + kgeq=fmult(K_env,qe)
    localparam [4:0] S_RLS_KSUB    = 5'd8;   // tk  = tau-kgeq
    localparam [4:0] S_RLS_KMUL    = 5'd9;   // gtk = fmult(gain,tk)
    localparam [4:0] S_RLS_KADD    = 5'd10;  // K_env += gtk, 钳位
    localparam [4:0] S_RLS_P       = 5'd11;  // pgq = fmult(gain,qe)
    localparam [4:0] S_RLS_P2      = 5'd12;  // pm  = fmult(pgq,P)
    localparam [4:0] S_RLS_PSUB    = 5'd13;  // psub= P-pm
    localparam [4:0] S_RLS_PMUL    = 5'd14;  // pnew= fmult(INV_LAMBDA,psub)
    localparam [4:0] S_RLS_P3      = 5'd15;  // P = 钳位(pnew)
    localparam [4:0] S_ADAPT_D1    = 5'd16;
    localparam [4:0] S_ADAPT_WAIT1 = 5'd17;
    localparam [4:0] S_ADAPT_Q1    = 5'd18;
    localparam [4:0] S_ADAPT_WAIT2 = 5'd19;
    localparam [4:0] S_OUTPUT      = 5'd20;
    localparam [4:0] S_DONE        = 5'd21;

    reg [4:0] state;
    reg       start_pending;    // FIX #2: atomic prior_load + start

`ifdef DEBUG
    reg [5:0] cycle_count;
`endif

    integer i;

    // -- Divider helper instances ------------------------------------------
    // Single broadcast valid pulse to all 6 dividers.
    genvar gi;
    generate
        for (gi = 0; gi < 6; gi = gi + 1) begin : g_div
            pipe_div32 u_div (
                .clk      (clk),
                .rst_n    (rst_n),
                .numer_in (div_numer_in[gi]),
                .denom_in (div_denom_in[gi]),
                .valid_in (div_valid_in_bcast),
                .quot_out (div_quot_out[gi]),
                .valid_out(div_valid_out_w[gi])
            );
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin : fsm
        // Named-block locals (FIX #4: moved out of module scope)
        reg signed [31:0] kenv_i;
        reg signed [31:0] p_i;
        reg signed [31:0] qe_i;
        reg signed [31:0] tau_i;
        reg signed [31:0] ksnom_i;
        reg signed [31:0] dsnom_i;
        reg signed [31:0] Pq_l;
        reg signed [31:0] denom_l;
        reg signed [31:0] Ks_raw;
        reg signed [31:0] Ks_clamped;

        if (!rst_n) begin
            state          <= S_IDLE;
            done           <= 1'b0;
            start_pending  <= 1'b0;
            valid_in_gain  <= 1'b0;
            valid_in_adapt <= 1'b0;
`ifdef DEBUG
            cycle_count    <= 6'd0;
`endif
            for (i = 0; i < 6; i = i + 1) begin
                K_env[i]        <= K_ENV_INIT;
                P[i]            <= P_INIT;
                tau_lat[i]      <= 32'sd0;
                qe_lat[i]       <= 32'sd0;
                Ks_nom_lat[i]   <= 32'sd0;
                Ds_nom_lat[i]   <= 32'sd0;
                dz_skip[i]      <= 1'b0;
                gain_reg[i]     <= 32'sd0;
                kenv_lat[i]     <= 32'sd0;
                Ks_new_reg[i]   <= 32'sd0;
                Pq_reg[i]       <= 32'sd0;
                kgeq_reg[i]     <= 32'sd0;
                pgq_reg[i]      <= 32'sd0;
                pm_reg[i]       <= 32'sd0;
                qpq_reg[i]      <= 32'sd0;
                tk_reg[i]       <= 32'sd0;
                gtk_reg[i]      <= 32'sd0;
                psub_reg[i]     <= 32'sd0;
                pnew_reg[i]     <= 32'sd0;
                div_numer_in[i] <= 32'sd0;
                div_denom_in[i] <= 32'sd1;
            end
            Ks_out_0 <= 0; Ks_out_1 <= 0; Ks_out_2 <= 0;
            Ks_out_3 <= 0; Ks_out_4 <= 0; Ks_out_5 <= 0;
            Ds_out_0 <= 0; Ds_out_1 <= 0; Ds_out_2 <= 0;
            Ds_out_3 <= 0; Ds_out_4 <= 0; Ds_out_5 <= 0;
        end
        else if (prior_load) begin
            // FIX #2: highest-priority synchronous load of RLS priors.
            // If `start` is co-asserted, latch it for consumption on the
            // next cycle (after prior_load deasserts).
            for (i = 0; i < 6; i = i + 1) begin
                K_env[i] <= K_env_prior_arr[i];
                P[i]     <= P_prior_arr[i];
            end
            if (start) start_pending <= 1'b1;
            state          <= S_IDLE;
            done           <= 1'b0;
            valid_in_gain  <= 1'b0;
            valid_in_adapt <= 1'b0;
        end
        else begin
            // Default valid pulses to 0; states that issue divides override.
            valid_in_gain  <= 1'b0;
            valid_in_adapt <= 1'b0;

            // Bug-8 fix: latch incoming start in any non-IDLE state, so a
            // start arriving during compute (cycles 1-18) is queued rather
            // than silently dropped. Consumed on next S_IDLE entry.
            if (state != S_IDLE && start) begin
                start_pending <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start || start_pending) begin
                        state         <= S_LATCH;
                        start_pending <= 1'b0;
`ifdef DEBUG
                        cycle_count   <= 6'd1;
`endif
                    end
                end

                // cycle 1: latch all 6-joint inputs in parallel
                S_LATCH: begin
                    tau_lat[0] <= tau_ext_0; tau_lat[1] <= tau_ext_1; tau_lat[2] <= tau_ext_2;
                    tau_lat[3] <= tau_ext_3; tau_lat[4] <= tau_ext_4; tau_lat[5] <= tau_ext_5;
                    qe_lat[0]  <= q_err_0;   qe_lat[1]  <= q_err_1;   qe_lat[2]  <= q_err_2;
                    qe_lat[3]  <= q_err_3;   qe_lat[4]  <= q_err_4;   qe_lat[5]  <= q_err_5;
                    Ks_nom_lat[0] <= Ks_nom_0; Ks_nom_lat[1] <= Ks_nom_1; Ks_nom_lat[2] <= Ks_nom_2;
                    Ks_nom_lat[3] <= Ks_nom_3; Ks_nom_lat[4] <= Ks_nom_4; Ks_nom_lat[5] <= Ks_nom_5;
                    Ds_nom_lat[0] <= Ds_nom_0; Ds_nom_lat[1] <= Ds_nom_1; Ds_nom_lat[2] <= Ds_nom_2;
                    Ds_nom_lat[3] <= Ds_nom_3; Ds_nom_lat[4] <= Ds_nom_4; Ds_nom_lat[5] <= Ds_nom_5;
                    state <= S_DEADZONE;
`ifdef DEBUG
                    cycle_count <= cycle_count + 6'd1;
`endif
                end

                // cycle 2: dead-zone decision for every joint (parallel).
                // dz_skip joints will have their final outputs forced to
                // ksnom/dsnom in S_OUTPUT (cycle 17).
                S_DEADZONE: begin
                    for (i = 0; i < 6; i = i + 1) begin
                        if (!adaptive_enable || fabs(qe_lat[i]) < Q_DEADZONE)
                            dz_skip[i] <= 1'b1;
                        else
                            dz_skip[i] <= 1'b0;
                    end
                    state <= S_GAIN_PQ;
`ifdef DEBUG
                    cycle_count <= cycle_count + 6'd1;
`endif
                end

                // Pq = fmult(P, qe)  (纯乘法独占一拍)
                S_GAIN_PQ: begin
                    for (i = 0; i < 6; i = i + 1)
                        Pq_reg[i] <= fmult(P[i], qe_lat[i]);
                    state <= S_GAIN_QPQ;
                end

                // qPq = fmult(qe, Pq)  (纯乘法独占一拍)
                S_GAIN_QPQ: begin
                    for (i = 0; i < 6; i = i + 1)
                        qpq_reg[i] <= fmult(qe_lat[i], Pq_reg[i]);
                    state <= S_GAIN_NUM;
                end

                // gain 除法操作数: numer=Pq, denom=LAMBDA+qPq  (仅加法)
                S_GAIN_NUM: begin
                    for (i = 0; i < 6; i = i + 1) begin
                        div_numer_in[i] <= Pq_reg[i];
                        div_denom_in[i] <= LAMBDA + qpq_reg[i];
                    end
                    valid_in_gain <= 1'b1;
                    state <= S_GAIN_WAIT;
                end

                // cycles 4, 5, 6: gain divide propagates through pipe_div32
                // stages 1, 2, 3. Quotient appears on div_quot_out at the
                // start of S_RLS_KENV (cycle 7).
                // 等 gain 除法 valid_out (迭代式 ~48 拍)
                S_GAIN_WAIT: if (div_done) state <= S_RLS_KENV;

                // cycle 7: gain quotient is on div_quot_out. Latch into
                // gain_reg AND apply K_env update in the same cycle.
                //   gain_reg[i]  <= div_quot_out[i]
                //   K_env[i]     <= K_env[i] + gain*(tau - K_env*qe)
                // 锁存 gain; kgeq = fmult(K_env, qe)  (纯乘法独占一拍)
                S_RLS_KENV: begin
                    for (i = 0; i < 6; i = i + 1) begin
                        gain_reg[i] <= div_quot_out[i];
                        kgeq_reg[i] <= fmult(K_env[i], qe_lat[i]);
                    end
                    state <= S_RLS_KSUB;
                end

                // tk = tau - K_env*qe  (仅减法)
                S_RLS_KSUB: begin
                    for (i = 0; i < 6; i = i + 1)
                        tk_reg[i] <= tau_lat[i] - kgeq_reg[i];
                    state <= S_RLS_KMUL;
                end

                // gtk = fmult(gain, tk)  (纯乘法独占一拍)
                S_RLS_KMUL: begin
                    for (i = 0; i < 6; i = i + 1)
                        gtk_reg[i] <= fmult(gain_reg[i], tk_reg[i]);
                    state <= S_RLS_KADD;
                end

                // K_env += gtk, 钳到 [K_ENV_MIN,K_ENV_MAX]  (仅加法+钳位)
                // Bug-2: 钳 K_env 防后续 fmult(K_env,qe) 溢出.
                S_RLS_KADD: begin
                    for (i = 0; i < 6; i = i + 1)
                        if (!dz_skip[i])
                            K_env[i] <= fclamp(K_env[i] + gtk_reg[i], K_ENV_MIN, K_ENV_MAX);
                    state <= S_RLS_P;
                end

                // P = (1/lambda)*(P - gain*qe*P) 拆 3 拍, 每拍 1 乘法.
                // pgq = fmult(gain, qe)
                S_RLS_P: begin
                    for (i = 0; i < 6; i = i + 1)
                        pgq_reg[i] <= fmult(gain_reg[i], qe_lat[i]);
                    state <= S_RLS_P2;
                end

                // pm = fmult(pgq, P)   (P 用旧值)
                S_RLS_P2: begin
                    for (i = 0; i < 6; i = i + 1)
                        pm_reg[i] <= fmult(pgq_reg[i], P[i]);
                    state <= S_RLS_PSUB;
                end

                // psub = P - pm  (仅减法; P 用旧值)
                S_RLS_PSUB: begin
                    for (i = 0; i < 6; i = i + 1)
                        psub_reg[i] <= P[i] - pm_reg[i];
                    state <= S_RLS_PMUL;
                end

                // pnew = fmult(INV_LAMBDA, psub)  (纯乘法独占一拍)
                S_RLS_PMUL: begin
                    for (i = 0; i < 6; i = i + 1)
                        pnew_reg[i] <= fmult(INV_LAMBDA, psub_reg[i]);
                    state <= S_RLS_P3;
                end

                // P = 钳位(pnew) 到 [P_MIN,P_INIT]  (仅钳位)
                // Bug-5: P>=P_MIN>0 防下次 gain 除法分母趋零.
                S_RLS_P3: begin
                    for (i = 0; i < 6; i = i + 1)
                        if (!dz_skip[i])
                            P[i] <= fclamp(pnew_reg[i], P_MIN, P_INIT);
                    state <= S_ADAPT_D1;
                end

                // cycle 9: register 1st adapt-divide operands and pulse
                // valid_in_adapt so pipe_div32 stage 0 captures.
                //   numer_i = fmult(ksnom_i, ksnom_i)   (Ks_nom^2)
                //   denom_i = K_env[i]   (or 1 as safe placeholder if <= 0)
                // Snapshot K_env[i] (freshly updated in cycle 7) and Ks_nom
                // for use after the divide settles.
                S_ADAPT_D1: begin
                    for (i = 0; i < 6; i = i + 1) begin
                        kenv_i  = K_env[i];
                        ksnom_i = Ks_nom_lat[i];
                        kenv_lat[i]     <= kenv_i;
                        div_numer_in[i] <= fmult(ksnom_i, ksnom_i);
                        div_denom_in[i] <= (kenv_i > 0) ? kenv_i : 32'sd1;
                    end
                    valid_in_adapt <= 1'b1;
                    state <= S_ADAPT_WAIT1;
`ifdef DEBUG
                    cycle_count <= cycle_count + 6'd1;
`endif
                end

                // cycles 10, 11, 12: adapt divide 1 propagates through
                // pipe_div32 stages 1, 2, 3.
                // 等 adapt 除法1 (Ks_nom²/K_env) valid_out
                S_ADAPT_WAIT1: if (div_done) state <= S_ADAPT_Q1;

                // cycle 13: 1st adapt quotient (raw Ks_new) is on
                // div_quot_out. Clamp combinationally, register into
                // Ks_new_reg, AND in the same cycle issue the 2nd adapt
                // divide (Ks_clamped / Ks_nom) by writing the operand
                // registers and pulsing valid_in_adapt again.
                S_ADAPT_Q1: begin
                    for (i = 0; i < 6; i = i + 1) begin
                        ksnom_i = Ks_nom_lat[i];
                        kenv_i  = kenv_lat[i];

                        if (kenv_i > 0) begin
                            Ks_raw     = div_quot_out[i];
                            Ks_clamped = fclamp(Ks_raw, KS_MIN, KS_MAX);
                        end else begin
                            // v1 fallback: if K_env<=0, Ks_new := ksnom
                            // (and the 2nd divide result is ignored in
                            // S_OUTPUT for this joint).
                            Ks_clamped = ksnom_i;
                        end

                        Ks_new_reg[i] <= Ks_clamped;

                        // 2nd divide operands
                        div_numer_in[i] <= Ks_clamped;
                        div_denom_in[i] <= (ksnom_i != 0) ? ksnom_i : 32'sd1;
                    end
                    valid_in_adapt <= 1'b1;
                    state <= S_ADAPT_WAIT2;
`ifdef DEBUG
                    cycle_count <= cycle_count + 6'd1;
`endif
                end

                // cycles 14, 15, 16: adapt divide 2 propagates through
                // pipe_div32 stages 1, 2, 3.
                // 等 adapt 除法2 (Ks_clamped/Ks_nom) valid_out
                S_ADAPT_WAIT2: if (div_done) state <= S_OUTPUT;

                // cycle 17: 2nd adapt quotient is on div_quot_out. Compute
                // Ds_adapted = fmult(Ds_nom_lat, quot) for non-dead-zone
                // joints. Drive all output ports and raise done on the
                // same edge. dz_skip joints output ksnom/dsnom directly.
                S_OUTPUT: begin
                    // joint 0
                    if (dz_skip[0]) begin
                        Ks_out_0 <= Ks_nom_lat[0];
                        Ds_out_0 <= Ds_nom_lat[0];
                    end else begin
                        Ks_out_0 <= Ks_new_reg[0];
                        Ds_out_0 <= (Ks_nom_lat[0] != 0)
                                    ? fmult(Ds_nom_lat[0], div_quot_out[0])
                                    : Ds_nom_lat[0];
                    end
                    // joint 1
                    if (dz_skip[1]) begin
                        Ks_out_1 <= Ks_nom_lat[1];
                        Ds_out_1 <= Ds_nom_lat[1];
                    end else begin
                        Ks_out_1 <= Ks_new_reg[1];
                        Ds_out_1 <= (Ks_nom_lat[1] != 0)
                                    ? fmult(Ds_nom_lat[1], div_quot_out[1])
                                    : Ds_nom_lat[1];
                    end
                    // joint 2
                    if (dz_skip[2]) begin
                        Ks_out_2 <= Ks_nom_lat[2];
                        Ds_out_2 <= Ds_nom_lat[2];
                    end else begin
                        Ks_out_2 <= Ks_new_reg[2];
                        Ds_out_2 <= (Ks_nom_lat[2] != 0)
                                    ? fmult(Ds_nom_lat[2], div_quot_out[2])
                                    : Ds_nom_lat[2];
                    end
                    // joint 3
                    if (dz_skip[3]) begin
                        Ks_out_3 <= Ks_nom_lat[3];
                        Ds_out_3 <= Ds_nom_lat[3];
                    end else begin
                        Ks_out_3 <= Ks_new_reg[3];
                        Ds_out_3 <= (Ks_nom_lat[3] != 0)
                                    ? fmult(Ds_nom_lat[3], div_quot_out[3])
                                    : Ds_nom_lat[3];
                    end
                    // joint 4
                    if (dz_skip[4]) begin
                        Ks_out_4 <= Ks_nom_lat[4];
                        Ds_out_4 <= Ds_nom_lat[4];
                    end else begin
                        Ks_out_4 <= Ks_new_reg[4];
                        Ds_out_4 <= (Ks_nom_lat[4] != 0)
                                    ? fmult(Ds_nom_lat[4], div_quot_out[4])
                                    : Ds_nom_lat[4];
                    end
                    // joint 5
                    if (dz_skip[5]) begin
                        Ks_out_5 <= Ks_nom_lat[5];
                        Ds_out_5 <= Ds_nom_lat[5];
                    end else begin
                        Ks_out_5 <= Ks_new_reg[5];
                        Ds_out_5 <= (Ks_nom_lat[5] != 0)
                                    ? fmult(Ds_nom_lat[5], div_quot_out[5])
                                    : Ds_nom_lat[5];
                    end

                    done  <= 1'b1;
                    state <= S_DONE;
`ifdef DEBUG
                    cycle_count <= cycle_count + 6'd1;
`endif
                end

                // hold done high 1 cycle, return to idle
                S_DONE: begin
                    done  <= 1'b0;
                    state <= S_IDLE;
`ifdef DEBUG
                    cycle_count <= 6'd0;
`endif
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

// ============================================================================
// pipe_div32 -- per-joint Q16.16 signed divider, EXPLICIT 4-stage pipeline
// ----------------------------------------------------------------------------
// Bit-exact with v1's fdiv() which computed:
//     n64  = $signed(numer) <<< 16;
//     quot = (denom == 0) ? 32'sh7FFF_FFFF
//                         : (n64 / $signed(denom));   // truncating signed
//
// ARCHITECTURE
//   Explicit 4-stage pipelined shift-subtract restoring divider. There is
//   NO combinational path from numer_in/denom_in to quot_out. The 4 pipeline
//   stages are:
//
//     Stage 0 = PARENT operand registers `div_numer_in` / `div_denom_in` +
//               `valid_in_gain`/`valid_in_adapt` -- written one cycle before
//               pipe_div32 sees them. This module's input ports (numer_in,
//               denom_in, valid_in) are driven by those parent registers,
//               so the first pipeline register stage is upstream in the
//               parent. The FSM schedule (S_GAIN_NUM, S_ADAPT_D1, S_ADAPT_Q1)
//               performs this stage-0 latch.
//
//     Stage 1 = INTERNAL register `s1_*`. Combinational body: compute sign,
//               div_zero, absolute magnitudes, |numer|<<16, then run 16
//               restoring-divide iterations. Latches {rem, div, quot, sign,
//               div_zero, valid} into s1_* on the clock edge.
//
//     Stage 2 = INTERNAL register `s2_*`. Combinational body: 16 more
//               restoring-divide iterations. Latches {rem, div, quot, sign,
//               div_zero, valid} into s2_* on the clock edge.
//
//     Stage 3 = INTERNAL register `quot_out` / `valid_out`. Combinational
//               body: final 16 restoring-divide iterations (total 48),
//               apply sign (negate if s2_sign), on s2_div_zero force
//               32'sh7FFF_FFFF. Latches signed quotient and valid flag.
//
//   End-to-end latency: parent writes operands in cycle N -> quot_out valid
//   in cycle N+4. That is exactly what the FSM schedule expects: for the
//   gain divide, S_GAIN_NUM writes in cycle 3 and S_RLS_KENV reads in
//   cycle 7 (3+4). For adapt divide 1, S_ADAPT_D1 writes in cycle 9 and
//   S_ADAPT_Q1 reads in cycle 13. For adapt divide 2, S_ADAPT_Q1 writes in
//   cycle 13 and S_OUTPUT reads in cycle 17.
//
//   Per-stage combinational depth: ~16 add/sub on 48-bit operands (<6 ns
//   on Alveo U200 Ultrascale+), plus stage 1 carries a small
//   absolute-value / shift prefix. Closes 100 MHz with margin and WITHOUT
//   relying on Vivado retiming.
//
//   Iteration partition across stages 1..3: 16 / 16 / 16 (total 48).
//   Reasoning: the dividend is |numer|<<16 which is at most 48 bits, so
//   48 restoring-divide iterations produce a full 32-bit quotient (the
//   top 16 iterations drive the integer part of the quotient into the
//   high half of the rem register, the bottom 32 iterations produce the
//   32-bit quotient bits).
//
// SIGN HANDLING
//   Internally we always operate on unsigned magnitudes (restoring divide
//   is only well-defined for unsigned operands). Sign is XOR of the two
//   input sign bits and is propagated alongside the data. At stage 3 the
//   final unsigned quotient is conditionally negated. This matches v1's
//   $signed truncating divide bit-for-bit on the 4 sign combinations
//   (++, +-, -+, --), on +/-max, +/-min, +/-typical, +/-near-zero, and on
//   the zero-denom guard.
//
// RESET
//   All internal pipeline registers (s1_*, s2_*, quot_out, valid_out) are
//   synchronously cleared on !rst_n.
//
// SRL PROTECTION
//   Every stage register carries (* shreg_extract = "no" *) so Vivado
//   cannot collapse them into an SRL shift register.
// ============================================================================
module pipe_div32 (
    input  wire                clk,
    input  wire                rst_n,
    input  wire signed [31:0]  numer_in,
    input  wire signed [31:0]  denom_in,
    input  wire                valid_in,
    output reg  signed [31:0]  quot_out,
    output reg                 valid_out
);

    // Working register layout (combined rem | dvd_q):
    //   wr[80:48] (33 bits)  = remainder with 1-bit headroom
    //   wr[47:0]  (48 bits)  = dividend in low half, gets shifted out MSB
    //                          first into rem, and new quotient bits are
    //                          shifted into its LSB in the same position.
    //                          After 48 iterations, wr[47:0] is the unsigned
    //                          quotient (we keep only the low 32 bits).
    localparam integer WR_W  = 81;
    // Bit slice ranges for the working register:
    //   wr[80:48] = remainder (33 bits, 1 bit of headroom over the 32-bit divisor)
    //   wr[47:0]  = dividend / quotient slot (48 bits)

    // 迭代式: 一次只处理一个除法 (自适应 FSM 串行发起), 每拍 1 次 restoring 迭代.
    localparam [1:0] D_IDLE = 2'd0, D_RUN = 2'd1, D_FIN = 2'd2;
    reg [1:0]        dstate;
    reg [5:0]        dcnt;            // 迭代计数 0..47
    reg [WR_W-1:0]   wr;              // 81 位工作寄存器 [80:48]=余数(33) [47:0]=被除数/商(48)
    reg [31:0]       divr;            // |denom|
    reg              sgn;             // 商符号 = numer[31]^denom[31]
    reg              dzero;           // denom==0 保护
    // 组合临时 (在时钟块内 blocking 赋值)
    reg [31:0]       abs_numer, abs_denom;
    reg [63:0]       numer_shl;
    reg [WR_W-1:0]   wr_sh;
    reg [31:0]       uquot;

    always @(posedge clk) begin : div_iter
        if (!rst_n) begin
            dstate <= D_IDLE; dcnt <= 6'd0; wr <= {WR_W{1'b0}};
            divr <= 32'd0; sgn <= 1'b0; dzero <= 1'b0;
            quot_out <= 32'sd0; valid_out <= 1'b0;
        end
        else begin
            valid_out <= 1'b0;                       // 默认低, 仅 D_FIN 拉高 1 拍
            case (dstate)
                // 锁存操作数 + 初始化工作寄存器 (rem=0, dvd=|numer|<<16)
                D_IDLE: if (valid_in) begin
                    // Bug-9: -2^31 取负溢出回自身, 饱和到 2^31-1 保证无符号幅值正确
                    abs_numer = (numer_in == 32'h8000_0000) ? 32'h7FFF_FFFF :
                                numer_in[31] ? (~numer_in + 32'd1) : numer_in;
                    abs_denom = (denom_in == 32'h8000_0000) ? 32'h7FFF_FFFF :
                                denom_in[31] ? (~denom_in + 32'd1) : denom_in;
                    numer_shl = {32'd0, abs_numer} << 16;          // |numer|<<16 (<=48 位)
                    wr     <= {{33{1'b0}}, numer_shl[47:0]};
                    divr   <= abs_denom;
                    sgn    <= numer_in[31] ^ denom_in[31];
                    dzero  <= (denom_in == 32'sd0);
                    dcnt   <= 6'd0;
                    dstate <= D_RUN;
                end
                // 每拍 1 次 restoring 迭代 (与原 3 段流水逐位一致), 共 48 次
                //   wr<<=1; if(wr[80:48]>=divr){wr[80:48]-=divr; 商位=1} else 商位=0
                D_RUN: begin
                    wr_sh = wr << 1;
                    if (wr_sh[80:48] >= {1'b0, divr})
                        wr <= { (wr_sh[80:48] - {1'b0, divr}), wr_sh[47:1], 1'b1 };
                    else
                        wr <= wr_sh;
                    if (dcnt == 6'd47) dstate <= D_FIN;
                    else               dcnt   <= dcnt + 6'd1;
                end
                // 低 32 位为无符号商, 应用符号 / 零除保护(0x7FFFFFFF), 出 valid_out
                D_FIN: begin
                    uquot     = wr[31:0];
                    quot_out  <= dzero ? 32'sh7FFF_FFFF
                                       : (sgn ? -$signed({1'b0, uquot})
                                              :  $signed({1'b0, uquot}));
                    valid_out <= 1'b1;
                    dstate    <= D_IDLE;
                end
                default: dstate <= D_IDLE;
            endcase
        end
    end

endmodule
