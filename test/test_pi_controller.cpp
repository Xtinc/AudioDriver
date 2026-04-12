#include "audio_process.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void test_check(bool condition, const std::string &name)
{
    if (!condition)
    {
        std::cout << "FAILED: " << name << std::endl;
        assert(false);
    }
    std::cout << "PASSED: " << name << std::endl;
}

static void expect_near(double actual, double expected, double tol, const std::string &name)
{
    if (std::fabs(actual - expected) > tol)
    {
        std::cout << "FAILED: " << name << "  actual=" << actual << "  expected=" << expected << "  tol=" << tol
                  << std::endl;
        assert(false);
    }
    std::cout << "PASSED: " << name << std::endl;
}

// ---------------------------------------------------------------------------
// Helper: build a standard controller for most tests
//   target = 20 ms, dt = 0.01 s, output ∈ [0.995, 1.005], integral clamp = 1e3
// ---------------------------------------------------------------------------
static PIController make_pi(double kp, double ki, unsigned int warmup = 0)
{
    return PIController(kp, ki,
                        /*target*/ 20.0,
                        /*dt*/ 0.01,
                        /*out_min*/ 0.995,
                        /*out_max*/ 1.005,
                        /*integral_clamp*/ 1e3,
                        warmup);
}

// ---------------------------------------------------------------------------
// 1. Warmup gate
//    During warmup every call must return exactly 1.0 regardless of measured value.
//    First call after warmup must return a non-1.0 value when there is error.
// ---------------------------------------------------------------------------
static void test_warmup_gate()
{
    const unsigned int WARMUP = 32;
    PIController pi = make_pi(1e-4, 1e-4, WARMUP);

    // Feed a value far from target during warmup
    for (unsigned int i = 0; i < WARMUP; ++i)
    {
        double out = pi.update(0.0); // large error (target=20, measured=0)
        test_check(out == 1.0, "warmup_gate: cycle " + std::to_string(i) + " returns 1.0");
    }

    // First post-warmup call should reflect proportional kick
    double out = pi.update(0.0);
    test_check(out != 1.0, "warmup_gate: first active call != 1.0");
    test_check(out > 1.0, "warmup_gate: first active call > 1.0 (error is positive)");
}

// ---------------------------------------------------------------------------
// 2. Proportional-only response (Ki = 0)
//    output = 1.0 + Kp * (target - measured)
// ---------------------------------------------------------------------------
static void test_proportional_only()
{
    PIController pi = make_pi(1e-3, 0.0, 0);

    // measured = 10, target = 20, e = 10
    // expected output = 1.0 + 1e-3 * 10 = 1.01  → clamped to 1.005
    double out = pi.update(10.0);
    expect_near(out, 1.005, 1e-9, "proportional_only: positive error clamped to out_max");

    PIController pi2 = make_pi(1e-3, 0.0, 0);
    // measured = 30, target = 20, e = -10
    // expected output = 1.0 - 1e-3 * 10 = 0.99  → clamped to 0.995
    out = pi2.update(30.0);
    expect_near(out, 0.995, 1e-9, "proportional_only: negative error clamped to out_min");

    PIController pi3 = make_pi(1e-3, 0.0, 0);
    // measured = 19, target = 20, e = 1
    // expected output = 1.0 + 1e-3 * 1 = 1.001  (within clamp)
    out = pi3.update(19.0);
    expect_near(out, 1.001, 1e-9, "proportional_only: small positive error unclamped");

    PIController pi4 = make_pi(1e-3, 0.0, 0);
    // measured = 20, error = 0
    out = pi4.update(20.0);
    expect_near(out, 1.0, 1e-9, "proportional_only: zero error → 1.0");
}

// ---------------------------------------------------------------------------
// 3. Integral accumulation (Kp = 0)
//    With constant error e and dt, after N steps:
//    integral = e * dt * N
//    output   = 1.0 + Ki * e * dt * N
// ---------------------------------------------------------------------------
static void test_integral_accumulation()
{
    const double Ki = 1e-3;
    const double e = 5.0; // measured = 15, target = 20
    const double dt = 0.01;
    PIController pi = make_pi(0.0, Ki, 0);

    for (int n = 1; n <= 5; ++n)
    {
        double out = pi.update(20.0 - e); // measured = 15
        double expected = 1.0 + Ki * e * dt * n;
        // clamp to [0.995, 1.005]
        if (expected > 1.005)
            expected = 1.005;
        expect_near(out, expected, 1e-9, "integral_accum: step " + std::to_string(n));
    }
}

// ---------------------------------------------------------------------------
// 4. Integral anti-windup
//    With a large constant positive error, the integral saturates at integral_clamp.
//    After saturation, output stays at out_max (not growing beyond clamp).
// ---------------------------------------------------------------------------
static void test_integral_antiwindup()
{
    // Use smaller clamp for faster saturation
    PIController pi(0.0, 1.0,
                    /*target*/ 20.0, /*dt*/ 0.01,
                    /*out_min*/ 0.995, /*out_max*/ 1.005,
                    /*integral_clamp*/ 0.1, // clamps after 10 steps at e=1
                    0);

    // After integral saturates, output should be clamped to out_max
    for (int i = 0; i < 200; ++i)
    {
        pi.update(19.0); // e = 1 each step
    }
    double out = pi.update(19.0);
    expect_near(out, 1.005, 1e-9, "antiwindup: output clamped at out_max after saturation");
}

// ---------------------------------------------------------------------------
// 5. Output clamping (direct bounds check)
// ---------------------------------------------------------------------------
static void test_output_clamp()
{
    // Large positive error → out_max
    PIController pi_hi = make_pi(1.0, 0.0, 0);
    double out = pi_hi.update(0.0); // e = 20, output would be 21 without clamp
    expect_near(out, 1.005, 1e-9, "output_clamp: out_max");

    // Large negative error → out_min
    PIController pi_lo = make_pi(1.0, 0.0, 0);
    out = pi_lo.update(40.0); // e = -20, output would be -19 without clamp
    expect_near(out, 0.995, 1e-9, "output_clamp: out_min");
}

// ---------------------------------------------------------------------------
// 6. reset()
//    After reset the controller behaves as newly constructed:
//    warmup cycles return 1.0, integral is cleared.
// ---------------------------------------------------------------------------
static void test_reset()
{
    const unsigned int WARMUP = 4;
    PIController pi = make_pi(1e-4, 1e-4, WARMUP);

    // Run past warmup and accumulate integral
    for (unsigned int i = 0; i < WARMUP + 10; ++i)
    {
        pi.update(10.0);
    }

    pi.reset();

    // After reset, warmup should be re-entered
    for (unsigned int i = 0; i < WARMUP; ++i)
    {
        double out = pi.update(0.0);
        test_check(out == 1.0, "reset: warmup restarted cycle " + std::to_string(i));
    }

    // First active call: no integral residue from before reset.
    // Kp=1e-4, e=20, Ki=1e-4, dt=0.01 → out = 1 + 1e-4*20 + 1e-4*(20*0.01) = 1.00202
    double out = pi.update(0.0);
    double expected_p = 1.0 + 1e-4 * 20.0 + 1e-4 * 20.0 * 0.01; // P+I on first step
    expect_near(out, expected_p, 1e-9, "reset: first active call matches fresh controller");
}

// ---------------------------------------------------------------------------
// 7. Closed-loop convergence with a first-order integrating plant
//
//    Plant model (models the device buffer drain dynamics):
//      val[n+1] = val[n] - gain * drift + gain * (adj[n] - 1.0)
//
//    Where:
//      gain  = ti ms  (ms of buffer change per unit fractional adj deviation)
//      drift = fractional clock offset (100 ppm = 1e-4): positive drift means
//              hardware consumes faster → buffer drains faster (negative term)
//      adj > 1: software drains/feeds more per period → compensates drain
//
//    Steady-state: drift = adj_ss - 1  →  adj_ss = 1 + drift
//    PI: adj = 1 + Kp*(target - val) + Ki*I
//      val < target  →  e > 0  →  adj > 1  →  val increases  (negative feedback ✓)
//
//    Pass criteria:
//      - val converges to within 1% of target within 10 000 cycles (~100 s)
//      - adj stays within [0.995, 1.005] at all times
// ---------------------------------------------------------------------------
static void test_convergence_simulation()
{
    const double ti_ms = 10.0; // playback period ms
    const double dt_s = ti_ms / 1000.0;
    const double target = 2.0 * ti_ms; // 20 ms
    const double drift = 100e-6;       // 100 ppm  (hardware faster than software)
    const double gain = ti_ms;         // latency gain per period

    PIController pi(1e-4, 1e-4, target, dt_s, 0.995, 1.005, 1e3, 64);

    double val = target; // start at target

    bool adj_ever_out_of_range = false;
    bool converged = false;

    for (int n = 0; n < 10000; ++n)
    {
        double adj = pi.update(val);

        // Check adj bounds
        if (adj < 0.995 - 1e-9 || adj > 1.005 + 1e-9)
        {
            adj_ever_out_of_range = true;
        }

        // Plant: positive drift drains the buffer; adj > 1 compensates
        val += -gain * drift + gain * (adj - 1.0);

        // Convergence check (after warmup, allow settling time)
        if (n > 1000 && std::fabs(val - target) < target * 0.01)
        {
            converged = true;
        }
    }

    test_check(!adj_ever_out_of_range, "convergence_sim: adj always within [0.995, 1.005]");
    test_check(converged, "convergence_sim: val converges to within 1% of target within 10000 cycles");

    // Final steady-state: adj should be ≈ 1 + drift
    double adj_final = pi.update(val);
    expect_near(adj_final, 1.0 + drift, 2e-4,
                "convergence_sim: steady-state adj ≈ 1 + drift (100 ppm offset)");
}

// ---------------------------------------------------------------------------
// 8. Negative drift: hardware slower than software
//    val should converge from above target, adj < 1.
// ---------------------------------------------------------------------------
static void test_convergence_negative_drift()
{
    const double ti_ms = 10.0;
    const double dt_s = ti_ms / 1000.0;
    const double target = 2.0 * ti_ms;
    const double drift = -50e-6; // 50 ppm hardware slower
    const double gain = ti_ms;

    PIController pi(1e-4, 1e-4, target, dt_s, 0.995, 1.005, 1e3, 64);

    double val = target;

    bool adj_ever_out_of_range = false;
    bool converged = false;
    bool adj_was_below_one = false;

    for (int n = 0; n < 10000; ++n)
    {
        double adj = pi.update(val);

        if (adj < 0.995 - 1e-9 || adj > 1.005 + 1e-9)
            adj_ever_out_of_range = true;
        if (adj < 1.0)
            adj_was_below_one = true;

        // drift=-50ppm: hardware slower → buffer fills; adj<1 drains it
        val += -gain * drift + gain * (adj - 1.0);

        if (n > 1000 && std::fabs(val - target) < target * 0.01)
            converged = true;
    }

    test_check(!adj_ever_out_of_range, "neg_drift: adj always within [0.995, 1.005]");
    test_check(converged, "neg_drift: val converges within 1% of target");
    test_check(adj_was_below_one, "neg_drift: adj goes below 1.0 to compensate");
}

// ---------------------------------------------------------------------------
// 9. Step disturbance rejection
//    System is at steady state, then a sudden +5 ms step in plant value occurs.
//    PI must drive it back to within 1% of target.
// ---------------------------------------------------------------------------
static void test_step_disturbance_rejection()
{
    const double ti_ms = 10.0;
    const double dt_s = ti_ms / 1000.0;
    const double target = 20.0;
    const double gain = ti_ms;

    PIController pi(1e-4, 1e-4, target, dt_s, 0.995, 1.005, 1e3, 0);

    double val = target;

    // Run to steady state (no drift)
    for (int n = 0; n < 2000; ++n)
    {
        double adj = pi.update(val);
        val += gain * (adj - 1.0); // adj>1 → val rises back to target when below
    }

    // Apply step disturbance (push below target; PI should raise it back)
    val -= 5.0;

    bool recovered = false;
    for (int n = 0; n < 5000; ++n)
    {
        double adj = pi.update(val);
        val += gain * (adj - 1.0); // adj>1 when val<target → val increases

        if (std::fabs(val - target) < target * 0.01)
        {
            recovered = true;
            break;
        }
    }

    test_check(recovered, "step_disturbance: val recovers to within 1% of target after +5ms step");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    std::cout << "=== PIController Tests ===" << std::endl;
    test_warmup_gate();
    test_proportional_only();
    test_integral_accumulation();
    test_integral_antiwindup();
    test_output_clamp();
    test_reset();
    test_convergence_simulation();
    test_convergence_negative_drift();
    test_step_disturbance_rejection();
    std::cout << "=== All PIController tests passed ===" << std::endl;
    return 0;
}
