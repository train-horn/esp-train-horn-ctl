/*
 * Host unit tests for train_horn_ctl. Build and run: `make -C test`.
 * Tiny assert harness, no dependencies beyond libc and libm.
 * SPDX-License-Identifier: MIT
 */
#include <math.h>
#include <stdio.h>

#include "train_horn_ctl.h"

static int g_checks, g_fail;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, eps) do { double a_ = (a), b_ = (b); g_checks++; \
    if (!(fabs(a_ - b_) <= (eps))) { g_fail++; \
    printf("  FAIL %s:%d: %s = %.12f, expected %.12f\n", __FILE__, __LINE__, #a, a_, b_); } } while (0)
#define RUN(t) do { int f0 = g_fail; t(); printf("%-40s %s\n", #t, g_fail == f0 ? "ok" : "FAILED"); } while (0)

/* Run the sequencer in 1 ms steps and record ON/OFF edges. */
static void test_fra_pattern_timing(void)
{
    th_step_t steps[TH_FRA_STEPS];
    th_pattern_t p;
    th_seq_t s;
    uint32_t t, on_ms = 0, edges = 0;
    uint32_t on_len[8] = { 0 };
    bool prev = false;

    CHECK(th_pattern_fra(steps, 3000, 1000, 500, false, &p));
    th_seq_init(&s);
    CHECK(!th_seq_output(&s));
    CHECK(th_seq_start(&s, &p));
    CHECK(th_seq_output(&s)); /* ON immediately at t=0 */
    prev = true;
    for (t = 0; t < 20000; t++) {
        bool now;
        if (prev) { on_ms++; on_len[edges / 2]++; }
        now = th_seq_tick(&s, 1);
        if (now != prev) { edges++; }
        prev = now;
    }
    /* long 3000, long 3000, short 1000, long 3000 */
    CHECK(on_len[0] == 3000);
    CHECK(on_len[1] == 3000);
    CHECK(on_len[2] == 1000);
    CHECK(on_len[3] == 3000);
    CHECK(on_ms == 10000);
    CHECK(edges == 7); /* 4 ON periods -> 7 edges after the initial ON */
    CHECK(!th_seq_running(&s)); /* one-shot finished after 12000 ms */
}

static void test_seq_boundaries_and_big_dt(void)
{
    th_step_t steps[TH_FRA_STEPS];
    th_pattern_t p;
    th_seq_t s;
    th_pattern_fra(steps, 2000, 1000, 500, false, &p);
    th_seq_init(&s);
    th_seq_start(&s, &p);
    CHECK(th_seq_tick(&s, 1999) == true);
    CHECK(th_seq_tick(&s, 1) == false);   /* t=2000: gap */
    CHECK(th_seq_tick(&s, 499) == false); /* t=2499 */
    CHECK(th_seq_tick(&s, 1) == true);    /* t=2500: 2nd long */
    /* jump across 2nd long (to 4500), gap, into the short (starts 5000) */
    CHECK(th_seq_tick(&s, 2700) == true); /* t=5200: inside short */
    CHECK(th_seq_tick(&s, 800) == false); /* t=6000: gap after short */
    CHECK(th_seq_tick(&s, 500) == true);  /* t=6500: final long */
    CHECK(th_seq_tick(&s, 100000) == false);
    CHECK(!th_seq_running(&s));
}

static void test_seq_repeat_and_stop(void)
{
    th_step_t steps[TH_FRA_STEPS];
    th_pattern_t p;
    th_seq_t s;
    th_pattern_fra_default(steps, true, &p); /* 2000/1000/500, period 9000 */
    th_seq_init(&s);
    th_seq_start(&s, &p);
    CHECK(th_seq_tick(&s, 8999) == false); /* last gap */
    CHECK(th_seq_tick(&s, 1) == true);     /* t=9000: wrapped to 1st long */
    CHECK(th_seq_running(&s));
    CHECK(th_seq_tick(&s, 10 * 9000 + 1999) == true);
    CHECK(th_seq_tick(&s, 1) == false);    /* gap after 1st long */
    CHECK(th_seq_running(&s));
    th_seq_stop(&s);
    CHECK(!th_seq_running(&s));
    CHECK(th_seq_tick(&s, 10) == false);
}

static void test_custom_and_invalid_patterns(void)
{
    static const th_step_t two_short[] = { { 300, 200 }, { 300, 0 } };
    static const th_step_t zero[] = { { 0, 0 } };
    th_step_t fra_buf[TH_FRA_STEPS];
    th_pattern_t p = { two_short, 2, false };
    th_pattern_t bad = { zero, 1, false };
    th_pattern_t empty = { two_short, 0, false };
    th_seq_t s;
    th_seq_init(&s);
    CHECK(th_seq_start(&s, &p));
    CHECK(th_seq_tick(&s, 299));
    CHECK(!th_seq_tick(&s, 1));
    CHECK(th_seq_tick(&s, 200));
    CHECK(!th_seq_tick(&s, 300));
    CHECK(!th_seq_running(&s));
    CHECK(!th_seq_start(&s, &bad));
    CHECK(!th_seq_start(&s, &empty));
    CHECK(!th_seq_start(&s, NULL));
    CHECK(!th_pattern_fra(NULL, 1, 1, 1, false, &p));
    CHECK(!th_pattern_fra(fra_buf, 0, 1, 1, false, &p));
    CHECK(!th_pattern_fra(fra_buf, 1, 0, 1, false, &p));
}

static void test_duty_cap(void)
{
    th_duty_cfg_t cfg = { 250, 1000 }; /* 25 %, 1 s burst */
    th_duty_t d;
    int i, on_ticks = 0;
    long on_total = 0;

    CHECK(th_duty_init(&d, &cfg));
    CHECK(th_duty_available_ms(&d) == 1000);
    /* continuous request: exactly 1000 ms ON, then lockout */
    for (i = 0; i < 100; i++) {
        if (th_duty_tick(&d, true, 10)) { on_ticks++; }
    }
    CHECK(on_ticks == 99); /* the tick reaching 1000 ms is cut */
    CHECK(th_duty_locked(&d));
    CHECK(th_duty_available_ms(&d) == 0);
    /* lockout lasts until drained: 1000*750/250 = 3000 ms OFF */
    CHECK(th_duty_tick(&d, true, 2990) == false);
    CHECK(th_duty_locked(&d));
    CHECK(th_duty_tick(&d, false, 10) == false);
    CHECK(!th_duty_locked(&d));
    CHECK(th_duty_tick(&d, true, 10) == true);

    /* long-run ON fraction never exceeds 25 % with a held button */
    th_duty_init(&d, &cfg);
    for (i = 0; i < 600000; i++) {
        if (th_duty_tick(&d, true, 1)) { on_total++; }
    }
    CHECK(on_total <= 600000L * 250 / 1000);
    CHECK(on_total >= 600000L * 250 / 1000 - 1000); /* and not far below */

    /* invalid configs; 100 % disables limiting */
    cfg.max_duty_permille = 0;    CHECK(!th_duty_init(&d, &cfg));
    cfg.max_duty_permille = 1001; CHECK(!th_duty_init(&d, &cfg));
    cfg.max_duty_permille = 500; cfg.max_on_ms = 0; CHECK(!th_duty_init(&d, &cfg));
    cfg.max_duty_permille = 1000; cfg.max_on_ms = 1;
    CHECK(th_duty_init(&d, &cfg));
    CHECK(th_duty_tick(&d, true, 100000) == true);
}

static void test_interlock_truth_table(void)
{
    th_interlock_cfg_t cfg;
    th_interlock_t il;
    th_interlock_out_t o;
    th_interlock_cfg_default(&cfg); /* horn 90/100, comp 120/150, max 200 */
    CHECK(th_interlock_init(&il, &cfg));

    /* pressure OK (130) x ignition */
    o = th_interlock_update(&il, 130.0f, true);
    CHECK(o.horn_allowed && !o.sensor_fault);
    o = th_interlock_update(&il, 130.0f, false);
    CHECK(!o.horn_allowed && !o.compressor_demand);
    /* pressure low (80) x ignition */
    o = th_interlock_update(&il, 80.0f, true);
    CHECK(!o.horn_allowed && o.compressor_demand);
    o = th_interlock_update(&il, 80.0f, false);
    CHECK(!o.horn_allowed && !o.compressor_demand);
    /* horn hysteresis: still blocked at 95, allowed at 100 */
    o = th_interlock_update(&il, 95.0f, true);  CHECK(!o.horn_allowed);
    o = th_interlock_update(&il, 100.0f, true); CHECK(o.horn_allowed);
    o = th_interlock_update(&il, 92.0f, true);  CHECK(o.horn_allowed);
    o = th_interlock_update(&il, 89.9f, true);  CHECK(!o.horn_allowed);
    /* compressor hysteresis: on until 150, stays off until <= 120 */
    o = th_interlock_update(&il, 149.0f, true); CHECK(o.compressor_demand);
    o = th_interlock_update(&il, 150.0f, true); CHECK(!o.compressor_demand);
    o = th_interlock_update(&il, 121.0f, true); CHECK(!o.compressor_demand);
    o = th_interlock_update(&il, 120.0f, true); CHECK(o.compressor_demand);
    /* ignition off keeps hysteresis state; back on resumes compressor */
    o = th_interlock_update(&il, 130.0f, false); CHECK(!o.compressor_demand);
    o = th_interlock_update(&il, 130.0f, true);  CHECK(o.compressor_demand);
    /* sensor faults: fail safe */
    o = th_interlock_update(&il, NAN, true);
    CHECK(o.sensor_fault && !o.horn_allowed && !o.compressor_demand);
    o = th_interlock_update(&il, 250.0f, true);
    CHECK(o.sensor_fault && !o.horn_allowed && !o.compressor_demand);
    o = th_interlock_update(&il, -10.0f, true);
    CHECK(o.sensor_fault && !o.compressor_demand);
    /* bad configs */
    cfg.comp_cut_out_psi = cfg.comp_cut_in_psi; CHECK(!th_interlock_init(&il, &cfg));
    th_interlock_cfg_default(&cfg); cfg.horn_resume_psi = 80.0f;
    CHECK(!th_interlock_init(&il, &cfg));
    th_interlock_cfg_default(&cfg); cfg.sensor_max_psi = 140.0f;
    CHECK(!th_interlock_init(&il, &cfg));
}

static void test_controller(void)
{
    th_duty_cfg_t duty = { 500, 5000 };
    th_interlock_cfg_t icfg;
    th_ctl_t c;
    th_ctl_out_t o;
    th_step_t steps[TH_FRA_STEPS];
    th_pattern_t p;
    th_interlock_cfg_default(&icfg);
    CHECK(th_ctl_init(&c, &duty, &icfg));
    th_pattern_fra_default(steps, false, &p);

    o = th_ctl_tick(&c, 10, 140.0f, true);
    CHECK(!o.solenoid && !o.compressor);
    th_seq_start(&c.seq, &p);
    o = th_ctl_tick(&c, 10, 140.0f, true);
    CHECK(o.solenoid && !o.blocked_by_interlock);
    o = th_ctl_tick(&c, 10, 140.0f, false); /* ignition off */
    CHECK(!o.solenoid && o.blocked_by_interlock);
    o = th_ctl_tick(&c, 10, 85.0f, true);   /* low pressure */
    CHECK(!o.solenoid && o.blocked_by_interlock && o.compressor);
    th_seq_stop(&c.seq);
    th_ctl_set_manual(&c, true);
    o = th_ctl_tick(&c, 10, 130.0f, true);  /* 85 -> 130 >= resume */
    CHECK(o.solenoid && o.compressor);
    o = th_ctl_tick(&c, 6000, 130.0f, true); /* held past 5 s burst */
    CHECK(!o.solenoid && o.blocked_by_duty);
    th_ctl_set_manual(&c, false);
    o = th_ctl_tick(&c, 10, NAN, true);
    CHECK(o.sensor_fault && !o.solenoid && !o.compressor);
}

static void test_math_reference_vectors(void)
{
    CHECK_NEAR(th_spl_at_distance(150.0, 100.0, 10.0), 130.0, 1e-6);
    CHECK_NEAR(th_spl_at_distance(150.0, 20.0, 10.0), 150.0 - 20.0 * log10(2.0), 1e-6);
    CHECK_NEAR(th_spl_at_distance(150.0, 1000.0, 10.0), 110.0, 1e-6);
    CHECK_NEAR(th_spl_at_distance(110.0, 5.0, 10.0), 116.020599913, 1e-6);
    CHECK_NEAR(th_distance_for_spl(150.0, 130.0, 10.0), 100.0, 1e-6);
    CHECK_NEAR(th_distance_for_spl(150.0, 150.0, 10.0), 10.0, 1e-6);
    CHECK(isnan(th_spl_at_distance(150.0, 0.0, 10.0)));
    CHECK(isnan(th_spl_at_distance(150.0, 10.0, -1.0)));

    /* trainhorn_calc reference values */
    CHECK_NEAR(th_free_air_cuft(1.0, 14.696, 0.0, TH_ATMOSPHERIC_PSI), 231.0 / 1728.0, 1e-6);
    CHECK_NEAR(th_tank_runtime_s(5.0, 150.0, 110.0, 10.0, TH_ATMOSPHERIC_PSI),
               10.91566866267465, 1e-6);
    CHECK_NEAR(th_tank_runtime_s(4.0, 150.0, 120.0, 5.0, TH_ATMOSPHERIC_PSI),
               2.0 * th_tank_runtime_s(2.0, 150.0, 120.0, 5.0, TH_ATMOSPHERIC_PSI), 1e-6);
    CHECK(th_tank_runtime_s(5.0, 150.0, 110.0, 10.0, 12.2) >
          th_tank_runtime_s(5.0, 150.0, 110.0, 10.0, TH_ATMOSPHERIC_PSI));
    CHECK(isnan(th_tank_runtime_s(0.0, 150.0, 110.0, 10.0, TH_ATMOSPHERIC_PSI)));
    CHECK(isnan(th_tank_runtime_s(5.0, 110.0, 150.0, 10.0, TH_ATMOSPHERIC_PSI)));
    CHECK(isnan(th_tank_runtime_s(5.0, 150.0, 150.0, 10.0, TH_ATMOSPHERIC_PSI)));
    CHECK(isnan(th_tank_runtime_s(5.0, 150.0, -1.0, 10.0, TH_ATMOSPHERIC_PSI)));
    CHECK(isnan(th_tank_runtime_s(5.0, 150.0, 110.0, 0.0, TH_ATMOSPHERIC_PSI)));
}

int main(void)
{
    printf("train_horn_ctl %s host tests\n", TRAIN_HORN_CTL_VERSION);
    RUN(test_fra_pattern_timing);
    RUN(test_seq_boundaries_and_big_dt);
    RUN(test_seq_repeat_and_stop);
    RUN(test_custom_and_invalid_patterns);
    RUN(test_duty_cap);
    RUN(test_interlock_truth_table);
    RUN(test_controller);
    RUN(test_math_reference_vectors);
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
