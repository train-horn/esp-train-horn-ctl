/*
 * train_horn_ctl - controller logic for an air train horn (solenoid valve +
 * compressor), plus small acoustic / air-system calculators.
 *
 * Pure C99. No ESP-IDF calls in this header or in src/, so the same code
 * runs in host unit tests and on any MCU. Time is passed in explicitly as
 * milliseconds via the *_tick() functions.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TRAIN_HORN_CTL_H
#define TRAIN_HORN_CTL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRAIN_HORN_CTL_VERSION "1.0.0"

/* ------------------------------------------------------------------------ */
/* 1. Pattern sequencer                                                      */
/* ------------------------------------------------------------------------ */

/** One element of a horn pattern: solenoid ON for on_ms, then OFF for off_ms. */
typedef struct {
    uint32_t on_ms;
    uint32_t off_ms;
} th_step_t;

/** A horn pattern. The steps array is NOT copied; keep it alive while playing. */
typedef struct {
    const th_step_t *steps;
    size_t n_steps;
    bool repeat; /**< restart from step 0 after the last step until stopped */
} th_pattern_t;

/** Number of steps in the grade-crossing pattern (long, long, short, long). */
#define TH_FRA_STEPS 4u

/** Default durations used by th_pattern_fra_default(). The US rule
 *  (49 CFR 222.21) fixes the long-long-short-long ORDER, not exact durations;
 *  these are common, adjustable values. */
#define TH_FRA_DEFAULT_LONG_MS  2000u
#define TH_FRA_DEFAULT_SHORT_MS 1000u
#define TH_FRA_DEFAULT_GAP_MS    500u

/**
 * Fill steps[0..3] with the grade-crossing pattern long-long-short-long and
 * point *out at it. The gap after the final long blast is also gap_ms (it
 * matters only when repeat is true).
 * @return false if an argument is NULL or long_ms/short_ms is 0.
 */
bool th_pattern_fra(th_step_t steps[TH_FRA_STEPS], uint32_t long_ms,
                    uint32_t short_ms, uint32_t gap_ms, bool repeat,
                    th_pattern_t *out);

/** Same as th_pattern_fra() with the TH_FRA_DEFAULT_* durations. */
bool th_pattern_fra_default(th_step_t steps[TH_FRA_STEPS], bool repeat,
                            th_pattern_t *out);

/** Non-blocking sequencer state. Treat the fields as private. */
typedef struct {
    th_pattern_t pattern;
    size_t step;
    uint32_t elapsed_ms; /**< time spent in the current step */
    bool running;
} th_seq_t;

void th_seq_init(th_seq_t *seq);
/** Start (or restart) a pattern from its first step. Returns false if the
 *  pattern is empty or has a step with zero total length. */
bool th_seq_start(th_seq_t *seq, const th_pattern_t *pattern);
/** Stop immediately; the output goes OFF. */
void th_seq_stop(th_seq_t *seq);
bool th_seq_running(const th_seq_t *seq);
/** Current requested solenoid state without advancing time. */
bool th_seq_output(const th_seq_t *seq);
/**
 * Advance the sequencer by dt_ms and return the requested solenoid state
 * (true = ON). Large dt values are handled correctly (steps are skipped
 * as needed), so a late tick never stretches a blast.
 */
bool th_seq_tick(th_seq_t *seq, uint32_t dt_ms);

/* ------------------------------------------------------------------------ */
/* 2. Duty-cycle limiter (solenoid / compressor protection)                  */
/* ------------------------------------------------------------------------ */

/**
 * Leaky-bucket limiter. While the output is ON the bucket fills at
 * (1000 - max_duty_permille) per ms; while OFF it drains at
 * max_duty_permille per ms. The capacity is max_on_ms * (1000 - duty), so:
 *   - from a rested state the horn may stay ON for exactly max_on_ms;
 *   - when the bucket is full the output is forced OFF (lockout) until the
 *     bucket has fully drained, so the valve never chatters;
 *   - the long-run ON fraction can never exceed max_duty_permille / 1000.
 */
typedef struct {
    uint16_t max_duty_permille; /**< 1..1000, e.g. 250 = 25 % */
    uint32_t max_on_ms;         /**< longest continuous blast from rest */
} th_duty_cfg_t;

typedef struct {
    th_duty_cfg_t cfg;
    uint64_t level;
    uint64_t capacity;
    bool locked;
} th_duty_t;

/** @return false on an invalid configuration (duty 0 or > 1000, max_on 0). */
bool th_duty_init(th_duty_t *d, const th_duty_cfg_t *cfg);
/**
 * Account dt_ms of the requested state and return the permitted state.
 * Pass the request that was applied during the last dt_ms.
 */
bool th_duty_tick(th_duty_t *d, bool requested_on, uint32_t dt_ms);
bool th_duty_locked(const th_duty_t *d);
/** Milliseconds of ON time currently available before lockout (0 if locked). */
uint32_t th_duty_available_ms(const th_duty_t *d);

/* ------------------------------------------------------------------------ */
/* 3. Pressure / ignition interlock and compressor control                   */
/* ------------------------------------------------------------------------ */

typedef struct {
    float horn_min_psi;     /**< horn blocked below this tank pressure */
    float horn_resume_psi;  /**< ...and allowed again at/above this (>= min) */
    float comp_cut_in_psi;  /**< compressor ON at/below this */
    float comp_cut_out_psi; /**< compressor OFF at/above this (> cut-in) */
    float sensor_max_psi;   /**< readings outside [-2, max] => sensor fault */
} th_interlock_cfg_t;

typedef struct {
    th_interlock_cfg_t cfg;
    bool pressure_ok;
    bool compressor_on;
    bool sensor_fault;
} th_interlock_t;

typedef struct {
    bool horn_allowed;      /**< ignition on, sensor valid, pressure OK */
    bool compressor_demand; /**< ignition on, sensor valid, hysteresis band */
    bool sensor_fault;
} th_interlock_out_t;

/** Typical values for a 150 psi on-board air system. */
void th_interlock_cfg_default(th_interlock_cfg_t *cfg);
/** @return false if thresholds are inconsistent. */
bool th_interlock_init(th_interlock_t *il, const th_interlock_cfg_t *cfg);
/**
 * Evaluate with the latest tank pressure (gauge psi) and ignition state.
 * Fail-safe: NaN or out-of-range pressure => horn blocked, compressor OFF.
 * Ignition off => horn blocked, compressor OFF (no battery drain); the
 * hysteresis state is kept so the compressor resumes correctly.
 */
th_interlock_out_t th_interlock_update(th_interlock_t *il, float tank_psi,
                                       bool ignition_on);

/* ------------------------------------------------------------------------ */
/* 4. Combined controller                                                    */
/* ------------------------------------------------------------------------ */

typedef struct {
    th_seq_t seq;
    th_duty_t duty;
    th_interlock_t interlock;
    bool manual; /**< momentary horn button held */
} th_ctl_t;

typedef struct {
    bool solenoid;   /**< drive the horn valve relay */
    bool compressor; /**< drive the compressor relay */
    bool blocked_by_interlock;
    bool blocked_by_duty;
    bool sensor_fault;
} th_ctl_out_t;

bool th_ctl_init(th_ctl_t *c, const th_duty_cfg_t *duty,
                 const th_interlock_cfg_t *interlock);
/** Momentary manual horn (button). Combined with the sequencer by OR. */
void th_ctl_set_manual(th_ctl_t *c, bool on);
/**
 * Advance everything by dt_ms. Solenoid = (sequencer OR manual) AND
 * interlock AND duty limiter. The sequencer keeps time while blocked, so a
 * pattern is never replayed late.
 */
th_ctl_out_t th_ctl_tick(th_ctl_t *c, uint32_t dt_ms, float tank_psi,
                         bool ignition_on);

/* ------------------------------------------------------------------------ */
/* 5. Calculators (same formulas as the trainhorn-calc Python package)       */
/* ------------------------------------------------------------------------ */

#define TH_ATMOSPHERIC_PSI 14.696
#define TH_CUFT_PER_GALLON (231.0 / 1728.0)

/** Inverse-square law: L = source_db - 20*log10(distance / ref_distance).
 *  Distances in any one unit. Returns NaN if a distance is <= 0. */
double th_spl_at_distance(double source_db, double distance,
                          double ref_distance);
/** Inverse: d = ref_distance * 10^((source_db - target_db) / 20). */
double th_distance_for_spl(double source_db, double target_db,
                           double ref_distance);
/** Boyle's law free air (cu ft at atmospheric) released between two gauge
 *  pressures: V_tank * (start - end) / P_atm. NaN on invalid input. */
double th_free_air_cuft(double tank_gallons, double start_psi, double end_psi,
                        double atmospheric_psi);
/** Continuous horn seconds from start_psi down to cutoff_psi for a horn that
 *  uses horn_cfm of free air: free_air / horn_cfm * 60. NaN on bad input. */
double th_tank_runtime_s(double tank_gallons, double start_psi,
                         double cutoff_psi, double horn_cfm,
                         double atmospheric_psi);

#ifdef __cplusplus
}
#endif

#endif /* TRAIN_HORN_CTL_H */
