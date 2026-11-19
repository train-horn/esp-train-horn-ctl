/*
 * train_horn_ctl - pure C99 implementation. See include/train_horn_ctl.h.
 * SPDX-License-Identifier: MIT
 */
#include "train_horn_ctl.h"

#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------- pattern */

bool th_pattern_fra(th_step_t steps[TH_FRA_STEPS], uint32_t long_ms,
                    uint32_t short_ms, uint32_t gap_ms, bool repeat,
                    th_pattern_t *out)
{
    if (steps == NULL || out == NULL || long_ms == 0u || short_ms == 0u) {
        return false;
    }
    steps[0].on_ms = long_ms;  steps[0].off_ms = gap_ms;
    steps[1].on_ms = long_ms;  steps[1].off_ms = gap_ms;
    steps[2].on_ms = short_ms; steps[2].off_ms = gap_ms;
    steps[3].on_ms = long_ms;  steps[3].off_ms = gap_ms;
    out->steps = steps;
    out->n_steps = TH_FRA_STEPS;
    out->repeat = repeat;
    return true;
}

bool th_pattern_fra_default(th_step_t steps[TH_FRA_STEPS], bool repeat,
                            th_pattern_t *out)
{
    return th_pattern_fra(steps, TH_FRA_DEFAULT_LONG_MS, TH_FRA_DEFAULT_SHORT_MS,
                          TH_FRA_DEFAULT_GAP_MS, repeat, out);
}

/* -------------------------------------------------------------- sequencer */

void th_seq_init(th_seq_t *seq)
{
    if (seq != NULL) {
        memset(seq, 0, sizeof(*seq));
    }
}

bool th_seq_start(th_seq_t *seq, const th_pattern_t *pattern)
{
    size_t i;
    if (seq == NULL || pattern == NULL || pattern->steps == NULL ||
        pattern->n_steps == 0u) {
        return false;
    }
    for (i = 0; i < pattern->n_steps; i++) {
        if ((uint64_t)pattern->steps[i].on_ms + pattern->steps[i].off_ms == 0u) {
            return false;
        }
    }
    seq->pattern = *pattern;
    seq->step = 0;
    seq->elapsed_ms = 0;
    seq->running = true;
    return true;
}

void th_seq_stop(th_seq_t *seq)
{
    if (seq != NULL) {
        seq->running = false;
        seq->step = 0;
        seq->elapsed_ms = 0;
    }
}

bool th_seq_running(const th_seq_t *seq)
{
    return seq != NULL && seq->running;
}

bool th_seq_output(const th_seq_t *seq)
{
    if (seq == NULL || !seq->running) {
        return false;
    }
    return seq->elapsed_ms < seq->pattern.steps[seq->step].on_ms;
}

bool th_seq_tick(th_seq_t *seq, uint32_t dt_ms)
{
    uint64_t remaining = dt_ms;
    if (seq == NULL || !seq->running) {
        return false;
    }
    while (remaining > 0u && seq->running) {
        const th_step_t *st = &seq->pattern.steps[seq->step];
        uint64_t len = (uint64_t)st->on_ms + st->off_ms;
        uint64_t left = len - seq->elapsed_ms;
        if (remaining < left) {
            seq->elapsed_ms += (uint32_t)remaining;
            remaining = 0;
        } else {
            remaining -= left;
            seq->elapsed_ms = 0;
            seq->step++;
            if (seq->step >= seq->pattern.n_steps) {
                if (seq->pattern.repeat) {
                    seq->step = 0;
                } else {
                    th_seq_stop(seq);
                }
            }
        }
    }
    return th_seq_output(seq);
}

/* ---------------------------------------------------------- duty limiter */

bool th_duty_init(th_duty_t *d, const th_duty_cfg_t *cfg)
{
    if (d == NULL || cfg == NULL || cfg->max_duty_permille == 0u ||
        cfg->max_duty_permille > 1000u || cfg->max_on_ms == 0u) {
        return false;
    }
    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
    d->capacity = (uint64_t)cfg->max_on_ms * (1000u - cfg->max_duty_permille);
    return true;
}

bool th_duty_tick(th_duty_t *d, bool requested_on, uint32_t dt_ms)
{
    uint64_t fill, drain;
    if (d == NULL) {
        return false;
    }
    if (d->cfg.max_duty_permille >= 1000u) {
        return requested_on; /* 100 %: limiter disabled */
    }
    fill = (uint64_t)dt_ms * (1000u - d->cfg.max_duty_permille);
    drain = (uint64_t)dt_ms * d->cfg.max_duty_permille;

    if (requested_on && !d->locked) {
        d->level += fill;
        if (d->level >= d->capacity) {
            d->level = d->capacity;
            d->locked = true;
        }
    } else {
        d->level = (d->level > drain) ? d->level - drain : 0u;
        if (d->locked && d->level == 0u) {
            d->locked = false;
        }
    }
    return requested_on && !d->locked;
}

bool th_duty_locked(const th_duty_t *d)
{
    return d != NULL && d->locked;
}

uint32_t th_duty_available_ms(const th_duty_t *d)
{
    uint64_t per_ms;
    if (d == NULL || d->locked) {
        return 0u;
    }
    if (d->cfg.max_duty_permille >= 1000u) {
        return UINT32_MAX;
    }
    per_ms = 1000u - d->cfg.max_duty_permille;
    return (uint32_t)((d->capacity - d->level) / per_ms);
}

/* --------------------------------------------------------------- interlock */

#define TH_SENSOR_MIN_PSI (-2.0f)

void th_interlock_cfg_default(th_interlock_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->horn_min_psi = 90.0f;
    cfg->horn_resume_psi = 100.0f;
    cfg->comp_cut_in_psi = 120.0f;
    cfg->comp_cut_out_psi = 150.0f;
    cfg->sensor_max_psi = 200.0f;
}

bool th_interlock_init(th_interlock_t *il, const th_interlock_cfg_t *cfg)
{
    if (il == NULL || cfg == NULL) {
        return false;
    }
    if (!(cfg->horn_min_psi >= 0.0f) ||
        !(cfg->horn_resume_psi >= cfg->horn_min_psi) ||
        !(cfg->comp_cut_in_psi >= 0.0f) ||
        !(cfg->comp_cut_out_psi > cfg->comp_cut_in_psi) ||
        !(cfg->sensor_max_psi > cfg->comp_cut_out_psi)) {
        return false;
    }
    memset(il, 0, sizeof(*il));
    il->cfg = *cfg;
    return true;
}

th_interlock_out_t th_interlock_update(th_interlock_t *il, float tank_psi,
                                       bool ignition_on)
{
    th_interlock_out_t out = { false, false, false };
    if (il == NULL) {
        out.sensor_fault = true;
        return out;
    }
    /* NaN fails every comparison, so it lands in the fault branch. */
    if (!(tank_psi >= TH_SENSOR_MIN_PSI && tank_psi <= il->cfg.sensor_max_psi)) {
        il->sensor_fault = true;
        il->pressure_ok = false;
        il->compressor_on = false;
        out.sensor_fault = true;
        return out;
    }
    il->sensor_fault = false;

    /* Horn pressure hysteresis. */
    if (tank_psi < il->cfg.horn_min_psi) {
        il->pressure_ok = false;
    } else if (tank_psi >= il->cfg.horn_resume_psi) {
        il->pressure_ok = true;
    }
    /* Compressor hysteresis. */
    if (tank_psi <= il->cfg.comp_cut_in_psi) {
        il->compressor_on = true;
    } else if (tank_psi >= il->cfg.comp_cut_out_psi) {
        il->compressor_on = false;
    }

    out.horn_allowed = ignition_on && il->pressure_ok;
    out.compressor_demand = ignition_on && il->compressor_on;
    return out;
}

/* -------------------------------------------------------------- controller */

bool th_ctl_init(th_ctl_t *c, const th_duty_cfg_t *duty,
                 const th_interlock_cfg_t *interlock)
{
    if (c == NULL) {
        return false;
    }
    memset(c, 0, sizeof(*c));
    th_seq_init(&c->seq);
    return th_duty_init(&c->duty, duty) &&
           th_interlock_init(&c->interlock, interlock);
}

void th_ctl_set_manual(th_ctl_t *c, bool on)
{
    if (c != NULL) {
        c->manual = on;
    }
}

th_ctl_out_t th_ctl_tick(th_ctl_t *c, uint32_t dt_ms, float tank_psi,
                         bool ignition_on)
{
    th_ctl_out_t out = { false, false, false, false, false };
    th_interlock_out_t il;
    bool want, gated;
    if (c == NULL) {
        return out;
    }
    want = th_seq_tick(&c->seq, dt_ms) || c->manual;
    il = th_interlock_update(&c->interlock, tank_psi, ignition_on);
    gated = want && il.horn_allowed;
    out.solenoid = th_duty_tick(&c->duty, gated, dt_ms);
    out.compressor = il.compressor_demand;
    out.blocked_by_interlock = want && !il.horn_allowed;
    out.blocked_by_duty = gated && !out.solenoid;
    out.sensor_fault = il.sensor_fault;
    return out;
}

/* -------------------------------------------------------------------- math */

double th_spl_at_distance(double source_db, double distance, double ref_distance)
{
    if (!(distance > 0.0) || !(ref_distance > 0.0) || !isfinite(source_db)) {
        return NAN;
    }
    return source_db - 20.0 * log10(distance / ref_distance);
}

double th_distance_for_spl(double source_db, double target_db, double ref_distance)
{
    if (!(ref_distance > 0.0) || !isfinite(source_db) || !isfinite(target_db)) {
        return NAN;
    }
    return ref_distance * pow(10.0, (source_db - target_db) / 20.0);
}

double th_free_air_cuft(double tank_gallons, double start_psi, double end_psi,
                        double atmospheric_psi)
{
    if (!(tank_gallons > 0.0) || !(end_psi >= 0.0) || !(start_psi > end_psi) ||
        !(atmospheric_psi > 0.0) || !isfinite(start_psi) || !isfinite(tank_gallons)) {
        return NAN;
    }
    return tank_gallons * TH_CUFT_PER_GALLON * (start_psi - end_psi) / atmospheric_psi;
}

double th_tank_runtime_s(double tank_gallons, double start_psi,
                         double cutoff_psi, double horn_cfm,
                         double atmospheric_psi)
{
    double v;
    if (!(horn_cfm > 0.0)) {
        return NAN;
    }
    v = th_free_air_cuft(tank_gallons, start_psi, cutoff_psi, atmospheric_psi);
    if (isnan(v)) {
        return NAN;
    }
    return v / horn_cfm * 60.0;
}
