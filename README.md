# train_horn_ctl

Controller logic for an ESP32-based air train horn: a horn solenoid valve and an on-board compressor. The core is pure C99 with no ESP-IDF calls, so you can unit-test it on a PC and reuse it on other MCUs.

What it does:

- **Pattern sequencer.** A non-blocking state machine for the grade-crossing pattern (long, long, short, long) and for your own patterns. You drive it with `tick(dt_ms)`, and it returns the solenoid ON/OFF state. A late or large tick skips steps correctly instead of stretching a blast.
- **Duty-cycle limiter.** A leaky-bucket cap on solenoid ON time. You set the longest continuous blast from rest and the long-run ON fraction. When the bucket is full, the output locks off until the bucket has fully drained, so the valve never chatters. This protects the valve coil and the compressor.
- **Interlock.** The horn is allowed only when the ignition is on and the tank pressure is OK, with hysteresis. Compressor demand uses cut-in/cut-out hysteresis in PSI. The interlock fails safe: a NaN, open-wire or out-of-range sensor reading blocks the horn and stops the compressor.
- **Calculators.** dB at distance (inverse-square law) and air-tank runtime (Boyle's law). These are the same formulas as the [`trainhorn-calc`](https://github.com/train-horn/trainhorn-calc) Python package.

## Install

```sh
idf.py add-dependency "train-horn/train_horn_ctl^1.0.0"
```

## Usage

```c
#include "train_horn_ctl.h"

static th_ctl_t ctl;
static th_step_t fra_steps[TH_FRA_STEPS];
static th_pattern_t fra;

void setup(void)
{
    th_duty_cfg_t duty = { .max_duty_permille = 250, .max_on_ms = 8000 };
    th_interlock_cfg_t il;
    th_interlock_cfg_default(&il);           /* horn 90/100 psi, compressor 120/150 psi */
    th_ctl_init(&ctl, &duty, &il);
    th_pattern_fra(fra_steps, 2000, 1000, 500, false, &fra); /* long, short, gap ms */
}

void every_10_ms(float tank_psi, bool ignition, bool horn_button)
{
    th_ctl_set_manual(&ctl, horn_button);    /* momentary horn */
    th_ctl_out_t o = th_ctl_tick(&ctl, 10, tank_psi, ignition);
    set_relay(SOLENOID, o.solenoid);
    set_relay(COMPRESSOR, o.compressor);
}

/* on a "crossing" button press: */
th_seq_start(&ctl.seq, &fra);
```

You can also use each block on its own: `th_seq_*`, `th_duty_*`, `th_interlock_*`.

Custom pattern, for example two short toots:

```c
static const th_step_t toot2[] = { { 300, 200 }, { 300, 0 } };
th_pattern_t p = { toot2, 2, false };
th_seq_start(&ctl.seq, &p);
```

Calculators:

```c
th_spl_at_distance(150.0, 100.0, 10.0);          /* 150 dB @10 ft -> 130.0 dB @100 ft */
th_distance_for_spl(150.0, 130.0, 10.0);         /* -> 100.0 ft */
th_tank_runtime_s(5.0, 150.0, 110.0, 10.0, TH_ATMOSPHERIC_PSI); /* 5 gal, 150->110 psi, 10 CFM horn -> 10.92 s */
```

The calculators return NaN on invalid input. They are estimates: inverse-square assumes a free field, and the runtime assumes isothermal expansion, so treat it as an upper bound.

### About the crossing pattern

In the US, 49 CFR 222.21 sets the *order* of the grade-crossing signal (long, long, short, long). It does not set exact blast lengths. The defaults here (2 s long, 1 s short, 0.5 s gaps) are adjustable. Using a train horn on a road vehicle is restricted in many places. Check your state and local rules; this README is not legal advice.

## Example

`examples/horn_controller` is an ESP-IDF 5.3+ project. A 10 ms `esp_timer` reads the buttons and ignition with `gpio_get_level()`, reads a 0.5-4.5 V pressure transducer through ADC1, and drives the two relays with `gpio_set_level()`.

```sh
cd examples/horn_controller
idf.py set-target esp32
idf.py build flash monitor
```

## Wiring notes

```
 +12V ──[fuse]──┬──────────────── relay COM
                │                 relay NO ───────┬──── solenoid (+)
                │                                 │
                │                      1N4007     ┴ cathode to (+)
                │                      flyback    ┬
                │                                 │
 GND ───────────┴─────────────────────────────────┴──── solenoid (-)

 ESP32 GPIO32 ──► relay module IN (logic-level input, or a transistor driver)
```

- **Relay.** Don't drive a horn solenoid or compressor from a GPIO. Use an automotive relay (30/40 A for the compressor) switched by a relay module or a logic-level MOSFET/transistor stage with its own flyback diode on the relay coil. Use a separate relay for the horn valve and for the compressor.
- **Flyback diode.** Put a diode (1N4007 or similar; a heavier one such as 1N5408 for large valves) across every inductive load: the solenoid, the relay coils and, if your compressor does not already have one, the compressor motor. Connect the cathode (band) to the positive side. Without it, the switch-off spike damages driver transistors and resets the ESP32.
- **Fuse.** Fuse every feed close to the battery. Size the fuse for the wire, not for the load: for example, a 30 A fuse on the compressor feed if the compressor's maker recommends it, and 5-10 A on the horn valve circuit. Use the wire gauge the compressor maker specifies for its fuse rating.
- **Ignition sense.** Feed the 12 V ignition line through an opto-isolator, or through a divider (e.g. 33k/10k) plus a 3.3 V zener clamp, into the ignition GPIO. Never connect 12 V straight to a pin.
- **Pressure sensor.** For a common 0.5-4.5 V, 0-200 psi transducer, bring the signal down below about 3.1 V with a divider (e.g. 10k/18k) and calibrate against a mechanical gauge. A reading below about 0.3 V is treated as an open wire.
- **Power.** Run the ESP32 from a proper automotive 12 V to 5 V buck converter that survives load-dump transients, not from a bare linear regulator.

For step-by-step install guides for horns, relays, compressors and air lines, see the component's Homepage link (the train-horn.com install guides).

## Safety notes

- This library is a convenience layer, **not a safety system**. Keep a mechanical pressure-relief valve on the tank (most kits include one) and the compressor's own pressure switch, or a hardware cut-out, as the final protection against over-pressure.
- The software interlock fails safe (sensor fault → horn and compressor off), but a welded relay contact can still keep a load on. A fuse limits the damage.
- Choose `max_duty_permille` and `max_on_ms` from your valve's and compressor's datasheets. Many small 12 V compressors are rated for 15-30 % duty.
- Train horns are very loud. 150 dB at 10 ft is still about 130 dB at 100 ft, so never sound one near people's ears, and follow local noise and equipment laws.

## Tests

The host tests use gcc or clang and need no ESP-IDF:

```sh
make -C test        # 102 checks: pattern timing, duty cap, interlock truth table, math vectors
```

## About

This component is maintained by [Train Horn](https://train-horn.com/), an independent reference on train horns (not a store). License: MIT.
