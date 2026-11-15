# M3 Webots tuning and perpendicular-parking results

Owner: Harshil Mittal. Measured on 15 November 2026 using Webots R2025b nightly 10/11/2026 on a MacBook Air.

## Outcome and code change

The selected default parking speed is **0.45 m/s** and lookahead is **0.55 m**, replacing 0.35 m/s and 0.65 m in `ManoeuvreConfig` (`behaviours/parking_manoeuvre.py`). Safety margins, stopping checks, steering limits, correction limits and the reference turning-radius setting are unchanged.

These settings were selected after controlled Webots comparisons and an additional model check of slow steering. They reduced the measured ALIGN-to-DONE parking duration by approximately 19% on A2, B0 and A5, with smaller final heading errors. All five selected-setting runs passed the existing full-footprint, alignment and stopped checks. This is validation on three bays, not evidence for every bay or every disturbance.

## Experimental setup

Base revision: `549b161d26c325540b89cd1c67c397c5f74c43c6`. Each Webots run reloaded the same world and started at rear-axle pose (2.0, 8.5, 0.0). M1 sensed the bays, M2 planned the route, M3 ran ALIGN/PARK, and M4 drove the four wheel and two steering motors. For targeted comparisons, other free bays were excluded from slot selection while remaining free on the map. They were not turned into obstacles. M1 still had to classify the requested bay as free.

The common world includes the separately proposed M1 lidar mount fix: height 0.28 → 0.35 m above the rear axle, clearing the 0.30 m tyre envelope while remaining below the 0.38 m cabin lower edge. Without it, self-returns from the wheels prevent route planning. The three-bay before/after comparisons all use this same fixed world. Overhead viewing and a plain floor appearance are visual changes; controller instrumentation logs every 32 ms.

The vehicle uses a 2.8 m wheelbase, a 4.6 × 1.9 m M3 envelope, 1.0 m rear overhang and 0.5 rad bicycle steering limit. The physical steering joints have a configured 3 rad/s maximum rate. Webots runs used fast mode; reported durations are simulation time, not wall-clock performance.

## Tuning process

1. Compare lookahead 0.50, 0.65 and 0.80 m on A2 at 0.35 m/s. The shorter setting improved alignment, with more steering adjustment.
2. At 0.50 m lookahead, compare 0.25, 0.35 and 0.45 m/s. Slowing to 0.25 m/s increased parking duration to 63.36 s without a useful alignment gain. The 0.45 m/s candidate was faster and worked in the live A2 and B0 cases.
3. Check slow steering (0.2 rad/s) using the existing kinematic motion-test fixture. The 0.45 m/s + 0.50 m combination exhausted its corrections on both sides, so it was rejected as the global default. A lookahead screen found 0.55 m to be the first tested value above it that passed both sides at 0.45 m/s. This model screen is separate from measured Webots results.
4. Validate 0.45 m/s + 0.55 m in Webots on A2, B0 and A5, and repeat A2 three times. The selected values passed all those runs.

## Paired Webots comparison

| Bay | Runs before / selected | Parking before → selected (s) | Reduction | Heading before → selected (°) | Selected centre error (cm) | Selected min lidar gap (m) |
|---|---:|---:|---:|---:|---:|---:|
| A2 | 3 / 3 | 45.18 → 36.67 | 18.8% | 0.76 → 0.40 | 5.93 | 0.184 |
| B0 | 1 / 1 | 46.05 → 37.18 | 19.2% | 0.74 → 0.36 | 6.02 | 0.209 |
| A5 | 1 / 1 | 45.15 → 36.67 | 18.8% | 0.78 → 0.40 | 5.94 | 0.196 |

A2 was repeated three times for each original/selected setting; B0 and A5 have one run per setting. Repeats are deterministic and match at the displayed precision. No statistical confidence or general parking success rate is inferred from this small set.

Parking duration starts at the first ALIGN and ends at the first DONE. Centre error uses the body centre reconstructed from the measured rear-axle pose. Heading error is wrapped relative to the bay's desired heading. A positive minimum lidar-derived body clearance is not an independent collision measurement; contact sensing was not instrumented.

## All measured parking trials

| Case ID | Bay | Speed (m/s) | Lookahead (m) | Outcome | Parking (s) | Heading error (°) |
|---|---|---:|---:|---|---:|---:|
| A2 original repeat | A2 | 0.35 | 0.65 | DONE | 45.18 | 0.76 |
| A2 original repeat | A2 | 0.35 | 0.65 | DONE | 45.18 | 0.76 |
| A2 original repeat | A2 | 0.35 | 0.65 | DONE | 45.18 | 0.76 |
| A2_L050_V035 | A2 | 0.35 | 0.50 | DONE | 45.41 | 0.24 |
| A2_L080_V035 | A2 | 0.35 | 0.80 | DONE | 45.12 | 0.98 |
| A2_L050_V025 | A2 | 0.25 | 0.50 | DONE | 63.36 | 0.33 |
| A2_L050_V045 | A2 | 0.45 | 0.50 | DONE | 36.74 | 0.18 |
| B0_DEFAULT | B0 | 0.35 | 0.65 | DONE | 46.05 | 0.74 |
| B0_TUNED | B0 | 0.45 | 0.50 | DONE | 37.38 | 0.21 |
| A5_DEFAULT | A5 | 0.35 | 0.65 | DONE | 45.15 | 0.78 |
| A2_REFINED_L055_V045 | A2 | 0.45 | 0.55 | DONE | 36.67 | 0.40 |
| B0_SELECTED | B0 | 0.45 | 0.55 | DONE | 37.18 | 0.36 |
| A5_SELECTED | A5 | 0.45 | 0.55 | DONE | 36.67 | 0.40 |
| A2_SELECTED_REPEAT_02 | A2 | 0.45 | 0.55 | DONE | 36.67 | 0.40 |
| A2_SELECTED_VIDEO_REPEAT_03 | A2 | 0.45 | 0.55 | DONE | 36.67 | 0.40 |

All 15 measured parking trials in this table reached DONE; some configurations were nevertheless rejected because of model-test results. The two earlier diagnostic starts that remained in SEARCH before the lidar fix are not parking trials in this table. For the selected A2 case, whole-run time was 52.928 s, of which 36.672 s was ALIGN → DONE. All selected-setting runs used zero correction manoeuvres.

## Regression tests

Changing the parking speed exposed a brittle obstacle test: it injected an obstacle at fixed times 1, 15 and 40 s, and a faster valid manoeuvre could already be DONE at the last time. The fixture now triggers the obstacle during actual setup motion, while reverse-turn heading is between 20° and 70°, and during final reverse with less than 0.8 m remaining and heading within 5° of the target. Each case still requires EMERGENCY_STOP and STOP mode. The expected safety outcome was preserved.

The unchanged slow-steering-response test passes with the selected defaults. In a clean validation copy containing the M3 changes plus the proposed M1 height fix and its regression test, the full suite passed: **115 tests, OK** (`python3 -B -m unittest discover -s tests`).

## Evidence

- [Paired comparison table](../results/m3_webots_tuning/paired_comparison.csv)
- [All 15 run metrics](../results/m3_webots_tuning/all_runs.csv)
- [Comparison figure](../results/m3_webots_tuning/tuning_comparison.png)
- [A2 measured speed and heading traces](../results/m3_webots_tuning/A2_before_after.png)
- [Tuned Webots video](../results/m3_webots_tuning/A2_tuned_demo.mp4)
- Raw per-step CSVs, console output, metadata and event logs: `results/m3_webots_tuning/run_01` through `run_15`.
- Slow-steering model screen: `results/m3_webots_tuning/model_steering_screen.csv`.

The video is Webots' native recording of selected A2 repeat 3, trimmed at 57 s after the car completes parking at 52.928 s. The playback speed was not changed. Local output-directory paths in shared console copies are replaced with a placeholder; measured values are unchanged.

## Remaining limits

Other bays, explicit pose disturbances, sensor noise and introduced-obstacle tests still need live Webots coverage. The emergency-stop phase tests above use the kinematic fixture. The six difficult end-of-aisle cases in the earlier offline sweep have not been rerun here. These results apply to the tested vehicle/world and preserve the safety settings; they do not establish a globally optimal tuning or independently prove contact-free motion.
