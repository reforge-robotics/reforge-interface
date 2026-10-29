# UF850 base-angle Shaper retraining

Status: retrained models are promoted to `src/robot/models/current/shaper` on
`ufactory-xarm-sdk`. This records the reproducible procedure and the results
that justified local promotion; physical off-angle validation is still needed.

## Provenance and method

- Source corpus: `src/robot/data/combined-payloads-2026-8-26-to-30`, the
  documented 277-row corpus. Its manifest SHA-256 was
  `49437d78f18a0e93590c873f10ae8406ef94027c09451fe549939a607d7bd3da`.
- Existing source FRF map shards were reused. Their hashes matched the source
  hashes documented in
  `reforge-core/docs/control/shaper-neural-network-smoothness.md`; the
  augmentation script records/validates this provenance when preparing maps.
- Nine evenly spaced base-joint labels cover the UF850 usable range
  `[-350°, +350°]`, including `0°`:
  `[-6.108652382, -4.581489286, -3.054326191, -1.527163095, 0,
  1.527163095, 3.054326191, 4.581489286, 6.108652382]` radians.
- Every measured axis row is retained at zero and duplicated at the eight
  nonzero labels, yielding 2,493 training rows (axis counts 819, 837, 837).
  Synthetic copies reuse the measured motion/vibration targets and only change
  angle-tagged base metadata. This encodes the assumed rotational symmetry; it
  is not new physical data.
- Exact existing per-axis network topologies were retained: axis 0 `[59]`,
  axis 1 `[79, 85]`, axis 2 `[121]`. Three 40,000-epoch, no-Optuna passes fit
  each target-axis model (nine fits total), seed base `20261029`.
- Reproduction entry point: `python scripts/retrain_ufactory_shaper_base_angle.py
  --help`. The commands are `augment`, `prepare-maps`, `train`, and `compare`;
  generated data/reports stay under ignored `src/robot/data` staging paths.

## Comparison

Measured zero-angle labels are evaluated in-sample. Regression metric is mean
squared error over finite modal targets. Off-angle results measure prediction
sensitivity when only the input base angle is changed; they are not accuracy
against physical off-angle measurements.

| Axis | Measured rows | Zero-angle MSE, old → new | Mean absolute mode change across off-angle grid, old → new | P95 change, old → new |
|---|---:|---:|---:|---:|
| 0 | 91 | 170.657 → 37.391 | 7,815.799 → 0.00642 | 42,026.398 → 0.03060 |
| 1 | 93 | 66.773 → 47.304 | 5,644.320 → 0.05069 | 21,454.617 → 0.24407 |
| 2 | 93 | 163.676 → 158.671 | 7,127.159 → 0.00301 | 26,221.686 → 0.01165 |

Classification accuracy was unchanged at 0.9780, 1.0, and 0.9785 respectively.
Axis 2 classification BCE increased (0.2194 → 0.2968) despite unchanged
accuracy; this is retained as a caveat for follow-up evaluation.

## Runtime and artifact notes

The promoted model bundle keeps schema v3, the four inputs
`j0_rad, v_deg, r_mm, inertia`, and omits the optional
`trained_on_multiple_base_angles` flag introduced by PR #546. The current Core
checkout predates that field, and omitting it avoids breaking older strict
runtime parsers. PR #546's gate is derived from base-prefix axis rows; this
training instead covers all per-axis models across the nine angle labels.

The native qualification status is `passed`. Public TorchScript SHA-256:

- axis 0: `f0b08884fbfa9238de63b078c0aa6fc94fb25df43cf3930be21d3d7baa292b8d`
- axis 1: `8aeb27f475a14b12cf2cb6500b9ef1b34e613cec16e8894d689ac6fd16711502`
- axis 2: `c64e47179fbca0de181c822fd55a1d7e4627005c0805f770e3147e15c25ba8f2`

PR #546 was not merged as part of this work, no Standard Bots data/models were
changed, and the public model changes have not been pushed.
