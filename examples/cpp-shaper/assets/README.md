# Shared Covalent Shaper Example Assets

These deterministic assets are shared inputs for the Python and C++ examples:

- `model/model_bundle.json`
- `model/shaper_models.native.json`
- `model/shaper_models.native.qualification.json`
- `modelone.urdf`

They were copied from `reforge-interface` commit `22dc942`. Source provenance,
SHA-256 values, feature order, units, and qualification status are frozen in
`docs/control/cpp-shaper-example-usage-phase1-baseline.md`.

The legacy one-axis Python baseline remains in `../python/axis0_model.pt` until
the cross-language native-backend transition. Keeping it separate prevents the
relocation phase from changing controller behavior.
