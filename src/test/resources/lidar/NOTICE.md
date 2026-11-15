# Test data for the LAS/LAZ reader (LidarRegression)

Pairs of LAZ files and the same points as LAS, written by reference tools, used to check that the
LAZ decoder returns the LAS records byte for byte.

- `point10`, `point-color`, `point-time`, `point-time-color`, `extra-bytes`, `point-version-1-point-wise`
  (`.las` / `.laz`): from the laz-rs repository (https://github.com/laz-rs/laz-rs, `tests/data`),
  Apache License 2.0.
- `1_4_w_evlr` (`.las` / `.laz`), `simple.las`, `simple.copc.laz`: from the laspy repository
  (https://github.com/laspy/laspy, `tests/data`), BSD 2-Clause License.
