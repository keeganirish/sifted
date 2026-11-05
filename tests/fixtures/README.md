# Test fixtures

Small, curated, committed inputs. Large recordings belong in `recordings/` (git-ignored).

- `measurement_bearing_only_v1.hex`: golden wire frame (v1), derived by hand from
  the format spec, not produced by either encoder. Both the C++ and Python tests
  must reproduce it exactly. If it has to change, the wire version changes too.
