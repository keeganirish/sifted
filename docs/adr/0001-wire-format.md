# ADR 0001: Custom framing for the wire format

Status: accepted (M0)

## Context
Sensors send measurements over UDP; the same bytes are written to the event
log. The format must be versioned, size-limited, strictly validated, and
produce *identical bytes for identical input* (replay is bit-for-bit).

## Decision
A small hand-rolled binary framing, version 1, specified in
`src/sifted/wire/codec.hpp`: 8-byte header (magic `SIFT`, version, type,
u16 length) + payload. Little-endian, IEEE-754 doubles, one frame per datagram.
The decoder is **strict and canonical**: unused fields must be zero, reserved
bytes zero, trailing bytes rejected, so each value has exactly one valid
encoding. A Python mirror (`sim/sifted_sim/wire.py`) and a shared golden
fixture (`tests/fixtures/`) keep both languages in agreement.

## Alternatives considered
- **Protocol Buffers:** mature, but no canonical-serialization guarantee,
  adds a codegen dependency, and the interesting engineering (bounds, limits,
  strictness) would be hidden in a library.
- **FlatBuffers:** zero-copy but needs a verifier pass and is similarly opaque.

## Consequences
+ No dependency; trivially fuzzable; stable bytes for replay and golden tests.
- We own the evolution story: new fields need a version bump, and the decoder
  rejects unknown versions rather than guessing. No checksum in v1 (UDP has its
  own; log integrity can be added at the log layer).
