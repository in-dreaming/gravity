# Deterministic Trigonometry Protocol 1

Task 02 freezes the previously unspecified CORDIC details. `TrigTable` has 1024
entries, indexed `i=0..1023`, with sample angle `i*tau/1024`, where `tau` is
Q32.32 raw `26986075410` and integer division truncates toward zero. Each entry
is `{ sin: i64 little-endian, cos: i64 little-endian }`, therefore 16,384 bytes
in ascending index order. The BLAKE3 input is ASCII domain tag
`gravity/trig-table/v1\0` followed by those bytes; callers must use the complete
32-byte digest (the first 16 bytes are the Hash128 form). The frozen complete
digest is `26b17852d03c28d8cafb5b0c6866f23310a927352be84fba9275b21affc76a15`.

Generation is entirely integer: angles and CORDIC state use signed Q32.224;
each sample is first reduced to `[-pi,pi]`, then reflected into
`[-pi/2,pi/2]` with both output signs flipped when needed, and receives exactly
96 CORDIC rotations. The 96 `atan(2^-i)` entries and gain compensation are
frozen nearest-even Q32.224 literals in `src/math/geometry.zig`, independently
generated from the real arctangent definition. Conversion to Q32.32 uses
round-to-nearest ties-to-even.
The cardinal golden vectors are index 0 `(0,4294967296)`, 256
`(4294967296,0)`, 512 `(0,-4294967296)`, and 768 `(-4294967296,0)` within the
documented final Q32.32 rounding bound (16 raw units).
