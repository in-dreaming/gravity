# Q32.32 expression-level range analysis

`Fp` stores signed Q32.32. Arithmetic creates no floating-point values.
Addition and subtraction detect `i64` overflow; multiplication, division, and
the final Q64.64 narrow use `i128` quotient/remainder round-to-nearest,
ties-to-even. `MathStatus` is caller-owned and retains only the first fault.

`WideScalar` is Q64.64. Dot products construct products and accumulate them in
this representation, then call the single explicit `narrow` operation. This
prevents per-term rounding from changing dot, cross, matrix, or inertia
expressions.

Before state allocation or mutation, World initialization must call
`ProductEnvelope.validate`. It checks with `i128` overflow detection the
expression families position/distance squared (`3p^2`), cross (`s^2`), inertia
(`m*s^2`), angular impulse (`m*v*s`), and GJK/EPA determinant terms (`p^3`).
A configuration that exceeds an intermediate is rejected rather than narrowed.
The documented default envelope is validated by the same function.
