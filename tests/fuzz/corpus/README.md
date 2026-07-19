# Task 25 minimized regression corpus

Each file is the smallest input that preserves its original failure class. New crashes, timeouts, or invariant failures must be minimized with `zig build fuzz-minimize -- <kind> <input> <output>` and committed here with a regression assertion.

- `asset-leading-decimal.json`: source decimal grammar previously disagreed with `Fp.parseCanonicalDecimal` by accepting a missing integer component.
