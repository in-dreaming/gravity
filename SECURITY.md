# Security policy

## Reporting a vulnerability

Report vulnerabilities privately to the repository owner using the hosting provider's private security-advisory channel. Do not open a public issue for an unpatched memory-safety, denial-of-service, snapshot/asset validation, or ABI pointer flaw. Include the affected commit, target, smallest reproducer, observed result, and expected invariant.

The project aims to acknowledge a report within three business days, confirm severity and scope within seven, and coordinate a fix and disclosure date with the reporter. A release advisory identifies affected versions, exploit preconditions, remediation, and the minimized regression corpus. There is no promise of support for unmodified forks, unsupported toolchains, or protocol-major compatibility.

## Supported surface

Security qualification covers the current `gravity_v1_*` ABI, canonical asset/snapshot/replay formats, the shipped tools, and the Gravity Spindle adapter at the pinned submodule commit. Spindle's other subsystems and downstream host application code are outside Gravity's boundary.
