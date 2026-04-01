# Fitting Status

Fitting is unstable on this branch. This page records only current status.

## Current State

The current branch contains multiple fitting approaches, including:

- GenFit-based fitting through `AtFitterTask` and `AtFITTER::AtGenfit`
- Monte-Carlo-based fitting through `AtMCFitterTask`
- Bragg-curve-related fitting logic in other parts of the codebase

No stable long-term fitting architecture is documented here.

## Guidance

- Treat the current branch code as the source of truth.
- Do not assume the current fitters represent the intended long-term design.
- Inspect source directly before editing fitter code.
