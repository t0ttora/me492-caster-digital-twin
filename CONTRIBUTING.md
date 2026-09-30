# Maintaining the project record

1. Monday: choose the active weekly issue and set exactly one status label. Status values are planned, in-progress, blocked and verified.
2. During work: add dated comments with actual hours, evidence, limitations and blockers. Use blocked only for a dependency that prevents the current output.
3. Friday: complete the weekly progress fields; commit public figures/reports and a dated progress record where useful. Update the [progress index](docs/progress/README.md); keep the root README focused on the project overview.
4. Mark verified and close only after the acceptance evidence is linked and checked. Gate issues additionally require the explicit reviewed decision and rationale. Milestone percentages measure issue closure only.
5. For a scope/access change, use the decision template and update the [decision log](docs/planning/decisions.md) and [access register](docs/access/access-register.md). Keep official deadlines; record any reduced validation claim.

Owner: Oluş Emre Demir. The advisor can read all public pages/issues without an account; comments or edits require a GitHub account. Nothing is automatically sent to the advisor. Company CAD, raw telemetry, weights and restricted reports require explicit publication permission. Local ignore patterns are a convenience, not an access-control mechanism.

## Weekly progress fields

Period; planned output; actual work; prepared/executed/verified state; evidence link/run ID; actual hours versus plan; blocker; next output; decision needed.

## Dashboard updates

The advisor dashboard reads the existing planning, decision and access tables at build time. Keep their headings intact or update the generator and tests together. Update the dated narrative in `site/index.html` when findings or dependencies change; a live issue-label refresh does not update research conclusions. See [the dashboard maintenance guide](site/README.md) for preview, checks, snapshot refresh and publication.
