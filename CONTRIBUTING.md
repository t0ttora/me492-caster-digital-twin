# Maintaining the project record

1. Monday: choose the active weekly issue and set exactly one status label. Status values are planned, in-progress, blocked and verified.
2. During work: add dated comments with actual hours, evidence, limitations and blockers. Use blocked only for a dependency that prevents the current output.
3. Friday: complete the weekly progress fields; commit public figures/reports and a dated progress record where useful. Update the [progress index](docs/progress/README.md); keep the root README focused on the project overview.
4. Mark verified and close only after the acceptance evidence is linked and checked. Gate issues additionally require the explicit reviewed decision and rationale. Milestone percentages measure issue closure only.
5. For a scope/access change, use the decision template and update the [decision log](docs/planning/decisions.md) and [access register](docs/access/access-register.md). Keep official deadlines; record any reduced validation claim.

Owner: Oluş Emre Demir. The advisor can read all public pages/issues without an account; comments or edits require a GitHub account. Nothing is automatically sent to the advisor. Company CAD, raw telemetry, weights and restricted reports require explicit publication permission. Local ignore patterns are a convenience, not an access-control mechanism.

## Weekly progress fields

Period; planned output; actual work; prepared/executed/verified state; evidence link/run ID; actual hours versus plan; blocker; next output; decision needed.

## Updating the research website

The [public research journal](https://t0ttora.github.io/me492-caster-digital-twin/) is built from these records, not maintained as a second task list.

- **Tasks:** edit GitHub issues and use one status label. Only closed issues with `status:verified` count as verified. Closed issues without that label remain explicitly unverified; an open issue cannot count as verified.
- **Progress:** add a dated Markdown file in `docs/progress/` and update its index, or add a dated evidence comment to the relevant issue. Public issue comments appear on the website, with author, timestamp and source link. Do not place restricted data in public comments.
- **Roadmap:** update `docs/planning/timetable.md`; keep the existing table columns and W01–W16 identifiers aligned with the weekly issues.
- **Decisions and access:** maintain the decision log and access register, with evidence, owners and actual approval state.
- **Page layout and navigation:** edit `site/layout.html`; page-specific content lives in the corresponding `site/*.html` template.
- **Reports:** add permitted PDF files to `reports/` and update `reports/README.md`. The homepage highlights the two initial baseline reports; update `site/library.html` if those highlights change.
- **Vision, manifesto and profile:** edit `site/vision.html`, `site/manifesto.html` and `site/about.html`. Present research aims as aims; add achieved results only with linked evidence.

Pushes to main, issue edits/status changes, public issue comments and milestone changes trigger `.github/workflows/pages.yml`. The published timestamp describes the build, not the date of a new experiment. Run the workflow manually from GitHub Actions if a refresh is needed. Publication takes effect after a successful workflow run.

Local preview (Python 3.9+):

```sh
python3 scripts/build_site.py
python3 -m http.server 8765 --directory _site
```

The build fetches public GitHub issues and comments, renders all project Markdown documents, and copies permitted reports and brand assets. `GH_TOKEN` can authenticate API requests; it is never copied into the output. For an offline preview, save `gh issue list --state all --limit 100 --json number,title,state,labels,body,url` to a temporary file and pass `--issues-file /path/to/issues.json` (comments are omitted in that preview).

Run `python3 scripts/build_site.py --check` for the escaping, evidence-status and Markdown checks. Every full build also checks generated local links. `_site/` is generated and ignored. No frontend dependency installation or separate CMS is required.
