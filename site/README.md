# Advisor dashboard

A small, accessible static website for the public ME492 project record. Its layout follows the ruler marks, thin rules and restrained type of the project PDFs, with the charcoal/ivory palette of [Oluş Emre Demir’s portfolio](https://www.olusemre.dev/en).

## Build and preview

Requirements: Python 3.10+; Node.js 20+ for JavaScript tests. No package installation, secret, database or frontend framework is needed.

```sh
python3 scripts/build_dashboard.py
python3 -m http.server 8000 --directory _site
```

Open `http://localhost:8000`. The generated `_site/` is ignored by Git and is the only directory uploaded to Pages. The builder copies only website assets and the two already-public report PDFs.

```sh
python3 -m unittest discover -s tests -v
node --test tests/*.test.mjs
node --check site/assets/app.mjs
```

CI also installs pinned Playwright browser tooling and runs `python3 tests/browser_check.py`, exercising desktop/mobile layouts, filters, search, gate details, API success/failure, safe text rendering and no-JavaScript fallbacks. Screenshots are saved as the `dashboard-browser-evidence` workflow artifact.

## Sources and update contract

- Weekly scope, hours and acceptance evidence come from `docs/planning/timetable.md`.
- Gate requirements come from `docs/planning/decision-gates.md`.
- Decisions and access entries come from their existing Markdown tables.
- The published run count comes from `docs/research/run-inventory.csv`.
- Offline issue labels come from the explicitly dated `site/issue-snapshot.json`.
- At page load or manual refresh, the browser requests public issue labels and recent main-branch commits from GitHub. No credentials are used. Network failures or rate limits retain the last displayed data and show an explicit warning. The issue query is paginated, with an explicit failure after 1,000 returned entries rather than silently presenting a partial list.
- The project narrative, current focus, academic dates and evidence commentary in `site/index.html` are a **dated editorial summary**. Update these and their snapshot labels when the research record changes. Live issue labels do not silently rewrite scientific conclusions or company permissions.
- Refresh the offline issue snapshot periodically, retaining its real check timestamp. Labels must be one of `status:planned`, `status:in-progress`, `status:blocked`, or `status:verified`. Unlabelled, conflicting and closed-without-evidence states stay distinct.
- Keep the root README about the enduring project. Log dated work under `docs/progress/`, link issues and artifacts, then update the dashboard summary when appropriate.

The website remains readable without JavaScript. Search, filters, the compact six-week view, and live GitHub refresh progressively enhance the static content. A print stylesheet expands all work packages. Issue closure is never treated as proof of research validity or gate approval.

## GitHub Pages

The `Project dashboard` workflow tests and builds every push to main, then deploys through the official GitHub Pages actions. Pull requests build only.

Initial repository setting: **Settings → Pages → Build and deployment → Source → GitHub Actions**. This one-time setting must be enabled by an authorized repository administrator. The workflow deliberately does not request or create a personal access token to enable it. If Pages is disabled, the build can pass while the configure/deploy job fails; enable the setting and rerun the workflow.

The deployment job’s `github-pages` environment exposes the actual published URL. Confirm that URL and the deployed commit before announcing a live release.

## Layout and files

- `index.html`: semantic page template and dated editorial summary
- `assets/styles.css`: responsive design, keyboard focus, reduced-motion and print styles
- `assets/app.mjs`: progressive UI and public GitHub refresh
- `assets/status.mjs`: tested status, filtering and input-validation rules
- `assets/olus-emre-logo.svg`: existing owner mark from the public portfolio
- `issue-snapshot.json`: dated public issue-label fallback
- `../scripts/build_dashboard.py`: dependency-free static generator
- `../tests/`: source consistency, static link, accessibility and presentation-rule tests

The dashboard introduces no robot simulation implementation and makes no measured-performance claims.
