# Open research journal

The portfolio-aligned public ME492 record: progress, vision, manifesto, tasks, roadmap, decisions, access, documents and owner profile. The actual Neue Montreal font, owner mark, portrait, palette and ruler/grid language come from olusemre.dev.

## Build and maintain

See [CONTRIBUTING.md](../CONTRIBUTING.md) for the source and publication contract.

```sh
python3 scripts/build_site.py
python3 -m http.server 8765 --directory _site
```

The `Publish research journal` workflow rebuilds on main pushes, issue changes, issue comments and milestone updates. Pages is configured for GitHub Actions. The workflow URL is the deployment evidence; the build timestamp is not research progress.

All 22 initial tasks and their public comments are rendered at build time. Status filters, search, task details, roadmap-to-task links and a manual public GitHub status refresh enhance that static record. API errors retain the published snapshot and show a warning. All tasks, reports and records remain readable without JavaScript.

- `index.html`: editorial narrative and semantic page template
- `style.css`: responsive portfolio visual system, keyboard focus, reduced motion and print
- `app.js`: filters, navigation, roadmap links and GitHub status refresh
- `assets/status.mjs`: status classification and API input validation
- `assets/`: existing portfolio font, owner mark and portrait
- `issue-snapshot.json`: timestamped public issue fallback for offline checks
- `../scripts/build_site.py`: stdlib generator, escaping and generated-link checks
- `../scripts/build_dashboard.py`: compatibility command for the earlier dashboard builder

Scientific claims and permissions remain in the research records; refreshing a task label cannot approve an experiment or a decision gate.

```sh
python3 -m unittest discover -s tests -v
node --test tests/*.test.mjs
node --input-type=module --check < site/app.js
```

Desktop/mobile screenshots and UI checks are performed in the browser. The optional `tests/browser_check.py` provides CI regression checks when Playwright is available. Its test fixtures are not project results. No runtime framework, CMS, database or frontend package installation is required.
