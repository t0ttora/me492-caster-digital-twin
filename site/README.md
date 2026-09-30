# Open research journal

The portfolio-aligned public ME492 record: progress, vision, manifesto, tasks, roadmap, decisions, access, documents and owner profile. The actual Neue Montreal font, owner mark, palette and ruler/grid language come from olusemre.dev.

## Build and maintain

See [CONTRIBUTING.md](../CONTRIBUTING.md) for the source and publication contract.

```sh
python3 scripts/build_site.py
python3 -m http.server 8765 --directory _site
```

The `Publish research journal` workflow rebuilds on main pushes, issue changes, issue comments and milestone updates. Pages is configured for GitHub Actions. The workflow URL is the deployment evidence; the build timestamp is not research progress.

All 22 initial tasks and their public comments are rendered at build time. Status filters, search, task details, roadmap-to-task links and a manual public GitHub status refresh enhance that static record. API errors retain the published snapshot and show a warning. All tasks, reports and records remain readable without JavaScript.

- `layout.html`: shared frame, navigation and page metadata
- `index.html`: short project overview and status table
- `progress.html`: dated journal and repository history
- `tasks.html`: filterable issue workbench
- `roadmap.html`: semester table and deadlines
- `vision.html`: research rationale and question/evidence matrix
- `manifesto.html`: separate editorial principles page
- `decisions.html`: decision and access ledgers
- `library.html`: report and document catalog
- `about.html`: text profile and professional facts
- `style.css`: responsive portfolio visual system, keyboard focus, reduced motion and print
- `app.js`: filters, navigation, roadmap links and GitHub status refresh
- `assets/status.mjs`: status classification and API input validation
- `assets/`: existing portfolio font and owner mark
- `issue-snapshot.json`: timestamped public issue fallback for offline checks
- `../scripts/build_site.py`: stdlib generator, escaping and generated-link checks
- `../scripts/build_dashboard.py`: compatibility command for the earlier dashboard builder

Scientific claims and permissions remain in the research records; refreshing a task label cannot approve an experiment or a decision gate.

```sh
python3 -m unittest discover -s tests -v
node --test tests/*.test.mjs
node --input-type=module --check < site/app.js
```

Desktop/mobile screenshots and UI checks are performed in the browser. The optional `tests/browser_check.py` provides CI regression checks when Playwright is available. Its test fixtures are not project results. No runtime framework, CMS or database is required. The PDF reader uses a pinned, locally vendored Mozilla PDF.js build; no CDN or third-party document upload is involved.

Each primary tab has a real HTML URL and a distinct content layout. Legacy homepage section hashes redirect to their matching pages. No generated illustrations, robot diagrams or synthetic data plots are included.

## Editorial voice

All interface copy is English. Use the reports’ technical terminology and plain, descriptive headings. Introductions can use first person and an occasional dry aside; evidence, permissions, deadlines and status messages stay precise. Keep source documents, issue text and reports faithful to their authors. Distinguish planned, executed and verified work, and avoid claims that the records cannot support.

Tasks show planned periods/budgets in the list, independent type/status filters with counts, search and reset. A native dialog presents one task at a time with previous/next navigation and focus restoration; without JavaScript, inline details remain readable. Report links resolve to dedicated site pages with PDF continuous scrolling, page navigation, fit-width/fit-page sizing, selectable text, zoom and a focus mode. Original downloads remain available. PDF.js provenance and licenses are in `assets/pdfjs/`; update the engine and worker together when upgrading.

The reader fills the available viewport with one document scroll area. Pages render near the viewport, and old canvas buffers are released on scale changes. The source PDFs remain unchanged.

Overview progress uses equally weighted weekly tasks and a separate decision-gate denominator. Completion requires a closed issue with an unambiguous `status:verified` label. Active work gets no partial completion credit; percentages refresh with the public GitHub status request. The scope and evidence panels reflect the published research records.
