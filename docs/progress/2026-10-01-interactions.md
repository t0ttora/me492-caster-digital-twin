# Progress record - 1 October 2026 / Task interactions and PDF reading

The header uses a smaller owner mark and tighter spacing. Task rows now have explicit open/close labels, selected-record styling, one expanded task at a time, stable task links and a filter reset. Changing tasks keeps the selected summary visible below the header. Live status refresh keeps current-focus links inside the site.

Both published PDFs have dedicated reader pages within the site frame. A locally vendored Mozilla PDF.js build renders the original files, with page controls, zoom, a page-text alternative and original downloads. The browser-native embedded viewer was tested and displayed blank in the Codex browser, so it was replaced. PDF content is unchanged.

Seven Python checks and four Node checks passed locally. Browser checks covered task switching, filtering/reset, linked task opening, close-button focus, both PDF renders, page navigation, zoom and the 390 px mobile reader without page overflow.

Publication is tracked in the [Pages workflow history](https://github.com/t0ttora/me492-caster-digital-twin/actions/workflows/pages.yml). These are website changes, with no additional robot evidence or access approval. Actual hours were not recorded.
