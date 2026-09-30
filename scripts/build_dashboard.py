#!/usr/bin/env python3
"""Build the advisor dashboard from the repository record; Python stdlib only."""
import csv
import html
import json
import re
import shutil
import subprocess
from pathlib import Path
from string import Template

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '_site'
REPO = 'https://github.com/t0ttora/me492-caster-digital-twin'

def esc(value):
    return html.escape(str(value), quote=True)

def table(path):
    """Read the first Markdown table, including escaped pipes in issue titles."""
    rows = []
    for line in (ROOT / path).read_text().splitlines():
        if not line.startswith('|'): continue
        cells = [s.strip().replace(r'\|', '|') for s in re.split(r'(?<!\\)\|', line.strip().strip('|'))]
        if all(re.fullmatch(r':?-+:?', cell) for cell in cells): continue
        rows.append(cells)
    if not rows: raise ValueError(f'No Markdown table in {path}')
    return [dict(zip(rows[0], row, strict=True)) for row in rows[1:]]

def issue_status(issue):
    labels = [label if isinstance(label, str) else label.get('name', '') for label in issue.get('labels', [])]
    states = {label[7:] for label in labels if label in {'status:planned', 'status:in-progress', 'status:blocked', 'status:verified'}}
    if len(states) > 1: return 'conflicting'
    return next(iter(states)) if states else ('closed' if issue.get('state') == 'closed' else 'unlabelled')

def status(label):
    return f'<span class="status {esc(label)}"><i aria-hidden="true"></i>{esc(label.replace("-", " ").capitalize())}</span>'

def build():
    snapshot = json.loads((ROOT / 'site/issue-snapshot.json').read_text())
    issues = {issue['number']: issue for issue in snapshot['issues']}
    weeks = [{'id':r['Week'], 'period':r['Period'], 'hours':int(r['Hours']), 'output':r['Output'], 'evidence':r['Acceptance evidence']} for r in table('docs/planning/timetable.md')]
    gates = [{'id':r['Gate'].split()[0], 'date':' '.join(r['Gate'].split()[1:]), 'evidence':r['Evidence and decision']} for r in table('docs/planning/decision-gates.md')]
    access = table('docs/access/access-register.md')
    decisions = table('docs/planning/decisions.md')
    with (ROOT / 'docs/research/run-inventory.csv').open() as source:
        runs = list(csv.DictReader(source))
    assert len(weeks) == 16 and len(gates) == 6, 'Baseline shape changed; review dashboard layout'
    assert sum(w['hours'] for w in weeks) == 252, 'Update dashboard capacity when the baseline changes'
    week_html = []
    for n, week in enumerate(weeks, 1):
        state = issue_status(issues.get(n, {}))
        week['status'] = state
        week_html.append(f'''<article class="week-row" data-week="{week['id']}" data-issue="{n}" data-status="{esc(state)}">
 <span class="week-number">{week['id']}</span><div class="week-content"><a class="week-title" href="{REPO}/issues/{n}">{esc(week['output'])}<span class="out-arrow" aria-hidden="true">↗</span></a><span class="week-date">{esc(week['period'])} <span>· {week['hours']} h planned</span></span><details><summary>Acceptance evidence</summary><p>{esc(week['evidence'])}</p></details></div><div class="week-status">{status(state)}</div></article>''')
    gate_html = []
    for n, gate in enumerate(gates, 17):
        gate_html.append(f'''<details class="gate" data-issue="{n}"><summary><span class="mono">{gate['id']}</span><span class="gate-date">{esc(gate['date'])}</span><span class="gate-label">{esc(issues[n]['title'].split('|',1)[-1].strip())}</span><span class="gate-state">{status(issue_status(issues[n]))}</span></summary><div class="gate-body"><p>{esc(gate['evidence'])}</p><a href="{REPO}/issues/{n}">Review gate evidence <span aria-hidden="true">↗</span></a></div></details>''')
    access_html = ''.join(f'<tr><th scope="row">{esc(r["Item"])}</th><td>{esc(r["State"])}</td><td>{esc(r["Evidence / owner"])}</td><td>{esc(r["Target"])}</td></tr>' for r in access)
    decision_html = ''.join(f'<article class="decision"><span class="mono">{esc(r["ID"])}</span><div><h3>{esc(r["Decision / rationale"])}</h3><p>{esc(r["Evidence and authority"])}</p><span class="small">{esc(r["Date"])}</span></div><span class="decision-state">{esc(r["State"])}</span></article>' for r in decisions)
    try:
        commit = subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip()
        log = subprocess.check_output(['git','log','-5','--format=%H%x09%cs%x09%s'], cwd=ROOT, text=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        commit = 'main'; log = ''
    commits = []
    for line in log.splitlines():
        sha, date, title = line.split('\t',2)
        commits.append(f'<li><a href="{REPO}/commit/{esc(sha)}">{esc(title)}</a><span class="mono">{esc(date)} · {sha[:7]}</span></li>')
    data = {'snapshot_date':'2026-09-30', 'issue_checked_at':snapshot['checked_at'], 'source_commit':commit, 'repository':REPO, 'weeks':weeks, 'gates':gates, 'access':access, 'decisions':decisions, 'published_runs':len(runs)}
    rendered = Template((ROOT / 'site/index.html').read_text()).substitute(
        repo=REPO, weeks='\n'.join(week_html), gates='\n'.join(gate_html), access=access_html,
        decisions=decision_html, commits=''.join(commits), commit=commit, short_commit=commit[:7], published_runs=len(runs))
    if OUT.is_symlink(): raise ValueError('Refusing a symlinked build output')
    if OUT.exists(): shutil.rmtree(OUT)
    OUT.mkdir()
    (OUT / 'index.html').write_text(rendered)
    (OUT / 'project.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    (OUT / '.nojekyll').write_text('')
    shutil.copytree(ROOT / 'site/assets', OUT / 'assets', dirs_exist_ok=True)
    (OUT / 'reports').mkdir(exist_ok=True)
    for name in ['ME492_Final_Roadmap_and_Arche_Visit_EN.pdf', 'ME492_Timetable_EN.pdf']:
        shutil.copy2(ROOT / 'reports' / name, OUT / 'reports' / name)
    print(f'Built {OUT.name}: {len(weeks)} work packages, {len(gates)} gates, {len(runs)} published run records')

if __name__ == '__main__': build()
