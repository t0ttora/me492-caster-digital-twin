"""Build the public research record. Python stdlib only; GitHub is the task source."""
from datetime import datetime, timezone
from html import escape, unescape
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import urlsplit, quote
from urllib.request import Request, urlopen
from zoneinfo import ZoneInfo
import argparse
import json
import os
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '_site'
REPO = 't0ttora/me492-caster-digital-twin'
GH = f'https://github.com/{REPO}'
STATUS = {'planned': 'Planlanan', 'in-progress': 'Devam ediyor', 'blocked': 'Engelli', 'verified': 'Doğrulandı', 'closed': 'Kapalı · doğrulanmadı', 'unclassified': 'Durum belirtilmedi'}


def api_all(endpoint):
    items = []
    for page in range(1, 1001):
        separator = '&' if '?' in endpoint else '?'
        request = Request(f'https://api.github.com/repos/{REPO}/{endpoint}{separator}per_page=100&page={page}', headers={'Accept': 'application/vnd.github+json', 'User-Agent': 'Arche-Research-Record'})
        token = os.environ.get('GH_TOKEN') or os.environ.get('GITHUB_TOKEN')
        if token:
            request.add_header('Authorization', f'Bearer {token}')
        with urlopen(request, timeout=45) as response:
            batch = json.load(response)
        items.extend(batch)
        if len(batch) < 100:
            return items
    raise RuntimeError('GitHub pagination exceeded expected limit')


def document_url(path):
    return f'records/{path.with_suffix(".html").as_posix()}'


def safe_link(url, source=None):
    parts = urlsplit(url)
    if parts.scheme:
        return url if parts.scheme in ('https', 'http', 'mailto') else '#'
    if parts.netloc:
        return '#'
    if url.startswith('#'):
        return url
    if source:
        target = (ROOT / source.parent / parts.path).resolve()
        if target.is_relative_to(ROOT) and target.exists():
            relative = target.relative_to(ROOT)
            mapped = document_url(relative) if relative.suffix == '.md' else relative.as_posix()
            return mapped + (f'#{parts.fragment}' if parts.fragment else '')
        return f'{GH}/blob/main/{quote(source.parent.as_posix() + "/" + url)}'
    return f'{GH}/{url.lstrip("/")}'


def inline(text, source=None):
    # Escape first: issue text and comments never become executable HTML.
    text = escape(text.replace('\\|', '|'))
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', lambda m: f'<a href="{escape(safe_link(unescape(m[2]), source), quote=True)}">{m[1]}</a>', text)
    text = re.sub(r'\*\*(.+?)\*\*', r'<strong>\1</strong>', text)
    text = re.sub(r'`([^`]+)`', r'<code>\1</code>', text)
    return text


def markdown(text, source=None):
    # ponytail: repository prose subset, use a maintained Markdown parser if nested lists or complex embeds become necessary.
    result, paragraph, bullets, table = [], [], [], []
    code = None
    def flush():
        if paragraph:
            result.append('<p>' + inline(' '.join(paragraph), source) + '</p>')
            paragraph.clear()
        if bullets:
            result.append('<ul>' + ''.join(f'<li>{inline(item, source)}</li>' for item in bullets) + '</ul>')
            bullets.clear()
        if table:
            rows = [row for row in table if not all(re.fullmatch(r':?-+:?', cell.strip()) for cell in row)]
            result.append('<div class="table-scroll"><table>' + ''.join('<tr>' + ''.join(f'<{"th" if i == 0 else "td"}>{inline(cell, source)}</{"th" if i == 0 else "td"}>' for cell in row) + '</tr>' for i, row in enumerate(rows)) + '</table></div>')
            table.clear()
    for line in text.splitlines():
        if line.startswith('```'):
            flush()
            if code is None:
                code = []
            else:
                result.append('<pre><code>' + escape('\n'.join(code)) + '</code></pre>')
                code = None
            continue
        if code is not None:
            code.append(line)
            continue
        if line.startswith('|'):
            if paragraph or bullets:
                flush()
            table.append([cell.strip() for cell in re.split(r'(?<!\\)\|', line.strip('|'))])
        elif line.startswith(('- ', '* ')) or re.match(r'^\d+\. ', line):
            if paragraph or table:
                flush()
            item = re.sub(r'^(?:[-*]|\d+\.) ', '', line)
            item = re.sub(r'^\[([ xX])\]', lambda m: '☑' if m[1].lower() == 'x' else '☐', item)
            bullets.append(item)
        elif match := re.match(r'^(#{1,6}) (.+)$', line):
            flush()
            level = len(match[1])
            result.append(f'<h{level}>{inline(match[2], source)}</h{level}>')
        elif not line.strip():
            flush()
        else:
            if bullets or table:
                flush()
            paragraph.append(line)
    flush()
    if code is not None:
        result.append('<pre><code>' + escape('\n'.join(code)) + '</code></pre>')
    return ''.join(result)


def state(issue):
    labels = [label['name'] for label in issue['labels']]
    if issue['state'].lower() == 'closed':
        return 'verified' if 'status:verified' in labels else 'closed'
    for status in ('blocked', 'in-progress', 'planned'):
        if f'status:{status}' in labels:
            return status
    return 'unclassified'


def badge(status):
    return f'<span class="badge {status}">{STATUS[status]}</span>'


def issue_url(issue):
    return issue.get('html_url') or issue['url']


def short_date(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00')).astimezone(ZoneInfo('Europe/Istanbul')).strftime('%d.%m.%Y · %H:%M')


def check():
    assert 'javascript:' not in inline('[bad](javascript:alert)')
    assert '&lt;script&gt;' in markdown('<script>alert(1)</script>')
    assert '<table>' in markdown('| A | B |\n| --- | --- |\n| a | b |')
    assert '☑' in markdown('- [x] Done')
    assert '<pre><code>&lt;script&gt;</code></pre>' in markdown('```sh\n<script>\n```')
    assert '<th>A | B</th>' in markdown('| A \\| B | C |\n| --- | --- |')
    assert state({'state': 'closed', 'labels': []}) == 'closed'
    assert state({'state': 'open', 'labels': [{'name': 'status:verified'}]}) == 'unclassified'
    assert state({'state': 'closed', 'labels': [{'name': 'status:verified'}]}) == 'verified'
    assert safe_link('../../reports/ME492_Timetable_EN.pdf', Path('docs/progress/2026-09-30.md')) == 'reports/ME492_Timetable_EN.pdf'
    print('PASS: escaping, tables, checklists, evidence status and document links')


def build(issues, comments):
    if OUT.exists():
        shutil.rmtree(OUT)
    shutil.copytree(ROOT / 'site', OUT)
    shutil.copytree(ROOT / 'reports', OUT / 'reports', ignore=shutil.ignore_patterns('*.md'))
    documents = [Path('CONTRIBUTING.md'), *sorted(p.relative_to(ROOT) for p in (ROOT / 'docs').rglob('*.md')), Path('reports/README.md')]
    doc_links = []
    for path in documents:
        text = (ROOT / path).read_text()
        title = text.splitlines()[0].lstrip('# ')
        dest = OUT / document_url(path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        relative_root = os.path.relpath(OUT, dest.parent).replace(os.sep, '/') + '/'
        page = f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><base href="{relative_root}"><title>{escape(title)} · Arche</title><link rel="icon" href="assets/olus-emre-logo.svg"><link rel="stylesheet" href="style.css"></head><body><div class="frame"><header class="record-header"><a href="index.html#library">← Arche / Kütüphane</a><a href="{GH}/blob/main/{path.as_posix()}">GitHub’da aslı ↗</a></header><main class="record"><p class="record-meta">PUBLIC RESEARCH RECORD / {escape(path.as_posix())}</p><article class="prose">{markdown(text, path)}</article></main></div></body></html>'
        dest.write_text(page)
        doc_links.append(f'<a href="{document_url(path)}"><span>{escape(title)}</span><small>{escape(path.parent.name.upper())} ↗</small></a>')
    shutil.copyfile(ROOT / 'docs/research/run-inventory.csv', OUT / 'records/run-inventory.csv')
    doc_links.append('<a href="records/run-inventory.csv"><span>Run inventory</span><small>CSV ↗</small></a>')
    issues = sorted((i for i in issues if 'pull_request' not in i), key=lambda i: i['number'])
    tasks, journal = [], []
    weekly = [i for i in issues if any(l['name'] == 'type:weekly' for l in i['labels'])]
    for issue in issues:
        status = state(issue)
        kind = 'gate' if any(l['name'] == 'type:gate' for l in issue['labels']) else 'weekly' if issue in weekly else 'task'
        code = issue['title'].split('|')[0].strip()
        if not re.fullmatch(r'[WG]\d+', code):
            code = f'#{issue["number"]}'
        title = issue['title'].split('|')[-1].strip()
        history = []
        for comment in comments:
            if comment['issue_url'].endswith(f'/{issue["number"]}'):
                history.append(f'<div class="issue-comment"><p>{escape(comment["user"]["login"])} · {short_date(comment["created_at"])} · <a href="{escape(comment["html_url"])}">Kaynak ↗</a></p><div class="prose">{markdown(comment.get("body") or "")}</div></div>')
        tasks.append(f'<details class="task" data-status="{status}" data-kind="{kind}" id="task-{issue["number"]}"><summary><span class="task-code">{escape(code)}</span><span>{escape(title)}</span>{badge(status)}<span class="task-arrow" aria-hidden="true">+</span></summary><div class="task-body"><div class="prose">{markdown(issue.get("body") or "")}</div>{"".join(history)}<a class="text-link" href="{escape(issue_url(issue))}">GitHub’da görev ve yorumlar ↗</a></div></details>')
    for path in sorted((ROOT / 'docs/progress').glob('????-??-??*.md'), reverse=True):
        source = path.relative_to(ROOT)
        title = path.read_text().splitlines()[0].lstrip('# ')
        journal.append((path.name[:10], f'<article class="journal-entry"><time datetime="{path.name[:10]}">{path.name[:10]}</time><h3>{escape(title)}</h3><p>Repo ilerleme kaydı · hazırlık, uygulama ve doğrulama sınırları belgenin içinde.</p><details><summary>Kaydı oku +</summary><div class="prose">{markdown(path.read_text(), source)}</div></details><a class="text-link" href="{document_url(source)}">Tam kayıt ↗</a></article>'))
    for comment in comments:
        number = int(comment['issue_url'].rsplit('/', 1)[1])
        journal.append((comment['created_at'], f'<article class="journal-entry"><time datetime="{comment["created_at"]}">{short_date(comment["created_at"])}</time><h3>Görev #{number} / güncelleme</h3><p>{escape(comment["user"]["login"])} tarafından GitHub’da kaydedildi.</p><details><summary>Güncellemeyi oku +</summary><div class="prose">{markdown(comment.get("body") or "")}</div></details><a class="text-link" href="{escape(comment["html_url"])}">Kaynak yorum ↗</a></article>'))
    journal.sort(key=lambda item: item[0], reverse=True)
    roadmap = []
    for line in (ROOT / 'docs/planning/timetable.md').read_text().splitlines():
        if not re.match(r'^\| W\d+', line):
            continue
        week, period, hours, output, evidence = [cell.strip() for cell in line.strip('|').split('|')]
        task_number = int(week[1:])
        roadmap.append(f'<details><summary><span><b>{week}</b>{escape(period)}</span><strong>{escape(output)}</strong><span>{hours} saat +</span></summary><div><p>{escape(evidence)}</p><a href="#task-{task_number}" class="roadmap-task" data-task="task-{task_number}">İlgili görevi aç ↗</a></div></details>')
    counts = {s: sum(state(i) == s for i in weekly) for s in STATUS}
    metrics = ''.join(f'<div class="metric"><b>{count:02d}</b><span>{label}</span></div>' for count,label in [(counts['in-progress'],'Devam eden haftalık görev'),(counts['verified'],'Doğrulanan haftalık görev'),(counts['blocked'],'Engelli haftalık görev'),(len(weekly),'Toplam haftalık görev')])
    active = [i for i in weekly if state(i) == 'in-progress']
    focus = ''.join(f'<h3>{escape(i["title"].split("|")[-1].strip())}</h3>{badge("in-progress")}<p><a href="#task-{i["number"]}">Görev #{i["number"]} ↗</a></p>' for i in active) or '<h3>Aktif görev işaretlenmedi.</h3><p>Güncel durumu görevler bölümünden inceleyin.</p>'
    built = datetime.now(timezone.utc)
    tokens = {'METRICS': metrics, 'FOCUS': focus, 'PROGRESS': ''.join(item[1] for item in journal), 'SYNC': f'GitHub ve repo kaydı · Son yayın: {short_date(built.isoformat())} (İstanbul).', 'TASKS': ''.join(tasks), 'ROADMAP': ''.join(roadmap), 'DOCUMENTS': ''.join(doc_links)}
    page = (ROOT / 'site/index.html').read_text()
    for key, value in tokens.items():
        page = page.replace('{{' + key + '}}', value)
    assert '{{' not in page
    (OUT / 'index.html').write_text(page)
    (OUT / '.nojekyll').touch()
    (OUT / 'snapshot.json').write_text(json.dumps({'published_at': built.isoformat(), 'source': GH, 'issues': issues, 'comments': comments}, ensure_ascii=False, indent=2))
    validate_links()
    print(f'Built {len(tasks)} tasks, {len(roadmap)} weeks, {len(documents)} documents and {len(journal)} journal entries.')


def validate_links():
    class Links(HTMLParser):
        def __init__(self):
            super().__init__()
            self.links = []
            self.base = None
        def handle_starttag(self, tag, attrs):
            attrs = dict(attrs)
            if tag == 'base': self.base = attrs.get('href')
            for attr in ('href', 'src'):
                if tag != 'base' and attrs.get(attr): self.links.append(attrs[attr])
    for path in OUT.rglob('*.html'):
        parser = Links()
        parser.feed(path.read_text())
        base = (path.parent / parser.base).resolve() if parser.base else path.parent
        for link in parser.links:
            parts = urlsplit(link)
            if parts.scheme or parts.netloc or not parts.path: continue
            assert (base / parts.path).exists(), f'Broken link: {path}: {link}'
    print('PASS: all generated local HTML links and assets resolve')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--issues-file', type=Path, help='Saved gh issue list JSON for an offline preview')
    args = parser.parse_args()
    check()
    if not args.check:
        issues = json.loads(args.issues_file.read_text()) if args.issues_file else api_all('issues?state=all')
        comments = [] if args.issues_file else api_all('issues/comments?sort=created&direction=asc')
        build(issues, comments)
