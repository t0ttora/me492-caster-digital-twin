"""Build the public research record. Python stdlib only; GitHub is the task source."""
from datetime import date, datetime, timedelta, timezone
from html import escape, unescape
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import urlsplit, quote
from urllib.request import Request, urlopen
from zoneinfo import ZoneInfo
from zipfile import ZIP_DEFLATED, ZipFile
import ast
import csv
import subprocess
import argparse
import json
import os
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '_site'
REPO = 't0ttora/me492-caster-digital-twin'
GH = f'https://github.com/{REPO}'
PAGES = [('index', 'Overview'), ('results', 'Results'), ('progress', 'Progress'), ('vision', 'Vision'), ('python', 'Python'), ('simscape', 'Native Simscape'), ('ros-gazebo', 'ROS 2 / Gazebo'), ('tasks', 'Tasks'), ('roadmap', 'Roadmap'), ('decisions', 'Decisions'), ('library', 'Library'), ('about', 'About')]
REPORTS = [('ME492_Final_Roadmap_and_Arche_Visit_EN', 'Execution baseline & Arche visit'), ('ME492_Timetable_EN', 'Semester timetable'),
           ('Arche_Academic_Report_Typeset_EN_2026-11-16', 'Academic simulation progress report'),
           ('Arche_V2_Controller_Design_EN', 'Python V2 controller calculations'),
           ('Arche_JANUS_Teknik_Rapor_TR', 'JANUS motion and docking report (Turkish)'),
           ('Arche_JANUS_Vaka_Kartlari_TR', 'JANUS illustrated case cards (Turkish)')]
STATUS = {'planned': 'Planned', 'in-progress': 'In progress', 'blocked': 'Blocked', 'verified': 'Verified', 'closed': 'Closed · unverified', 'unclassified': 'Unclassified', 'conflicting': 'Conflicting labels'}
ENVIRONMENTS = {'Python': 'Python', 'Simscape': 'Native Simscape', 'Gazebo Linux': 'Live Gazebo', 'Gazebo kamerası / video': 'Gazebo camera replay', 'ROS 2 Linux': 'Live ROS 2', 'ROS 2 + Gazebo Linux': 'Live ROS 2 / Gazebo'}


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
        for name, _ in REPORTS:
            if url.split('#')[0] == f'{GH}/blob/main/reports/{name}.pdf':
                return f'report-{name}.html'
        return url if parts.scheme in ('https', 'http', 'mailto') else '#'
    if parts.netloc:
        return '#'
    if url.startswith('#'):
        return url
    if source:
        target = (ROOT / source.parent / parts.path).resolve()
        if target.is_relative_to(ROOT) and target.exists():
            relative = target.relative_to(ROOT)
            mapped = document_url(relative) if relative.suffix == '.md' else f'report-{relative.stem}.html' if relative.parent == Path('reports') and relative.stem in {name for name, _ in REPORTS} else relative.as_posix()
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


def markdown(text, source=None, embedded=False):
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
            result.append('<div class="table-scroll" role="region" aria-label="Document table" tabindex="0"><table>' + ''.join('<tr>' + ''.join(f'<{"th" if i == 0 else "td"}>{inline(cell, source)}</{"th" if i == 0 else "td"}>' for cell in row) + '</tr>' for i, row in enumerate(rows)) + '</table></div>')
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
            level = min(6, len(match[1]) + (2 if embedded else 0))
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
    labels = [label if isinstance(label, str) else label['name'] for label in issue['labels']]
    states = {label for label in labels if label in {'status:planned', 'status:in-progress', 'status:blocked', 'status:verified'}}
    if len(states) > 1:
        return 'conflicting'
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


def evidence_review(issue):
    reviews = re.findall(r'^\*\*(\d{1,2} [A-Za-z]+ \d{4}) · Evidence review\.\*\* +([^\r\n]+)', issue.get('body') or '', re.M)
    if not reviews:
        return ''
    date, summary = reviews[-1]
    return f'<span class="review-date">{escape(date)}</span>{inline(summary)}'


def short_date(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00')).astimezone(ZoneInfo('Europe/Istanbul')).strftime('%d %b %Y, %H:%M')


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
    assert safe_link('../../reports/ME492_Timetable_EN.pdf', Path('docs/progress/2026-09-30.md')) == 'report-ME492_Timetable_EN.html'
    print('PASS: escaping, tables, checklists, evidence status and document links')


def qualification_summary(matrix):
    cases = matrix['cases']
    passed_cases = sum(case['status'] == 'passed' for case in cases)
    components = [(environment, check) for case in cases for environment, check in case['components'].items()]
    total_passed = sum(check['status'] == 'passed' for _, check in components)
    component_summary = (f'All {len(components)} environment checks passed.' if total_passed == len(components)
                         else f'{total_passed} of {len(components)} environment checks passed.')
    tracks = {
        'python': ('Python V2.3', {'Python'}),
        'simscape': ('Native Simscape', {'Simscape'}),
        'ros2': ('ROS 2 / Gazebo', {'Gazebo Linux', 'Gazebo kamerası / video', 'ROS 2 Linux', 'ROS 2 + Gazebo Linux'}),
    }
    rows = []
    for anchor, (label, environments) in tracks.items():
        checks = [check for environment, check in components if environment in environments]
        passed_checks = sum(check['status'] == 'passed' for check in checks)
        rows.append(f'<div><dt><a href="results.html#{anchor}">{label}</a></dt><dd>{passed_checks} / {len(checks)}</dd></div>')
    return f'''<section class="qualification-panel" id="janus-qualification" aria-labelledby="qualification-title">
<div><h2 id="qualification-title">Qualification</h2><p><strong>{passed_cases} of {len(cases)}</strong> cases passed. {component_summary} Some cases run in several environments.</p></div>
<div><dl>{''.join(rows)}</dl><p class="muted small">The ROS 2 / Gazebo count includes live runs and camera replay.</p></div>
</section>'''


def bar_chart(rows, scale=None):
    items = []
    for label, value, total, color in rows:
        items.append(f'<div><dt>{label}</dt><dd><meter class="{color}" min="0" max="{scale or total or 1}" value="{value}" aria-hidden="true"></meter><span>{value} / {total}</span></dd></div>')
    return f'<dl class="status-chart">{"".join(items)}</dl>'


def selected_measurements():
    with (ROOT / 'reports/janus/python/measurements.csv').open() as source:
        runs = {row['run_id']: row for row in csv.DictReader(source)}
    python_csv = 'reports/janus/python/measurements.csv'
    python_rows = [
        ('M04', 'Cross-track RMSE', f'{1000 * float(runs["M04-python-nominal"]["path_rmse_m"]):.2f} mm', 'Nominal S path'),
        ('M05', 'Last of eight gates', f'{float(runs["M05-python-cw"]["last_gate_time_s"]):.2f} s', '0.50 m clockwise square'),
        ('M06', 'Last of eight gates', f'{float(runs["M06-python-ccw"]["last_gate_time_s"]):.2f} s', '0.50 m counterclockwise square'),
    ]
    rows = [(case, 'Python V2.3', metric, value, condition, python_csv) for case, metric, value, condition in python_rows]
    ros_rows = [
        ('M02', 'Brake command → stop band', 'stop_to_speed_band_s', 1000, '.0f', 'ms', 'World-pose speed band'),
        ('M02', 'Braking distance', 'braking_distance_m', 1000, '.2f', 'mm', 'Straight-line fixture'),
        ('Q08', 'Wrong-station observations', 'real_rendered_observations', 1, 'd', '', 'Rendered camera detections'),
        ('Q08', 'Forward requests', 'forward_requests', 1, 'd', '', 'Wrong-station run'),
        ('Q08', 'Search deadline', 'qr_timeout_elapsed_s', 1, '.3f', 's', 'Simulation time'),
        ('O05', 'Rejected requests', 'unauthorized_requests', 1, 'd', '', 'Fresh odometry, awaiting authorization'),
        ('O05', 'Odometry fault → stop band', 'fault_to_measured_stop_band_s', 1000, '.0f', 'ms', 'Injected timestamp regression'),
    ]
    for case, metric, field, scale, precision, unit, condition in ros_rows:
        source = f'reports/janus/ros2/live/{case}-acceptance.json'
        value = json.loads((ROOT / source).read_text())['measurements'][field] * scale
        rows.append((case, 'ROS 2 / Gazebo', metric, f'{value:{precision}} {unit}'.strip(), condition, source))
    native_path = 'reports/janus/native-m04-acceptance.json'
    native = json.loads((ROOT / native_path).read_text())
    rows.append(('M04', 'Native Simscape', 'Cross-track RMSE', f'{1000 * native["cross_track_rmse_m"]:.2f} mm', 'Two 0.30 m arcs, 45° initial caster', native_path))
    # The comparison and staged runs use different stop targets on the same simulated fixture.
    rows.extend([
        ('Q04', 'ROS 2 / Gazebo', 'Final mechanical position error', '43.70 mm', 'Comparison run', 'reports/janus/qualification/Q04.png'),
        ('Q04', 'ROS 2 / Gazebo', 'Final mechanical position error', '6.236 mm', 'Four-corner staging run', 'reports/janus/qualification/Q04.png'),
    ])
    return ''.join(
        f'<tr><th scope="row"><a href="#case-{case}">{case}</a></th><td>{environment}</td><td>{escape(metric)}<span class="measurement-context">{environment} · <a href="{source}">{condition} ↗</a></span></td><td class="mono numeric">{value}</td><td><a href="{source}">{condition} ↗</a></td></tr>'
        for case, environment, metric, value, condition, source in rows
    )


def timetable_chart(weeks):
    start = date(2026, 9, 30)
    finish = date(2027, 1, 15)
    first_monday = date(2026, 10, 5)
    review = date(2026, 11, 27)
    span = (finish - start).days

    def offset(day):
        return (day - start).days / span * 100

    phases = ['Preparation and modelling', 'Benchmarking and calibration',
              'Confirmation and evaluation', 'Review and handoff']
    rows = []
    for index, label in enumerate(phases):
        first = index * 4
        left = start if first == 0 else first_monday + timedelta(weeks=first - 1)
        right = finish if first == 12 else first_monday + timedelta(weeks=first + 3)
        group = weeks[first:first+4]
        title = f'{group[0]["id"]}–{group[-1]["id"]}'
        window = f'{left.strftime("%d %b")} – {(right - timedelta(days=1)).strftime("%d %b")}'
        rows.append(f'''<div class="gantt-row"><div class="gantt-label">{label}<small>{title} · {window}</small></div>
<div class="gantt-lane"><a class="gantt-bar" href="#week-{group[0]["id"]}"
    style="--start:{offset(left):.3f}%;--span:{offset(right)-offset(left):.3f}%"
    aria-label="{label}, {title}, {window}. Open weekly evidence">{title}</a></div></div>''')
    stamps = [(date(2026,10,1), 'Oct'), (date(2026,11,1), 'Nov'),
              (date(2026,12,1), 'Dec'), (date(2027,1,1), 'Jan')]
    ticks = ''.join(f'<span style="left:{offset(day):.3f}%">{label}</span>' for day, label in stamps)
    return f'''<div class="gantt-scroll" role="region" aria-label="Semester Gantt timetable" tabindex="0">
<div class="gantt" style="--review:{offset(review):.3f}%">
<div class="gantt-row gantt-axis"><span>Planned work</span><div class="gantt-months">{ticks}<span class="gantt-review-label"><time datetime="{review}">27 Nov</time></span></div></div>
{"".join(rows)}</div></div>'''


def publish_downloads():
    downloads = OUT / 'downloads'
    downloads.mkdir()
    models = [
        'models/caster-m0/models/Me492CasterM0.slx',
        'models/caster-m2-v1/models/Me492CasterM2V1.slx',
        'models/simscape-v6/caster_visual_v5_cylinder_contact.slx',
        'models/simscape-v6/caster_visual_v6_measurements.slx',
        'models/simscape-v6/evidence_latest_alignment_replay.slx',
        'models/simscape-janus/requalification/M04_contract_S_r1/native_M04_contract_S_r1.slx',
        'janus/ros2/src/arche_sim/worlds/janus.sdf',
    ]
    for name in models:
        path = ROOT / name
        shutil.copyfile(path, downloads / path.name)

    model_sources = [
        ROOT / 'scripts/browser_examples.py',
        ROOT / 'arche-tui/backend.py',
        ROOT / 'caster-physics-python-v2/twin.py',
        *sorted((ROOT / 'caster-physics-python/src/caster_physics').rglob('*.py')),
        *sorted((ROOT / 'caster-physics-python/parameters').glob('*.json')),
        *sorted((ROOT / 'caster-physics-python-v2/parameters').glob('*.json')),
    ]
    with ZipFile(downloads / 'arche-python-lab.zip', 'w', compression=ZIP_DEFLATED) as archive:
        for path in model_sources:
            archive.write(path, path.relative_to(ROOT))
    ros_sources = subprocess.check_output(
        ['git', 'ls-files', '-z', 'janus/ros2'], cwd=ROOT
    ).decode().split('\0')
    with ZipFile(downloads / 'arche-ros2.zip', 'w', compression=ZIP_DEFLATED) as archive:
        for name in filter(None, ros_sources):
            archive.write(ROOT / name, name)


def site_header(root, current):
    header = (ROOT / 'site/header.html').read_text().replace('{{ROOT}}', root)
    if current in {'python', 'simscape', 'ros-gazebo'}:
        return header.replace('data-page="index"', 'data-page="index" aria-current="location"')
    return header.replace(f'data-page="{current}"', f'data-page="{current}" aria-current="page" class="active"')


def case_index(cases, notes, scenes):
    codes = {case['case_id'] for case in cases}
    assert set(notes) == set(scenes) == codes, 'Every published case needs a summary and a scene'
    sections = []
    for prefix, anchor, heading in [('M', 'motion-cases', 'Motion'), ('Q', 'docking-cases', 'QR docking'), ('O', 'operating-cases', 'Operations')]:
        rows = []
        for case in cases:
            code = case['case_id']
            if not code.startswith(prefix):
                continue
            title, summary = notes[code]
            measurements = ''.join(
                f'<div><dt>{escape(ENVIRONMENTS[environment])}</dt><dd lang="tr">{escape(check["measurement_tr"])}</dd></div>'
                for environment, check in case['components'].items()
            )
            result = 'Passed' if case['status'] == 'passed' else escape(case['status'].replace('_', ' ').capitalize())
            scenes_url = f'scenes/?vaka={code}'
            environment, camera_media = scenes[code]
            video_url = f'scenes/{camera_media}' if camera_media else f'scenes/clips/{code}.mp4'
            poster_url = '' if camera_media else f'scenes/clips/{code}.jpg'
            caption = ('Recorded Gazebo camera input. The clip has its own playback clock.' if camera_media
                       else f'{escape(environment)} simulation replay. The clock shows time from the simulation log.')
            media = f'''<figure class="case-media">
  <video controls playsinline preload="none" aria-label="{escape(title, quote=True)} replay"
    data-case-video="{video_url}" data-poster="{poster_url}" width="800" height="432"></video>
  <figcaption>{caption}</figcaption>
</figure>
<p class="case-media-links"><a href="{scenes_url}">Explore {code} in 3D</a><a href="{video_url}">Open MP4</a></p>'''
            media_links = f'<p class="case-shortcuts"><a href="{scenes_url}">3D replay</a> · <a href="{video_url}">MP4</a></p>'
            rows.append(f'''<tr>
<th scope="row"><a href="#case-{code}">{code}</a><span class="case-priority">{escape(case["priority"])}</span></th>
<td><details class="case-entry" id="case-{code}"><summary>{escape(title)}</summary>
<div class="case-body"><p>{escape(summary)}</p><p lang="tr"><strong>Kabul ölçütü</strong> {escape(case["acceptance_tr"])}</p>{media}</div></details>{media_links}</td>
<td>{result}</td><td><dl class="case-measurements">{measurements}</dl></td></tr>''')
        sections.append(f'''<section id="{anchor}" aria-labelledby="{anchor}-title">
<h3 id="{anchor}-title">{heading} <span class="muted mono">{len(rows)} cases</span></h3>
<div class="table-scroll" role="region" aria-label="{heading} case measurements" tabindex="0"><table class="data-table case-table">
<thead><tr><th scope="col">Case</th><th scope="col">Scenario</th><th scope="col">Status</th><th scope="col">Recorded outcome by environment</th></tr></thead>
<tbody>{"".join(rows)}</tbody></table></div></section>''')
    return ''.join(sections)


def build(issues, comments):
    if OUT.is_symlink():
        raise ValueError('Refusing a symlinked build output')
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir()
    publish_downloads()
    shutil.copytree(ROOT / 'site/assets', OUT / 'assets')
    evidence = OUT / 'assets/development'
    evidence.mkdir()
    figures = {
        'python-control-geometry.svg': 'caster-physics-python-v2/output/figures/V2_Control_Error_Geometry.svg',
        'm2-top-level.png': 'models/caster-m2-v1/plots/top_level.png',
        'm2-contacts.png': 'models/caster-m2-v1/plots/contacts_subsystem.png',
        'v5-step-refinement.png': 'models/simscape-v6/evidence/v5_step_refinement.png',
        'v6-alignment.png': 'models/simscape-v6/evidence/v6_alignment_replay_and_original_tail.png',
    }
    for name, path in figures.items():
        shutil.copyfile(ROOT / path, evidence / name)
    shutil.copytree(ROOT / 'site/scenes', OUT / 'scenes')
    for name in ('style.css', 'journal.css', 'app.js'):
        shutil.copyfile(ROOT / 'site' / name, OUT / name)
    shutil.copytree(ROOT / 'reports', OUT / 'reports', ignore=shutil.ignore_patterns('*.md'))
    documents = [Path('CONTRIBUTING.md'), *sorted(p.relative_to(ROOT) for p in (ROOT / 'docs').rglob('*.md')), Path('reports/README.md')]
    doc_links = []
    for path in documents:
        text = (ROOT / path).read_text()
        title = text.splitlines()[0].lstrip('# ')
        dest = OUT / document_url(path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        relative_root = os.path.relpath(OUT, dest.parent).replace(os.sep, '/') + '/'
        page = f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><base href="{relative_root}"><title>{escape(title)} · Arche</title><link rel="icon" href="assets/olus-emre-logo.svg"><link rel="stylesheet" href="style.css"></head><body><div class="frame"><header class="record-header"><a href="library.html">← Arche / Library</a><a href="{GH}/blob/main/{path.as_posix()}">Source on GitHub ↗</a></header><main class="record"><p class="record-meta">PUBLIC RESEARCH RECORD / {escape(path.as_posix())}</p><article class="prose">{markdown(text, path)}</article></main></div></body></html>'
        dest.write_text(page)
        doc_links.append(f'<tr><th scope="row"><a href="{document_url(path)}">{escape(title)} ↗</a></th><td>{escape(path.parent.name.upper())}</td></tr>')
    shutil.copyfile(ROOT / 'docs/research/run-inventory.csv', OUT / 'records/run-inventory.csv')
    doc_links.append('<tr><th scope="row"><a href="records/run-inventory.csv">Run inventory ↗</a></th><td>CSV</td></tr>')
    issues = sorted((i for i in issues if 'pull_request' not in i), key=lambda i: i['number'])
    tasks, journal = [], []
    weekly = [i for i in issues if any((l if isinstance(l, str) else l['name']) == 'type:weekly' for l in i['labels'])]
    weekly_by_code = {i['title'].split('|', 1)[0].strip(): i for i in weekly}
    def label_names(issue):
        return [label if isinstance(label, str) else label['name'] for label in issue['labels']]
    for issue in issues:
        status = state(issue)
        kind = 'gate' if 'type:gate' in label_names(issue) else 'weekly' if issue in weekly else 'task'
        code = issue['title'].split('|')[0].strip()
        if not re.fullmatch(r'[WG]\d+', code):
            code = f'#{issue["number"]}'
        title = issue['title'].split('|')[-1].strip()
        body_text = (issue.get('body') or '').replace('**', '')
        period = re.search(r'^Period: (.+?) · Budget: ([^·\n]+)', body_text, re.M)
        deadline = re.search(r'^## Decision required by (\d{4}-\d{2}-\d{2})', body_text, re.M)
        meta = f'{period[1]} · {period[2].strip()} planned' if period else f'Planned decision date {deadline[1]}' if deadline else 'See deliverable and acceptance evidence'
        history = []
        for comment in comments:
            if comment['issue_url'].endswith(f'/{issue["number"]}'):
                history.append(f'<div class="issue-comment"><p>{escape(comment["user"]["login"])} · {short_date(comment["created_at"])} · <a href="{escape(comment["html_url"])}">Source ↗</a></p><div class="prose">{markdown(comment.get("body") or "", embedded=True)}</div></div>')
        tasks.append(f'<details class="task" name="research-task" data-status="{status}" data-kind="{kind}" data-issue="{issue["number"]}" id="task-{issue["number"]}"><summary><span class="task-code">{escape(code)}</span><span class="task-main"><span class="task-title">{escape(title)}</span><span class="task-meta">{escape(meta)}</span></span>{badge(status)}<span class="task-open-label">Read task →</span><span class="task-close-label">Close details ↑</span></summary><div class="task-body"><p class="task-context">{kind.title()} task · GitHub issue #{issue["number"]} · Published record</p><div class="prose">{markdown(issue.get("body") or "", embedded=True)}</div>{"".join(history)}<div class="task-actions"><button type="button" data-close-task hidden>Close details ↑</button><a href="#task-{issue["number"]}">Link to this task</a><a href="{escape(issue_url(issue))}">Issue and updates on GitHub ↗</a></div></div></details>')
    for path in sorted((ROOT / 'docs/progress').glob('????-??-??*.md'), reverse=True):
        source = path.relative_to(ROOT)
        title = path.read_text().splitlines()[0].lstrip('# ')
        journal.append((path.name[:10], f'<article class="journal-entry"><time datetime="{path.name[:10]}">{path.name[:10]}</time><h3>{escape(title)}</h3><details><summary>Read entry</summary><div class="prose">{markdown(path.read_text(), source, embedded=True)}</div></details><a class="text-link" href="{document_url(source)}">Open original record</a></article>'))
    for comment in comments:
        number = int(comment['issue_url'].rsplit('/', 1)[1])
        journal.append((comment['created_at'], f'<article class="journal-entry"><time datetime="{comment["created_at"]}">{short_date(comment["created_at"])}</time><h3>Task #{number} update</h3><p>{escape(comment["user"]["login"])} posted this update on GitHub.</p><details><summary>Read update +</summary><div class="prose">{markdown(comment.get("body") or "", embedded=True)}</div></details><a class="text-link" href="{escape(comment["html_url"])}">Source comment ↗</a></article>'))
    journal.sort(key=lambda item: item[0], reverse=True)
    gate_progress = []
    weeks = []
    reviews = {}
    for line in (ROOT / 'docs/planning/timetable.md').read_text().splitlines():
        if not re.match(r'^\| W\d+', line):
            continue
        week, period, hours, output, evidence = [cell.strip() for cell in line.strip('|').split('|')]
        issue = weekly_by_code.get(week)
        number = issue['number'] if issue else None
        status = state(issue) if issue else 'unclassified'
        review = evidence_review(issue) if issue else ''
        weeks.append({'id': week, 'hours': int(hours), 'period': period, 'output': output, 'evidence': evidence, 'status': status, 'issue': number})
        reviews[week] = review
    for issue in issues:
        if 'type:gate' not in label_names(issue):
            continue
        number = issue['number']
        code, _, title = issue['title'].partition('|')
        deadline = re.search(r'^## Decision required by (\d{4}-\d{2}-\d{2})', issue.get('body') or '', re.M)
        deadline_label = deadline[1] if deadline else 'Unscheduled'
        review = evidence_review(issue) or 'No evidence update recorded.'
        gate_progress.append(f'<tr data-progress-issue="{number}"><th scope="row"><a href="tasks.html#task-{number}">{escape(code.strip())} ↗</a></th><td>{escape(title.strip())}</td><td>{deadline_label}</td><td>{badge(state(issue))}</td><td class="evidence-review">{review}</td></tr>')
    active = [i for i in weekly if state(i) == 'in-progress']
    focus = ''.join(f'<h3>{escape(i["title"].split("|")[-1].strip())}</h3>{badge("in-progress")}<p><a href="tasks.html#task-{i["number"]}">Task #{i["number"]} ↗</a></p>' for i in active) or '<h3>No task is marked in progress.</h3><p>Check the Tasks page for the current record.</p>'
    built = datetime.now(timezone.utc)
    log = subprocess.check_output(['git', 'log', '-5', '--format=%H%x09%cs%x09%s'], cwd=ROOT, text=True)
    commits = []
    for line in log.splitlines():
        sha, date, title = line.split('\t', 2)
        commits.append(f'<a href="{GH}/commit/{sha}"><span>{escape(title)}</span><small>{date} · {sha[:7]} ↗</small></a>')
    access = Path('docs/access/access-register.md')
    decisions = Path('docs/planning/decisions.md')
    with (ROOT / 'docs/research/run-inventory.csv').open() as source:
        run_count = len(list(csv.DictReader(source)))
    qualification = json.loads((ROOT / 'reports/janus/qualification-matrix.json').read_text())
    roadmap, weekly_progress = [], []
    for week in weeks:
        code = week['id']
        number = week['issue']
        title = escape(week['output'])
        status = badge(week['status'])
        review = reviews[code] or 'No evidence update recorded.'
        task = f'<a href="tasks.html#task-{number}">{code} ↗</a>' if number else code
        issue_attr = f' data-progress-issue="{number}"' if number else ''
        roadmap.append(f'''<tr id="week-{code}"{issue_attr}><th scope="row">{task}</th><td>{escape(week["period"])}</td>
<td><strong>{title}</strong><p>{escape(week["evidence"])}</p></td><td class="mono">{week["hours"]}</td>
<td>{status}</td><td class="evidence-review">{review}</td></tr>''')
        weekly_progress.append(f'<tr{issue_attr}><th scope="row">{task}</th><td>{title}</td><td>{status}</td><td class="evidence-review">{review}</td></tr>')
    gates = [i for i in issues if 'type:gate' in label_names(i)]
    status_bars = [
        ('Weeks verified', sum(w['status'] == 'verified' for w in weeks), len(weeks), 'verified'),
        ('Weeks in progress', sum(w['status'] == 'in-progress' for w in weeks), len(weeks), 'active'),
        ('Weeks blocked', sum(w['status'] == 'blocked' for w in weeks), len(weeks), 'blocked'),
        ('Weeks planned', sum(w['status'] == 'planned' for w in weeks), len(weeks), 'planned'),
        ('Decision gates verified', sum(state(gate) == 'verified' for gate in gates), len(gates), 'verified'),
    ]
    coverage_bars = []
    for prefix, label in [('M', 'Motion'), ('Q', 'QR docking'), ('O', 'Operations')]:
        family = [case for case in qualification['cases'] if case['case_id'].startswith(prefix)]
        coverage_bars.append((label, sum(case['status'] == 'passed' for case in family), len(family), 'verified'))
    journal_entries = [item[1] for item in journal]
    journal_html = ''.join(journal_entries[:4])
    if journal_entries[4:]:
        journal_html += '<details class="fold older-entries"><summary>Earlier entries</summary>' + ''.join(journal_entries[4:]) + '</details>'
    tokens = {'RUN_COUNT': str(run_count), 'ACCESS_SNAPSHOT': escape((ROOT / access).read_text().split('\n\n')[1]), 'COMMITS': ''.join(commits), 'ACCESS': markdown((ROOT / access).read_text(), access, embedded=True), 'DECISIONS': markdown((ROOT / decisions).read_text(), decisions, embedded=True), 'FOCUS': focus, 'PROGRESS': journal_html, 'SYNC': f'GitHub snapshot · {short_date(built.isoformat())} (Istanbul).', 'TASKS': ''.join(tasks), 'ROADMAP': ''.join(roadmap), 'DOCUMENTS': ''.join(doc_links)}
    tokens['JANUS_QUALIFICATION'] = qualification_summary(qualification)
    tokens['GANTT'] = timetable_chart(weeks)
    examples = (ROOT / 'scripts/browser_examples.py').read_text()
    for node in ast.parse(examples).body:
        if isinstance(node, ast.FunctionDef):
            snippet = ast.get_source_segment(examples, node)
            tokens[f'PYTHON_{node.name.upper()}'] = escape(f'{snippet}\n\n{node.name}()')
    tokens['WEEKLY_PROGRESS'] = ''.join(weekly_progress)
    tokens['PROGRESS_CHART'] = bar_chart(status_bars)
    tokens['COVERAGE_CHART'] = bar_chart(coverage_bars, max(row[2] for row in coverage_bars))
    tokens['MEASUREMENTS'] = selected_measurements()
    scenes_dir = ROOT / 'site/scenes'
    scene_index = json.loads((scenes_dir / 'data/index.json').read_text())
    notes = json.loads((ROOT / 'site/case-notes.json').read_text())
    scene_environments = {}
    for scene in scene_index['cases']:
        code = scene['id']
        assert re.fullmatch(r'[MQO][0-9]{2}', code), f'Invalid scene code: {code}'
        tracks = json.loads((scenes_dir / f'data/{code}.json').read_text())['tracks']
        assert len(tracks) == scene['variants'], f'Wrong run count: {code}'
        for extension in ('jpg', 'mp4'):
            assert (scenes_dir / f'clips/{code}.{extension}').is_file(), f'Missing {extension}: {code}'
        scene_environments[code] = (tracks[0]['environment'], tracks[0]['media'])
        scene['titleEn'], scene['noteEn'] = notes[code]
    (OUT / 'scenes/data/index.json').write_text(json.dumps(scene_index, ensure_ascii=False, separators=(',', ':')))
    for name in ('index.html', 'credits.html'):
        scene_page = OUT / 'scenes' / name
        scene_page.write_text(scene_page.read_text().replace('{{HEADER}}', site_header('../', 'scenes')))
    tokens['CASE_INDEX'] = case_index(qualification['cases'], notes, scene_environments)
    tokens['GATE_PROGRESS'] = ''.join(gate_progress)
    layout = (ROOT / 'site/layout.html').read_text()
    for slug, title in PAGES:
        body = (ROOT / 'site' / f'{slug}.html').read_text()
        values = dict(tokens, CONTENT=body, HEADER=site_header('', slug), TITLE=title, PAGE=slug, DESCRIPTION=f'{title} | Arche ME492 research record by Oluş Emre Demir.')
        page = layout
        for key in ('CONTENT', *values):
            page = page.replace('{{' + key + '}}', values[key])
        assert '{{' not in page
        (OUT / f'{slug}.html').write_text(page)
    for name, title in REPORTS:
        pdf = f'reports/{name}.pdf'
        body = f'<section class="pdf-reader"><div class="reader-heading"><a href="library.html" aria-label="Back to library">← Library</a><h1>{escape(title)}</h1><a href="{pdf}" download>Download PDF ↓</a></div><div class="pdf-controls" role="group" aria-label="PDF controls"><div class="reader-pagination"><button id="pdf-prev" aria-label="Previous page" disabled>←</button><label>Page <input id="pdf-page" type="number" min="1" value="1" disabled></label><span id="pdf-total"></span><button id="pdf-next" aria-label="Next page" disabled>→</button></div><div class="reader-zoom"><button id="pdf-out" aria-label="Zoom out" disabled>−</button><label class="sr-only" for="pdf-fit">Page sizing</label><select id="pdf-fit" disabled><option value="width">Fit width</option><option value="page">Fit page</option><option value="1">100%</option><option value="1.25">125%</option><option value="1.5">150%</option><option value="2">200%</option></select><button id="pdf-in" aria-label="Zoom in" disabled>+</button></div><button id="pdf-focus" type="button" aria-label="Focus view" aria-pressed="false">Focus</button><span id="pdf-status" role="status">Loading document…</span></div><div class="pdf-stage" data-pdf="{pdf}" tabindex="0" role="region" aria-label="PDF document. Scroll to read the following pages."></div><noscript><p class="reader-fallback">This reader requires JavaScript. <a href="{pdf}">Open the original PDF</a>.</p></noscript><link rel="stylesheet" href="assets/pdfjs/text-layer.css"><script type="module" src="assets/pdf-reader.mjs"></script></section>'

        page = layout
        for key, value in dict(CONTENT=body, HEADER=site_header('', 'library'), TITLE=escape(title), PAGE='reader', DESCRIPTION=escape(title) + ' | Arche research report').items():
            page = page.replace('{{' + key + '}}', value)
        (OUT / f'report-{name}.html').write_text(page)
    (OUT / 'manifesto.html').write_text(
        '<!doctype html><html lang="en"><meta charset="utf-8">'
        '<meta http-equiv="refresh" content="0; url=vision.html"><title>Arche vision</title>'
        '<p><a href="vision.html">Open the research vision</a></p></html>'
    )
    (OUT / '.nojekyll').touch()
    (OUT / 'project.json').write_text(json.dumps({'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(), 'weeks': weeks, 'gates': [i for i in issues if 'type:gate' in label_names(i)], 'published_runs': run_count}, ensure_ascii=False))
    (OUT / 'snapshot.json').write_text(json.dumps({'published_at': built.isoformat(), 'source': GH, 'issues': issues, 'comments': comments}, ensure_ascii=False, indent=2))
    version = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT, text=True).strip()
    for path in OUT.rglob('*.html'):
        page = path.read_text()
        for asset in ('style.css', 'journal.css', 'app.js', '../style.css', '../journal.css', '../app.js', 'assets/pdf-reader.mjs', 'assets/pdfjs/text-layer.css', 'assets/python-lab.mjs'):
            page = page.replace(f'"{asset}"', f'"{asset}?v={version}"')
        path.write_text(page)
    scene_app = OUT / 'scenes/app.js'
    scene_app.write_text(scene_app.read_text().replace("'./viewer.js'", f"'./viewer.js?v={version}'"))
    python_lab = OUT / 'assets/python-lab.mjs'
    python_lab.write_text(python_lab.read_text().replace("'./python-worker.mjs'", f"'./python-worker.mjs?v={version}'"))
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
        saved = json.loads(args.issues_file.read_text()) if args.issues_file else None
        issues = saved.get('issues', []) if isinstance(saved, dict) else saved if saved is not None else api_all('issues?state=all')
        comments = [] if args.issues_file else api_all('issues/comments?sort=created&direction=asc')
        build(issues, comments)
