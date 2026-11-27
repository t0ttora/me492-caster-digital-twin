import csv
import json
import re
import subprocess
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import build_site

class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run(['python3', 'scripts/build_site.py', '--issues-file', 'site/issue-snapshot.json'], cwd=ROOT, capture_output=True, text=True)
        if result.returncode:
            raise AssertionError('Build failed: ' + result.stderr)
        cls.pages = {slug: (ROOT / '_site' / f'{slug}.html').read_text() for slug, _ in build_site.PAGES}
        cls.html = ''.join(cls.pages.values())
        cls.data = json.loads((ROOT / '_site/project.json').read_text())

    def test_source_grounded_scope(self):
        self.assertEqual(len(self.data['weeks']), 16)
        self.assertEqual(sum(w['hours'] for w in self.data['weeks']), 252)
        self.assertEqual(len(self.data['gates']), 6)
        with (ROOT / 'docs/research/run-inventory.csv').open() as source:
            self.assertEqual(self.data['published_runs'], len(list(csv.DictReader(source))))
        for phrase in ['Oluş Emre Demir', 'Independent pose reference', 'D01']:
            self.assertIn(phrase, self.html)

    def test_real_content_without_javascript(self):
        self.assertEqual(self.html.count('class="task"'), 22)
        self.assertEqual(self.html.count('data-kind="gate"'), 6)
        for phrase in ['Scope and preparation', 'How I work', 'Published reports', 'Oluş Emre Demir']:
            self.assertIn(phrase, self.html)

    def test_links_and_public_artifact(self):
        build_site.validate_links()
        files = [p for p in (ROOT / '_site').rglob('*') if p.is_file()]
        self.assertEqual(
            {path.stem for path in (ROOT / '_site/reports').glob('*.pdf')},
            {name for name, _ in build_site.REPORTS},
        )
        self.assertFalse(any('.git' in p.parts or p.suffix in {'.py', '.md', '.env'} for p in files))
        self.assertTrue((ROOT / '_site/.nojekyll').exists())

    def test_task_reading_and_report_links(self):
        page = self.pages['tasks']
        self.assertIn('<dialog id="task-inspector"', page)
        self.assertIn('data-kind-filter="weekly"', page)
        self.assertIn('10 person-hours planned', page)
        self.assertIn('Planned decision date 2026-10-16', page)
        url = build_site.GH + '/blob/main/reports/ME492_Timetable_EN.pdf'
        self.assertEqual(build_site.safe_link(url), 'report-ME492_Timetable_EN.html')

    def test_qualification_has_one_home(self):
        matrix = json.loads((ROOT / 'reports/janus/qualification-matrix.json').read_text())
        passed = sum(case['status'] == 'passed' for case in matrix['cases'])
        self.assertEqual(self.html.count('id="janus-qualification"'), 1)
        results = self.pages['results']
        self.assertIn(f'<strong>{passed} of {len(matrix["cases"])}</strong> cases passed', results)
        self.assertIn('All 72 environment checks passed.', results)
        for track, result in [('Python V2.3', '29 / 29'), ('Native Simscape', '16 / 16'), ('ROS 2 / Gazebo', '27 / 27')]:
            self.assertIn(f'>{track}</a></dt><dd>{result}</dd>', results)
        for slug in ('index', 'progress'):
            self.assertNotIn('environment checks', self.pages[slug])
        self.assertIn(f'lists {self.data["published_runs"]} physical runs', results)
        self.assertIn('src="scenes/?vaka=Q04&amp;embed=1"', self.pages['index'])
        self.assertIn('href="roadmap.html#weekly-progress"', self.pages['progress'])

    def test_results_index_and_live_trace_downloads(self):
        page = self.pages['results']
        matrix = json.loads((ROOT / 'reports/janus/qualification-matrix.json').read_text())
        self.assertEqual(page.count('class="case-entry"'), len(matrix['cases']))
        for case in matrix['cases']:
            self.assertEqual(page.count(f'id="case-{case["case_id"]}"'), 1)
        for case in ('M02', 'Q08', 'O05'):
            directory = ROOT / 'reports/janus/ros2/live'
            self.assertEqual(json.loads((directory / f'{case}-acceptance.json').read_text())['status'], 'passed')
            with (directory / f'{case}-telemetry.csv').open() as source:
                self.assertTrue({'truth', 'request', 'guard_output'} <= {row['source'] for row in csv.DictReader(source)})
            for file in (f'{case}-telemetry.csv', f'{case}-acceptance.json'):
                self.assertIn(f'href="reports/janus/ros2/live/{file}"', page)

    def test_each_case_has_a_replay_and_video(self):
        output = ROOT / '_site/scenes'
        index = json.loads((output / 'data/index.json').read_text())['cases']
        self.assertEqual(len(index), 40)
        self.assertEqual(sum(case['variants'] for case in index), 59)
        page = self.pages['results']
        notes = json.loads((ROOT / 'site/case-notes.json').read_text())
        for case in index:
            code = case['id']
            self.assertEqual([case['titleEn'], case['noteEn']], notes[code])
            entry = page.split(f'id="case-{code}"', 1)[1].split('</details>', 1)[0]
            self.assertIn(f'href="scenes/?vaka={code}"', entry)
            self.assertIn(f'data-case-video="scenes/clips/{code}.mp4"', entry)
            self.assertIn(f'href="scenes/clips/{code}.mp4"', entry)
            with (output / f'clips/{code}.mp4').open('rb') as clip:
                self.assertEqual(clip.read(8)[4:8], b'ftyp')
            tracks = json.loads((output / f'data/{code}.json').read_text())['tracks']
            self.assertEqual(len(tracks), case['variants'])
            for track in tracks:
                self.assertNotIn('source', track)
                self.assertNotIn('fixtureFile', track['setup'])
        self.assertTrue((output / 'assets/janus-kit.glb').exists())

    def test_weekly_and_gate_views_share_task_status_and_evidence(self):
        fixture = json.loads((ROOT / 'site/issue-snapshot.json').read_text())['issues']
        by_number = {issue['number']: issue for issue in fixture}
        self.assertNotIn('data-progress-issue=', self.pages['progress'])
        for slug, numbers, close_tag in [('roadmap', range(1, 17), '</details>'), ('decisions', range(17, 23), '</tr>')]:
            page = self.pages[slug]
            self.assertEqual(page.count('data-progress-issue='), len(numbers))
            for number in numbers:
                issue = by_number[number]
                row = page.split(f'data-progress-issue="{number}"', 1)[1].split(close_tag, 1)[0]
                self.assertIn(build_site.badge(build_site.state(issue)), row)
                self.assertIn(f'href="tasks.html#task-{number}"', row)
                review = build_site.evidence_review(issue)
                self.assertTrue(review)
                self.assertIn(review, row)
        roadmap = self.pages['roadmap']
        for number in range(1, 17):
            self.assertIn(f'href="#week-W{number:02d}"', roadmap)
            self.assertIn(f'id="week-W{number:02d}"', roadmap)

    def test_embedded_report_readers(self):
        for name, title in build_site.REPORTS:
            page = (ROOT / '_site' / f'report-{name}.html').read_text()
            self.assertIn(f'data-pdf="reports/{name}.pdf"', page)
            self.assertIn('assets/pdf-reader.mjs', page)
            self.assertIn('id="pdf-page"', page)
            self.assertIn('Download PDF', page)
            self.assertIn(f'href="report-{name}.html"', self.pages['library'])
        self.assertTrue((ROOT / '_site/assets/pdfjs/pdf.worker.min.mjs').exists())

    def test_accessibility_basics(self):
        for page in self.pages.values():
            self.assertIn('lang="en"', page)
            interface = re.sub(r'<(?:p|dd) lang="tr">.*?</(?:p|dd)>', '', page)
            for phrase in ['Görev', 'İlerleme', 'Kütüphane', 'koşulları', 'Yol haritası']:
                self.assertNotIn(phrase, interface)
        self.assertIn('Skip to content', self.html)
        for slug, page in self.pages.items():
            self.assertEqual(page.count('<h1'), 1, slug)
        self.assertIn('aria-live="polite"', self.html)
        self.assertIn('aria-label="Arche home"', self.html)

    def test_separate_routes_without_invented_visuals(self):
        self.assertEqual(len(self.pages), 10)
        for slug, page in self.pages.items():
            self.assertIn(f'data-page="{slug}" aria-current="page"', page)
            self.assertNotIn('<svg', page)
            self.assertNotIn('experiment-figure', page)
            self.assertIn('href="tasks.html"', page)
        self.assertNotIn('class="task"', self.pages['index'])
        self.assertIn('class="task"', self.pages['tasks'])
        self.assertIn('class="roadmap-stops"', self.pages['roadmap'])
        self.assertIn('journal-entry', self.pages['progress'])

    def test_untrusted_text_and_evidence_status(self):
        build_site.check()
        self.assertEqual(build_site.state({'state':'open','labels':[{'name':'status:planned'},{'name':'status:blocked'}]}), 'conflicting')
        self.assertIn('&lt;img', build_site.markdown('<img src=x onerror=alert(1)>'))
        self.assertEqual(build_site.safe_link('javascript:alert(1)'), '#')

if __name__ == '__main__':
    unittest.main()
