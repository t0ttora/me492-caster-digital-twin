import csv
import json
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
        for phrase in ['Scope and preparation', 'MANIFESTO', 'Published reports', 'PROFILE / PRACTICE']:
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

    def test_weekly_counts_do_not_appear_as_project_completion(self):
        self.assertNotIn('PROJECT PROGRESS', self.pages['index'])
        self.assertNotIn('data-summary=', self.html)
        self.assertNotIn('class="metric-value"', self.pages['index'])
        self.assertIn('class="metric-value"', self.pages['progress'].split('id="weekly-progress"')[1])

    def test_simulation_qualification_on_both_progress_views(self):
        matrix = json.loads((ROOT / 'reports/janus/qualification-matrix.json').read_text())
        passed = sum(case['status'] == 'passed' for case in matrix['cases'])
        for slug in ('index', 'progress'):
            page = self.pages[slug]
            self.assertIn(f'<strong>{passed} / {len(matrix["cases"])}</strong> cases passed', page)
            for track, result in [('Python V2.3', '29 / 29'), ('Native Simscape', '16 / 16'), ('ROS 2 / Gazebo', '27 / 27')]:
                self.assertIn(f'<dt>{track}</dt><dd>{result}</dd>', page)
            self.assertIn('href="reports/janus/qualification-matrix.tsv"', page)
        self.assertIn(f'Physical runs in the public inventory: <b>{self.data["published_runs"]}</b>', self.pages['progress'])

    def test_weekly_and_gate_views_share_task_status_and_evidence(self):
        fixture = json.loads((ROOT / 'site/issue-snapshot.json').read_text())['issues']
        by_number = {issue['number']: issue for issue in fixture}
        for slug, numbers in [('progress', range(1, 17)), ('roadmap', range(1, 17)), ('decisions', range(17, 23))]:
            page = self.pages[slug]
            self.assertEqual(page.count('data-progress-issue='), len(numbers))
            for number in numbers:
                issue = by_number[number]
                row = page.split(f'<tr data-progress-issue="{number}">')[1].split('</tr>')[0]
                self.assertIn(build_site.badge(build_site.state(issue)), row)
                self.assertIn(f'href="tasks.html#task-{number}"', row)
                review = build_site.evidence_review(issue)
                self.assertTrue(review)
                self.assertIn(review, row)

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
            for phrase in ['Görev', 'İlerleme', 'Kütüphane', 'koşulları', 'Yol haritası']:
                self.assertNotIn(phrase, page)
        self.assertIn('Skip to content', self.html)
        for slug, page in self.pages.items():
            self.assertEqual(page.count('<h1'), 1, slug)
        self.assertIn('aria-live="polite"', self.html)
        self.assertIn('aria-label="Arche home"', self.html)

    def test_separate_routes_without_invented_visuals(self):
        self.assertEqual(len(self.pages), 9)
        for slug, page in self.pages.items():
            self.assertIn(f'data-page="{slug}" aria-current="page"', page)
            self.assertNotIn('<svg', page)
            self.assertNotIn('experiment-figure', page)
            self.assertIn('href="tasks.html"', page)
        self.assertNotIn('class="task"', self.pages['index'])
        self.assertIn('class="task"', self.pages['tasks'])
        self.assertIn('data-table roadmap', self.pages['roadmap'])
        self.assertIn('journal-entry', self.pages['progress'])

    def test_untrusted_text_and_evidence_status(self):
        build_site.check()
        self.assertEqual(build_site.state({'state':'open','labels':[{'name':'status:planned'},{'name':'status:blocked'}]}), 'conflicting')
        self.assertIn('&lt;img', build_site.markdown('<img src=x onerror=alert(1)>'))
        self.assertEqual(build_site.safe_link('javascript:alert(1)'), '#')

if __name__ == '__main__':
    unittest.main()
