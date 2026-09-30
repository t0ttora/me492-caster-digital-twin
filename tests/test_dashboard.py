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
        self.assertEqual(len(list((ROOT / '_site/reports').glob('*.pdf'))), 2)
        self.assertFalse(any('.git' in p.parts or p.suffix in {'.py', '.md', '.env'} for p in files))
        self.assertTrue((ROOT / '_site/.nojekyll').exists())

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
