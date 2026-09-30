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
        cls.html = (ROOT / '_site/index.html').read_text()
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
        for phrase in ['Scope and preparation', 'Temsili şema', 'MANIFESTO', 'ADVISOR DESK']:
            self.assertIn(phrase, self.html)

    def test_links_and_public_artifact(self):
        build_site.validate_links()
        files = [p for p in (ROOT / '_site').rglob('*') if p.is_file()]
        self.assertEqual(len(list((ROOT / '_site/reports').glob('*.pdf'))), 2)
        self.assertFalse(any('.git' in p.parts or p.suffix in {'.py', '.md', '.env'} for p in files))
        self.assertTrue((ROOT / '_site/.nojekyll').exists())

    def test_accessibility_basics(self):
        self.assertIn('lang="tr"', self.html)
        self.assertIn('İçeriğe geç', self.html)
        self.assertEqual(self.html.count('<h1'), 1)
        self.assertIn('aria-live="polite"', self.html)
        self.assertIn('alt="Oluş Emre Demir"', self.html)

    def test_untrusted_text_and_evidence_status(self):
        build_site.check()
        self.assertEqual(build_site.state({'state':'open','labels':[{'name':'status:planned'},{'name':'status:blocked'}]}), 'conflicting')
        self.assertIn('&lt;img', build_site.markdown('<img src=x onerror=alert(1)>'))
        self.assertEqual(build_site.safe_link('javascript:alert(1)'), '#')

if __name__ == '__main__':
    unittest.main()
