import csv
import json
import subprocess
import unittest
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import urlsplit, unquote

ROOT = Path(__file__).resolve().parents[1]

class Links(HTMLParser):
    def __init__(self):
        super().__init__(); self.urls = []; self.ids = []; self.tags = []
    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs); self.tags.append((tag, attrs))
        if 'id' in attrs: self.ids.append(attrs['id'])
        for key in ['href', 'src']:
            if key in attrs: self.urls.append(attrs[key])

class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run(['python3', 'scripts/build_dashboard.py'], cwd=ROOT, capture_output=True, text=True)
        if result.returncode: raise AssertionError('Dashboard build failed: ' + result.stderr)
        cls.html = (ROOT / '_site/index.html').read_text()
        cls.data = json.loads((ROOT / '_site/project.json').read_text())
        cls.links = Links(); cls.links.feed(cls.html)

    def test_source_grounded_scope(self):
        self.assertEqual(len(self.data['weeks']), 16)
        self.assertEqual(sum(int(w['hours']) for w in self.data['weeks']), 252)
        self.assertEqual(len(self.data['gates']), 6)
        self.assertEqual(len(self.data['access']), 7)
        with (ROOT / 'docs/research/run-inventory.csv').open() as source:
            self.assertEqual(self.data['published_runs'], len(list(csv.DictReader(source))))
        self.assertIn('Oluş Emre Demir', self.html)
        self.assertNotIn('Veysel', self.html)

    def test_real_content_without_javascript(self):
        for phrase in ['Caster-aware', 'digital twin.', 'Scope and preparation', 'G1', 'Independent pose reference', 'D01']:
            self.assertIn(phrase, self.html)
        self.assertEqual(self.html.count('data-week='), 16)
        self.assertIn('Snapshot', self.html)

    def test_local_links_resolve_and_ids_unique(self):
        self.assertEqual(len(self.links.ids), len(set(self.links.ids)))
        for href in self.links.urls:
            url = urlsplit(href)
            if url.scheme or url.netloc: continue
            if url.path:
                target = ROOT / '_site' / unquote(url.path)
                self.assertTrue(target.exists(), href)
            elif url.fragment:
                self.assertIn(url.fragment, self.links.ids, href)

    def test_only_allowed_public_assets_are_built(self):
        files = [p for p in (ROOT / '_site').rglob('*') if p.is_file()]
        self.assertEqual(len(list((ROOT / '_site/reports').glob('*.pdf'))), 2)
        self.assertFalse(any('.git' in p.parts for p in files))
        self.assertFalse(any(p.suffix in ['.py', '.md', '.env'] for p in files))
        self.assertTrue((ROOT / '_site/.nojekyll').exists())

    def test_accessibility_basics(self):
        self.assertIn('lang="en"', self.html)
        self.assertIn('Skip to content', self.html)
        self.assertEqual(sum(tag == 'h1' for tag, _ in self.links.tags), 1)
        for tag, attrs in self.links.tags:
            if tag == 'img': self.assertIn('alt', attrs)
        self.assertIn('aria-live="polite"', self.html)

if __name__ == '__main__': unittest.main()
