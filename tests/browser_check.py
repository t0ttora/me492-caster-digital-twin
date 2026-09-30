"""Optional CI browser regression; synthetic API data are not project evidence."""
import atexit
import json
import subprocess
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '_qa'
OUT.mkdir(exist_ok=True)
server = subprocess.Popen(['python3', '-m', 'http.server', '8766', '--bind', '127.0.0.1', '--directory', str(ROOT / '_site')], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
atexit.register(server.terminate)
with sync_playwright() as p:
    browser = p.chromium.launch(headless=True)
    page = browser.new_page(viewport={'width':1280, 'height':900})
    errors = []
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.route('https://api.github.com/**', lambda route: route.fulfill(status=403, content_type='application/json', body='{"message":"Rate limit"}'))
    page.goto('http://127.0.0.1:8766', wait_until='networkidle')
    assert page.locator('.task:visible').count() == 22
    page.get_by_role('button', name='Devam eden', exact=True).click()
    assert page.locator('.task:visible').count() == 1
    page.get_by_role('button', name='Doğrulanan', exact=True).click()
    assert page.locator('#empty-tasks').is_visible()
    page.get_by_role('button', name='Karar kapıları', exact=True).click()
    assert page.locator('.task:visible').count() == 6
    page.get_by_role('searchbox').fill('freeze')
    assert page.locator('.task:visible').count() == 3
    page.locator('.roadmap summary').first.click()
    page.locator('.roadmap-task').first.click()
    assert page.locator('#task-1').get_attribute('open') is not None
    assert page.locator('#task-1').is_visible()
    page.get_by_role('button', name='GitHub durumunu yenile').click()
    page.wait_for_function("!document.querySelector('#refresh').disabled")
    assert 'kullanılamıyor' in page.locator('#sync-status').inner_text()
    for width in [320, 390, 768, 1280]:
        page.set_viewport_size({'width':width, 'height':844})
        page.goto('http://127.0.0.1:8766', wait_until='networkidle')
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth'), f'Overflow at {width}'
        page.screenshot(path=str(OUT / f'{width}.png'))
    fixture = json.loads((ROOT / 'site/issue-snapshot.json').read_text())['issues']
    fixture[0]['labels'] = [{'name':'status:blocked'}, {'name':'type:gate'}]
    page.unroute_all()
    page.route('https://api.github.com/**', lambda route: route.fulfill(status=200, content_type='application/json', body=json.dumps(fixture)))
    page.get_by_role('button', name='GitHub durumunu yenile').click()
    page.wait_for_function("!document.querySelector('#refresh').disabled")
    assert page.locator('#task-22 .badge').inner_text() == 'Engelli'
    assert 'yenilendi' in page.locator('#sync-status').inner_text()
    nojs = browser.new_context(java_script_enabled=False, viewport={'width':390,'height':844})
    static = nojs.new_page()
    static.goto('http://127.0.0.1:8766')
    assert static.locator('.task:visible').count() == 22
    assert not errors, errors
    browser.close()
print('PASS: desktop/mobile, filters/search, roadmap links, refresh success/failure and no-JavaScript')
