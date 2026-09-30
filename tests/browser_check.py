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
    page.goto('http://127.0.0.1:8766/tasks.html', wait_until='networkidle')
    assert page.locator('.task:visible').count() == 22
    page.locator('#task-1 > summary').click()
    assert page.locator('#task-inspector').is_visible()
    page.get_by_role('button', name='Next task →', exact=True).click()
    assert 'Arche technical visit' in page.locator('#inspector-title').inner_text()
    page.keyboard.press('Escape')
    assert not page.locator('#task-inspector').is_visible()
    page.get_by_role('button', name='In progress', exact=True).click()
    assert page.locator('.task:visible').count() == 1
    page.get_by_role('button', name='Verified', exact=True).click()
    assert page.locator('#empty-tasks').is_visible()
    page.get_by_role('button', name='Decision gates', exact=True).click()
    assert page.locator('.task:visible').count() == 6
    page.get_by_role('searchbox').fill('freeze')
    assert page.locator('.task:visible').count() == 3
    page.goto('http://127.0.0.1:8766/roadmap.html')
    page.locator('.roadmap-task').first.click()
    assert page.locator('#task-inspector').is_visible()
    assert 'Scope and preparation' in page.locator('#inspector-title').inner_text()
    page.get_by_role('button', name='Close task details', exact=True).click()
    page.goto('http://127.0.0.1:8766/progress.html')
    page.get_by_role('button', name='Refresh GitHub status').click()
    page.wait_for_function("!document.querySelector('#refresh').disabled")
    assert 'unavailable' in page.locator('#sync-status').inner_text()
    for width in [320, 390, 768, 1280]:
        page.set_viewport_size({'width':width, 'height':844})
        page.goto('http://127.0.0.1:8766/tasks.html', wait_until='networkidle')
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth'), f'Overflow at {width}'
        if width <= 760:
            assert not page.locator('#site-navigation').is_visible()
            page.locator('.nav-toggle').click()
            assert page.locator('#site-navigation').is_visible()
            assert page.locator('#site-navigation a').count() == 10
            page.get_by_role('link', name='Roadmap', exact=True).press('Escape')
            assert page.locator('.nav-toggle').get_attribute('aria-expanded') == 'false'
        else:
            assert page.locator('#site-navigation').is_visible()
        page.screenshot(path=str(OUT / f'{width}.png'))
    fixture = json.loads((ROOT / 'site/issue-snapshot.json').read_text())['issues']
    fixture[0]['labels'] = [{'name':'status:blocked'}, {'name':'type:gate'}]
    page.unroute_all()
    page.route('https://api.github.com/**', lambda route: route.fulfill(status=200, content_type='application/json', body=json.dumps(fixture)))
    page.goto('http://127.0.0.1:8766/progress.html')
    page.get_by_role('button', name='Refresh GitHub status').click()
    page.wait_for_function("!document.querySelector('#refresh').disabled")
    assert 'refreshed' in page.locator('#sync-status').inner_text()
    page.goto('http://127.0.0.1:8766/report-ME492_Timetable_EN.html')
    page.get_by_text('Page 1 of 2', exact=True).wait_for()
    page.get_by_role('button', name='Next page', exact=True).click()
    page.get_by_text('Page 2 of 2', exact=True).wait_for()
    page.locator('.pdf-sheet[data-page="2"][data-ready="true"]').wait_for()
    assert page.locator('.pdf-sheet').count() == 2
    assert page.evaluate('document.documentElement.scrollHeight <= innerHeight')
    page.get_by_role('button', name='Focus view', exact=True).click()
    assert page.locator('body').get_attribute('class') == 'page-reader reader-focus'
    page.keyboard.press('Escape')
    assert page.locator('body').get_attribute('class') == 'page-reader'
    nojs = browser.new_context(java_script_enabled=False, viewport={'width':390,'height':844})
    static = nojs.new_page()
    static.goto('http://127.0.0.1:8766/tasks.html')
    assert static.locator('.task:visible').count() == 22
    assert not errors, errors
    browser.close()
print('PASS: desktop/mobile, filters/search, roadmap links, refresh success/failure and no-JavaScript')
