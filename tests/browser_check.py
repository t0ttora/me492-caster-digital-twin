import json
import os
import subprocess
import atexit
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '_qa'; OUT.mkdir(exist_ok=True)
server = subprocess.Popen(['python3','-m','http.server','8765','--bind','127.0.0.1','--directory',str(ROOT / '_site')], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
atexit.register(server.terminate)
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True)
    page=browser.new_page(viewport={'width':1536,'height':1024}, device_scale_factor=1)
    errors=[]
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.route('https://api.github.com/**', lambda route:route.fulfill(status=403,content_type='application/json',body='{"message":"Rate limit"}'))
    page.goto('http://127.0.0.1:8765',wait_until='networkidle')
    assert 'Caster-aware' in page.title()
    assert page.locator('.week-row:visible').count()==6
    assert 'snapshot' in page.locator('#sync-status').inner_text().lower()
    page.screenshot(path=str(OUT/'desktop.png'),full_page=False)
    page.screenshot(path=str(OUT/'desktop-full.png'),full_page=True)
    page.get_by_role('button',name='Show all 16 weeks').click()
    assert page.locator('.week-row:visible').count()==16
    page.get_by_role('button',name='Verified',exact=True).click()
    assert page.locator('#no-results').is_visible()
    assert page.locator('.week-row:visible').count()==0
    page.get_by_role('button',name='All',exact=True).click()
    page.get_by_role('searchbox').fill('caster')
    assert page.locator('.week-row:visible').count()==1
    assert 'Caster' in page.locator('.week-row:visible').inner_text()
    page.get_by_role('searchbox').fill('')
    page.locator('.gate summary').first.click()
    assert page.locator('.gate[open]').count()==1
    page.locator('.gate summary').first.click()
    assert page.locator('.gate[open]').count()==0
    page.get_by_role('button',name='Refresh GitHub').click()
    page.wait_for_function("!document.querySelector('#refresh').disabled")
    assert 'unavailable' in page.locator('#sync-status').inner_text()
    print('PASS desktop: identity, fallback, show-all, empty filter, search, gate open/close, retry')
    for width in [320,390,768]:
        page.set_viewport_size({'width':width,'height':844})
        page.goto('http://127.0.0.1:8765',wait_until='networkidle')
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth'), f'Overflow at {width}'
        assert page.locator('.scope-stats dt').first.evaluate('(e) => parseFloat(getComputedStyle(e).fontSize)') >= 9
        assert 'Open work. Explicit limits.' in page.locator('.project-links h2').inner_text().replace('\n',' ')
        if width==390:
            page.screenshot(path=str(OUT/'mobile.png'),full_page=False)
            page.screenshot(path=str(OUT/'mobile-full.png'),full_page=True)
        print(f'PASS responsive {width}px: no page overflow')
    # Successful API refresh with synthetic data verifies UI behavior only, not real project progress.
    snapshot=json.loads((ROOT / 'site/issue-snapshot.json').read_text())
    issues=[dict(i,html_url=i['url']) for i in snapshot['issues']]
    issues[0]['labels']=['status:blocked']
    fake_commit={'sha':'a'*40,'html_url':'https://github.com/t0ttora/me492-caster-digital-twin/commit/'+'a'*40,'commit':{'message':'<img src=x onerror=alert(1)> test','committer':{'date':'2026-09-30T00:00:00Z'}}}
    page.unroute_all()
    page.route('https://api.github.com/**',lambda route:route.fulfill(status=200,content_type='application/json',body=json.dumps([fake_commit] if '/commits?' in route.request.url else issues)))
    page.goto('http://127.0.0.1:8765',wait_until='networkidle')
    assert 'Live issue labels' in page.locator('#sync-status').inner_text()
    assert page.locator('[data-issue="22"] .status').inner_text()=='Blocked'
    assert page.locator('#commit-list img').count()==0
    assert '<img' in page.locator('#commit-list').inner_text()
    print('PASS live API fixture: explicit labels and safe text rendering')
    nojs=browser.new_context(java_script_enabled=False,viewport={'width':390,'height':844})
    static=nojs.new_page();static.goto('http://127.0.0.1:8765')
    assert static.locator('.week-row:visible').count()==16
    assert static.locator('.reports').get_by_role('link',name='Execution baseline').is_visible()
    print('PASS no-JavaScript: all 16 rows and report links remain available')
    assert not errors, errors
    print('PASS no uncaught browser errors')
    browser.close()
