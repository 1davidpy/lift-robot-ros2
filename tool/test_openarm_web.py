#!/usr/bin/env python3
"""Browser regression using HTTP state fixtures; never sends robot commands."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
with sync_playwright() as p:
    browser = p.chromium.launch(headless=True)
    page = browser.new_page()
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto('http://127.0.0.1:8000')
    expect(page.locator("#arm-joints tr")).to_have_count(16)
    live = page.request.get('http://127.0.0.1:8000/api/state').json()
    print('Live:', page.locator('#arms-detail').inner_text(), page.locator('#arm-phase').inner_text())
    fixture = json.loads(json.dumps(live))
    fixture['arms'] = dict(fresh=True, valid=[True]*8+[False]*8, position=[1.25]*16,
                           velocity=[0.0]*16, effort=[0.0]*16, mos_temperature=[30]*16,
                           rotor_temperature=[31]*16, enabled=False, phase='monitor_only')
    page.route('**/api/state', lambda route: route.fulfill(json=fixture))
    expect(page.locator("#arms-detail")).to_contain_text("左臂 0/8 · 右臂 8/8")
    assert page.locator('#arm-joints tr').nth(0).locator('td').nth(3).inner_text() == '—'
    assert page.locator('#arm-joints tr').nth(8).locator('td').nth(3).inner_text() == '1.250'
    fixture['arms']['fresh'] = False
    expect(page.locator("#arm-phase")).to_have_text("采集节点离线")
    assert all(t == '—' for t in page.locator('#arm-joints tr td:nth-child(4)').all_inner_texts())
    page.unroute('**/api/state')
    page.route('**/api/state', lambda route: route.abort())
    expect(page.locator("#arm-joints")).to_contain_text("反馈已失效")
    assert not errors, errors
    browser.close()
print('PASS: left/right mapping, joint values, stale feedback and HTTP disconnect')
