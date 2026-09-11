#!/usr/bin/env python3
"""Live lifecycle acceptance. Starts acquisition/programs, NEVER enables motion.
Stops/restarts only the configured absent left camera; leaves other devices running.
"""
import json, time
from pathlib import Path
from playwright.sync_api import sync_playwright, expect
ROOT=Path(__file__).resolve().parents[1]
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True)
 page=browser.new_page(viewport={"width":1440,"height":1000})
 errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 page.goto('http://127.0.0.1:8000')
 expect(page.locator('[data-device]')).to_have_count(7)
 expect(page.locator('[data-device="arms"] [data-service-action="toggle"]')).to_be_disabled()
 assert page.request.post('http://127.0.0.1:8000/api/devices/arms',data={'action':'restart'}).status==403
 token=(ROOT/'log/web-control.token').read_text().strip()
 page.locator('#token').fill(token);page.locator('#unlock').click()
 expect(page.locator('#auth')).to_have_text('可操作')
 for id in ['program','left_wrist_rgb','head_rgb','right_wrist_rgb']:
  box=page.locator(f'[data-device="{id}"]')
  toggle=box.locator('[data-service-action="toggle"]')
  expect(toggle).to_be_enabled(timeout=15000)
  if toggle.get_attribute('aria-checked')=='false': toggle.click()
  expect(toggle).to_have_attribute('aria-checked','true',timeout=15000)
  if id=='left_wrist_rgb':
   box.locator('[data-service-action="restart"]').click()
   expect(toggle).to_have_attribute('aria-checked','true',timeout=20000)
   expect(box.locator('.service-state')).to_have_text('服务运行中',timeout=20000)
   expect(toggle).to_be_enabled(timeout=15000)
   toggle.click()
   expect(toggle).to_have_attribute('aria-checked','false',timeout=20000)
 headers={'Authorization':'Bearer '+token}
 assert page.request.post('http://127.0.0.1:8000/api/devices/arms',headers=headers,data={'action':'restart; id'}).status==400
 assert page.request.post('http://127.0.0.1:8000/api/devices/unknown',headers=headers,data={'action':'start'}).status==400
 samples=[]
 for _ in range(60):
  s=page.request.get('http://127.0.0.1:8000/api/state').json()
  assert not s.get('arms',{}).get('enabled',False)
  assert not s.get('base_status',{}).get('enabled',False)
  samples.append({'arms':s.get('arms',{}).get('fresh',False),'valid_joints':sum(s.get('arms',{}).get('valid',[])), 'base':s.get('base_status',{}).get('feedback_fresh',False),'program':s['system']['services']['control'],'cameras':{k:v['fresh'] for k,v in s['cameras'].items()}})
  page.wait_for_timeout(1000)
 page.locator('#token').fill('')
 page.screenshot(path=str(ROOT/'log/device-management.png'),full_page=True)
 assert not errors,errors
 (ROOT/'log/device-management-test.json').write_text(json.dumps(samples,indent=2))
 print('PASS: login, seven device controls, start/restart/stop, unauthorized/invalid requests rejected, no motor enable, no browser errors')
 print('Fresh sample counts:',{k:sum(x[k] for x in samples) for k in ['arms','base','program']})
 print('Camera fresh sample counts:',{k:sum(x['cameras'][k] for x in samples) for k in samples[0]['cameras']})
 browser.close()
