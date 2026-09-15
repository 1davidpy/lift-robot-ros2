#!/usr/bin/env python3
"""Isolated mock ROS + real browser WASD regression. Requires Playwright Chromium."""
import json, os, pathlib, signal, subprocess, threading, time, urllib.request
import rclpy
from std_msgs.msg import String
from playwright.sync_api import sync_playwright

ROOT = pathlib.Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID'] = '86'
processes = []
def launch(package, binary, *args):
    log = open(ROOT / 'log' / ('keyboard-test-' + binary + '.log'), 'w')
    p = subprocess.Popen([str(ROOT / 'build/install' / package / 'lib' / package / binary), '--ros-args', *args], stdout=log, stderr=log)
    processes.append((p, log))
def state():
    return json.load(urllib.request.urlopen('http://127.0.0.1:8001/api/state'))
def command(cmd, detail=''):
    req = urllib.request.Request('http://127.0.0.1:8001/api/keyboard', json.dumps({'command':cmd, 'detail':detail}).encode(), {'Content-Type':'application/json'})
    return json.load(urllib.request.urlopen(req))
messages = []
try:
    launch('luxi_device_adapters', 'r680_adapter', '-p', 'mock:=true')
    launch('luxi_web_ctrl', 'web_ctrl', '-p', 'port:=8001', '-p', 'base_enabled:=true', '-p', 'no_cameras:=true', '-p', 'directory:=' + str(ROOT / 'build/test_runs/keyboard'))
    rclpy.init()
    node = rclpy.create_node('keyboard_probe')
    sub = node.create_subscription(String, '/luxi/base/applied', lambda m: messages.append(json.loads(m.data)), 100)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    for _ in range(100):
        try:
            if state()['base']['valid']: break
        except Exception: pass
        time.sleep(.1)
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto('http://127.0.0.1:8001')
        page.locator('#keyboard-start').wait_for()
        page.wait_for_function("!document.getElementById('keyboard-start').disabled")
        page.click('#keyboard-start')
        page.wait_for_timeout(150)
        assert page.locator('#keyboard-start').get_attribute('aria-checked') == 'true'
        page.click('#keyboard-start')
        page.wait_for_timeout(300)
        assert not state()['base_status']['enabled']
        assert page.locator('#keyboard-start').get_attribute('aria-checked') == 'false'
        page.click('#keyboard-start')
        page.wait_for_timeout(150)
        print('PASS token-free keyboard switch on/off/on', flush=True)
        for key, axis, sign in [('w',0,1),('s',0,-1),('a',1,1),('d',1,-1)]:
            start = len(messages)
            page.keyboard.down(key)
            page.wait_for_timeout(400)
            page.keyboard.up(key)
            page.wait_for_timeout(130)
            seen = messages[start:]
            assert any(m['velocity'][axis]*sign >= .04 for m in seen), (key, seen[-3:])
            assert all(abs(m['velocity'][1-axis]) < .001 and abs(m['velocity'][2]) < .001 for m in seen)
            assert messages[-1]['velocity'] == [0,0,0], messages[-1]
            print('PASS', key, 'direction and release stop', flush=True)
        page.keyboard.down('w'); page.keyboard.down('d'); page.wait_for_timeout(300)
        assert max((m['velocity'][0]**2+m['velocity'][1]**2)**.5 for m in messages) <= .051
        page.evaluate("window.dispatchEvent(new Event('blur'))")
        page.wait_for_timeout(250)
        assert not state()['base_status']['enabled'] and messages[-1]['velocity'] == [0,0,0]
        print('PASS diagonal limit and blur disarm', flush=True)
        assert not errors, errors
        browser.close()
    command('keyboard_start')
    command('keyboard', '[1,0]')
    time.sleep(.4)
    assert not state()['base_status']['enabled'] and messages[-1]['velocity'] == [0,0,0]
    assert state()['base_status']['reason'] == 'keyboard_timeout'
    print('PASS server watchdog without browser heartbeats', flush=True)
    try: command('keyboard', '[1,0]'); raise AssertionError('expired command accepted')
    except urllib.error.HTTPError as e: assert e.code == 409
    print('PASS no automatic re-enable after timeout', flush=True)
finally:
    for proc, log in reversed(processes):
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=10)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        log.close()
