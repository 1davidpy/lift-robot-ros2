#!/usr/bin/env python3
"""Isolated fake-SDK recovery and browser preview test. No physical camera opened."""
import json, os, pathlib, signal, subprocess, tempfile, time, urllib.request
from playwright.sync_api import sync_playwright, expect
ROOT = pathlib.Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
processes = []
def launch(package, binary, *args):
    p = subprocess.Popen([str(ROOT/'build/install'/package/'lib'/package/binary), '--ros-args', *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    processes.append(p)
    return p
def state():
    return json.load(urllib.request.urlopen('http://127.0.0.1:18022/api/state'))
def until(predicate, timeout=20):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        try:
            if predicate(): return
        except Exception: pass
        time.sleep(.2)
    raise AssertionError('condition timed out')
try:
    with tempfile.TemporaryDirectory(prefix='luxi-rgb-test-') as tmp:
        sdk=pathlib.Path(tmp)/'sdk.so'
        launch('luxi_web_ctrl','web_ctrl','-p','port:=18022','-p','directory:='+tmp)
        camera=launch('luxi_device_adapters','hikrobot_camera','-p','mock:=false','-p','serial:=test','-p','sdk_library:='+str(sdk))
        until(lambda: state()['cameras']['head_rgb'].get('driver',{}).get('phase')=='error')
        with sync_playwright() as p:
            browser=p.chromium.launch(headless=True)
            page=browser.new_page(); errors=[]
            page.on('pageerror',lambda e:errors.append(str(e)))
            page.goto('http://127.0.0.1:18022')
            expect(page.locator('#head_rgb-meta')).to_contain_text('SDK')
            sdk.symlink_to(ROOT/'build/packages/luxi_device_adapters/libfake_mv3d.so')
            until(lambda: state()['cameras']['head_rgb']['fresh'])
            expect(page.locator('#head_rgb')).to_be_visible(timeout=15000)
            until(lambda: page.locator('#head_rgb').evaluate('e => e.complete && e.naturalWidth === 640'))
            jpeg=urllib.request.urlopen('http://127.0.0.1:18022/api/cameras/head_rgb').read()
            assert jpeg[:2]==b'\xff\xd8'
            page.locator('#preview').uncheck(); expect(page.locator('#head_rgb')).to_be_hidden()
            page.locator('#preview').check(); expect(page.locator('#head_rgb')).to_be_visible()
            camera.send_signal(signal.SIGINT);camera.wait(timeout=10)
            until(lambda: not state()['cameras']['head_rgb']['fresh'])
            expect(page.locator('#head_rgb')).to_be_hidden()
            launch('luxi_device_adapters','hikrobot_camera','-p','mock:=false','-p','serial:=luxi_test_absent_usb','-p','require_usb:=true','-p','sdk_library:='+str(sdk))
            until(lambda: 'USB 相机未连接' in state()['cameras']['head_rgb'].get('driver',{}).get('reason',''))
            assert not state()['cameras']['head_rgb']['fresh']
            assert not errors,errors
            browser.close()
        print('PASS missing SDK, automatic recovery, JPEG preview, pause/resume, stale image hiding, absent USB guard')
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:p.kill();p.wait()
