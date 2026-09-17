"""Optional browser integration check. Runs synthetic sources, no hardware."""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import tempfile
import threading
import time

from playwright.sync_api import expect, sync_playwright

from nero_pico_data.capture import CaptureController
from nero_pico_data.schema import load_config
from nero_pico_data.server import create_server
from nero_pico_data.storage import DatasetStore
from nero_pico_data.synthetic import SyntheticSource


def main():
    artifacts = Path("artifacts")
    artifacts.mkdir(exist_ok=True)
    config = load_config(Path(__file__).resolve().parents[1] / "config/demo.json")
    with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
        store = stack.enter_context(DatasetStore(directory, config, synthetic=True))
        source = stack.enter_context(SyntheticSource(config))
        controller = stack.enter_context(CaptureController(config, store, source, source.cameras))
        server = create_server(controller, 0)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(executable_path=os.environ.get("PLAYWRIGHT_CHROMIUM_EXECUTABLE"), headless=True,
                    args=["--no-sandbox", "--disable-dev-shm-usage"])
                page = browser.new_page(viewport={"width": 1440, "height": 1000}, device_scale_factor=1)
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                page.goto(f"http://127.0.0.1:{server.server_port}")
                expect(page.locator('#connection')).to_have_text('已就绪')
                expect(page.locator('#start')).to_be_disabled()
                expect(page.locator('#readiness')).to_have_text('任务指令未填写')
                expect(page.locator('#success')).to_be_disabled()
                expect(page.locator('#success-count')).to_have_text('0')
                expect(page.locator('#failure-count')).to_have_text('0')
                expect(page.locator('#capture-fps')).to_have_value('30')
                page.locator('#capture-fps').fill('31')
                expect(page.locator('#apply-fps')).to_be_disabled()
                page.locator('#capture-fps').fill('15')
                page.locator('#apply-fps').click()
                expect(page.locator('#rate')).to_have_text('15 Hz')
                page.locator("#task").fill("将红色方块放入盒子")
                page.keyboard.press('Space')
                page.keyboard.press('l')
                assert controller.status()['state'] == 'idle'
                expect(page.locator('#task')).to_have_value('将红色方块放入盒子 l')
                page.locator('#task').fill('将红色方块放入盒子')
                page.locator('#capture-view h1').click()
                page.keyboard.press('l')
                assert controller.status()['state'] == 'idle'
                expect(page.locator('#start')).to_be_enabled()
                expect(page.locator('#live-cameras img')).to_have_count(2)
                for image in page.locator('#live-cameras img').all():
                    expect(image).to_have_js_property('naturalWidth', 640)
                page.keyboard.down('Space')
                expect(page.locator('#record-state')).to_have_text('录制中')
                page.keyboard.down('Space')
                page.keyboard.up('Space')
                assert controller.status()['state'] == 'recording'
                expect(page.locator('#capture-fps')).to_be_disabled()
                response = page.request.post(f"http://127.0.0.1:{server.server_port}/api/command",
                    headers={"X-Nero-Request": "1"}, data={"command": "configure", "fps": 20})
                assert response.status == 400
                deadline = time.monotonic() + 5
                while int(page.locator('#frame-count').inner_text().split()[0]) < 25 and time.monotonic() < deadline:
                    page.wait_for_timeout(100)
                assert int(page.locator('#frame-count').inner_text().split()[0]) >= 25
                page.locator('#failure').focus()
                page.keyboard.press('Space')
                expect(page.locator('#episode-count')).to_have_text('1')
                expect(page.locator('#success-count')).to_have_text('1')
                expect(page.locator('#failure-count')).to_have_text('0')
                assert controller.status()['last_episode'] == 'episode_01.h5'
                assert json.loads(page.request.get(f'http://127.0.0.1:{server.server_port}/api/episodes').text())[0]['outcome'] == 'success'
                page.screenshot(path=str(artifacts / "workbench-desktop.png"), full_page=True)
                page.locator("#all-episodes").click()
                page.locator(".episode-item").first.click()
                page.locator("#pass").click()
                expect(page.locator('#episode-list .verdict')).to_have_text('PASS')
                page.locator("#play").click()
                page.wait_for_timeout(450)
                assert int(page.locator('#timeline').input_value()) >= 5
                page.locator("#play").click()
                page.screenshot(path=str(artifacts / "workbench-replay.png"), full_page=True)
                for episode in (2, 3):
                    page.locator('[data-view=capture]').click()
                    page.locator('#capture-fps').fill('30')
                    if episode == 2:
                        page.locator('#apply-fps').click()
                    expect(page.locator('#rate')).to_have_text('30 Hz')
                    page.locator('#task').fill(f'将红色方块放入盒子，第 {episode} 段')
                    page.locator('#start').click()
                    expect(page.locator('#record-state')).to_have_text('录制中')
                    if episode == 2:
                        page.locator('[data-view=episodes]').click()
                        expect(page.locator('#select-episodes')).to_be_disabled()
                        expect(page.locator('#delete-episodes')).to_be_disabled()
                        response = page.request.post(f'http://127.0.0.1:{server.server_port}/api/command',
                            headers={'X-Nero-Request': '1'}, data={'command': 'delete', 'episode_ids': ['episode_01.h5']})
                        assert response.status == 400
                        assert (Path(directory) / 'episodes/episode_01.h5').exists()
                        page.keyboard.press('l')
                        assert controller.status()['state'] == 'recording'
                        page.locator('[data-view=capture]').click()
                    deadline = time.monotonic() + 5
                    while int(page.locator('#frame-count').inner_text().split()[0]) < 25 and time.monotonic() < deadline:
                        page.wait_for_timeout(100)
                    assert controller.status()['state'] == 'recording', controller.status()
                    page.locator('#success').click()
                    expect(page.locator('#episode-count')).to_have_text(str(episode))
                    page.locator('#all-episodes').click()
                    page.locator('.episode-item').first.click()
                    page.locator('#pass').click()
                    expect(page.locator('#episode-list .verdict').first).to_have_text('PASS')
                page.locator("[data-view=export]").click()
                repo_id = 'local/nero_pick_place_selected_test'
                page.locator('#repo-id').fill(repo_id)
                page.reload()
                page.locator('[data-view=export]').click()
                expect(page.locator('#repo-id')).to_have_value(repo_id)
                page.locator("#allow-synthetic").check()
                expect(page.locator('#export-episode-list input')).to_have_count(3)
                expect(page.locator('#export-button')).to_be_disabled()
                page.locator('#select-all').check()
                expect(page.locator('#selection-error')).to_be_visible()
                expect(page.locator('#export-button')).to_be_disabled()
                page.locator('#clear-selection').click()
                page.locator('#export-rate-filter').select_option('30')
                expect(page.locator('#export-episode-list input')).to_have_count(2)
                page.locator('#select-all').check()
                expect(page.locator('#export-fps')).to_have_text('30 Hz')
                page.locator('#export-episode-list input').first.uncheck()
                expect(page.locator('.export-name').last).to_have_text('episode_000000')
                page.locator('#export-episode-list input').first.check()
                expect(page.locator('.export-name').first).to_have_text('episode_000000')
                expect(page.locator('.export-name').last).to_have_text('episode_000001')
                page.locator("#export-button").click()
                expect(page.locator('#export-status')).to_contain_text('已导出', timeout=45000)
                assert "2 个片段" in page.locator("#export-status").inner_text()
                expect(page.locator('#export-status')).to_contain_text(repo_id)
                expect(page.locator('#export-status')).to_contain_text('2 个 Parquet · 4 个 MP4')
                export_path = next((Path(directory) / 'exports').glob(f'nero_pick_place_selected_test_*/{repo_id}'))
                report = json.loads((export_path / 'meta/nero_provenance.json').read_text())
                spec = json.loads((export_path / 'meta/nero_openpi.json').read_text())
                assert report['repo_id'] == spec['repo_id'] == repo_id
                assert spec['dataset_root'] == str(export_path)
                assert [item['source'] for item in report['sources']] == ['episode_02.h5', 'episode_03.h5']
                assert len(list((export_path / 'data').rglob('*.parquet'))) == 2
                assert len(list((export_path / 'videos').rglob('*.mp4'))) == 4
                assert not (export_path / 'clips').exists()
                page.reload()
                page.locator('[data-view=export]').click()
                expect(page.locator('#repo-id')).to_have_value(repo_id)
                expect(page.locator('#export-files tbody tr')).to_have_count(2)
                for item in report['sources']:
                    expect(page.locator('#export-files')).to_contain_text(item['source'])
                    expect(page.locator('#export-files')).to_contain_text(item['parquet'])
                    for video in item['videos'].values():
                        expect(page.locator('#export-files')).to_contain_text(video)
                page.screenshot(path=str(artifacts / 'workbench-export-desktop.png'), full_page=True)
                page.locator("[data-view=capture]").click()
                page.set_viewport_size({"width": 390, "height": 844})
                page.wait_for_timeout(400)
                page.screenshot(path=str(artifacts / "workbench-mobile.png"), full_page=True)
                for view in ("capture", "episodes", "export"):
                    page.locator(f"[data-view={view}]").click()
                    assert page.evaluate("document.documentElement.scrollWidth <= window.innerWidth"), view
                    assert page.locator("header").bounding_box()["height"] > 0
                    page.screenshot(path=str(artifacts / f'workbench-{view}-mobile.png'), full_page=True)
                page.locator('[data-view=capture]').click()
                page.locator('#task').fill('等待数据时手动保存')
                page.locator('#start').click()
                expect(page.locator('#record-state')).to_have_text('录制中')
                deadline = time.monotonic() + 5
                while controller.status()['frames'] < 25 and time.monotonic() < deadline:
                    page.wait_for_timeout(100)
                assert controller.status()['frames'] >= 25
                source.stop.set()
                source.worker.join()
                expect(page.locator('#frame-count')).to_contain_text('缺失', timeout=5000)
                expect(page.locator('#record-state')).to_have_text('录制中 · 等待数据')
                for button in ('success', 'failure', 'discard'):
                    expect(page.locator(f'#{button}')).to_be_enabled()
                for width, height, label in ((390, 844, 'mobile'), (1440, 1000, 'desktop')):
                    page.set_viewport_size({'width': width, 'height': height})
                    assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
                    page.screenshot(path=str(artifacts / f'workbench-wait-{label}.png'), full_page=True)
                page.locator('#capture-view h1').click()
                page.keyboard.press('l')
                expect(page.locator('#episode-count')).to_have_text('4')
                expect(page.locator('#success-count')).to_have_text('3')
                expect(page.locator('#failure-count')).to_have_text('1')
                assert controller.status()['state'] == 'idle'
                assert controller.status()['skipped_frames'] > 0
                page.locator('[data-view=episodes]').click()
                page.locator('.episode-item').first.click()
                for identifier in ('episode_02.h5', 'episode_04.h5'):
                    page.get_by_role('checkbox', name=f'选择片段 {identifier}', exact=True).check()
                expect(page.locator('#delete-count')).to_have_text('已选 2 个片段')
                page.locator('#delete-episodes').click()
                expect(page.locator('#delete-dialog')).to_be_visible()
                page.locator('#cancel-delete').click()
                expect(page.locator('#episode-count')).to_have_text('4')
                page.locator('#delete-episodes').click()
                for width, height, label in ((1440, 1000, 'desktop'), (390, 844, 'mobile')):
                    page.set_viewport_size({'width': width, 'height': height})
                    assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
                    page.screenshot(path=str(artifacts / f'workbench-delete-{label}.png'), full_page=True)
                page.locator('#confirm-delete').click()
                expect(page.locator('#episode-count')).to_have_text('2')
                expect(page.locator('#replay')).to_be_hidden()
                expect(page.locator('#delete-count')).to_have_text('已选 0 个片段')
                for identifier in ('episode_02.h5', 'episode_04.h5'):
                    path = Path(directory) / 'episodes' / identifier
                    assert not path.exists() and not path.with_suffix('.review.json').exists()
                assert len(list((export_path / 'data').rglob('*.parquet'))) == 2
                assert len(list((export_path / 'videos').rglob('*.mp4'))) == 4
                page.reload()
                expect(page.locator('#episode-count')).to_have_text('2')
                expect(page.locator('#success-count')).to_have_text('2')
                expect(page.locator('#failure-count')).to_have_text('0')
                source.stop.clear()
                source.__enter__()
                page.locator('[data-view=capture]').click()
                page.locator('#task').fill('删除后继续采集')
                expect(page.locator('#start')).to_be_enabled()
                page.locator('#capture-view h1').click()
                page.keyboard.press('Space')
                expect(page.locator('#record-state')).to_have_text('录制中')
                deadline = time.monotonic() + 5
                while controller.status()['frames'] < 25 and time.monotonic() < deadline:
                    page.wait_for_timeout(100)
                assert controller.status()['frames'] >= 25
                page.keyboard.press('L')
                expect(page.locator('#episode-count')).to_have_text('3')
                expect(page.locator('#success-count')).to_have_text('2')
                expect(page.locator('#failure-count')).to_have_text('1')
                assert controller.status()['last_episode'] == 'episode_05.h5'
                assert not errors, errors
                print(json.dumps({"browser_errors": errors, "desktop": [1440, 1000], "mobile": [390, 844],
                                  "record_review_replay_export": "passed", "manual_finish_during_dropout": "passed",
                                  "keyboard_recording_and_episode_deletion": "passed"}))
                browser.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join()
            server.export_pool.shutdown(wait=True)


if __name__ == "__main__":
    main()
