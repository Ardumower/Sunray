#!/usr/bin/env python3
"""Exercise service.sh without sudo, package installation, or a running robot."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'service.sh'
MOCK = r'''#!/usr/bin/env python3
import json, os, pathlib, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['CALL_LOG'], 'a') as f:
    f.write(json.dumps([name] + args) + '\n')
if name == 'uname':
    print({'-s': 'Linux', '-m': 'aarch64', '-r': '6.1.43'}.get(args[0], 'Linux 6.1.43'))
elif name == 'systemctl' and args[:1] == ['is-active']:
    sys.exit(0 if args[-1] == os.environ.get('ACTIVE_UNIT') else 3)
elif name == 'cmake':
    sys.exit(int(os.environ.get('CMAKE_RESULT', '0')))
elif name == 'sudo':
    if os.environ.get('SUDO_RESULT'):
        sys.exit(int(os.environ['SUDO_RESULT']))
    if args[:1] == ['install']:
        for filename in args[3:-1]:
            p = pathlib.Path(filename)
            (pathlib.Path(os.environ['UNIT_OUTPUT']) / p.name).write_text(p.read_text())
'''


class SimulatorServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name)
        self.log = self.directory / 'calls.jsonl'
        mock = self.directory / 'mock'
        mock.write_text(MOCK)
        mock.chmod(0o755)
        for name in ('uname', 'systemctl', 'sudo', 'cmake', 'journalctl'):
            (self.directory / name).symlink_to(mock)
        self.env = dict(os.environ, PATH=str(self.directory)+os.pathsep+os.environ['PATH'],
                        CALL_LOG=str(self.log), UNIT_OUTPUT=str(self.directory))
        self.env.pop('ACTIVE_UNIT', None)
        self.env.pop('CMAKE_RESULT', None)
        self.env.pop('SUDO_RESULT', None)

    def tearDown(self):
        self.tmp.cleanup()

    def run_action(self, action):
        return subprocess.run(['bash', str(SCRIPT), 'simulator', action],
                              cwd=self.directory, env=self.env, text=True,
                              capture_output=True, timeout=10)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_start_starts_main_dependency_chain(self):
        self.assertEqual(self.run_action('start').returncode, 0)
        self.assertIn(['sudo', 'systemctl', 'start', 'sunray-sim3d'], self.calls())

    def test_real_robot_service_blocks_simulator(self):
        self.env['ACTIVE_UNIT'] = 'sunray'
        self.assertNotEqual(self.run_action('start').returncode, 0)
        self.assertFalse(any(c[0] == 'sudo' for c in self.calls()))

    def test_stop_includes_world_and_display_without_disabling_boot(self):
        self.assertEqual(self.run_action('stop').returncode, 0)
        self.assertEqual(self.calls(), [['sudo', 'systemctl', 'stop', 'sunray-sim3d',
                                        'sunray-sim3d-world', 'sunray-sim3d-display']])

    def test_restart_preserves_world(self):
        self.assertEqual(self.run_action('restart').returncode, 0)
        self.assertIn(['sudo', 'systemctl', 'restart', 'sunray-sim3d'], self.calls())

    def test_failure_is_reported(self):
        self.env['SUDO_RESULT'] = '5'
        self.assertEqual(self.run_action('start').returncode, 5)

    def test_build_selects_cloud_config_and_requires_npu(self):
        result = self.run_action('build')
        self.assertEqual(result.returncode, 0, result.stderr)
        configure, build = [c for c in self.calls() if c[0] == 'cmake']
        self.assertIn('CONFIG_FILE='+str(SCRIPT.parent/'config_sim_ws.h'), configure)
        self.assertIn('SUNRAY_REQUIRE_RKNN=ON', configure)
        self.assertIn(str(SCRIPT.parent/'build-sim3d'), build)
        self.assertFalse(any(c[0] == 'sudo' for c in self.calls()))

    def test_failed_configure_does_not_build(self):
        self.env['CMAKE_RESULT'] = '7'
        self.assertEqual(self.run_action('build').returncode, 7)
        self.assertEqual(len([c for c in self.calls() if c[0] == 'cmake']), 1)

    def test_running_simulator_blocks_build(self):
        self.env['ACTIVE_UNIT'] = 'sunray-sim3d'
        self.assertNotEqual(self.run_action('build').returncode, 0)
        self.assertFalse(any(c[0] == 'cmake' for c in self.calls()))

    def test_install_renders_current_checkout_and_user(self):
        result = self.run_action('install')
        self.assertEqual(result.returncode, 0, result.stderr)
        units = list(self.directory.glob('*.service'))
        self.assertEqual(len(units), 3)
        main = (self.directory/'sunray-sim3d.service').read_text()
        self.assertIn('WorkingDirectory='+str(SCRIPT.parent), main)
        self.assertIn('SupplementaryGroups=render video', main)
        self.assertIn('Environment=LD_LIBRARY_PATH=', main)
        self.assertIn('Wants=sunray-sim3d-world.service', main)

    def run_download(self, payload, checksum, target):
        # Source function definitions only; never run the interactive menu.
        definitions = self.directory / 'definitions.sh'
        definitions.write_text(SCRIPT.read_text().split('# Noninteractive entry point')[0])
        return subprocess.run(['bash', '-c',
                               'source "$1"; simulator_download "$2" "$3" "$4"',
                               'test', str(definitions), payload.as_uri(), checksum, str(target)],
                              cwd=self.directory, env=self.env, capture_output=True,
                              text=True, timeout=10)

    def test_download_verifies_content_and_repairs_corrupt_cache(self):
        payload = self.directory/'source.bin'
        payload.write_bytes(b'verified vendor component')
        target = self.directory/'download.bin'
        target.write_bytes(b'corrupt cache')
        digest = hashlib.sha256(payload.read_bytes()).hexdigest()
        result = self.run_download(payload, digest, target)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(target.read_bytes(), payload.read_bytes())
        payload.unlink()  # Valid cache must work even without the source.
        self.assertEqual(self.run_download(payload, digest, target).returncode, 0)

    def test_wrong_checksum_does_not_replace_existing_download(self):
        payload = self.directory/'source.bin'
        payload.write_bytes(b'unexpected content')
        target = self.directory/'download.bin'
        target.write_bytes(b'previous download')
        self.assertNotEqual(self.run_download(payload, '0'*64, target).returncode, 0)
        self.assertEqual(target.read_bytes(), b'previous download')

    def test_unknown_action_has_no_side_effects(self):
        self.assertEqual(self.run_action('invalid').returncode, 2)
        self.assertEqual(self.calls(), [])


if __name__ == '__main__':
    unittest.main()
