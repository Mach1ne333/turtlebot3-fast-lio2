"""Exercise workspace scripts without network, real builds, or ROS processes."""

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
PIN = 'a4743b095409588842a5b30ddfa27e29d2f99164'
COMMAND_STUB = '''#!/usr/bin/python3
import json, os, pathlib, sys
command = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['TEST_LOG'], 'a') as stream:
    stream.write(json.dumps(dict(command=command, args=args, cwd=os.getcwd())) + '\\n')
if command == 'git':
    if 'rev-parse' in args:
        print(os.environ['TEST_PIN'])
    elif 'apply' in args and '--reverse' in args:
        sys.exit(0 if os.environ.get('TEST_PATCHED', '1') == '1' else 1)
elif command == 'ros2':
    assert args[:2] == ['pkg', 'prefix'], args
    print(json.loads(os.environ['TEST_PREFIXES'])[args[2]])
'''


class WorkspaceScriptsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='tb3-script-tests-')
        self.addCleanup(self.temporary.cleanup)
        self.temp = Path(self.temporary.name)
        self.project = self.temp / 'project with spaces'
        self.scripts = self.project / 'scripts'
        self.scripts.mkdir(parents=True)
        for source in (ROOT / 'scripts').glob('*.sh'):
            shutil.copy2(source, self.scripts / source.name)
        self.workspaces = {
            'TB3_SIM_WS': self.project / 'turtlebot3_ws',
            'TB3_CARTOGRAPHER_WS': self.project / 'mapping_ws/cartographer',
            'TB3_LIO_WS': self.project / 'mapping_ws/fast_lio2',
            'TB3_NAV_WS': self.project / 'tb3_nav_ws',
        }
        variables = {'TB3_PROJECT_ROOT': self.project, **self.workspaces}
        (self.scripts / 'workspaces.bash').write_text(''.join(
            f'{name}={shlex.quote(str(path))}\n' for name, path in variables.items()))
        self.bin = self.temp / 'bin'
        self.bin.mkdir()
        for command in ('git', 'colcon', 'ros2'):
            stub = self.bin / command
            stub.write_text(COMMAND_STUB)
            stub.chmod(0o755)
        (self.scripts / 'humble-base-env.bash').write_text(
            'source "$(dirname "${BASH_SOURCE[0]}")/workspaces.bash"\n'
            f'export PATH={shlex.quote(str(self.bin))}:/usr/bin:/bin\n')
        self.log = self.temp / 'calls.jsonl'
        self.env = dict(os.environ, PATH=f'{self.bin}:/usr/bin:/bin',
                        TEST_LOG=str(self.log), TEST_PIN=PIN)
        prefixes = {
            'turtlebot3_gazebo': '/opt/ros/humble',
            'turtlebot3_description': '/opt/ros/humble',
            'tb3_sim': str(self.workspaces['TB3_SIM_WS'] / 'install/tb3_sim'),
            'cartographer_bringup': str(self.workspaces['TB3_CARTOGRAPHER_WS'] / 'install/cartographer_bringup'),
        }
        for name in ('fast_planner', 'fast_planner_nav2'):
            prefixes[name] = str(self.workspaces['TB3_NAV_WS'] / f'install/{name}')
        self.env['TEST_PREFIXES'] = json.dumps(prefixes)
        for ws in self.workspaces.values():
            install = ws / 'install'
            install.mkdir(parents=True)
            event = json.dumps(dict(command='source', workspace=str(ws)))
            (install / 'local_setup.bash').write_text(
                f'printf "%s\\n" {shlex.quote(event)} >> "$TEST_LOG"\n')
            (install / 'setup.bash').write_text(
                'echo "Unexpected legacy overlay chain" >&2; return 99\n')

    def run_script(self, name, **environment):
        return subprocess.run(
            ['bash', str(self.scripts / name)], cwd=self.temp,
            env={**self.env, **environment}, text=True, capture_output=True, timeout=10)

    def calls(self, command):
        return [entry for line in self.log.read_text().splitlines()
                if (entry := json.loads(line))['command'] == command]

    def test_prepare_is_idempotent_and_never_fetches_official_tb3_sources(self):
        result = self.run_script('prepare.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls('git')
        self.assertEqual(calls[0]['args'], ['submodule', 'update', '--init', '--recursive'])
        self.assertEqual(len(calls), 3)
        for call in calls:
            self.assertFalse({'clone', 'fetch', 'checkout'} & set(call['args']))
        self.assertFalse((self.workspaces['TB3_SIM_WS'] / 'src').exists())

    def test_prepare_applies_missing_fast_lio_patch(self):
        result = self.run_script('prepare.sh', TEST_PATCHED='0')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls('git')
        patch = str(self.workspaces['TB3_LIO_WS'] / 'patches/fast_lio2_gazebo.patch')
        self.assertEqual(calls[-2]['args'][-3:], ['apply', '--check', patch])
        self.assertEqual(calls[-1]['args'][-2:], ['apply', patch])
        self.assertEqual(len(calls), 5)

    def test_prepare_rejects_wrong_upstream_commit(self):
        result = self.run_script('prepare.sh', TEST_PIN='unexpected-commit')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not at tested commit', result.stderr)
        self.assertEqual(len(self.calls('git')), 2)

    def test_full_build_selects_only_tb3_sim_then_sources_local_overlays(self):
        result = self.run_script('build.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        builds = self.calls('colcon')
        expected = list(map(str, self.workspaces.values()))
        self.assertEqual([call['cwd'] for call in builds], expected)
        self.assertEqual(builds[0]['args'][:3], ['build', '--packages-select', 'tb3_sim'])
        for call in builds[1:]:
            self.assertNotIn('--packages-select', call['args'])
        self.assertEqual([call['workspace'] for call in self.calls('source')], expected)

    def test_2d_build_uses_system_resources_and_only_local_simulation_package(self):
        result = self.run_script('build-2d.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call['args'][-1] for call in self.calls('ros2')[:2]],
                         ['turtlebot3_gazebo', 'turtlebot3_description'])
        builds = self.calls('colcon')
        self.assertEqual(len(builds), 3)
        self.assertEqual(builds[0]['args'][:3], ['build', '--packages-select', 'tb3_sim'])
        self.assertIn('cartographer_bringup', builds[1]['args'])
        self.assertEqual(builds[2]['args'][1:4],
                         ['--packages-select', 'fast_planner', 'fast_planner_nav2'])

    def test_shell_syntax(self):
        for source in sorted((ROOT / 'scripts').iterdir()):
            if source.suffix in ('.sh', '.bash'):
                with self.subTest(path=source.name):
                    result = subprocess.run(['bash', '-n', str(source)],
                                            text=True, capture_output=True, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stderr)

    def test_smoke_test_does_not_mix_building_scene_with_demo_maps(self):
        text = (ROOT / 'scripts/smoke-test.sh').read_text()
        self.assertIn('3d-nav scene:=turtlebot3_world gui:=false', text)
        self.assertIn('/humble-base-env.bash', text)
        self.assertNotIn('/install/setup.bash', text)


if __name__ == '__main__':
    unittest.main()
