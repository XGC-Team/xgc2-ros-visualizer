"""Exercise the existing builder's SDK selection without compiling dependencies."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class PublishedSdkConsumption(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        # Capture the real Docker payload; execute only its dependency selection
        # with fake apt. No containers, network, compiler or Cargo is started.
        docker = self.bin / 'docker'
        docker.write_text('''#!/usr/bin/env python3
import json, os, pathlib, subprocess, sys
body = sys.stdin.read()
log = pathlib.Path(os.environ['SDK_TEST_LOG'])
with log.open('a') as out:
    out.write(json.dumps({'args':sys.argv[1:], 'body':body})+'\\n')
env = dict(os.environ)
for index, arg in enumerate(sys.argv):
    if arg == '-e' and '=' in sys.argv[index+1]:
        key, value = sys.argv[index+1].split('=', 1)
        env[key] = value
if 'sdk_packages=()' in body:
    selection = body.split('sdk_packages=()',1)[1].split('chown ',1)[0]
    result = subprocess.run(['bash','-ec','sdk_packages=()'+selection], env=env)
    if result.returncode: sys.exit(result.returncode)
    start = body.index('if [[ "$XGC2_EXTERNAL_ZENOHC" == 1 ]]; then')
    end = body.index('ros_prefix_path=', start)
    selection = body[start:end]+'printf "%s" "$sdk_prefix"'
    result = subprocess.run(['bash','-ec',selection],env=env,capture_output=True)
    pathlib.Path(os.environ['SDK_TEST_PREFIX']).write_bytes(result.stdout)
    sys.exit(result.returncode)
''')
        docker.chmod(0o755)
        apt = self.bin / 'apt-get'
        apt.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$SDK_TEST_APT"\nexit "${SDK_TEST_APT_EXIT:-0}"\n')
        apt.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ['PATH'],
                        SDK_TEST_LOG=str(self.root/'docker.jsonl'),
                        SDK_TEST_APT=str(self.root/'apt.txt'), SDK_TEST_PREFIX=str(self.root/'prefix'))
        self.env.pop('XGC2_ZENOHC_PREFIX', None)

    def build(self, *args):
        return subprocess.run(['bash', str(ROOT/'.xgc2/scripts/build_debs_in_docker.sh'),
                               '--work-dir', str(self.root/'work'), '--output-dir', str(self.root/'out'), *args],
                              env=self.env, capture_output=True, text=True)

    def calls(self):
        return [json.loads(line) for line in (self.root/'docker.jsonl').read_text().splitlines()]

    def test_default_installs_exact_production_sdk_without_source_build(self):
        result = self.build()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        self.assertEqual(len(calls), 2)
        self.assertIn('xgc2-zenoh=1.8.0-1~focal', calls[0]['body'])
        self.assertIn('xgc2-zenoh=1.8.0-1~focal', (self.root/'apt.txt').read_text())
        self.assertEqual((self.root/'prefix').read_text(), '/opt/xgc2/zenoh')
        for forbidden in ('cargo', 'rustup', 'build_zenoh_sdk', 'codeload.github.com'):
            self.assertNotIn(forbidden, calls[0]['body'])
        self.assertIn('https://xgc2.apt.xiaokang.ink focal main', calls[0]['body'])

    def test_offline_prefix_is_read_only_and_does_not_install_another_sdk(self):
        sdk = self.root/'offline-sdk'
        (sdk/'include').mkdir(parents=True)
        (sdk/'lib').mkdir()
        (sdk/'include/zenoh.h').touch()
        (sdk/'lib/libzenohc.a').touch()
        result = self.build('--zenoh-prefix', str(sdk))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(sdk)+':/zenoh-sdk:ro', self.calls()[0]['args'])
        self.assertNotIn('xgc2-zenoh', (self.root/'apt.txt').read_text())
        self.assertEqual((self.root/'prefix').read_text(), '/zenoh-sdk')

    def test_missing_published_dependency_fails_without_source_fallback(self):
        self.env['SDK_TEST_APT_EXIT'] = '42'
        result = self.build()
        self.assertEqual(result.returncode, 42, result.stderr)
        self.assertEqual(len(self.calls()), 1)
        self.assertFalse((self.root/'prefix').exists())

    def test_build_metadata_matches_static_dependency_consumption(self):
        package = ET.parse(ROOT/'package.xml').getroot()
        dependency = package.find("build_depend[.='xgc2-zenoh']")
        self.assertIsNotNone(dependency)
        self.assertEqual(dependency.get('version_eq'), '1.8.0-1~focal')
        self.assertIsNone(package.find("exec_depend[.='xgc2-zenoh']"))
        self.assertIn('find_package(zenohc 1.8.0 EXACT CONFIG REQUIRED)', (ROOT/'CMakeLists.txt').read_text())
        self.assertFalse((ROOT/'.xgc2/scripts/build_zenoh_sdk.sh').exists())
        self.assertFalse((ROOT/'.xgc2/Dockerfile.zenoh-sdk').exists())
        self.assertFalse(any((ROOT/'.xgc2/zenoh-c-1.8.0').glob('*')))
