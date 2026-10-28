#!/usr/bin/env python3
"""Config-driven installation checks without packages, sudo or robot services."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'service.sh'
SOURCE = SCRIPT.read_text()
START = SOURCE.index('setup_configured_direct_link() {')
FUNCTION = SOURCE[START:SOURCE.index('\n}\n', START)+3]

class DirectSetupTests(unittest.TestCase):
    def run_config(self, contents, inherited=None, fail=False, cache=False):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            (root/'sunray').mkdir(); (root/'linux/build').mkdir(parents=True)
            config=root/'sunray/config.h';config.write_text(contents)
            if inherited is not None: (root/'sunray/base.h').write_text(inherited)
            if cache:
                chosen=root/'chosen.h';chosen.write_text('#define DIRECT_LINK_ENABLE true\n')
                (root/'linux/build/CMakeCache.txt').write_text('CONFIG_FILE:UNINITIALIZED='+str(chosen)+'\n')
            env=dict(os.environ,SIM_ROOT=d,SCRIPT_DIR=str(root/'linux'),SETUP_RESULT='7' if fail else '0')
            code=FUNCTION+'\nbash() { test "$2" = direct-setup || return 99; echo "INSTALL:$3"; return "$SETUP_RESULT"; }\nsetup_configured_direct_link\n'
            return subprocess.run(['bash','-c',code],env=env,text=True,capture_output=True)
    def test_enabled_installs_for_normal_service(self):
        r=self.run_config('#define DIRECT_LINK_ENABLE true\n');self.assertEqual(r.returncode,0);self.assertIn('INSTALL:sunray',r.stdout)
    def test_inherited_option_is_resolved(self):
        r=self.run_config('#include "base.h"\n','#define DIRECT_LINK_ENABLE 1\n');self.assertEqual(r.returncode,0);self.assertIn('INSTALL:sunray',r.stdout)
    def test_disabled_missing_and_override_skip_packages(self):
        for config in ('', '#define DIRECT_LINK_ENABLE false\n', '#include "base.h"\n#undef DIRECT_LINK_ENABLE\n#define DIRECT_LINK_ENABLE 0\n'):
            r=self.run_config(config,'#define DIRECT_LINK_ENABLE true\n');self.assertEqual(r.returncode,0);self.assertNotIn('INSTALL:',r.stdout)
    def test_selected_normal_build_overrides_shared_header(self):
        r=self.run_config('#define DIRECT_LINK_ENABLE false\n',cache=True);self.assertIn('INSTALL:sunray',r.stdout)
    def test_invalid_config_fails_instead_of_silently_skipping(self):
        r=self.run_config('#include "missing.h"\n');self.assertNotEqual(r.returncode,0);self.assertNotIn('INSTALL:',r.stdout)
    def test_install_failure_is_propagated(self):
        r=self.run_config('#define DIRECT_LINK_ENABLE true\n',fail=True);self.assertEqual(r.returncode,7)

if __name__=='__main__': unittest.main()
