#!/usr/bin/env python3
"""Check the real make hook against old and CubeIDE-regenerated object lists."""
from pathlib import Path
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[1]
hook = repo / '6storm32-test/makefile.defs'
objects = ['Core/Src/gimbal_' + name + '.o' for name in ('app', 'control', 'motor')]
cases = [
    ('old list without modules', [], [], objects),
    ('bare paths', objects, [], []),
    ('CubeIDE ./ paths', ['./' + obj for obj in objects], [], []),
    ('mixed paths and partial discovery', [objects[0], './' + objects[1]], [], [objects[2]]),
    ('existing user objects', [], ['./' + obj for obj in objects], []),
]

with tempfile.TemporaryDirectory(prefix='gimbal-make-') as directory:
    project = Path(directory)
    source = project / 'Core/Src'
    source.mkdir(parents=True)
    build = project / 'Debug'
    build.mkdir()
    for obj in objects:
        (project / obj).with_suffix('.c').touch()
    for name, generated, user, expected in cases:
        makefile = (
            'OBJS := ' + ' '.join(generated) + '\n'
            'USER_OBJS := ' + ' '.join(user) + '\n'
            'include ' + str(hook) + '\n'
            '.PHONY: check\ncheck:\n'
            '\t@printf "%s\\n" "$(GIMBAL_EXTRA_OBJS)" "$(USER_OBJS)"\n'
        )
        result = subprocess.run(
            ['make', '-s', '-f', '-', 'check'], input=makefile, text=True,
            cwd=build, capture_output=True, check=True,
        )
        lines = result.stdout.splitlines()
        assert lines[0].split() == expected, (name, result.stdout)
        # The linker consumes the generated response file plus USER_OBJS.
        linked = [obj.removeprefix('./') for obj in generated + lines[1].split()]
        assert sorted(linked) == sorted(objects), (name, linked)
        print('PASS:', name)
