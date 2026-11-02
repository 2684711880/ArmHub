#!/usr/bin/env python3
"""只读复制已验证工程的官方 SDK 与生命周期修正，不连接设备。"""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = Path('/home/a/Panthera-HT')


def main():
    target = ROOT / 'vendor/panthera/sdk'
    sources = [
        (REFERENCE / 'hardware_ws/src/hightorque_robot', target / 'native'),
        (REFERENCE / 'hardware_ws/deps/root/usr', target / 'deps/usr'),
        (REFERENCE / '.venv_sdk/lib/python3.10/site-packages/pybind11', target / 'build_deps/pybind11'),
        (REFERENCE / 'vendor/Panthera_HT_SDK_Extensions/Panthera-HT_vision_servo_sdk/panthera_python/scripts/Panthera_lib',
         target / 'official_python/Panthera_lib'),
    ]
    for source, destination in sources:
        shutil.copytree(source, destination, symlinks=True, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__', '.git'))
    for source, destination in [
        (REFERENCE / 'hardware_ws/src/python_sdk/bindings.cpp', target / 'bindings.cpp'),
        (REFERENCE / 'hardware_ws/python/hightorque_robot/__init__.py', target / 'python/hightorque_robot/__init__.py'),
        (REFERENCE / 'hardware_ws/config/motors.yaml', target / 'reference_motors.yaml'),
        (REFERENCE / 'hardware_ws/config/python_sdk.yaml', target / 'reference_python_sdk.yaml'),
        (REFERENCE / 'hardware_ws/sdk_commission.patch', target / 'reference_commission.patch'),
    ]:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    lock = {'source_project': str(REFERENCE), 'native_library_and_binding_version': '4.4.7',
            'native_banner_version': '4.6.0',
            'note': 'Official native SDK with existing passive-lifecycle/snapshot-lock fixes; high-level Python copied for reference, not imported for initial commissioning.',
            'files': {}}
    for path in sorted(target.rglob('*')):
        if path.is_file():
            lock['files'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (ROOT / 'deps/panthera_sdk.lock.json').write_text(json.dumps(lock, ensure_ascii=False, indent=2) + '\n')
    print(f'Imported {len(lock["files"])} SDK/dependency files without opening devices.')


if __name__ == '__main__':
    main()
