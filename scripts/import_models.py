#!/usr/bin/env python3
"""从参考工程复制最小模型快照。只读来源，不启动任何旧工程节点。"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def copy_file(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def main():
    panthera = Path('/home/a/Panthera-HT/vendor/Panthera-HT_ROS2')
    nero = Path('/home/a/Nero-7/sim_ws/src/agx_arm_sim')
    descriptions = [
        (panthera / 'src/panthera_ht_description', ROOT / 'vendor/panthera/description', ['urdf', 'meshes']),
        (nero / 'agx_arm_description/agx_arm_urdf/nero', ROOT / 'vendor/agilex/nero', ['meshes']),
    ]
    for source, target, directories in descriptions:
        for directory in directories:
            shutil.copytree(source / directory, target / directory, dirs_exist_ok=True)
    copy_file(panthera / 'LICENSE', ROOT / 'vendor/panthera/LICENSE')
    copy_file(nero / 'agx_arm_description/urdf/nero_gripper_d435.urdf',
              ROOT / 'vendor/agilex/nero/robot_with_camera.urdf')
    # 单臂原始语义与规划参数作为来源快照保存。
    for robot, source in [
        ('panthera', panthera / 'src/panthera_moveit_config/config'),
        ('agilex', nero / 'Moveit2/nero_gripper_moveit_config/config'),
    ]:
        shutil.copytree(source, ROOT / f'vendor/{robot}/moveit_reference', dirs_exist_ok=True)
    # AgileX 许可沿用旧工程保留的上游说明；同时登记真实文件出处。
    license_files = list(nero.rglob('LICENSE*'))
    for index, source in enumerate(license_files):
        if source.is_file():
            copy_file(source, ROOT / f'vendor/agilex/licenses/{index}_{source.name}')
    manifest = {'sources': [], 'files': {}}
    for name, path in [('Panthera-HT_ROS2', panthera), ('agx_arm_sim', nero)]:
        manifest['sources'].append({
            'name': name, 'local_source': str(path),
            'origin': subprocess.check_output(['git', '-C', str(path), 'remote', 'get-url', 'origin'], text=True).strip(),
            'commit': subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip(),
            'working_tree_changes': subprocess.check_output(['git', '-C', str(path), 'status', '--short'], text=True),
        })
    for path in sorted((ROOT / 'vendor').rglob('*')):
        if path.is_file():
            manifest['files'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (ROOT / 'deps').mkdir(exist_ok=True)
    (ROOT / 'deps/models.lock.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f'Imported {len(manifest["files"])} resource files; old projects unchanged.')


if __name__ == '__main__':
    main()
