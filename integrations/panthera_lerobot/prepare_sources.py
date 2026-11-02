"""Restore pinned official snapshots atomically, then apply the local patch."""
from pathlib import Path
import json
import hashlib
import subprocess
import tempfile
import tarfile

ROOT = Path(__file__).resolve().parent


def main():
    lock = json.loads((ROOT/'sources.lock.json').read_text())
    if hashlib.sha256((ROOT/lock['patch']).read_bytes()).hexdigest() != lock['patch_sha256']:
        raise SystemExit('本地补丁哈希与来源记录不匹配')
    for source in lock['sources']:
        target = ROOT/source['directory']
        if target.exists():
            marker = target/'.armhub-source.json'
            if not marker.is_file() or json.loads(marker.read_text()) != source:
                raise SystemExit(f'来源标记不匹配，保留现有目录供核对：{target}')
            print(f'保留现有固定版本目录：{target}')
            continue
        with tempfile.TemporaryDirectory(prefix='armhub-lerobot-source-') as temp:
            checkout = Path(temp)/'repo'
            subprocess.run(['git','init','-q',str(checkout)], check=True)
            subprocess.run(['git','-C',str(checkout),'fetch','--depth','1',source['url'],source['commit']], check=True)
            archive = Path(temp)/'source.tar'
            subprocess.run(['git','-C',str(checkout),'archive','--output',str(archive),'FETCH_HEAD'], check=True)
            # Same filesystem as target: incomplete extraction never becomes a source directory.
            with tempfile.TemporaryDirectory(prefix='.source-',dir=ROOT) as stage:
                staged = Path(stage)/source['directory']
                staged.mkdir()
                with tarfile.open(archive) as tar:
                    members = [m for m in tar.getmembers() if '__pycache__' not in m.name and '.egg-info' not in m.name]
                    tar.extractall(staged, members=members)
                if source['directory'] == 'Panthera-HT_lerobot':
                    subprocess.run(['patch','-p1','-i',str(ROOT/lock['patch'])], cwd=staged, check=True)
                (staged/'.armhub-source.json').write_text(json.dumps(source,indent=2)+'\n')
                staged.rename(target)


if __name__ == '__main__':
    main()
