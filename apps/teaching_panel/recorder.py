"""Bounded asynchronous raw recording, review and official LeRobot export."""
from datetime import datetime
import json
import fcntl
from pathlib import Path
import queue
import re
import threading
import time
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
NAMES=[f'joint{i}.pos' for i in range(1,7)]+['gripper.pos']


def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path)


def read_episode(folder):
    folder=Path(folder)
    meta=json.loads((folder/'episode.json').read_text())
    rows=[json.loads(s) for s in (folder/'samples.jsonl').read_text().splitlines() if s.strip()]
    return meta,rows


def _export_episode(folder, dataset_base=None):
    """Called after approval, outside the SDK owner. Failed/demo sources stay distinct."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    folder=Path(folder);meta,rows=read_episode(folder)
    if meta['outcome']!='success' or not meta['quality_ok']:
        raise ValueError('只有人工标记成功且采样检查通过的回合可以进入训练集')
    if meta.get('exported'):
        return meta['exported']
    if not rows:raise ValueError('空回合不可导出')
    base=Path(dataset_base) if dataset_base is not None else ROOT/'datasets'/('demo_lerobot' if meta['source']=='demo' else 'lerobot')
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+',meta['repo_id']):raise ValueError('数据集名称无效')
    root=base/meta['repo_id']
    names=NAMES
    first=np.asarray(Image.open(folder/'images'/rows[0]['image']),dtype=np.uint8)
    features={
        'observation.state':{'dtype':'float32','shape':(7,),'names':names},
        'action':{'dtype':'float32','shape':(7,),'names':names},
        'observation.images.d435':{'dtype':'video','shape':tuple(first.shape),
                                   'names':['height','width','channels']},
    }
    provenance=dict(source=meta['source'],joint_units='rad',gripper_units='raw_motor_rad',
                    action_semantics='six measured teaching joint positions plus commanded gripper target',
                    camera='d435',image_shape=list(first.shape),fps=meta['fps'])
    pending=root/'teaching_export_pending.json'
    if pending.exists():raise RuntimeError(f'此数据集上次导出未完整结束，保留原始回合并拒绝继续追加：{root}')
    ledger_path=root/'teaching_exports.json'
    ledger=json.loads(ledger_path.read_text()) if ledger_path.exists() else {}
    identity=str(folder.resolve())
    if identity in ledger:
        meta.update(exported=ledger[identity],status='exported');atomic_json(folder/'episode.json',meta)
        return ledger[identity]
    exists=(root/'meta/info.json').is_file()
    if root.exists() and not exists:
        raise ValueError(f'目标目录不是完整 LeRobot 数据集，请换数据集名称：{root}')
    if exists:
        origin=root/'teaching_provenance.json'
        if not origin.exists() or json.loads(origin.read_text())!=provenance:
            raise ValueError('已有数据集的来源、FPS 或图像规格不同，请使用新的数据集名称')
        ds=LeRobotDataset(meta['repo_id'],root=root,video_backend='pyav')
        index=ds.meta.total_episodes
    else:
        ds=LeRobotDataset.create(meta['repo_id'],fps=meta['fps'],root=root,
            robot_type='panthera_follower',features=features,use_videos=True,video_backend='pyav')
        index=0
        atomic_json(root/'teaching_provenance.json',provenance)
    atomic_json(pending,dict(raw_episode=identity,episode_index=index))
    try:
        for row in rows:
            image=np.asarray(Image.open(folder/'images'/row['image']).convert('RGB'),dtype=np.uint8)
            ds.add_frame({'observation.state':np.array(row['observation'],np.float32),
                          'action':np.array(row['action'],np.float32),
                          'observation.images.d435':image,'task':meta['task']})
        ds.save_episode(parallel_encoding=False)
    finally:
        ds.finalize()
    result=dict(root=str(root),episode_index=index,frames=len(rows))
    ledger[identity]=result;atomic_json(ledger_path,ledger)
    pending.unlink()
    meta['exported']=result;meta['status']='exported'
    atomic_json(folder/'episode.json',meta)
    return result


def export_episode(folder, dataset_base=None):
    folder=Path(folder);meta,_=read_episode(folder)
    base=Path(dataset_base) if dataset_base is not None else ROOT/'datasets'/('demo_lerobot' if meta['source']=='demo' else 'lerobot')
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+',meta['repo_id']):raise ValueError('数据集名称无效')
    root=base/meta['repo_id']
    root.parent.mkdir(parents=True,exist_ok=True)
    with (root.parent/('.'+root.name+'.export.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return _export_episode(folder,dataset_base)


class EpisodeStore(threading.Thread):
    def __init__(self,emit,base=None,dataset_base=None):
        super().__init__(name='teaching-disk-writer',daemon=False)
        self.emit=emit;self.base=Path(base) if base else ROOT/'datasets/teaching_raw'
        self.dataset_base=dataset_base
        self.queue=queue.Queue(maxsize=12);self.exit_event=threading.Event()
        self.active=False;self.busy=False;self.folder=None;self.meta=None;self.stream=None
        self.count=0;self.first_stamp=None;self.last_stamp=None;self.max_gap=0.

    def post(self,kind,value=None):
        try:self.queue.put_nowait((kind,value));return True
        except queue.Full:return False

    def shutdown(self):self.exit_event.set()

    def run(self):
        try:
            while not self.exit_event.is_set() or not self.queue.empty():
                try:kind,value=self.queue.get(timeout=.05)
                except queue.Empty:continue
                try:
                    self.handle(kind,value)
                except Exception as exc:
                    try:
                        if self.stream:self.stream.close()
                    except Exception:pass
                    self.stream=None;self.active=False;self.busy=False
                    self.emit('writer_error',str(exc))
                    try:
                        if kind=='decide':
                            folder=Path(value[0]);meta,_=read_episode(folder)
                            meta.update(status='review',export_error=str(exc))
                            atomic_json(folder/'episode.json',meta)
                            self.emit('review',dict(folder=str(folder),**meta))
                        elif self.meta is not None and self.folder is not None:
                            self.meta.update(status='error',quality_ok=False,error=str(exc),frames=self.count)
                            atomic_json(self.folder/'episode.json',self.meta)
                            if (self.folder/'samples.jsonl').is_file():
                                self.emit('review',dict(folder=str(self.folder),**self.meta))
                    except Exception as persist_error:
                        self.emit('writer_error',f'原始文件保留；错误状态写入失败：{persist_error}')
                finally:self.queue.task_done()
        finally:
            if self.stream:self.stream.close()

    def handle(self,kind,value):
        if kind=='begin':
            if self.active or self.busy:raise RuntimeError('上一回合仍在处理')
            self.folder=None;self.meta=None
            self.meta=dict(value,status='recording',outcome='unlabelled',quality_ok=False,exported=None)
            if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+',self.meta['repo_id']):
                raise ValueError('数据集名称使用 local/任务名，仅字母数字下划线和连字符')
            stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.folder=self.base/self.meta['source']/stamp
            (self.folder/'images').mkdir(parents=True,exist_ok=False)
            self.stream=(self.folder/'samples.jsonl').open('w',buffering=1)
            self.count=0;self.first_stamp=self.last_stamp=None;self.max_gap=0.
            self.active=True
            atomic_json(self.folder/'episode.json',self.meta)
            self.emit('record_started',str(self.folder))
        elif kind=='frame':
            if not self.active:return
            row,image=value
            name=f'frame-{self.count:06d}.png'
            Image.fromarray(image).save(self.folder/'images'/name,compress_level=1)
            row=dict(row,image=name,frame_index=self.count)
            self.stream.write(json.dumps(row,ensure_ascii=False)+'\n')
            stamp=row['monotonic_s']
            if self.first_stamp is None:self.first_stamp=stamp
            if self.last_stamp is not None:self.max_gap=max(self.max_gap,stamp-self.last_stamp)
            self.last_stamp=stamp;self.count+=1
        elif kind=='end':
            if not self.active:return
            self.stream.close();self.stream=None;self.active=False
            duration=0. if self.count<2 else self.last_stamp-self.first_stamp
            actual_fps=(self.count-1)/duration if duration>0 else 0.
            quality=self.count>=5 and self.max_gap<=2.5/self.meta['fps'] and abs(actual_fps/self.meta['fps']-1)<.2 and value=='completed'
            self.meta.update(status='review',frames=self.count,duration_s=duration,
                             actual_fps=actual_fps,max_gap_s=self.max_gap,quality_ok=quality,
                             end_reason=value,host_timestamp_note='camera host receipt; no hardware exposure synchronization')
            atomic_json(self.folder/'episode.json',self.meta)
            self.emit('review',dict(folder=str(self.folder),**self.meta))
        elif kind=='decide':
            folder,outcome=value
            meta,rows=read_episode(folder)
            meta.update(outcome=outcome,status='discarded' if outcome=='discard' else 'archived')
            atomic_json(Path(folder)/'episode.json',meta)
            if outcome=='success':
                if not meta['quality_ok']:raise ValueError('采样不完整，保留原始回合供检查，请重录')
                self.busy=True;self.emit('exporting',True)
                try:result=export_episode(folder,self.dataset_base)
                finally:self.busy=False;self.emit('exporting',False)
                self.emit('exported',dict(folder=str(folder),**result))
            self.emit('decision',dict(folder=str(folder),outcome=outcome))
