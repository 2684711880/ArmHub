"""Complete synthetic recording -> review -> official LeRobot video export."""
import argparse
import json
from pathlib import Path
import tempfile
import threading
import time

from apps.teaching_panel.camera import CameraWorker
from apps.teaching_panel.controller import TeachingWorker
from apps.teaching_panel.recorder import read_episode


def wait_for(predicate,message,timeout=15):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.02)
    raise RuntimeError('验证超时：'+message)


def validate(destination):
    destination=Path(destination).resolve()
    if (destination/'settings.json').exists():raise ValueError('验证输出目录已使用，请指定一个新目录')
    destination.mkdir(parents=True,exist_ok=True)
    events=[];lock=threading.Lock()
    def emit(event,value):
        with lock:events.append((event,value))
    camera=CameraWorker(emit,demo=True)
    worker=TeachingWorker(emit,camera,demo=True,settings_path=destination/'settings.json',
                           store_base=destination/'raw',dataset_base=destination/'lerobot')
    camera.start();worker.start();camera.open();worker.permission(True)
    try:
        worker.submit('connect');wait_for(lambda:worker.mode=='readonly','只读连接')
        wait_for(worker.stationary,'静止反馈')
        worker.submit('save_prepare');wait_for(lambda:worker.options['prepared'] is not None,'保存准备位')
        worker.submit('options',dict(fps=20))
        worker.submit('teach');wait_for(lambda:worker.mode=='teach','拖动模式')
        wait_for(lambda:camera.latest() is not None,'合成图像')
        worker.submit('record');wait_for(lambda:worker.recording and worker.store.active,'开始录制')
        wait_for(lambda:worker.store.count>=7,'前七帧')
        worker.submit('close');wait_for(lambda:worker.store.count>=21,'闭合后数据')
        worker.submit('open')
        wait_for(lambda:worker.store.count>=40,'手动结束前的帧数')
        worker.submit('end')
        wait_for(lambda:any(e=='review' for e,v in events),'回合审阅')
        folder=worker.store.folder
        meta,rows=read_episode(folder)
        assert meta['quality_ok'],meta
        assert meta['source']=='demo'
        assert any(r['action'][6]==0. for r in rows)
        assert any(abs(r['action'][6]-r['observation'][6])>.1 for r in rows),'必须分别记录夹爪目标与实际位置'
        assert worker.pending_review.is_set()
        wait_for(lambda:worker.mode=='hold','结束后的保持确认')
        worker.submit('decision',(str(folder),'success'))
        wait_for(lambda:any(e=='exported' for e,v in events),'官方视频数据集导出',90)
        wait_for(lambda:not worker.writer_busy.is_set(),'导出完成')
        final,_=read_episode(folder)
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        ds=LeRobotDataset(meta['repo_id'],root=final['exported']['root'],video_backend='pyav')
        frame=ds[0]
        assert tuple(frame['observation.state'].shape)==(7,)
        assert tuple(frame['action'].shape)==(7,)
        assert tuple(frame['observation.images.d435'].shape)==(3,480,640)
        # Append another independently reviewed episode to the same local dataset.
        prior_reviews=sum(event=='review' for event,value in events)
        worker.submit('teach');wait_for(lambda:worker.mode=='teach','再次进入拖动')
        previous_folder=worker.store.folder
        worker.submit('record');wait_for(lambda:worker.recording and worker.store.active and worker.store.folder!=previous_folder,'第二回合开始')
        wait_for(lambda:worker.store.count>=20,'第二回合手动结束前帧数')
        worker.submit('end')
        wait_for(lambda:sum(event=='review' for event,value in events)>prior_reviews,'第二回合审阅')
        wait_for(lambda:worker.mode=='hold','第二回合保持确认')
        folder2=worker.store.folder
        worker.submit('decision',(str(folder2),'success'))
        wait_for(lambda:sum(event=='exported' for event,value in events)==2,'同一数据集追加回合',90)
        wait_for(lambda:not worker.writer_busy.is_set(),'第二回合导出完成')
        ds2=LeRobotDataset(meta['repo_id'],root=final['exported']['root'],video_backend='pyav')
        assert ds2.meta.total_episodes==2
        assert tuple(ds2[len(rows)]['observation.images.d435'].shape)==(3,480,640)
        worker.submit('preview');wait_for(lambda:worker.plan is not None,'显式预览准备位')
        worker.submit('return');wait_for(lambda:worker.mode=='returning','显式回准备位')
        wait_for(lambda:worker.mode=='hold','准备位到位反馈')
        assert max(abs(a-b) for a,b in zip(worker.sample['q'],worker.options['prepared']['q']))<.001
        result=dict(status='passed',source='synthetic_only',hardware_connected=False,
                    frames=len(rows),actual_fps=meta['actual_fps'],total_episodes=ds2.meta.total_episodes,
                    append_and_reload_verified=True,explicit_prepare_return_verified=True,
                    gripper_target_and_feedback_distinct=True,
                    raw_episode=str(folder),dataset=final['exported'],
                    decoded_image_shape=list(frame['observation.images.d435'].shape),
                    events=[event for event,value in events if event not in ('feedback','camera_frame','record_progress')])
        (destination/'acceptance.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    finally:
        worker.shutdown();camera.shutdown();worker.join(timeout=100);camera.join(timeout=5)
        if worker.is_alive() or camera.is_alive():raise RuntimeError('演示线程未完整退出')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    validate(parser.parse_args().output)
