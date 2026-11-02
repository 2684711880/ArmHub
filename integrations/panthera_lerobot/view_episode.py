"""Offline LeRobot episode -> Rerun timeline. Never imports a robot controller."""
import json
from pathlib import Path


def validate_episode(root, episode):
    root = Path(root)
    info_path = root/'meta/info.json'
    if not info_path.is_file():
        raise ValueError(f'找不到本地数据集：{root}')
    info = json.loads(info_path.read_text())
    if not 0 <= episode < info['total_episodes']:
        raise ValueError(f'数据集只有 {info["total_episodes"]} 个完整回合，不能播放第 {episode} 回合（从 0 编号）')
    return info


def view_episode(repo_id, root, episode=0, save=None):
    validate_episode(root, episode)
    import numpy as np
    import rerun as rr
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset = LeRobotDataset(repo_id, root=Path(root), episodes=[episode], video_backend='pyav')
    rr.init(f'{repo_id} / episode {episode}')
    if save:
        output = Path(save).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(output))
    else:
        rr.spawn(memory_limit='10%')
    count = 0
    try:
        for index in range(len(dataset)):
            frame = dataset[index]
            if int(frame['episode_index']) != episode:
                continue
            rr.set_time('episode', duration=float(frame['timestamp']))
            for key in ('observation.state', 'action'):
                values = np.asarray(frame[key])
                for name, value in zip(dataset.features[key]['names'], values):
                    rr.log(f'{key}/{name}', rr.Scalars(float(value)))
            for key, feature in dataset.features.items():
                if key.startswith('observation.images.') and feature['dtype'] in ('video', 'image'):
                    image = np.asarray(frame[key])
                    # LeRobot decoding returns float CHW in [0,1].
                    if image.ndim == 3 and image.shape[0] in (1,3,4):
                        image = image.transpose(1,2,0)
                    if image.dtype.kind == 'f':
                        image = (np.clip(image,0,1)*255).astype(np.uint8)
                    rr.log(key, rr.Image(image))
            count += 1
        if not count:
            raise ValueError('本回合没有可播放的帧')
    finally:
        rr.disconnect()
    print(f'已载入第 {episode} 回合，共 {count} 帧。仅回看图像和曲线，没有连接机械臂。')
    if save:
        print(f'回看文件：{output}')
    else:
        print('在 Rerun 底部选择 episode 时间轴，拖动或点击播放。')
    return count
