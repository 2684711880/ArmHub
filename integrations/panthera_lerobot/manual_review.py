"""Human-gated official recording: keep/retry, explicit preparation and start."""
from datetime import datetime
import json
from pathlib import Path
import time
from terminal_keys import console_input


def clear_events(events):
    for key in ('exit_early','rerecord_episode','stop_recording'):
        events[key]=False


def prepare_review_dataset(root, repo_id):
    """Return resume mode; preserve zero-episode attempts before creating anew."""
    root=Path(root)
    if not root.exists():
        return False
    info_path=root/'meta/info.json'
    if not info_path.is_file():
        raise ValueError(f'目标目录不是完整数据集，保留原样：{root}')
    info=json.loads(info_path.read_text())
    if info['total_episodes'] > 0:
        return True
    # An interrupted first attempt cannot be loaded by LeRobot's data loader.
    # Rename the whole folder, including recoverable raw frames; never delete it.
    archived=root.with_name(root.name+'.empty-'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    root.rename(archived)
    print(f'未完成的空数据集已完整保留到：{archived}，本次重新创建 {repo_id}',flush=True)
    return False


def begin_round(robot, events, kept, goal, *, ask=None, prepare=None):
    ask=console_input if ask is None else ask
    if robot.is_connected:
        robot.disconnect()
    print(f'已成功保留 {kept}/{goal} 条；下一条为第 {kept+1} 条。',flush=True)
    try:
        if not getattr(robot,'_armhub_review_ready',False):
            answer=ask('首次准备：确认路径可通行后按回车回准备位并开爪，输入 q 退出：').strip().lower()
            while answer and answer!='q':
                answer=ask('未开始移动；按回车回准备位，或输入 q 退出：').strip().lower()
            if answer=='q':return False
            reset_ready(robot,prepare=prepare)
        answer=ask('已在准备位并开爪。摆好蓝块，输入 n 回车开始下一条，q 退出：').strip().lower()
        while answer not in ('n','q'):
            answer=ask('输入 n 回车才会开始录制，或输入 q 退出：').strip().lower()
        if answer=='q':return False
        robot._armhub_review_ready=False
        preview=getattr(robot,'_armhub_continuous_preview',None)
        if preview is not None:
            preview.check_ready()
        robot.connect()
        robot.enable_teaching_mode()
        clear_events(events)
        print(f'正在录制第 {kept+1}/{goal} 条；已成功保留 {kept} 条。松爪并抬离盒口后按 → 结束。',flush=True)
        return True
    except EOFError:
        return False


def reset_ready(robot, *, prepare=None):
    """s/r authorizes this return; no further motion confirmation is needed."""
    robot._armhub_review_ready=False
    if robot.is_connected:
        robot.disconnect()
    if prepare is None:
        from prepare_pose import return_prepared
        prepare=return_prepared
    print('正在回准备位并张开夹爪；此过程不录制。',flush=True)
    prepare(execute=True)
    robot._armhub_review_ready=True
    print('准备位与开爪已完成；等待 n 开始，不会自动录制。',flush=True)


def review_round(robot, dataset, events, *, ask=None):
    ask=console_input if ask is None else ask
    # Stop the SDK writer, feedback-confirmed hold, close serial before input; a continuous camera proxy retains preview.
    # Preparation can then reuse the existing HardwareSession without a second owner.
    if robot.is_connected:
        robot.disconnect()
    size=int(dataset.episode_buffer['size'])
    if size < 5:
        print(f'本轮只有 {size} 帧，不可保留，请重录。',flush=True)
        return 'quit' if events.get('stop_recording') else 'retry'
    if events.get('rerecord_episode'):
        print('已按左方向键：本轮作为失败重录，不计入成功数量。',flush=True)
        return 'retry'
    while True:
        try:
            answer=ask('本轮结束。s 成功保留并回准备位 / r 失败重录并回准备位 / q 丢弃本轮退出（输入后回车）：').strip().lower()
        except EOFError:
            return 'quit'
        if answer in ('s','r','q'):
            return {'s':'keep','r':'retry','q':'quit'}[answer]
        print('请输入小写 s、r 或 q；回车不会默认保存。',flush=True)


def write_review_event(dataset, decision, frames=None):
    path=Path(dataset.root)/'manual_review.jsonl'
    row=dict(unix_s=time.time(),decision=decision,total_kept=dataset.num_episodes,
             episode_index=dataset.num_episodes-1 if decision=='kept' else int(dataset.episode_buffer['episode_index']),
             frames=int(dataset.episode_buffer['size']) if frames is None else int(frames))
    with path.open('a') as stream:
        stream.write(json.dumps(row,ensure_ascii=False)+'\n')
