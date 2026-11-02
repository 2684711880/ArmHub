"""Project entry points; check/help never construct or connect a robot."""
import argparse
import importlib
import json
import re
from pathlib import Path
import runpy
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UPSTREAM = HERE/'Panthera-HT_lerobot'
SDK = HERE/'Panthera-HT_SDK/panthera_python'


def prepare_config(config, role):
    import yaml
    device = config[f'{role}_device']
    if not device:
        raise ValueError(f'请先在 configs/local.json 配置 {role}_device')
    folder = HERE/'.runtime'
    folder.mkdir(exist_ok=True)
    native = yaml.safe_load((ROOT/'robots/panthera_ht/hardware/motors.yaml').read_text())
    native['robot']['Serial_Type'] = device
    motor_path = folder/f'{role}_motors.yaml'
    motor_path.write_text(yaml.safe_dump(native))
    param = yaml.safe_load((SDK/f'robot_param/{role.title()}.yaml').read_text())
    param['robot'].update(name='Panthera-HT', commission_passive_lifecycle=True, param_file=str(motor_path))
    param['urdf']['file_path'] = str((SDK/'robot_param'/param['urdf']['file_path']).resolve())
    arm = json.loads((ROOT/'robots/panthera_ht/hardware/config.json').read_text())
    param['robot']['max_torque'] = arm['torque_caps_nm'][:6]
    output = folder/f'{role}.yaml'
    output.write_text(yaml.safe_dump(param,sort_keys=False))
    return output


def check():
    modules = ['torch','torchvision','torchcodec','pinocchio','pyrealsense2','hightorque_robot',
               'lerobot','lerobot_robot_panthera','lerobot.scripts.lerobot_record','lerobot.scripts.lerobot_train']
    errors=[]
    for name in modules:
        try:
            module=importlib.import_module(name)
            print(f'OK {name}: {getattr(module,"__version__", "imported")}')
        except Exception as error:
            errors.append(name)
            print(f'FAIL {name}: {type(error).__name__}: {error}')
    if errors:
        raise SystemExit(1)
    from armhub_lerobot_runtime import configure_runtime
    configure_runtime()
    from lerobot_robot_panthera.panthera_follower import PantheraFollowerConfig
    from lerobot_robot_panthera.panthera_leader import PantheraLeaderConfig
    # Explicit empty cameras avoids upstream camera enumeration during this check.
    config=json.loads((HERE/'configs/local.json').read_text())
    follower=prepare_config(config,'follower')
    assert follower.is_file()
    assert PantheraFollowerConfig(cameras={}).type == 'panthera_follower'
    assert PantheraLeaderConfig().type == 'panthera_leader'
    from lerobot.scripts.lerobot_record import RecordConfig
    from lerobot.configs.train import TrainPipelineConfig
    import draccus
    record_cfg = draccus.parse(RecordConfig,args=[
        '--robot.type=panthera_follower',f'--robot.param_path={follower}',
        f'--robot.cameras={json.dumps(config["cameras"])}','--dataset.repo_id=local/offline_check',
        '--dataset.single_task=offline check',f'--dataset.root={ROOT/"datasets/lerobot/local/offline_check"}',
        '--dataset.push_to_hub=false','--play_sounds=false'])
    assert record_cfg.robot.type == 'panthera_follower'
    assert set(record_cfg.robot.cameras) == set(config['cameras'])
    train_cfg = draccus.parse(TrainPipelineConfig,args=[
        '--dataset.repo_id=local/offline_check',f'--dataset.root={ROOT/"datasets/lerobot/local/offline_check"}',
        '--policy.type=act','--policy.device=cpu','--policy.push_to_hub=false'])
    assert train_cfg.policy.type == 'act'
    from lerobot.scripts.lerobot_replay import ReplayConfig
    replay_cfg = draccus.parse(ReplayConfig,args=[
        '--robot.type=panthera_follower',f'--robot.param_path={follower}',
        f'--robot.cameras={json.dumps(config["cameras"])}','--dataset.repo_id=local/offline_check','--dataset.episode=0'])
    assert replay_cfg.robot.type == 'panthera_follower'
    print('OK record / replay / ACT CLI config decoding')
    import numpy as np
    import pinocchio as pin
    import yaml
    model_path = yaml.safe_load(follower.read_text())['urdf']['file_path']
    model = pin.buildModelFromUrdf(model_path)
    gravity = pin.computeGeneralizedGravity(model, model.createData(), pin.neutral(model))
    assert model.nq == 6 and np.isfinite(gravity).all()
    print(f'OK official URDF / gravity: nq={model.nq}')
    print('PASS: imports / plugin configs / SDK paths; no device opened')
    print('Camera config:', config['cameras'] or 'empty (joint-only recording; configure serial for image data)')


def main():
    parser=argparse.ArgumentParser(description='ArmHub official Panthera LeRobot integration')
    parser.add_argument('command',choices=['check','cameras','record','train','replay','evaluate','panel','prepare','view'])
    parser.add_argument('--execute',action='store_true',help='允许采集/回放连接机械臂并发送命令')
    parser.add_argument('--leader',action='store_true',help='使用已配置的主臂；默认单臂拖动示教')
    parser.add_argument('--repo-id',default='local/panthera_demo')
    parser.add_argument('--checkpoint')
    parser.add_argument('--episode',type=int,default=0)
    parser.add_argument('--review',action='store_true',help='record：s 保留并回准备位，r 重录，n 开始；累计 50 条成功数据并显示进度')
    parser.add_argument('--save',help='view：把图像与曲线保存为 .rrd 文件，不打开查看器')
    parser.add_argument('--gripper-log',nargs='?',const='auto',help='replay：记录夹爪位置、速度和反馈力矩，可指定新的 CSV 路径')
    parser.add_argument('--gripper-assist',nargs='?',type=float,const=.1,help='replay：接近夹持开度时叠加闭合前馈力矩；默认 0.1 SDK Nm，最大 0.3')
    parser.add_argument('--gripper-close-at',type=float,help='辅助加力的闭合开度阈值；本轮蓝块试录默认 0.85 rad')
    args,extra=parser.parse_known_args()
    if args.review:
        if args.command!='record' or args.leader:
            parser.error('--review 只用于单臂 record')
        if not sys.stdin.isatty():
            parser.error('--review 需要可输入 S/R/Q 的交互终端')
    if args.gripper_log and args.command!='replay':
        parser.error('--gripper-log 只用于 replay')
    assist_config = None
    if args.gripper_assist is not None:
        if args.command!='replay':parser.error('--gripper-assist 只用于 replay')
        from gripper_assist import AssistConfig
        kwargs = dict(torque_nm=args.gripper_assist)
        if args.gripper_close_at is not None:kwargs['close_at_rad']=args.gripper_close_at
        try:assist_config = AssistConfig(**kwargs)
        except ValueError as exc:parser.error(str(exc))
        if args.repo_id!='local/blue_pick_trial_01' and args.gripper_close_at is None:
            parser.error('其他数据集请显式提供 --gripper-close-at，默认 0.85 只针对本轮蓝块试录')
        args.gripper_log = args.gripper_log or 'auto'
    elif args.gripper_close_at is not None:
        parser.error('--gripper-close-at 需与 --gripper-assist 一起使用')
    if extra and extra[0]=='--':extra=extra[1:]
    if args.command=='panel':
        sys.argv=['teaching-panel',*extra]
        runpy.run_module('apps.teaching_panel',run_name='__main__'); return
    if args.command=='check':
        check(); return
    if args.command=='cameras':
        sys.argv=['lerobot-find-cameras','realsense',*extra]
        runpy.run_module('lerobot.scripts.lerobot_find_cameras',run_name='__main__'); return
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+',args.repo_id):
        parser.error('数据集名称需为 local/名称，仅字母数字下划线和连字符')
    if args.command=='prepare':
        if extra:parser.error('prepare 不接受额外参数')
        from prepare_pose import return_prepared
        return_prepared(execute=args.execute)
        return
    if args.command=='view':
        if extra or args.execute:parser.error('view 只做离线回看，不使用 --execute 或额外参数')
        from view_episode import view_episode
        view_episode(args.repo_id,ROOT/'datasets/lerobot'/args.repo_id,args.episode,args.save)
        return
    if '--help' in extra:
        module = {'train':'lerobot.scripts.lerobot_train',
                  'record':'lerobot.scripts.lerobot_record',
                  'evaluate':'lerobot.scripts.lerobot_record',
                  'replay':'lerobot.scripts.lerobot_replay'}[args.command]
        sys.argv=[module,'--help']
        runpy.run_module(module,run_name='__main__')
        return
    config=json.loads((HERE/'configs/local.json').read_text())
    dataset_root=ROOT/'datasets/lerobot'/args.repo_id
    if args.command=='train':
        module='lerobot.scripts.lerobot_train'
        options=[f'--dataset.repo_id={args.repo_id}',f'--dataset.root={dataset_root}',
                 '--policy.type=act','--policy.device=cpu','--policy.push_to_hub=false',
                 f'--output_dir={ROOT/"outputs/lerobot/panthera_act"}','--wandb.enable=false',
                 '--batch_size=4','--steps=50000']
    else:
        if not args.execute and '--help' not in extra:
            raise SystemExit('采集/回放会启用官方控制循环。准备好设备后加 --execute；离线检查用 check。')
        if args.command=='evaluate' and not args.checkpoint:
            raise SystemExit('策略回放需要 --checkpoint /absolute/path/to/pretrained_model')
        # Device semantics are configured here; prevent raw CLI flags bypassing ownership.
        forbidden=('--robot.','--teleop.')
        if any(x.startswith(forbidden) for x in extra):
            raise SystemExit('机械臂和相机参数请在 configs/local.json 配置；其余官方参数可直接追加。')
        from armhub_lerobot_runtime import configure_runtime
        configure_runtime()
        follower=prepare_config(config,'follower')
        camera_json=json.dumps(config['cameras'])
        options=['--robot.type=panthera_follower',f'--robot.param_path={follower}',
                 f'--robot.cameras={camera_json}',f'--dataset.repo_id={args.repo_id}',
                 f'--dataset.root={dataset_root}',f'--dataset.single_task={config["task"]}',
                 f'--dataset.fps={config["fps"]}',f'--dataset.num_episodes={config["num_episodes"]}',
                 f'--dataset.episode_time_s={config["episode_time_s"]}',
                 f'--dataset.reset_time_s={config["reset_time_s"]}',
                 '--dataset.push_to_hub=false','--display_data=false','--play_sounds=false']
        if args.review:
            options += ['--manual_review=true','--dataset.num_episodes=50',
                        '--dataset.episode_time_s=inf','--dataset.reset_time_s=0',
                        '--display_data=true',
                        '--dataset.single_task=pick up the blue block and place it in the box']
        if args.leader:
            options += ['--teleop.type=panthera_leader',f'--teleop.param_path={prepare_config(config,"leader")}']
        if args.command=='evaluate':
            options += [f'--policy.path={Path(args.checkpoint).resolve()}']
        module='lerobot.scripts.lerobot_record'
        if args.command=='replay':
            if args.leader:
                raise SystemExit('数据回放不使用主臂')
            options=['--robot.type=panthera_follower',f'--robot.param_path={follower}',
                     '--robot.cameras={}',f'--dataset.repo_id={args.repo_id}',
                     f'--dataset.root={dataset_root}',f'--dataset.episode={args.episode}',
                     '--play_sounds=false']
            module='lerobot.scripts.lerobot_replay'
    # Upstream relative paths must never put artifacts into the source snapshot.
    sys.argv=[module,'--help'] if '--help' in extra else [module,*options,*extra]
    trace = None
    uninstall_assist = None
    try:
        if args.gripper_log:
            from gripper_trace import install_gripper_trace, default_trace_path
            path = default_trace_path(ROOT) if args.gripper_log=='auto' else Path(args.gripper_log)
            trace = install_gripper_trace(path)
            print(f'将记录夹爪反馈：{trace.path}',flush=True)
        if assist_config is not None:
            from gripper_assist import install_gripper_assist
            uninstall_assist = install_gripper_assist(assist_config)
            trace.assist_config = vars(assist_config)
            print(f'夹持辅助：闭合到 {assist_config.close_at_rad} rad 附近后逐渐叠加 '
                  f'{assist_config.torque_nm} SDK Nm 闭合力矩，张开时撤销。',flush=True)
        runpy.run_module(module,run_name='__main__')
    except BaseException as exc:
        if trace is not None:
            trace.failure=dict(type=type(exc).__name__,message=str(exc))
        raise
    finally:
        if uninstall_assist is not None:
            uninstall_assist()
        if trace is not None:
            trace.close()
        if args.review:
            from terminal_keys import close_console
            close_console()


if __name__=='__main__':
    main()
