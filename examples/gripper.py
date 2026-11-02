"""独立夹爪命令，六轴保持；接触停滞只报告可能接触，不认定夹稳。"""
import argparse
import json
import math
import time
from armhub import create_arm


def move_gripper(arm, target, speed, torque, *, allow_contact=False):
    arm.command_gripper(target,speed=speed,torque=torque)
    began=time.monotonic();anchor=None;since=began
    while True:
        if arm.poll_gripper():
            return {'status':'target_reached','grasp_confirmed':False}
        now=time.monotonic();q=arm._last_states[6].position
        if anchor is None or abs(q-anchor)>.003:
            anchor=q;since=now
        # 夹笔时不强求空爪的零位；停止并确认七轴保持，避免长时间堵转。
        if allow_contact and now-began>=1. and now-since>=.6:
            arm.stop()
            return {'status':'stopped_before_target','possible_contact':True,
                    'grasp_confirmed':False,'note':'已切到实测位置保持；请手动确认笔是否夹稳'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['open','close'])
    p.add_argument('--position',type=float,help='覆盖开合目标，原始电机 rad')
    p.add_argument('--speed',type=float,default=.3)
    p.add_argument('--torque',type=float,default=.3,help='动作力矩上限 Nm；停止保持沿用 ArmHub 配置')
    p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    target=args.position if args.position is not None else (1.6 if args.action=='open' else 0.)
    if not all(math.isfinite(v) for v in [target,args.speed,args.torque]):p.error('参数必须有限')
    if not 0<=target<=2 or not 0<args.speed<=.5 or not 0<args.torque<=1:p.error('目标/速度/力矩越界')
    arm=create_arm('panthera_ht','hardware')
    with arm:
        states=arm.get_joint_feedback()
        before=states[6].position
        print(json.dumps(dict(action=args.action,current_raw_rad=before,target_raw_rad=target,
                              execute=args.execute,arm_motion='hold_current'),ensure_ascii=False),flush=True)
        if args.execute:
            arm.enable()
            result=move_gripper(arm,target,args.speed,args.torque,
                                allow_contact=args.action=='close' and target<before)
            result['actual_raw_rad']=arm.get_joint_feedback()[6].position
            print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        else:print('只读预览；添加 --execute 才开合夹爪')


if __name__=='__main__':main()
