import json
from pathlib import Path
import numpy as np

def write_report(path,data):
 p=Path(path);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(data,indent=2)+'\n')
def show_plan(plan):
 if 'T_gripper_tool' in plan:print('杆尖 TCP 相对夹爪 XYZ(mm):',np.round(np.asarray(plan['T_gripper_tool'])[:3,3]*1000,2).tolist(),'；后端:',plan['execution_backend'])
 print('板面补偿 右/下(mm):',np.round(np.asarray(plan.get('plane_offset_right_down_m',[0.,0.]))*1000,2).tolist(),'；负值为左/上')
 print('\n目标 base_link XYZ(m):',np.round(plan['target_position_m'],5).tolist())
 print('目标 RPY(deg):',np.round(plan['target_rpy_deg'],2).tolist())
 print('目标关节(rad):',np.round(plan['target_joints_rad'],5).tolist(),'对准参考最短时长:',round(plan['duration_s'],3),'s')
 print('按 g 锁定点击时目标并解算执行两段；p 只预览；r 回 home；空格保持；q 退出。',flush=True)


def execute_gripper(robot,args,report,closing):
    position=0.0 if closing else 1.6
    speed=.5 if args.grip_speed is None else args.grip_speed
    torque=.5 if args.grip_max_torque is None else args.grip_max_torque
    result=robot.command_gripper(position,speed=speed,torque=torque)
    report.update(action='gripper_close' if closing else 'gripper_open',**result)
    write_report(args.report,report)


class TaskTrace:
    """轻量状态转移与失败原因日志，不记录相机图像。"""
    def __init__(self,report_path):
        from datetime import datetime,timezone
        folder=Path(report_path).resolve().parent/'events'
        folder.mkdir(parents=True,exist_ok=True)
        self.path=folder/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'.jsonl')
        self.file=self.path.open('w',buffering=1)
    def record(self,event,**data):
        from datetime import datetime,timezone
        import time
        self.file.write(json.dumps({'event':event,'utc':datetime.now(timezone.utc).isoformat(),
                                   'monotonic_s':time.monotonic(),**data},ensure_ascii=False)+'\n')
    def close(self):self.file.close()
