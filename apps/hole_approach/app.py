"""Process isolation: UI, depth capture/detection, IK, and one motor owner.

UI never calls SDK, waits for frames, solves IK or waits for motion. Every key
advances a generation so a late IK result cannot issue a superseded command.
"""
import multiprocessing as mp
import os
import queue
import select
import sys
import time
import textwrap
import cv2
import numpy as np
from perception.detectors.depth_hole import detect_depth_hole
from .overlay import HoleOverlay


def latest_put(channel, value):
    try:
        channel.put_nowait(value); return
    except queue.Full:
        pass
    try:
        channel.get_nowait()
    except queue.Empty:
        pass
    try:
        channel.put_nowait(value)
    except queue.Full:
        pass


def drain(channel):
    result = None
    while True:
        try:
            result = channel.get_nowait()
        except queue.Empty:
            return result


def camera_worker(options, vision, display, quit_event):
    from perception.cameras.realsense import LiveCamera, SnapshotCamera
    vision.cancel_join_thread(); display.cancel_join_thread()
    camera = None; previous = None
    try:
        camera = LiveCamera(serial=options.get('camera_serial'),width=options['camera_width'],height=options['camera_height'],fps=options['camera_fps']) if options['camera_source'] == 'live' else SnapshotCamera(options['snapshot'])
        while not quit_event.is_set():
            try:
                rgb, depth, k, stamp = camera.read()
                if stamp == previous:
                    quit_event.wait(.01); continue
                previous = stamp
                try:
                    observation = detect_depth_hole(depth, k)
                    small = {key: observation[key] for key in ['center', 'normal', 'radius', 'plane_rms']}
                    error = None
                except ValueError as exc:
                    observation = None; small = None; error = str(exc)
                latest_put(vision, {'stamp': stamp, 'observation': small, 'detection_error': error})
                # Display depth, with RGB only as an optional context inset.
                mapped = np.nan_to_num(depth, nan=0., posinf=0., neginf=0.)
                image = cv2.applyColorMap(np.uint8(np.clip(mapped/1.5, 0, 1)*255), cv2.COLORMAP_TURBO)
                if observation is not None:
                    cv2.ellipse(image, observation['ellipse'], (0, 255, 0), 2)
                height,width=image.shape[:2]
                inset_w,inset_h=min(160,width),min(120,height)
                image[-inset_h:,-inset_w:]=cv2.resize(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),(inset_w,inset_h))
                display_k=k.copy();display_k[0,:]*=640/width;display_k[1,:]*=480/height
                image=cv2.resize(image,(640,480))
                latest_put(display, {'image': image, 'error': error, 'stamp': stamp, 'K':display_k.tolist(), 'measured_center_optical_m':None if small is None else small['center'].tolist()})
            except (RuntimeError, OSError, FileNotFoundError) as exc:
                latest_put(vision, {'stamp': time.time(), 'camera_error': str(exc), 'observation': None})
                latest_put(display, {'error': str(exc)})
                quit_event.wait(.2)
    except Exception as exc:
        latest_put(vision, {'stamp': time.time(), 'camera_error': str(exc), 'observation': None})
        latest_put(display, {'error': str(exc)})
    finally:
        if camera is not None:
            camera.close()


def planner_worker(tasks, results, quit_event):
    from .planning import solve_two_stage
    np.random.seed(258)
    results.cancel_join_thread()
    while not quit_event.is_set():
        try:
            task = tasks.get(timeout=.1)
        except queue.Empty:
            continue
        newer = drain(tasks)
        if newer is not None:
            task = newer
        try:
            result = {'id': task['id'], 'epoch': task['epoch'], 'plan': solve_two_stage(task)}
        except Exception as exc:
            result = {'id': task['id'], 'epoch': task['epoch'], 'error': str(exc)}
        latest_put(results, result)


def motor_worker(options, calibration, calibration_status, commands, vision, tasks, results, statuses, generation, quit_event):
    from types import SimpleNamespace
    from armhub.task_runtime import ArmTaskRuntime
    from .config import load_config
    from .controller import Controller
    robot = None; controller = None; last_packet = None; last_status = 0.
    trace=None;last_trace_state=None;last_trace_time=0.
    statuses.cancel_join_thread(); tasks.cancel_join_thread()
    args = SimpleNamespace(**options)
    auto_executed = False; auto_go_started=False; failure = None
    try:
        from .reporting import TaskTrace
        trace=TaskTrace(args.report)
        print('任务状态日志：',trace.path,flush=True)
        latest_put(statuses, {'state': 'INITIALIZING', 'message': 'Connecting arm and reading feedback; UI remains responsive'})
        robot = ArmTaskRuntime(args.robot,load_config(args.config),allow_motion=args.execute)
        controller = Controller(robot, args, calibration, calibration_status,
                                lambda task: latest_put(tasks, task), lambda: generation.value)
        while not quit_event.is_set():
            # Consume newest intent first, ahead of perception/planning results.
            command = drain(commands)
            try:
                if command is not None:
                    trace.record('command',key=command['key'],epoch=command['epoch'],state=controller.state,tracking=controller.tracking)
                    controller.command(command['key'], command['epoch'],command.get('click_time'),command.get('settings'))
                q = controller.feedback()
                packet = drain(vision)
                if packet is not None:
                    last_packet = packet
                if last_packet is not None:
                    controller.observe(last_packet)
                result = drain(results)
                if result is not None:
                    controller.planned(result)
                if args.execute_once and args.execute and not auto_go_started and controller.good>=3 and controller.vision=='visible' and controller.fresh():
                    controller.command('g',generation.value); auto_go_started=True
                controller.tick(q)
                if args.plan_only and controller.state == 'PREVIEW_READY':
                    break
                if args.execute_once and controller.active is not None:
                    auto_executed = True
                if auto_executed and controller.state == 'IDLE' and controller.active is None:
                    break
            except (ValueError, RuntimeError, TimeoutError) as exc:
                trace.record('control_error',reason=str(exc),state=controller.state,tracking=controller.tracking)
                try:
                    controller.cancel()
                except Exception as stop_exc:
                    controller.message = f'{exc}; hold unconfirmed: {stop_exc}'
                else:
                    controller.message = str(exc)
                controller.state = 'PAUSED'
                print('控制暂停：', controller.message, flush=True)
                quit_event.wait(.05)
            if time.monotonic()-last_status > .1:
                snapshot=controller.status()
                latest_put(statuses,snapshot);last_status=time.monotonic()
                key=(snapshot['state'],snapshot['tracking'],snapshot['vision'],snapshot['message'])
                if key!=last_trace_state or last_status-last_trace_time>=1.:
                    trace.record('status',**{field:snapshot.get(field) for field in ['state','tracking','vision','message','depth_confirmed_frames','camera_age_s','pending_age_s','vision_rejection','hold_pending']})
                    last_trace_state=key;last_trace_time=last_status
            quit_event.wait(.005)
    except Exception as exc:
        failure = str(exc)
        latest_put(statuses, {'state': 'ERROR', 'message': failure})
        print(f'控制器启动失败：{exc}', flush=True)
    finally:
        if controller is not None:
            try:
                controller.cancel()
            except Exception as exc:
                print(f'退出保持未确认：{exc}', flush=True)
        if robot is not None:
            try:robot.close()
            except Exception as exc:
                failure=str(exc)
                print(f'关闭机械臂时停止未确认：{exc}',flush=True)
        if trace is not None:
            trace.record('closed',error=failure)
            trace.close()
        latest_put(statuses, {'state': 'ERROR' if failure else 'CLOSED', 'message': failure or 'Transport closed; no automatic home or gripper release'})
    if failure is not None:
        raise SystemExit(1)


def publish_key(key, commands, generation, quit_event, settings=None):
    if key not in ['p', 'g', 'r', 'c', 'o', 't', 'q', ' ']:
        return
    with generation.get_lock():
        generation.value += 1
        epoch = generation.value
    latest_put(commands, {'key': key, 'epoch': epoch, 'click_time':time.time(), 'settings':settings})
    if key == 'q':
        quit_event.set()


def run_hardware(args, calibration, status, worker_targets=None):
    context = mp.get_context('spawn')
    quit_event = context.Event(); generation = context.Value('L', 0)
    commands, vision, display, tasks, results, statuses = [context.Queue(maxsize=2) for _ in range(6)]
    commands.cancel_join_thread()
    options = vars(args).copy()
    applied_settings=None
    def send_key(key):publish_key(key,commands,generation,quit_event,applied_settings)
    planner_target, camera_target, motor_target = worker_targets or (planner_worker, camera_worker, motor_worker)
    workers = [
        context.Process(target=planner_target, args=(tasks, results, quit_event), name='hole-ik'),
        context.Process(target=camera_target, args=(options, vision, display, quit_event), name='hole-depth'),
        context.Process(target=motor_target, args=(options, calibration.tolist(), status, commands, vision, tasks, results, statuses, generation, quit_event), name='hole-motors'),
    ]
    image = np.zeros((480, 640, 3), np.uint8); frame_data={}; overlay=HoleOverlay()
    state = {'state': 'STARTING', 'message': 'Starting independent workers'}
    last_text = None; started = []; mouse_ready = False; failure = None; terminal_buffer = ''
    panel=ToolCalibrationPanel(args) if not args.no_gui else None
    print('u 打开 TCP/offset 微调，a 应用（暂停跟踪并作废旧规划）；p 仅预览，g 自动plan并执行两段，t 跟踪/丢孔回收，空格保持，r home，c 抓取，o 释放，f 切换显示坐标系，q 退出。', flush=True)
    try:
        if not args.no_gui:
            cv2.namedWindow('ArmHub depth hole control',cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow('ArmHub depth hole control',1100,700)
        for process in workers:
            process.start(); started.append(process)
        if args.plan_only:
            send_key('p')
        while not quit_event.is_set():
            packet = drain(display)
            if packet is not None and 'image' in packet:
                image = packet['image']; frame_data=packet
            update = drain(statuses)
            if update is not None:
                state = update
            if state.get('state') in ['ERROR', 'CLOSED']:
                print(state.get('message'), flush=True)
                if state.get('state') == 'ERROR':failure = state.get('message')
                break
            if args.no_gui:
                text = state.get('state')+': '+state.get('message', '')
                if text != last_text:
                    print(text, flush=True); last_text = text
                if select.select([sys.stdin], [], [], .03)[0]:
                    data = os.read(sys.stdin.fileno(),4096)
                    if not data:
                        send_key('q')
                    else:
                        terminal_buffer += data.decode(errors='replace')
                        while '\n' in terminal_buffer:
                            line,terminal_buffer=terminal_buffer.split('\n',1)
                            key=' ' if line.strip() in ['s','space'] or line==' ' else line.strip().lower()
                            if key=='f':overlay.toggle(); print('Display frame:',overlay.frame,flush=True)
                            else:send_key(key)
            else:
                canvas = overlay.render(image,frame_data,state)
                cv2.putText(canvas, 'p:plan g:go t:track SPACE:hold r:home u:TCP a:apply q:quit', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
                cv2.putText(canvas, state.get('state', ''), (8, 44), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
                if 'gripper_position' in state:
                    cv2.putText(canvas, f"gripper {state['gripper_position']:.3f} rad / {state['gripper_torque']:.3f} Nm", (8, 65), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
                if 'hole_estimate_mode' in state:
                    cv2.putText(canvas,f"Hole {state['hole_estimate_mode']} age={state['hole_estimate_age_s']:.2f}s sigma={state['hole_position_std_m']*1000:.1f}mm",(8,108),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                    xyz=state['hole_estimate_base_m']
                    cv2.putText(canvas,f'Hole base XYZ(m): {xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f}',(8,130),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                if packet is not None and packet.get('error'):
                    cv2.putText(canvas, 'Depth hole unavailable', (8, 88), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
                for index, (key,label) in enumerate([('p','PLAN'),('g','GO'),('t','TRACK'),(' ','HOLD'),('r','HOME'),('c','GRIP'),('o','OPEN'),('q','QUIT')]):
                    x=8+index*78
                    cv2.rectangle(canvas,(x,447),(x+72,477),(70,70,70),-1)
                    cv2.putText(canvas,label,(x+4,467),cv2.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1)
                tool=state.get('tool_settings',{})
                cv2.putText(canvas,f"Tool +X: {tool.get('rod_length',.04)*1000:.0f}mm  backend: {'Ruckig' if state.get('ruckig_enabled') else 'moveJ'}  u: TCP / a: APPLY",(8,152),cv2.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1)
                cv2.putText(canvas,'EXECUTION ENABLED' if args.execute else 'READ ONLY - restart with --execute to allow motion',(8,174),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,220,255),1)
                for line,message in enumerate(textwrap.wrap(state.get('message',''),width=75)[:2]):
                    cv2.putText(canvas,message,(8,196+line*18),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                age=state.get('camera_age_s')
                details=f"Track: {'ON' if state.get('tracking') else 'OFF'} | Vision: {state.get('vision','waiting')} | Confirmed: {state.get('depth_confirmed_frames',0)} | Camera age: {age:.2f}s" if age is not None else 'Waiting for camera feedback'
                cv2.putText(canvas,details,(8,236),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
                cv2.imshow('ArmHub depth hole control', canvas)
                if not mouse_ready:
                    def mouse(event,x,y,flags,param):
                        if event==cv2.EVENT_LBUTTONDOWN and image.shape[1]+10<=x<=image.shape[1]+410 and 12<=y<=47:
                            overlay.toggle(); return
                        if event==cv2.EVENT_LBUTTONDOWN and 447<=y<=477:
                            index=(x-8)//78
                            if 0<=index<8 and 8+index*78<=x<=80+index*78:
                                send_key('pgt rcoq'[index])
                    cv2.setMouseCallback('ArmHub depth hole control',mouse);mouse_ready=True
                code = cv2.waitKey(20) & 255
                if code==ord('u'):panel.open()
                elif code==ord('a') and panel.visible():
                    applied_settings=panel.values();publish_settings(applied_settings,commands,generation)
                elif code==ord('f'):overlay.toggle()
                elif code in [ord(k) for k in 'pgrcotq ']:
                    send_key(chr(code))
            if panel is not None and panel.apply_requested:
                panel.apply_requested=False
                if panel.visible():
                    applied_settings=panel.values();publish_settings(applied_settings,commands,generation)
            if not workers[2].is_alive():
                print('Motor worker exited; stopping UI.', flush=True)
                if workers[2].exitcode:failure = f'Motor worker exited with code {workers[2].exitcode}'
                break
    except KeyboardInterrupt:
        send_key('q')
    finally:
        quit_event.set()
        # SDK owner exits cooperatively; never kill it merely because IK is busy.
        for process in reversed(started):
            process.join(timeout=6. if process.name == 'hole-depth' else 2.)
        for process in started:
            if process.is_alive():
                if process.name == 'hole-motors':
                    print(f'Motor worker PID {process.pid} has not exited; transport/hold unconfirmed.', flush=True)
                else:
                    process.terminate(); process.join(timeout=1.)
        cv2.destroyAllWindows()

    if failure is not None:
        raise RuntimeError(failure)


def publish_settings(settings,commands,generation):
    from armhub.tools import validate_settings
    values=validate_settings(settings)
    with generation.get_lock():
        generation.value+=1;epoch=generation.value
    latest_put(commands,{'key':'calibration','epoch':epoch,'settings':values})


class ToolCalibrationPanel:
    """UI-local editing; no SDK call, IK or motion until explicit Apply then GO."""
    name='TCP calibration - a APPLY, u reopen'
    def __init__(self,args):
        self.opened=False;self.apply_requested=False
        self.initial={'Rod mm':round(getattr(args,'rod_length',.04)*1000)}
        for prefix,values,shift in [('TCP',getattr(args,'tcp_xyz_mm',[0,0,0]),100),('RPY',getattr(args,'tcp_rpy_deg',[0,0,0]),90),('Offset',np.asarray(args.plane_offset_m)*1000,200)]:
            for axis,value in zip('XYZ' if prefix!='Offset' else ['Right','Down'],values):self.initial[prefix+' '+axis]=round(value)+shift
    def visible(self):
        try:return self.opened and cv2.getWindowProperty(self.name,cv2.WND_PROP_VISIBLE)>=1
        except cv2.error:return False
    def open(self):
        if self.visible():return
        self.opened=True;cv2.namedWindow(self.name,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.name,600,460)
        for name,value in self.initial.items():
            maximum=300 if name=='Rod mm' else (200 if name.startswith('TCP') else 180 if name.startswith('RPY') else 400)
            cv2.createTrackbar(name,self.name,int(value),maximum,lambda value:None)
        canvas=np.zeros((70,600,3),np.uint8)
        cv2.putText(canvas,'Sliders: mm / degrees. a APPLY, then p PREVIEW / g GO.',(8,35),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        cv2.rectangle(canvas,(450,42),(590,68),(50,100,50),-1)
        cv2.putText(canvas,'APPLY',(485,61),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        cv2.imshow(self.name,canvas)
        def mouse(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN and 450<=x<=590 and 42<=y<=68:self.apply_requested=True
        cv2.setMouseCallback(self.name,mouse)
    def values(self):
        self.initial={name:cv2.getTrackbarPos(name,self.name) for name in self.initial}
        return {'rod_length':self.initial['Rod mm']/1000,
                'tcp_xyz_mm':[self.initial['TCP '+axis]-100 for axis in 'XYZ'],
                'tcp_rpy_deg':[self.initial['RPY '+axis]-90 for axis in 'XYZ'],
                'plane_offset_m':[(self.initial['Offset '+axis]-200)/1000 for axis in ['Right','Down']]}
