"""Exercise the actual official record loop with human gates and no hardware."""
from contextlib import ExitStack, nullcontext
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock,patch
import manual_review as gates

class ReviewTests(unittest.TestCase):
    def test_review_disconnects_before_prompt_and_does_not_keep_on_enter(self):
        robot=SimpleNamespace(is_connected=True)
        events=[]
        def disconnect():robot.is_connected=False;events.append('held_and_closed')
        robot.disconnect=disconnect
        replies=iter(['','invalid','r'])
        def ask(prompt):
            self.assertFalse(robot.is_connected)
            return next(replies)
        ds=SimpleNamespace(episode_buffer={'size':10})
        self.assertEqual(gates.review_round(robot,ds,{},ask=ask),'retry')
        self.assertEqual(events,['held_and_closed'])

    def test_begin_waits_for_both_explicit_inputs(self):
        robot=Mock(is_connected=False,_armhub_review_ready=False);events={}
        order=[]
        replies=iter(['','n'])
        def ask(prompt):order.append('input');return next(replies)
        prepare=lambda **kw:order.append('prepare')
        robot.connect.side_effect=lambda:order.append('connect')
        self.assertTrue(gates.begin_round(robot,events,0,50,ask=ask,prepare=prepare))
        self.assertEqual(order,['input','prepare','input','connect'])
        self.assertFalse(any(events.values()))
        robot.enable_teaching_mode.assert_called_once()

    def test_quit_before_preparation_never_moves_or_connects(self):
        robot=Mock(is_connected=False,_armhub_review_ready=False);prepare=Mock()
        self.assertFalse(gates.begin_round(robot,{},0,50,ask=lambda p:'q',prepare=prepare))
        prepare.assert_not_called();robot.connect.assert_not_called()

    def test_ready_pose_waits_for_n_without_another_preparation_and_reports_index(self):
        from contextlib import redirect_stdout
        from io import StringIO
        robot=Mock(is_connected=False,_armhub_review_ready=True);prepare=Mock()
        replies=iter(['','n'])
        def ask(prompt):
            robot.connect.assert_not_called()
            return next(replies)
        output=StringIO()
        with redirect_stdout(output):
            self.assertTrue(gates.begin_round(robot,{},3,50,ask=ask,prepare=prepare))
        prepare.assert_not_called();robot.connect.assert_called_once()
        self.assertIn('正在录制第 4/50 条',output.getvalue())
        self.assertIn('已成功保留 3 条',output.getvalue())

    def test_empty_dataset_is_archived_with_recoverable_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'test';(root/'meta').mkdir(parents=True)
            (root/'meta/info.json').write_text('{"total_episodes":0}')
            (root/'recoverable.png').write_bytes(b'raw')
            self.assertFalse(gates.prepare_review_dataset(root,'local/test'))
            archived=list(Path(temp).glob('test.empty-*'))
            self.assertEqual(len(archived),1)
            self.assertEqual((archived[0]/'recoverable.png').read_bytes(),b'raw')
            self.assertFalse(root.exists())

    def run_official(self, initial, decisions, goal, fail_second_prepare=False):
        import lerobot.scripts.lerobot_record as m
        with tempfile.TemporaryDirectory() as temp,ExitStack() as stack:
            root=Path(temp)/'data';root.mkdir();(root/'meta').mkdir()
            (root/'meta/info.json').write_text(json.dumps({'total_episodes':initial}))
            robot=SimpleNamespace(name='panthera_follower',is_connected=False,cameras={},action_features={},observation_features={},teaching_mode=False)
            def connect():robot.is_connected=True
            def disconnect():robot.is_connected=False
            robot.connect=Mock(side_effect=connect);robot.disconnect=Mock(side_effect=disconnect)
            robot.enable_teaching_mode=Mock()
            class Dataset:
                def __init__(self):
                    self.root=root;self.num_episodes=initial;self.saved=[];self.cleared=0
                    self.episode_buffer={'size':0,'episode_index':initial}
                def clear_episode_buffer(self):
                    self.cleared+=1;self.episode_buffer={'size':0,'episode_index':self.num_episodes}
                def save_episode(self):
                    self.saved.append(self.episode_buffer['attempt'])
                    self.num_episodes+=1
                    self.episode_buffer={'size':0,'episode_index':self.num_episodes}
                def finalize(self):pass
            ds=Dataset();attempts=[]
            def record_loop(**kw):
                self.assertTrue(robot.is_connected)
                self.assertEqual(kw['control_time_s'],float('inf'))
                attempts.append(len(attempts));ds.episode_buffer.update(size=10,attempt=len(attempts))
            replies=['']
            for decision in decisions:replies+=['n',decision]
            replies=iter(replies)
            ask=lambda p:next(replies)
            real_begin=gates.begin_round;real_review=gates.review_round
            prep_count=[0]
            def fake_prepare(**kw):
                prep_count[0]+=1
                if fail_second_prepare and prep_count[0]==2:
                    raise RuntimeError('return failed after save')
            prep=Mock(side_effect=fake_prepare)
            real_reset=gates.reset_ready
            stack.enter_context(patch.object(gates,'reset_ready',side_effect=lambda robot,**kw:real_reset(robot,prepare=kw.get('prepare',prep))))
            stack.enter_context(patch.object(gates,'begin_round',side_effect=lambda *a:real_begin(*a,ask=ask,prepare=prep)))
            stack.enter_context(patch.object(gates,'review_round',side_effect=lambda *a:real_review(*a,ask=ask)))
            dataset_cls=Mock(return_value=ds);dataset_cls.create.return_value=ds
            def create_dataset(*args,**kwargs):
                root.mkdir(exist_ok=True)
                return ds
            dataset_cls.create.side_effect=create_dataset
            replacements={'asdict':lambda x:{},'init_logging':lambda:None,'log_say':lambda *a,**kw:None,
                'make_robot_from_config':lambda cfg:robot,'make_default_processors':lambda:(None,None,None),
                'combine_feature_dicts':lambda *a:{},'aggregate_pipeline_dataset_features':lambda **kw:{},
                'create_initial_features':lambda **kw:{},'LeRobotDataset':dataset_cls,
                'sanity_check_dataset_name':lambda *a:None,'sanity_check_dataset_robot_compatibility':lambda *a:None,
                'init_keyboard_listener':lambda:(None,{'exit_early':False,'rerecord_episode':False,'stop_recording':False}),
                'VideoEncodingManager':lambda d:nullcontext(),'record_loop':record_loop,'is_headless':lambda:True}
            for name,value in replacements.items():stack.enter_context(patch.object(m,name,value))
            dataset_cfg=SimpleNamespace(root=root,repo_id='local/test',num_episodes=goal,fps=20,
                video=True,video_encoding_batch_size=1,num_image_writer_processes=0,
                num_image_writer_threads_per_camera=1,single_task='blue block to box',
                episode_time_s=float('inf'),reset_time_s=0,push_to_hub=False)
            cfg=SimpleNamespace(robot=None,dataset=dataset_cfg,manual_review=True,display_data=False,
                teleop=None,policy=None,resume=False,play_sounds=False)
            if fail_second_prepare:
                with self.assertRaisesRegex(RuntimeError,'return failed after save'):
                    m.record.__wrapped__(cfg)
            else:
                m.record.__wrapped__(cfg)
            review_rows=[json.loads(line) for line in (root/'manual_review.jsonl').read_text().splitlines()] if (root/'manual_review.jsonl').exists() else []
            return ds.saved,ds.cleared,ds.num_episodes,len(attempts),prep.call_count,review_rows

    def test_real_official_loop_reject_does_not_count_and_keep_appends(self):
        saved,cleared,count,attempts,preps,rows=self.run_official(0,['r','s','s'],2)
        self.assertEqual((saved,cleared,count,attempts,preps),([2,3],1,2,3,4))
        self.assertEqual([r['decision'] for r in rows],['retry','kept','kept'])
        self.assertEqual([r['episode_index'] for r in rows],[0,0,1])
        self.assertEqual(rows[-1]['frames'],10)

    def test_resume_counts_existing_successes_towards_goal(self):
        saved,cleared,count,attempts,preps,_=self.run_official(1,['s'],2)
        self.assertEqual((saved,cleared,count,attempts,preps),([1],0,2,1,2))

    def test_quit_discards_current_without_saving_or_starting_another(self):
        saved,cleared,count,attempts,preps,_=self.run_official(0,['q'],2)
        self.assertEqual((saved,cleared,count,attempts,preps),([],1,0,1,1))

    def test_return_failure_after_s_does_not_lose_saved_data(self):
        saved,cleared,count,attempts,preps,rows=self.run_official(0,['s'],2,fail_second_prepare=True)
        self.assertEqual((saved,cleared,count,attempts,preps),([1],0,1,1,2))
        self.assertEqual(rows[-1]['decision'],'kept')

    def test_real_dataset_reject_keep_append_and_decode(self):
        import numpy as np
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        names=[f'joint{i}.pos' for i in range(1,7)]+['gripper.pos']
        features={'observation.state':{'dtype':'float32','shape':(7,),'names':names},
                  'action':{'dtype':'float32','shape':(7,),'names':names},
                  'observation.images.d435':{'dtype':'video','shape':(32,32,3),'names':['height','width','channels']}}
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'dataset'
            ds=LeRobotDataset.create('demo/review_test',20,root=root,robot_type='panthera_follower',
                features=features,use_videos=True,video_backend='pyav')
            robot=SimpleNamespace(is_connected=False)
            try:
                for attempt,decision in enumerate(('r','s','s')):
                    for i in range(8):
                        ds.add_frame({'observation.state':np.full(7,attempt,np.float32),
                            'action':np.full(7,attempt,np.float32),
                            'observation.images.d435':np.full((32,32,3),attempt*50,np.uint8),'task':'blue to box'})
                    outcome=gates.review_round(robot,ds,{},ask=lambda p:decision)
                    if outcome=='keep':ds.save_episode(parallel_encoding=False)
                    else:ds.clear_episode_buffer()
                self.assertEqual(ds.num_episodes,2)
            finally:
                ds.finalize();ds.stop_image_writer()
            loaded=LeRobotDataset('demo/review_test',root=root,video_backend='pyav')
            self.assertEqual(loaded.meta.total_episodes,2)
            self.assertEqual(len(loaded),16)
            self.assertEqual(float(loaded[0]['action'][0]),1.)
            self.assertEqual(float(loaded[8]['action'][0]),2.)
            self.assertEqual(tuple(loaded[0]['observation.images.d435'].shape),(3,32,32))

if __name__=='__main__':unittest.main()
