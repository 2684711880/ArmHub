"""Tool geometry layered over the SDK gripper-tip model, never camera extrinsics."""
import numpy as np
from scipy.spatial.transform import Rotation


def validate_settings(values):
    expected={'rod_length','tcp_xyz_mm','tcp_rpy_deg','plane_offset_m'}
    if values is None or set(values)!=expected:raise ValueError('Incomplete tool calibration settings')
    result={k:np.asarray(v,dtype=float) for k,v in values.items()}
    if result['rod_length'].shape!=() or not 0<=result['rod_length']<=.3:raise ValueError('Rod length must be 0..0.3 m')
    for k,n,limit in [('tcp_xyz_mm',3,100),('tcp_rpy_deg',3,90),('plane_offset_m',2,.2)]:
        if result[k].shape!=(n,) or not np.isfinite(result[k]).all() or np.max(np.abs(result[k]))>limit:raise ValueError(f'Invalid {k}')
    return {k:(float(v) if v.shape==() else v.tolist()) for k,v in result.items()}


def tool_transform(args):
    values=validate_settings({k:getattr(args,k,d) for k,d in [('rod_length',.04),('tcp_xyz_mm',[0,0,0]),('tcp_rpy_deg',[0,0,0]),('plane_offset_m',[0,0])]})
    transform=np.eye(4)
    transform[:3,:3]=Rotation.from_euler('xyz',values['tcp_rpy_deg'],degrees=True).as_matrix()
    transform[:3,3]=np.asarray(values['tcp_xyz_mm'])/1000+np.array([values['rod_length'],0,0])
    return transform


class ToolKinematics:
    def __init__(self,robot,transform):
        self.robot=robot;self.transform=transform
    def __getattr__(self,name):return getattr(self.robot,name)
    @property
    def q(self):return self.robot.q
    @q.setter
    def q(self,value):self.robot.q=value
    def forward_kinematics(self,q):
        t=np.asarray(self.robot.forward_kinematics(q)['transform'])@self.transform
        return {'transform':t,'position':t[:3,3].tolist(),'rotation':t[:3,:3]}
    def inverse_kinematics(self,position,rotation,*args,**kwargs):
        target=np.eye(4);target[:3,3]=position;target[:3,:3]=rotation
        grip=target@np.linalg.inv(self.transform)
        return self.robot.inverse_kinematics(grip[:3,3].tolist(),grip[:3,:3],*args,**kwargs)
