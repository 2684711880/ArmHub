"""Read-only FK from ArmHub model; no ROS package or motor construction."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
def rigid(xyz,rpy):
 r,p,y=rpy;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
 t=np.eye(4);t[:3,3]=xyz
 t[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
 return t
class ArmFK:
 def __init__(self):
  model=Path(__file__).resolve().parents[2]/'robots/panthera_ht/description/planning.urdf'
  root=ET.parse(model).getroot();self.joints={}
  for j in root.findall('joint'):
   origin=j.find('origin');axis=j.find('axis')
   xyz=[float(x) for x in (origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()]
   rpy=[float(x) for x in (origin.get('rpy','0 0 0') if origin is not None else '0 0 0').split()]
   ax=[float(x) for x in (axis.get('xyz','1 0 0') if axis is not None else '1 0 0').split()]
   self.joints[j.find('child').get('link')]={'name':j.get('name'),'parent':j.find('parent').get('link'),'type':j.get('type'),'origin':rigid(xyz,rpy),'axis':np.asarray(ax)}
 def transform(self,positions,base='base_link',tip='gripper_center'):
  chain=[];child=tip;seen=set()
  while child!=base:
   if child in seen or child not in self.joints:raise ValueError('No valid base-to-TCP chain')
   seen.add(child);j=self.joints[child];chain.append(j);child=j['parent']
  t=np.eye(4)
  for j in reversed(chain):
   t=t@j['origin'];motion=np.eye(4)
   if j['type'] in ['revolute','continuous']:
    q=float(positions[j['name']]);axis=j['axis']/np.linalg.norm(j['axis']);x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    motion[:3,:3]=np.eye(3)+math.sin(q)*k+(1-math.cos(q))*(k@k)
   elif j['type']=='prismatic':motion[:3,3]=j['axis']*float(positions[j['name']])
   elif j['type']!='fixed':raise ValueError('Unsupported chain joint type')
   t=t@motion
  if not np.isfinite(t).all():raise ValueError('Nonfinite FK')
  return t

def quaternion(m):
 tr=np.trace(m)
 if tr>0:
  s=math.sqrt(tr+1)*2;q=[(m[2,1]-m[1,2])/s,(m[0,2]-m[2,0])/s,(m[1,0]-m[0,1])/s,.25*s]
 else:
  i=int(np.argmax(np.diag(m)));j=(i+1)%3;k=(i+2)%3;s=math.sqrt(1+m[i,i]-m[j,j]-m[k,k])*2
  q=[0.,0.,0.,0.];q[i]=.25*s;q[j]=(m[j,i]+m[i,j])/s;q[k]=(m[k,i]+m[i,k])/s;q[3]=(m[k,j]-m[j,k])/s
 return np.asarray(q)/np.linalg.norm(q)
