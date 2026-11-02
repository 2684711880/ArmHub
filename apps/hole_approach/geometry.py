import numpy as np

def plane_compensation(normal_base,t_base_camera,right_down_m):
 """Metric correction on the board plane, right/down as seen by the camera."""
 values=np.asarray(right_down_m,dtype=float)
 if values.shape!=(2,) or not np.isfinite(values).all():raise ValueError('Require two finite right/down compensation values')
 if not np.any(values):return np.zeros(3)
 n=np.asarray(normal_base,dtype=float);n=n/np.linalg.norm(n)
 right=t_base_camera[:3,0];right=right-n*np.dot(n,right)
 if np.linalg.norm(right)<.1:raise ValueError('Camera right axis nearly normal to board; compensation direction undefined')
 right/=np.linalg.norm(right);down=np.cross(n,right)
 if np.dot(down,t_base_camera[:3,1])<0:down=-down
 return values[0]*right+values[1]*down

def target_from_observation(center,normal,fk,t_tcp_camera,standoff,orientation='align'):
 t=np.asarray(fk['transform'])@t_tcp_camera;p=(t@np.r_[center,1])[:3];n=t[:3,:3]@normal;n/=np.linalg.norm(n)
 pos=p+standoff*n
 if orientation=='keep':rot=np.asarray(fk['rotation'])
 else:
  x=-n;z=np.array([0.,0.,1.]);z-=x*(z@x)
  if np.linalg.norm(z)<.1:raise ValueError('Near-vertical hole: choose --orientation keep or define tool roll')
  z/=np.linalg.norm(z);y=np.cross(z,x);rot=np.column_stack([x,y,z])
 return p,n,pos,rot
