"""Rectify supported RealSense models via SDK deprojection, not coefficient reinterpretation."""
import numpy as np
import cv2
import pyrealsense2 as rs
from scipy.interpolate import LinearNDInterpolator


def build_maps(intr,step=8):
    if intr.model not in (rs.distortion.none,rs.distortion.brown_conrady,rs.distortion.inverse_brown_conrady):
        raise ValueError('Unsupported RealSense deprojection model: '+str(intr.model))
    u=np.unique(np.r_[np.arange(0,intr.width,step),intr.width-1]);v=np.unique(np.r_[np.arange(0,intr.height,step),intr.height-1])
    xx,yy=np.meshgrid(u,v);raw=np.c_[xx.ravel(),yy.ravel()].astype(float)
    rays=np.array([rs.rs2_deproject_pixel_to_point(intr,p.tolist(),1.) for p in raw])
    ideal=rays[:,:2]/rays[:,2,None]*[intr.fx,intr.fy]+[intr.ppx,intr.ppy]
    gx,gy=np.meshgrid(np.arange(intr.width),np.arange(intr.height));query=np.c_[gx.ravel(),gy.ravel()]
    maps=LinearNDInterpolator(ideal,raw,fill_value=-1)(query).reshape(intr.height,intr.width,2).astype(np.float32)
    return maps[:,:,0].copy(),maps[:,:,1].copy()


def rectify(image,maps):return cv2.remap(image,*maps,interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
