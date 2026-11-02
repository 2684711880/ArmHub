"""Read-only overlays: no SDK, control queues, or movement commands."""
import time
import cv2
import numpy as np

COLORS = {'DEPTH': (70, 240, 90), 'KALMAN': (255, 220, 40),
          'PLAN HOLE': (30, 170, 255), 'TCP GOAL': (230, 80, 240)}


def camera_pose(status, stamp):
    poses = status.get('camera_pose_history', [])
    if not poses:
        return None, None
    closest = min(poses, key=lambda entry: abs(entry['stamp']-stamp))
    age = abs(closest['stamp']-stamp)
    if age > .12:
        return None, age
    return np.asarray(closest['T_base_camera'], dtype=float), age


def project(point, k, shape):
    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= .001:
        return None, 'BEHIND CAMERA'
    uv = np.asarray(k) @ point; uv = uv[:2]/uv[2]
    h, w = shape[:2]
    if not (0 <= uv[0] < w and 0 <= uv[1] < h):
        return None, 'OUTSIDE IMAGE'
    return tuple(np.rint(uv).astype(int)), ''


class HoleOverlay:
    def __init__(self):
        self.frame = 'camera'

    def toggle(self):
        self.frame = 'base' if self.frame == 'camera' else 'camera'

    def points(self, packet, status):
        transform, pose_age = camera_pose(status, packet.get('stamp', 0.))
        rows = []
        measured = packet.get('measured_center_optical_m')
        if measured is not None:
            camera = np.asarray(measured, dtype=float)
            base = None if transform is None else transform[:3, :3]@camera+transform[:3, 3]
            rows.append({'name': 'DEPTH', 'camera': camera, 'base': base})
        for name, key in [('KALMAN', 'hole_estimate_base_m'), ('PLAN HOLE', 'plan_hole_base_m'), ('TCP GOAL', 'tcp_goal_base_m')]:
            if status.get(key) is None:
                continue
            base = np.asarray(status[key], dtype=float)
            camera = None if transform is None else transform[:3, :3].T@(base-transform[:3, 3])
            rows.append({'name': name, 'camera': camera, 'base': base})
        tcp = status.get('current_tcp_base_m')
        for row in rows:
            row['coordinates'] = row[self.frame]
            row['frame_range_m']=None if row['coordinates'] is None else float(np.linalg.norm(row['coordinates']))
            row['camera_range_m'] = None if row['camera'] is None else float(np.linalg.norm(row['camera']))
            row['tcp_distance_m'] = None if row['base'] is None or tcp is None else float(np.linalg.norm(row['base']-tcp))
            normal=status.get('plan_plane_normal_base' if row['name']=='PLAN HOLE' else 'hole_plane_normal_base')
            row['plane_gap_m']=None if row['name']=='TCP GOAL' or normal is None or row['base'] is None or tcp is None else float((np.asarray(tcp)-row['base'])@np.asarray(normal))
        return rows, pose_age

    def render(self, image, packet, status):
        h, w = image.shape[:2]; panel_width = 420
        canvas = np.zeros((max(h, 480), w+panel_width, 3), np.uint8)
        canvas[:, w:] = (26, 29, 34); canvas[:h, :w] = image
        rows, pose_age = self.points(packet, status)
        k = packet.get('K')
        # Feature labels avoid overlapping each other when all centers coincide.
        offsets = {'DEPTH': (12, -24), 'KALMAN': (12, -5), 'PLAN HOLE': (12, 14), 'TCP GOAL': (12, 33)}
        for row in rows:
            if k is not None and row['camera'] is not None:
                uv, visibility = project(row['camera'], k, image.shape)
                row['visibility'] = visibility
                if uv is not None:
                    color = COLORS[row['name']]
                    marker = cv2.MARKER_CROSS if row['name'] == 'DEPTH' else (cv2.MARKER_DIAMOND if row['name'] == 'KALMAN' else cv2.MARKER_TILTED_CROSS)
                    cv2.drawMarker(canvas, uv, color, marker, 16, 2)
                    dx, dy = offsets[row['name']]
                    cv2.putText(canvas, row['name'], (min(w-110, max(0, uv[0]+dx)), min(h-10, max(15, uv[1]+dy))), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1)
        left = w+14
        frame = 'CAMERA OPTICAL (right/down/forward)' if self.frame == 'camera' else 'BASE_LINK (robot base axes)'
        cv2.rectangle(canvas, (w+10, 12), (w+panel_width-10, 47), (65, 72, 82), -1)
        cv2.putText(canvas, 'f / click: SWITCH COORDINATE FRAME', (left+4, 35), cv2.FONT_HERSHEY_SIMPLEX, .49, (255, 255, 255), 1)
        cv2.putText(canvas, frame, (left, 70), cv2.FONT_HERSHEY_SIMPLEX, .45, (210, 220, 230), 1)
        for i, row in enumerate(rows):
            y = 102+i*76; color = COLORS[row['name']]
            cv2.putText(canvas, row['name'], (left, y), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1)
            xyz = row['coordinates']
            text = 'XYZ unavailable: waiting for matching pose' if xyz is None else f'XYZ(m): {xyz[0]:+.3f} {xyz[1]:+.3f} {xyz[2]:+.3f}'
            cv2.putText(canvas, text, (left, y+21), cv2.FONT_HERSHEY_SIMPLEX, .45, (230, 230, 230), 1)
            cam = '--' if row['camera_range_m'] is None else f"{row['camera_range_m']:.3f}"
            tcp = '--' if row['tcp_distance_m'] is None else f"{row['tcp_distance_m']:.3f}"
            origin='--' if row['frame_range_m'] is None else f"{row['frame_range_m']:.3f}"
            cv2.putText(canvas, f'Cam: {cam}m | TCP: {tcp}m | Origin: {origin}m', (left, y+41), cv2.FONT_HERSHEY_SIMPLEX, .42, (190, 200, 210), 1)
            if row.get('visibility'):
                cv2.putText(canvas, row['visibility'], (left+135, y), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        plane=next((r for r in rows if r['name']=='PLAN HOLE' and r['plane_gap_m'] is not None),None)
        if plane is None:plane=next((r for r in rows if r['plane_gap_m'] is not None),None)
        if plane is not None:cv2.putText(canvas,f"TCP to hole plane: {plane['plane_gap_m']:+.3f}m",(left,394),cv2.FONT_HERSHEY_SIMPLEX,.46,(230,230,230),1)
        y = 416
        if status.get('hole_estimate_mode'):
            cv2.putText(canvas, f"KF {status['hole_estimate_mode']} age={status['hole_estimate_age_s']:.2f}s std={status['hole_position_std_m']*1000:.1f}mm", (left, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (190, 215, 230), 1)
        cv2.putText(canvas, 'Plan: '+status.get('overlay_plan_mode', 'NONE'), (left, y+21), cv2.FONT_HERSHEY_SIMPLEX, .44, (190, 215, 230), 1)
        age = time.time()-packet.get('stamp', time.time())
        quality = 'NO MATCHING POSE' if pose_age is None or pose_age > .12 else f'pose offset={pose_age*1000:.0f}ms'
        cv2.putText(canvas, f'Image age={age:.2f}s | {quality}', (left, y+43), cv2.FONT_HERSHEY_SIMPLEX, .40, (160, 180, 200), 1)
        if age > .7:
            cv2.putText(canvas, 'STALE IMAGE', (12, h-36), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
        return canvas
