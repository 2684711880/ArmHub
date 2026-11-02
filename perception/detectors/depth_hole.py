"""Depth-only board aperture geometry. RGB is deliberately not an input."""
import cv2
import numpy as np


def _plane(points):
    center = points.mean(axis=0)
    _, _, axes = np.linalg.svd(points - center, full_matrices=False)
    normal = axes[2]
    if normal @ center > 0:
        normal = -normal
    rms = float(np.sqrt(np.mean(((points - center) @ normal) ** 2)))
    return center, normal, axes[:2], rms


def detect_depth_hole(depth, k, diameter_range=(.0425, .12)):
    depth = np.asarray(depth, dtype=float)
    k = np.asarray(k, dtype=float)
    if depth.ndim != 2 or k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Invalid depth image or intrinsics')
    h, w = depth.shape
    yy, xx = np.mgrid[:h, :w]
    rays = np.stack(((xx-k[0, 2])/k[0, 0], (yy-k[1, 2])/k[1, 1], np.ones_like(depth)), axis=-1)
    valid = np.isfinite(depth) & (depth > .08) & (depth < 3.)
    points = (rays * np.where(valid, depth, 0)[..., None])[valid]
    if len(points) < 1000:
        raise ValueError('Too few valid depth pixels')
    rng = np.random.default_rng(258)
    points = points[rng.choice(len(points), min(len(points), 1800), replace=False)]
    remaining = points.copy()
    candidates = []
    # Remove each dominant plane to also find a smaller foreground board when
    # the far background occupies most of the image.
    for _ in range(4):
        if len(remaining) < 120:
            break
        best = None
        for _ in range(100):
            a, b, c = remaining[rng.choice(len(remaining), 3, replace=False)]
            n = np.cross(b-a, c-a)
            norm = np.linalg.norm(n)
            if norm < 1e-8:
                continue
            n /= norm
            if abs(n[2]) < .2:
                continue
            mask = np.abs((remaining-a) @ n) < .006
            if best is None or mask.sum() > best.sum():
                best = mask
        if best is None or best.sum() < 120:
            break
        center, n, _, _ = _plane(remaining[best])
        remaining = remaining[~best]
        den = rays @ n
        predicted = np.divide(center @ n, den, out=np.full_like(depth, np.nan), where=np.abs(den) > .15)
        on_board = valid & (np.abs(depth-predicted) < .008)
        board = cv2.morphologyEx(on_board.astype(np.uint8)*255, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, hierarchy = cv2.findContours(board, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        if hierarchy is None:
            continue
        for i, contour in enumerate(contours):
            if hierarchy[0, i, 3] < 0 or len(contour) < 20 or cv2.contourArea(contour) < 100:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if x < 3 or y < 3 or x+bw > w-3 or y+bh > h-3:
                continue
            mask = np.zeros((h, w), np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            inside = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
            far_or_missing = (~valid) | (depth-predicted > .02)
            if inside.sum() < 50 or np.mean(far_or_missing[inside]) < .7:
                continue
            rim = (cv2.dilate(mask, np.ones((17, 17), np.uint8)) > 0) & ~(cv2.dilate(mask, np.ones((5, 5), np.uint8)) > 0) & on_board
            py, px = np.where(rim)
            if len(px) < 100:
                continue
            stride = max(1, len(px)//1800)
            py, px = py[::stride], px[::stride]
            rim_points = rays[py, px] * depth[py, px, None]
            pc, normal, axes, rms = _plane(rim_points)
            if rms > .004:
                continue
            uv = contour.reshape(-1, 2)
            rr = rays[uv[:, 1], uv[:, 0]]
            denominators = rr @ normal
            if np.min(np.abs(denominators)) < .15:
                continue
            boundary = rr * ((pc @ normal)/denominators)[:, None]
            local = (boundary-pc) @ axes.T
            (ex, ey), (aw, bw), _ = cv2.fitEllipse(local.astype(np.float32).reshape(-1, 1, 2))
            radius = float((aw+bw)/4)
            shape_rms = float(np.sqrt(np.mean((np.linalg.norm(local-[ex, ey], axis=1)-radius)**2)))
            if not diameter_range[0] < 2*radius < diameter_range[1] or min(aw, bw)/max(aw, bw) < .45 or shape_rms > radius*.25:
                continue
            candidates.append({'center': pc+np.array([ex, ey])@axes, 'normal': normal, 'radius': radius,
                               'plane_rms': rms, 'circle_rms': shape_rms, 'rim_samples': len(px),
                               'ellipse': cv2.fitEllipse(contour), 'mask': mask, 'source': 'depth_only'})
    if not candidates:
        raise ValueError('Depth: no enclosed aperture with a valid rim plane')
    # Require an unambiguous hole instead of switching among different objects.
    unique = []
    for c in candidates:
        if all(np.linalg.norm(c['center']-other['center']) > .02 for other in unique):
            unique.append(c)
    if len(unique) != 1:
        raise ValueError('Depth: multiple apertures; target ambiguous')
    return unique[0]
