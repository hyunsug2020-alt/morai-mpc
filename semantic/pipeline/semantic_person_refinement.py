"""Recheck small person components at two scales of the same camera frame.

The original person label survives ambiguous, clipped or conflicting evidence.
Only two current obstacle classifications may replace that component; fusion
continues to publish and check its measured physical obstacle footprint.
"""
import cv2
import numpy as np
from functools import lru_cache


def small_person_component(mask, placement, image_shape):
    """Select a complete small component nearest the camera in image ground order."""
    height, width = image_shape[:2]
    left, top, pw, ph = (float(placement[k]) for k in ('left', 'top', 'width', 'height'))
    if min(height, width, pw, ph) <= 0:
        return None
    count, labels, boxes, _ = cv2.connectedComponentsWithStats((mask == 1).astype(np.uint8), 8)
    candidates = []
    for index in range(1, count):
        x, y, w, h, area = (int(v) for v in boxes[index])
        if (not 8 <= area <= 1600 or not 4 <= h <= 64 or w > 40
                or x <= left or y <= top or x+w >= left+pw or y+h >= top+ph):
            continue
        # A farther post can have a larger noisy blob than the near post
        # currently holding the car. Spend the same two-context GPU budget on
        # the lowest complete component first, then prefer its image support.
        candidates.append((y+h, area, index))
    if not candidates:
        return None
    _, _, index = max(candidates)
    yy, xx = np.nonzero(labels == index)
    source = np.column_stack(((xx+.5-left)*width/pw, (yy+.5-top)*height/ph))
    return dict(yy=yy, xx=xx, source_xy=source, image_shape=(height, width))


def context_rois(component):
    """Two distinct, bounded contexts in original camera coordinates."""
    height, width = component['image_shape']
    center = component['source_xy'].mean(axis=0)
    result = []
    for fraction in (.4, .3):
        w, h = max(1, round(width*fraction)), max(1, round(height*fraction))
        x = max(0, min(width-w, int(center[0]-w/2)))
        y = max(0, min(height-h, int(center[1]-h/2)))
        source = component['source_xy']
        if (np.any(source[:, 0] < x) or np.any(source[:, 0] >= x+w)
                or np.any(source[:, 1] < y) or np.any(source[:, 1] >= y+h)):
            return []
        result.append((x, y, w, h))
    return result if result[0] != result[1] else []


def merge_contexts(original, component, contexts):
    """Require obstacle majority and almost no person support at both scales."""
    unchanged = dict(relabelled_pixels=0, obstacle_fractions=[])
    if len(contexts) != 2 or contexts[0]['roi'] == contexts[1]['roi']:
        return original, unchanged
    fractions = []
    for context in contexts:
        x, y, w, h = context['roi']
        placement, mask = context['placement'], context['mask']
        source = component['source_xy']
        xx = np.floor(placement['left']+(source[:, 0]-x)*placement['width']/w).astype(int)
        yy = np.floor(placement['top']+(source[:, 1]-y)*placement['height']/h).astype(int)
        if (np.any(xx < 0) or np.any(yy < 0) or np.any(xx >= mask.shape[1])
                or np.any(yy >= mask.shape[0])):
            return original, unchanged
        observed = mask[yy, xx]
        fraction = float(np.mean(observed == 2))
        fractions.append(fraction)
        if fraction < .6 or float(np.mean(observed == 1)) > .02:
            return original, dict(unchanged, obstacle_fractions=fractions)
    yy, xx = component['yy'], component['xx']
    if not np.all(original[yy, xx] == 1):
        return original, unchanged
    result = original.copy()
    result[yy, xx] = 2
    return result, dict(relabelled_pixels=len(xx), obstacle_fractions=fractions)


def forward_person_rois(image_shape):
    """Magnify the forward crossing view without changing camera resolution."""
    height, width = image_shape[:2]
    return [((width-round(width*fraction))//2, (height-round(height*fraction))//2,
             round(width*fraction), round(height*fraction)) for fraction in (.5, .4)]


@lru_cache(maxsize=4)
def _context_projection(shape, placement, image_shape, roi, context_place, context_shape):
    left, top, width, height = placement
    x, y, rw, rh = roi
    cl, ct, cw, ch = context_place
    if min(width, height, rw, rh, cw, ch, *context_shape) <= 0:
        raise ValueError('invalid person context geometry')
    yy, xx = np.indices(shape, dtype=np.float32)
    source_x = (xx+.5-left)*image_shape[1]/width
    source_y = (yy+.5-top)*image_shape[0]/height
    cx = np.floor(cl+(source_x-x)*cw/rw).astype(np.int32)
    cy = np.floor(ct+(source_y-y)*ch/rh).astype(np.int32)
    valid = ((source_x >= x) & (source_x < x+rw) & (source_y >= y) & (source_y < y+rh)
             & (cx >= 0) & (cx < context_shape[1]) & (cy >= 0) & (cy < context_shape[0]))
    return np.clip(cy, 0, context_shape[0]-1), np.clip(cx, 0, context_shape[1]-1), valid


def recover_person_contexts(original, placement, image_shape, contexts, protected=None):
    """Recover people only where two current magnified views agree.

    Vehicles, every existing person and independently confirmed posts remain
    unchanged. Two positive magnified views may correct one weak base label.
    The same source frame owns both magnified observations and the base mask.
    """
    unchanged = dict(added_person_pixels=0, contexts=len(contexts))
    if len(contexts) != 2 or contexts[0]['roi'] == contexts[1]['roi']:
        return original, unchanged
    consensus = (original != 0) & (original != 1) & (original != 254)
    if protected is not None:
        consensus &= ~np.asarray(protected, bool)
    keys = ('left', 'top', 'width', 'height')
    for context in contexts:
        mask = context['mask']
        yy, xx, valid = _context_projection(tuple(original.shape),
            tuple(placement[k] for k in keys), tuple(image_shape[:2]), tuple(context['roi']),
            tuple(context['placement'][k] for k in keys), tuple(mask.shape))
        consensus &= valid & (mask[yy, xx] == 1)
    count, labels, boxes, _ = cv2.connectedComponentsWithStats(consensus.astype(np.uint8), 8)
    accepted = [index for index in range(1, count)
                if boxes[index, cv2.CC_STAT_AREA] >= 8 and boxes[index, cv2.CC_STAT_HEIGHT] >= 4]
    if not accepted:
        return original, unchanged
    recovered = np.isin(labels, accepted)
    result = original.copy()
    result[recovered] = 1
    return result, dict(added_person_pixels=int(recovered.sum()), contexts=2)
