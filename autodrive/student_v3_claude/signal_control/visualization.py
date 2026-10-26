"""Render the exact signal frame and HSV masks; never alter perception/control."""
import cv2
import numpy as np

from .signal_logic import roi_pixels

COLORS = {"red": (40, 50, 255), "yellow": (0, 225, 255), "green": (70, 230, 70)}


def jpeg(image, max_width=960):
    if image.shape[1] > max_width:
        image = cv2.resize(image, (max_width, round(image.shape[0]*max_width/image.shape[1])))
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 78])
    if not ok:
        raise ValueError("signal preview encoding failed")
    return encoded.tobytes()


def crop_panel(image):
    if image is None or not image.size:
        panel = np.full((120, 480, 3), 20, np.uint8)
        cv2.putText(panel, "No signal housing", (90, 67), cv2.FONT_HERSHEY_SIMPLEX, .6, (160, 160, 160), 1)
        return panel
    scale = min(8., 640./image.shape[1], 220./image.shape[0])
    return cv2.resize(image, (max(1, round(image.shape[1]*scale)),
                              max(1, round(image.shape[0]*scale))), interpolation=cv2.INTER_NEAREST)


def signal_debug_images(image, semantic_mask, report, crop, masks, config):
    original = image.copy()
    x1, y1, x2, y2 = roi_pixels(config, image.shape)
    cv2.rectangle(original, (x1, y1), (x2-1, y2-1), (220, 180, 70), 2)
    search = report.get("mapped_search_box")
    if search is not None:
        a, b, c, d = (round(value) for value in search)
        cv2.rectangle(original, (a, b), (c, d), (255, 180, 60), 2)
        cv2.putText(original, report.get("traffic_light_id", ""), (a, max(14, b-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 180, 60), 1)
    for candidate in report.get("candidates", ()):
        x, y, w, h = candidate["box"]
        cv2.rectangle(original, (x, y), (x+w, y+h), (150, 150, 150), 1)
    box = report.get("target_box")
    if box is not None:
        x, y, w, h = box
        cv2.rectangle(original, (x, y), (x+w, y+h), (70, 240, 120), 2)
        cv2.putText(original, report.get("raw_state", "UNKNOWN"), (x, max(18, y-7)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (70, 240, 120), 2)
    semantic = np.repeat((semantic_mask > 0).astype(np.uint8)[:, :, None]*255, 3, axis=2)
    panel = crop.copy() if crop is not None else None
    if panel is not None:
        for slot in report.get("slots", {}).values():
            x, y, w, h = slot["box"]
            cv2.rectangle(panel, (x, y), (x+w-1, y+h-1),
                          (70, 240, 120) if slot["on"] else (100, 100, 100), 1)
    result = {"original": jpeg(original), "semantic": jpeg(semantic), "crop": jpeg(crop_panel(panel))}
    for color, bgr in COLORS.items():
        mask = masks.get(color)
        tile = np.zeros((*mask.shape, 3), np.uint8) if mask is not None else np.zeros((24, 100, 3), np.uint8)
        if mask is not None:
            tile[mask > 0] = bgr
        result[color] = jpeg(crop_panel(tile))
    return result
