"""Join semantic masks, HSV lamp classification, temporal stability, and freshness."""
import time

from .signal_logic import StableSignal, analyze_frame


class SignalFramePipeline:
    def __init__(self, config):
        self.config = config
        self.stable = StableSignal()

    def set_config(self, config):
        if config != self.config:
            self.config = config
            self.stable.reset()

    def process(self, image_bgr, semantic_mask, run_id, frame_wall=None, now=None, source="live", target=None):
        now = time.time() if now is None else float(now)
        frame_wall = now if frame_wall is None else float(frame_wall)
        observation, crop, masks = analyze_frame(image_bgr, semantic_mask, self.config, target=target)
        result = self.stable.update(observation, self.config)
        fresh = 0.0 <= now - frame_wall <= self.config["stale_seconds"]
        result.update(
            time=now,
            frame_wall=frame_wall,
            fresh=fresh,
            source=source,
            run_id=run_id,
        )
        if not fresh:
            self.stable.reset()
            result.update(
                state="UNKNOWN",
                raw_state="UNKNOWN",
                decision="STOP",
                stop_required=True,
                permission_stable=False,
                reason="signal frame stale",
            )
        return result, crop, masks
