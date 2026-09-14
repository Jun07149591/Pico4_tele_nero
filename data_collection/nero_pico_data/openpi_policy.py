"""Nero-specific OpenPI input/output transforms; no ALOHA joint conversions."""

from dataclasses import dataclass

import numpy as np

from .schema import arm_order

IMAGE_KEYS = {"front": "base_0_rgb", "left_wrist": "left_wrist_0_rgb", "right_wrist": "right_wrist_0_rgb"}


def parse_image(image):
    image = np.asarray(image)
    if image.ndim != 3:
        raise ValueError("RGB image must have three dimensions")
    if image.shape[0] == 3 and image.shape[-1] != 3:
        image = np.transpose(image, (1, 2, 0))
    if image.shape[-1] != 3:
        raise ValueError("RGB image must have three channels")
    if np.issubdtype(image.dtype, np.floating):
        if not np.isfinite(image).all() or np.min(image) < 0 or np.max(image) > 1:
            raise ValueError("floating RGB image must be in [0, 1]")
        image = np.rint(image * 255).astype(np.uint8)
    if image.dtype != np.uint8:
        raise ValueError("RGB image must be uint8 or float [0, 1]")
    return image


@dataclass(frozen=True)
class NeroInputs:
    mode: str = "single"
    cameras: tuple[str, ...] = ("front", "right_wrist")

    def __call__(self, data):
        dim = 8 * len(arm_order(self.mode))
        state = np.asarray(data["state"], dtype=np.float32)
        if state.shape != (dim,) or not np.isfinite(state).all():
            raise ValueError(f"expected {dim}-dimensional Nero state")
        parsed = {role: parse_image(data["images"][role]) for role in self.cameras}
        base = parsed["front"]
        output = {"state": state, "image": {}, "image_mask": {}}
        for role, key in IMAGE_KEYS.items():
            output["image"][key] = parsed[role] if role in parsed else np.zeros_like(base)
            output["image_mask"][key] = np.bool_(role in parsed)
        if "actions" in data:
            actions = np.asarray(data["actions"], dtype=np.float32)
            if actions.shape[-1] != dim or not np.isfinite(actions).all():
                raise ValueError(f"expected {dim}-dimensional Nero actions")
            output["actions"] = actions
        if "prompt" in data:
            output["prompt"] = data["prompt"]
        return output


@dataclass(frozen=True)
class NeroOutputs:
    mode: str = "single"

    def __call__(self, data):
        return {"actions": np.asarray(data["actions"])[..., :8 * len(arm_order(self.mode))]}
