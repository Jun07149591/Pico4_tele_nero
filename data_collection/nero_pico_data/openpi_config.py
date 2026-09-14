"""Loaded only inside an installed OpenPI environment."""

import dataclasses

from openpi import transforms
from openpi.training import config as training

from .openpi_policy import NeroInputs, NeroOutputs


@dataclasses.dataclass(frozen=True)
class NeroDataConfig(training.DataConfigFactory):
    mode: str = "single"
    cameras: tuple[str, ...] = ("front", "right_wrist")

    def create(self, assets_dirs, model_config):
        repack = transforms.Group(inputs=[transforms.RepackTransform({
            "state": "observation.state", "actions": "action", "prompt": "prompt",
            "images": {role: f"observation.images.{role}" for role in self.cameras}})])
        mask = transforms.make_bool_mask(*( (7, -1) if self.mode == "single" else (7, -1, 7, -1)))
        data = transforms.Group(inputs=[NeroInputs(self.mode, self.cameras)], outputs=[NeroOutputs(self.mode)])
        data = data.push(inputs=[transforms.DeltaActions(mask)], outputs=[transforms.AbsoluteActions(mask)])
        return dataclasses.replace(self.create_base_config(assets_dirs, model_config),
            repack_transforms=repack, data_transforms=data,
            model_transforms=training.ModelTransformFactory()(model_config), action_sequence_keys=("action",))


def register(spec):
    name = f"pi05_nero_{spec['mode']}"
    base = training.get_config("pi05_libero")
    config = dataclasses.replace(base, name=name,
        model=dataclasses.replace(base.model, action_horizon=spec.get("action_horizon", 20)),
        data=NeroDataConfig(repo_id=spec["repo_id"], mode=spec["mode"], cameras=tuple(spec["cameras"]),
                            base_config=training.DataConfig(prompt_from_task=True)),
        batch_size=8, num_workers=2, wandb_enabled=False)
    training._CONFIGS_DICT[name] = config
    return name
