"""Read both controller streams in one LeRobot/Isaac Teleop session."""

import numpy as np

from examples.isaac_teleop_to_so101.isaac_teleop.teleop_xr_controller import XRController
from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
from isaacteleop.retargeting_engine.interface import OutputCombiner, ValueInput
from isaacteleop.retargeting_engine.tensor_types import TransformMatrix
from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex as Index


class BothXRControllers(XRController):
    """Reuse the single-controller lifecycle and coordinate transform, not its reader."""

    @property
    def action_features(self):
        features = super().action_features
        return {f"{side}.{key}": value for side in ("left", "right")
                for key, value in features.items()}

    def _build_pipeline(self):
        source = ControllersSource(name="controllers")
        transform = ValueInput("base_T_anchor", TransformMatrix())
        controllers = source.transformed(transform.output("value"))
        return OutputCombiner({side: controllers.output(f"controller_{side}")
                               for side in ("left", "right")})

    def get_action(self):
        result = self._step(execution_events=self._running_events(), external_inputs=self._external_inputs)
        action = {}
        self.tracking_by_hand = {}
        for side in ("left", "right"):
            controller = result[side]
            values = dict(grip_pos=np.zeros(3), grip_quat=np.array([0., 0., 0., 1.]),
                          squeeze=0., trigger=0.)
            tracked = controller is not None and not getattr(controller, "is_none", False)
            if tracked:
                try:
                    values = dict(grip_pos=np.asarray(controller[Index.GRIP_POSITION], dtype=np.float32),
                                  grip_quat=np.asarray(controller[Index.GRIP_ORIENTATION], dtype=np.float32),
                                  squeeze=float(controller[Index.SQUEEZE_VALUE]),
                                  trigger=float(controller[Index.TRIGGER_VALUE]))
                except (IndexError, KeyError, TypeError, ValueError):
                    tracked = False
            self.tracking_by_hand[side] = tracked
            action.update({f"{side}.{key}": value for key, value in values.items()})
        self._is_tracking = any(self.tracking_by_hand.values())
        return action
