import time

import omni.usd
import omni.kit.app
from pxr import UsdGeom, Gf


class FlowSimulation:
    """Moves a flow item prim through an ordered list of station prims, pausing
    at each one, and logs a timestamped event for every arrival/departure so
    cycle-time stats can be computed after a run."""

    def __init__(self, flow_item_path: str, station_paths: list, move_speed: float = 150.0,
                 process_time: float = 2.0, loop: bool = False):
        self.flow_item_path = flow_item_path
        self.station_paths = station_paths
        self.move_speed = move_speed
        self.process_time = process_time
        self.loop = loop

        self.event_log = []  # list of dicts: {time, event, station}

        self._sub = None
        self._on_status = None

    def start(self, on_status=None):
        self._on_status = on_status
        self.event_log = []
        stage = omni.usd.get_context().get_stage()

        flow_prim = stage.GetPrimAtPath(self.flow_item_path)
        if not flow_prim.IsValid():
            self._status(f"Flow item not found: {self.flow_item_path}")
            return

        self._translate_attr = flow_prim.GetAttribute("xformOp:translate")
        if not self._translate_attr:
            self._status(f"{self.flow_item_path} has no xformOp:translate attribute")
            return

        up_axis = UsdGeom.GetStageUpAxis(stage)
        self._up_index = 1 if up_axis == "Y" else 2

        def get_pos(path):
            prim = stage.GetPrimAtPath(path)
            _, _, _, translation = omni.usd.get_local_transform_SRT(prim)
            return translation

        start_pos = get_pos(self.flow_item_path)
        ground_height = start_pos[self._up_index]

        def to_ground(pos):
            p = Gf.Vec3d(pos)
            p[self._up_index] = ground_height
            return p

        self._station_positions = [to_ground(get_pos(p)) for p in self.station_paths]
        self._pos = start_pos
        self._station_index = 0
        self._state = "MOVING"
        self._wait = 0.0

        self.stop()  # cancel any previous run first
        self._sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="factory_flow_sim"
        )
        self._log("started", None)
        self._status("Simulation started.")

    def stop(self):
        if self._sub:
            self._sub.unsubscribe()
            self._sub = None

    def _log(self, event: str, station_path):
        self.event_log.append({"time": time.time(), "event": event, "station": station_path})

    def _status(self, message):
        print(message)
        if self._on_status:
            self._on_status(message)

    def _step_towards(self, current, target, dt):
        direction = target - current
        distance = direction.GetLength()
        step = self.move_speed * dt
        if distance <= step:
            return target, True
        return current + direction.GetNormalized() * step, False

    def _on_update(self, e):
        dt = e.payload["dt"]

        if self._state == "MOVING":
            target = self._station_positions[self._station_index]
            self._pos, arrived = self._step_towards(self._pos, target, dt)
            self._translate_attr.Set(self._pos)
            if arrived:
                station_path = self.station_paths[self._station_index]
                self._log("arrived", station_path)
                self._status(f"Arrived at station {self._station_index + 1} ({station_path})")
                self._state = "WAITING"
                self._wait = 0.0

        elif self._state == "WAITING":
            self._wait += dt
            if self._wait >= self.process_time:
                station_path = self.station_paths[self._station_index]
                self._log("departed", station_path)
                self._station_index += 1
                if self._station_index >= len(self._station_positions):
                    if self.loop:
                        self._status("Looping back to station 1...")
                        self._station_index = 0
                        self._state = "MOVING"
                    else:
                        self._status("All stations visited. Simulation complete.")
                        self.stop()
                else:
                    self._status(f"Moving to station {self._station_index + 1}...")
                    self._state = "MOVING"
