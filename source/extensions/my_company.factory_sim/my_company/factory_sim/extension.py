import omni.ext
import omni.ui as ui

from .flow_sim import FlowSimulation
from .forklift_ops import kit_runner as FK
from .forklift_ops.driver import DEFAULT_CHARACTER
from .forklift_ops.planner import Settings


class FactorySimExtension(omni.ext.IExt):
    """Panel with two tools: a simple multi-station flow simulation, and forklift operations
    (find forklifts / racks / pallets, plan who stores which pallet where, bake the animation)."""

    def on_startup(self, _ext_id):
        self._sim = None

        old = ui.Workspace.get_window("Factory Sim")   # a leftover from a reload before on_shutdown destroyed it
        if old is not None:
            old.visible = False
        self._window = ui.Window("Factory Sim", width=460, height=720)
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    with ui.CollapsableFrame("Forklift operations", collapsed=False):
                        self._build_forklift_ui()
                    with ui.CollapsableFrame("Flow simulation", collapsed=True):
                        self._build_flow_ui()
        # jobs dropped into C:/OmniverseData/kit_jobs run inside the app (job_runner.py)
        from .job_runner import JobRunner
        self._jobs = JobRunner()
        # the Ergonomics window (ergo_panel.py): live RULA / spine-load readout of the assembly demos
        self._ergo = None
        self._ergo_menu = None
        try:
            from .ergo_panel import ErgoWindow
            self._ergo = ErgoWindow()
            import omni.kit.menu.utils as mu
            from omni.kit.menu.utils import MenuItemDescription
            self._ergo_menu = [MenuItemDescription(name="Ergonomics", onclick_fn=self._show_ergo)]
            mu.add_menu_items(self._ergo_menu, "Window")
        except Exception as e:   # noqa: BLE001
            import carb
            carb.log_error(f"[factory_sim] Ergonomics window failed: {e}")

    # ------------------------------------------------------------------ forklift operations
    def _build_forklift_ui(self):
        with ui.VStack(spacing=6, height=0):
            ui.Label("Drop forklifts, racks, pallets (and a floor plane for a dock) anywhere. Names containing "
                     "Forklift / Rack / Pallet / Dock are found automatically, or tag the selected prims:", word_wrap=True)
            with ui.HStack(height=26, spacing=4):
                for label, role in (("Forklift", "forklift"), ("Rack", "rack"), ("Pallet", "pallet"),
                                    ("Dock", "dock"), ("Obstacle", "obstacle"), ("Untag", None)):
                    ui.Button(label, clicked_fn=lambda r=role: self._tag(r))
            ui.Button("Scan scene", height=26, clicked_fn=self._scan)

            with ui.HStack(height=24):
                ui.Label("Task:", width=150)
                self._mode = ui.ComboBox(0, "Store floor pallets in racks", "Retrieve rack pallets to the dock")
            with ui.HStack(height=24):
                ui.Label("Fill shelves:", width=150)
                self._fill = ui.ComboBox(0, "Bottom first", "Top first")
            with ui.HStack(height=24):
                ui.Label("Drive speed (cm/s):", width=150)
                self._speed = ui.FloatField()
                self._speed.model.set_value(140.0)
            with ui.HStack(height=24):
                self._home_cb = ui.CheckBox(width=20)
                self._home_cb.model.set_value(True)
                ui.Label("Return to start when done")
            with ui.HStack(height=24):
                self._drivers_cb = ui.CheckBox(width=20)
                self._drivers_cb.model.set_value(True)
                ui.Label("Seat drivers (forklift models with a known seat)")
            ui.Label("Driver character (.usd):")
            self._character = ui.StringField(height=24)
            self._character.model.set_value(DEFAULT_CHARACTER)

            with ui.HStack(height=30, spacing=4):
                ui.Button("Plan && Bake", clicked_fn=self._plan)
                ui.Button("Clear animation", clicked_fn=self._clear)
            with ui.HStack(height=26, spacing=4):
                ui.Button("Overview camera", clicked_fn=lambda: self._set_status(FK.overview_camera(True)))
                ui.Button("Back to Perspective", clicked_fn=lambda: self._set_status(FK.overview_camera(False)))
            self._fk_status = ui.Label("Press Scan scene to see what was found.", word_wrap=True,
                                       alignment=ui.Alignment.LEFT_TOP)

    def _set_status(self, text):
        self._fk_status.text = text

    def _tag(self, role):
        n = FK.tag_selection(role)
        self._set_status(f"Tagged {n} prim(s) as {role}." if role else f"Removed the tag from {n} prim(s).")

    def _scan(self):
        try:
            self._set_status(FK.scan_text())
        except Exception as e:
            self._set_status(f"Scan failed: {e}")

    def _plan(self):
        s = Settings()
        s.drive_speed = max(20.0, self._speed.model.get_value_as_float())
        s.fill = "top" if self._fill.model.get_item_value_model().get_value_as_int() == 1 else "bottom"
        s.return_home = self._home_cb.model.get_value_as_bool()
        s.mode = "retrieve" if self._mode.model.get_item_value_model().get_value_as_int() == 1 else "store"
        FK.launch(FK.run(s, drivers=self._drivers_cb.model.get_value_as_bool(),
                         character=self._character.model.get_value_as_string().strip() or DEFAULT_CHARACTER,
                         status=self._set_status))

    def _clear(self):
        FK.launch(FK.clear(status=self._set_status))

    # ------------------------------------------------------------------ flow simulation (unchanged)
    def _build_flow_ui(self):
        with ui.VStack(spacing=6, height=0):
            ui.Label("Flow item prim path:")
            self._flow_item_field = ui.StringField(height=24)
            self._flow_item_field.model.set_value("/World/WoodenCrate_D1")

            ui.Label("Station prim paths (comma separated, in visit order):")
            self._stations_field = ui.StringField(height=24)
            self._stations_field.model.set_value(
                "/World/Cone_01, /World/Cone_02, /World/Cone_03"
            )

            with ui.HStack(height=24):
                ui.Label("Move speed (units/sec):", width=180)
                self._speed_field = ui.FloatField()
                self._speed_field.model.set_value(150.0)

            with ui.HStack(height=24):
                ui.Label("Process time (sec/station):", width=180)
                self._process_field = ui.FloatField()
                self._process_field.model.set_value(2.0)

            with ui.HStack(height=24):
                self._loop_checkbox = ui.CheckBox(width=20)
                ui.Label("Loop continuously")

            ui.Spacer(height=6)
            with ui.HStack(height=28):
                ui.Button("Start", clicked_fn=self._on_start)
                ui.Button("Stop", clicked_fn=self._on_stop)

            ui.Spacer(height=6)
            self._status_label = ui.Label("Idle.")

    def _on_start(self):
        flow_item_path = self._flow_item_field.model.get_value_as_string().strip()
        station_paths = [
            p.strip() for p in self._stations_field.model.get_value_as_string().split(",")
            if p.strip()
        ]
        move_speed = self._speed_field.model.get_value_as_float()
        process_time = self._process_field.model.get_value_as_float()
        loop = self._loop_checkbox.model.get_value_as_bool()

        self._sim = FlowSimulation(
            flow_item_path, station_paths,
            move_speed=move_speed, process_time=process_time, loop=loop,
        )
        self._sim.start(on_status=self._status_label.model.set_value)

    def _on_stop(self):
        if self._sim:
            self._sim.stop()
            self._status_label.model.set_value("Stopped.")

    def _show_ergo(self, *_):
        if self._ergo and self._ergo.window:
            self._ergo.window.visible = True
            self._ergo.window.focus()

    def on_shutdown(self):
        if getattr(self, "_jobs", None):
            self._jobs.destroy()
            self._jobs = None
        if getattr(self, "_ergo_menu", None):
            try:
                import omni.kit.menu.utils as mu
                mu.remove_menu_items(self._ergo_menu, "Window")
            except Exception:   # noqa: BLE001
                pass
        if getattr(self, "_ergo", None):
            self._ergo.destroy()
            self._ergo = None
        if self._sim:
            self._sim.stop()
        if self._window:
            self._window.destroy()
        self._window = None
