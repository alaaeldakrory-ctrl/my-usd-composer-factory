"""Runs Python jobs dropped into a folder inside the running app (renders, batch captures), so long tasks can be
started without driving the UI.

A job is a .py file in JOBS (C:/OmniverseData/kit_jobs) that defines `async def run(log)`; `log(text)` appends to
<job>.log. The file is renamed .running, then .done or .failed (traceback in the log). One job at a time.
Note: editing this extension's files reloads it and stops a running job.
"""
import asyncio
import os
import traceback

import omni.kit.app

JOBS = r"C:\OmniverseData\kit_jobs"


class JobRunner:
    def __init__(self, folder=JOBS, every=2.0):
        self.folder = folder
        self.every = every
        self._task = asyncio.ensure_future(self._loop())

    def destroy(self):
        if self._task:
            self._task.cancel()
            self._task = None

    async def _loop(self):
        os.makedirs(self.folder, exist_ok=True)
        while True:
            try:
                jobs = sorted(f for f in os.listdir(self.folder) if f.endswith(".py"))
            except OSError:
                jobs = []
            if jobs:
                await self._run(os.path.join(self.folder, jobs[0]))
            await asyncio.sleep(self.every)

    async def _run(self, path):
        base = os.path.splitext(path)[0]
        running = base + ".running"
        os.replace(path, running)
        logf = base + ".log"

        def log(text):
            with open(logf, "a", encoding="utf-8") as fh:
                fh.write(str(text) + "\n")
            print("[kit_job]", text)
        log(f"start {os.path.basename(path)}")
        ns = {"__name__": "kit_job", "__file__": path}
        try:
            code = open(running, encoding="utf-8").read()
            exec(compile(code, path, "exec"), ns)
            if "run" in ns:
                await ns["run"](log)
            os.replace(running, base + ".done")
            log("done")
        except asyncio.CancelledError:
            log("cancelled (extension reloaded?)")
            os.replace(running, base + ".failed")
            raise
        except Exception:   # noqa: BLE001
            log(traceback.format_exc())
            os.replace(running, base + ".failed")
        await omni.kit.app.get_app().next_update_async()
