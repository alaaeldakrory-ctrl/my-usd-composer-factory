import omni.ext


class ErgoSimExtension(omni.ext.IExt):
    """Placeholder: the panel and live scoring come in later steps. The scoring logic lives in `core/`
    and has no Omniverse dependencies, so it can be tested outside the app."""

    def on_startup(self, _ext_id):
        print("[my_company.ergo_sim] started (scoring core only, no UI yet)")

    def on_shutdown(self):
        pass
