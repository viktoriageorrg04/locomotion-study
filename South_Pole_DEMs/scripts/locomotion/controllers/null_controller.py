from .base import BaseController, ControllerSpec

class NullController(BaseController):
    spec = ControllerSpec(name="null", locomotion="unknown")
    # does nothing, just a placeholder