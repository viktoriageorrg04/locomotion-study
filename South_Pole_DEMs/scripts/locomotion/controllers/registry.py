from .base import BaseController
from .null_controller import NullController
from .wheeled_diff import WheeledDiffController
try:
    from .base_twist import BaseTwistController
except Exception:
    BaseTwistController = None

def _is_wheeled(feats) -> bool:
    if feats and feats.drive_joint_paths:
        return True
    return False

def pick_controller(stage, art_root: str, feats, preferred: str = "auto") -> BaseController:
    preferred = (preferred or "auto").lower()

    if preferred in ("dc", "wheeled-dc"):
        return WheeledDiffController(stage, art_root, feats)

    if preferred in ("none", "null"):
        return NullController(stage, art_root, feats)

    if preferred in ("base-twist",) and BaseTwistController is not None:
        return BaseTwistController(stage, art_root, feats)

    # auto
    if _is_wheeled(feats):
        return WheeledDiffController(stage, art_root, feats)

    # No legged/humanoid gait yet: default to Null (or BaseTwist if you want)
    if BaseTwistController is not None:
        return BaseTwistController(stage, art_root, feats)
    return NullController(stage, art_root, feats)
