from rnm.paths import *
from rnm.paths import RobotPath
import numpy as np

class ConvexObstacle:
    def from_points():
        """
        create a convex obstacle as a boundary of 3D-points from a pointcloud
        """

class AbstractPathPlanner:
    """
        Path planner interface,
        
    """
    def plan_path(self, start_configuration: np.ndarray, end_configuration: np.ndarray, obstacles) -> RobotPath:
        """
        
        """
        pass


class PotentialPathPlanner(AbstractPathPlanner):
    """
    Example: Path Planner using the potential algorithm
    """
    def __init__(self) -> None:
        super().__init__()

    def plan_path(self, obstacles) -> RobotPath:
        pass



def plan_needle_path(start_config, end_config):
    """
    Plan a path through configuration space such that the needle path created is   approximately linear
    """

