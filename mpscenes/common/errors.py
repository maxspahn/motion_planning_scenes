class ComponentIncompleteError(Exception):
    pass

class DimensionNotSuitableForEnv(Exception):
    pass

class TrajectoryNotSupported(Exception):
    pass

class TrajectoryComponentMissingError(Exception):
    pass

class JointSpaceGoalsNotSupportedError(Exception):
    pass

class MultiplePrimeGoalsError(Exception):
    pass

class UnknownSubGoalType(Exception):
    pass

class MissmatchDimensionError(Exception):
    pass


