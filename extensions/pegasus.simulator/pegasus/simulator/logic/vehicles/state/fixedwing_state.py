from pegasus.simulator.logic.state import State


class FixedWingState(State):
    """
    Stores the state of a fixed wing vehicle.
    """

    def __init__(self, aoa, throttle):
        """
        Initialize the FixedWingState object
        """

        # Call the parent class constructor
        super().__init__()

        self.angle_of_attack = aoa
        self.throttle = throttle