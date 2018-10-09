from .state import Car


class Commander:
    def __init__(self, car: Car, lock, config: dict):
        self._car = car
        self._lock = lock
        self.out_speed = 0
        self.out_trajectories = (0, 1, 2, 3, 4, 1, 2, 3, 4, 0, 1, True)

    def decide(self):
        """
        Decide about the next outputs.
        """

        if self._car.perception.is_the_cake_a_lie():
            # the cake is a lie!
            self.out_speed = 0
