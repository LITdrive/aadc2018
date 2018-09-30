class Receptor:
    """
    Receptors get and process sensor information to update the world state
    """

    def __init__(self, perception):
        self._perception = perception

    def init(self):
        """
        Initialize and start this receptor
        """
        raise NotImplementedError

    def update(self, data=None):
        """
        Process new (sensor) information and update internal state
        """
        raise NotImplementedError
