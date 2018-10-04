from .receptor import Receptor


class SirenReceptor(Receptor):
    def __init__(self, perception):
        super().__init__(perception)

    def init(self):
        pass

    def update(self, data=None):
        pass
