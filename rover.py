class Rover:

    def __init__(self, render, loader):

        # create rover
        self.model = loader.loadModel("models/box")
        self.model.reparentTo(render)
        self.model.setScale(1.5, 2.5, 0.8)
        self.model.setPos(0, 0, -2)

    def update(self, dt):

        pass

    def getModel(self):

        return self.model
