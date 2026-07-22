from direct.showbase.ShowBase import ShowBase


class Rover:

    def __init__(self, render, loader):

        # Create rover
        self.model = loader.loadModel("models/box")
        self.model.reparentTo(render)

        self.model.setScale(1.5, 2.5, 0.8)
        self.model.setPos(0, -20, 2)

        self.speed = 5


    def update(self, dt):

        # Move forward
        self.model.setY(self.model, self.speed * dt)


    def getModel(self):
        return self.model