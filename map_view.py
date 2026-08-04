from direct.gui.DirectGui import DirectFrame
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import CardMaker, PNMImage, TextNode, Texture


class MapView:

    def __init__(self, parent, gridMap):

        self.gridMap = gridMap
        self.width = gridMap.width
        self.height = gridMap.height

        # create map panel
        self.panel = DirectFrame(
            parent=parent,
            frameColor=(0.08, 0.08, 0.08, 0.9),
            frameSize=(-0.78, 0.02, -0.83, 0.08),
            pos=(-0.04, 0, -0.08)
        )

        self.title = OnscreenText(
            parent=self.panel,
            text="SLAM MAP",
            pos=(-0.38, 0.025),
            scale=0.055,
            align=TextNode.ACenter,
            fg=(1, 1, 1, 1),
            mayChange=False
        )

        self.status = OnscreenText(
            parent=self.panel,
            text="Unknown: 0  Free: 0  Walls: 0",
            pos=(-0.38, -0.79),
            scale=0.035,
            align=TextNode.ACenter,
            fg=(1, 1, 1, 1),
            mayChange=True
        )

        # create map texture
        self.image = PNMImage(
            self.width,
            self.height,
            4
        )

        self.texture = Texture("slamMapTexture")

        card = CardMaker("slamMapCard")
        card.setFrame(-0.72, -0.04, -0.72, -0.04)

        self.mapCard = self.panel.attachNewNode(
            card.generate()
        )

        self.mapCard.setTexture(
            self.texture
        )

        self.lastDrawTime = -1

    # update the visual map
    def draw(self, taskTime, rover, path, target):

        drawTime = int(taskTime * 5)

        if drawTime == self.lastDrawTime:
            return

        self.lastDrawTime = drawTime

        grid = self.gridMap.getGrid()

        unknown = 0
        free = 0
        walls = 0

        for y in range(self.height):

            imageY = self.height - 1 - y

            for x in range(self.width):

                cell = grid[y][x]

                if cell == -1:
                    color = (0.18, 0.18, 0.18, 1)
                    unknown += 1

                elif cell == 0:
                    color = (0.82, 0.82, 0.82, 1)
                    free += 1

                else:
                    color = (0.02, 0.02, 0.02, 1)
                    walls += 1

                self.image.setXelA(
                    x,
                    imageY,
                    color[0],
                    color[1],
                    color[2],
                    color[3]
                )

        # draw current path
        for pathX, pathY in path:

            if self.gridMap.isInside(pathX, pathY):

                self.image.setXelA(
                    pathX,
                    self.height - 1 - pathY,
                    0.15,
                    0.45,
                    1,
                    1
                )

        # draw current target
        if target is not None:

            if self.gridMap.isInside(target.x, target.y):

                self.drawMarker(
                    target.x,
                    target.y,
                    1,
                    0.15,
                    0.1
                )

        # draw rover position
        roverModel = rover.getModel()

        roverX, roverY = self.gridMap.worldToGrid(
            roverModel.getX(),
            roverModel.getY()
        )

        if self.gridMap.isInside(roverX, roverY):

            self.drawMarker(
                roverX,
                roverY,
                0.1,
                1,
                0.2
            )

        self.texture.load(
            self.image
        )

        self.status.setText(
            "Unknown: " + str(unknown) +
            "  Free: " + str(free) +
            "  Walls: " + str(walls)
        )

    # draw a larger map marker
    def drawMarker(self, centerX, centerY, red, green, blue):

        for offsetY in range(-1, 2):

            for offsetX in range(-1, 2):

                x = centerX + offsetX
                y = centerY + offsetY

                if self.gridMap.isInside(x, y):

                    self.image.setXelA(
                        x,
                        self.height - 1 - y,
                        red,
                        green,
                        blue,
                        1
                    )
