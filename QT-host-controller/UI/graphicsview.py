from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QGraphicsItem, QGraphicsPixmapItem,QGraphicsScene, QGraphicsView,QGraphicsLineItem,QGraphicsPolygonItem
from PyQt5.QtCore import QTimer, Qt, QCoreApplication, QRect, QRectF, QSize,QLineF,QPointF
from PyQt5.QtGui import QIcon,QFont,QTextCursor,QPainter, QPixmap, QWheelEvent,QPen, QColor, QPolygonF
import math

class GraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(GraphicsView, self).__init__(parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setOptimizationFlags(QGraphicsView.DontAdjustForAntialiasing | QGraphicsView.DontSavePainterState)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setMouseTracking(True)


        self._isRightButtonPressed = False
        self._lastMousePos = None

        self._setTargetPoseMode = True
        self._arrow = None
        self._arrowHead = None  # Add this line to save the reference to the arrow head
        self._arrowLine = None  # Add this line to save the reference to the arrow line



    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self._isRightButtonPressed = True
            self._lastMousePos = event.pos()
        if self._setTargetPoseMode and event.button() == Qt.LeftButton:
            if not self._arrow:  # Only create a new arrow if there isn't one already
                self._startPos = self.mapToScene(event.pos())
                self._arrow = QGraphicsLineItem()
                self.scene().addItem(self._arrow)
        super(GraphicsView, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self._isRightButtonPressed = False
        if self._setTargetPoseMode and event.button() == Qt.LeftButton:
            self._endPos = self.mapToScene(event.pos())
            self.drawArrow(self._startPos, self._endPos)
            self._arrow = None  # Remove the reference to the old arrow
        super(GraphicsView, self).mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        if self._isRightButtonPressed:
            newPos = event.pos()
            diff = newPos - self._lastMousePos
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - diff.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - diff.y())
            self._lastMousePos = newPos
        if self._setTargetPoseMode and self._arrow:
            self._endPos = self.mapToScene(event.pos())
            self.scene().removeItem(self._arrow)  # Remove the old arrow before drawing a new one
            self.drawArrow(self._startPos, self._endPos)
        super(GraphicsView, self).mouseMoveEvent(event)
       


    def drawArrow(self, start, end):
        if self._arrowLine:
            self.scene().removeItem(self._arrowLine)  # Remove the old arrow line before drawing a new one
        if self._arrowHead:
            self.scene().removeItem(self._arrowHead)  # Remove the old arrow head before drawing a new one
        line = QLineF(start, end)
        self._arrowLine = QGraphicsLineItem(line)  # Save the reference to the arrow line
        self.scene().addItem(self._arrowLine)

        # Draw the arrow head
        arrowHead = QPolygonF()
        arrowSize = 5.0
        angle = math.acos(line.dx() / (line.length() + 1e-10))  # Add a small offset to avoid division by zero
        if line.dy() >= 0:
            angle = (math.pi * 2) - angle
        arrowHead.append(line.p2())
        arrowHead.append(line.p2() - QPointF(math.sin(angle + math.pi / 3) * arrowSize, math.cos(angle + math.pi / 3) * arrowSize))
        arrowHead.append(line.p2() - QPointF(math.sin(angle + math.pi - math.pi / 3) * arrowSize, math.cos(angle + math.pi - math.pi / 3) * arrowSize))
        self._arrowHead = QGraphicsPolygonItem(arrowHead)  # Save the reference to the arrow head
        self._arrowHead.setBrush(QColor(Qt.red))
        self._arrowHead.setPen(QPen(Qt.transparent))  # Set the pen to be transparent
        self.scene().addItem(self._arrowHead)