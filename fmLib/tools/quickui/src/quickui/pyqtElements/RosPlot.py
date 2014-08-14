

from python_qt_binding import QtGui
from python_qt_binding.QtCore import QTimer
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np

import RosHelper
import collections

class RosPlot(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
    def __init__(self, parent, label_name,topic_name,topic_type,topic_field,buffer_size):
        
        self.label_name= label_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.topic_field = topic_field
        self.buffer_size = buffer_size
        
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_figure)
        
        fig = Figure(figsize=(5, 4), dpi=100)
        self.axes = fig.add_subplot(111)
        # We want the axes cleared every time plot() is called
        self.axes.hold(False)

        self.compute_initial_figure()
        self.buffer = collections.deque(maxlen=self.buffer_size)

        
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        
        self.subscriber = RosHelper.create_subscriber_from_type(self.topic_name,self.topic_type,self.on_message)
        self.timer.start()

    def compute_initial_figure(self):
        pass
    
    def on_message(self,msg):
        r = msg
        for subfields in self.topic_field.split(".")[1:]:
            r = getattr(r,subfields) 
        self.buffer.append(r)
        
    def update_figure(self):
        x = np.array(range(0,len(self.buffer)))
        y = np.array(self.buffer)
        self.axes.plot(x, y.T, 'r')
        self.draw()