from python_qt_binding.QtGui import QSlider
from python_qt_binding import QtCore
import RosHelper

class RosSlider(QSlider):
    def __init__(self,parent,label_name,topic_name,topic_type,topic_field,min_max_tuple,default):
        super(RosSlider,self).__init__(QtCore.Qt.Horizontal,parent)
        self.label_name = label_name
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_field = topic_field
        self.setMinimum(0)
        self.setMaximum(100)
        self.min = min_max_tuple[0]
        self.max = min_max_tuple[1]
        self.factor_a =  (self.max - self.min) / 100.0
        self.factor_b = self.max  - self.factor_a*100.0
        self.default = int(( default - self.factor_b ) / self.factor_a) 
        
        if self.default is not None:
            print "setting default to:", self.default, "%"
            self.setValue(self.default )
        
        self.publisher = RosHelper.create_publisher_from_type(self.topic_name,self.topic_type)
        self.valueChanged.connect(self.onval)
        
        self.onval(self.default)    
    
    def onval(self,val):
        val = float(val) * self.factor_a + self.factor_b
        #print "value:", val, " a: ", self.factor_a, " b: ",self.factor_b
        msg = RosHelper.create_msg_from_type(self.topic_type)

        #  iterate through msg attributes according to topic field and write value
        r = msg
        fields =self.topic_field.split(".")[1:]
        for subfields in fields[:len(fields)-1]:
            r = getattr(r,subfields) 
        setattr(r, fields[len(fields)-1], val)
        self.publisher.publish(msg)
