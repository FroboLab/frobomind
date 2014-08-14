from python_qt_binding.QtGui import QPushButton
from python_qt_binding import QtCore
import RosHelper

class RosToggleButton(QPushButton):
    def __init__(self,parent,btn_name,topic_name,topic_type,topic_field,toggle_dict):
        super(RosToggleButton,self).__init__(btn_name,parent)
        self.btn_name = btn_name
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_field = topic_field
        self.toggle_dict = toggle_dict
        self.setText("N/A")
        self.publisher = RosHelper.create_publisher_from_type(self.topic_name,self.topic_type)
        self.clicked.connect(self.onclick)
        self.cur_idx = 0
        self.max_idx = len(toggle_dict.keys())-1
    
    def onclick(self,msg):

        key = self.toggle_dict.keys()[self.cur_idx]
        val = self.toggle_dict[key]
        
        self.setText(str(key))
        msg = RosHelper.create_msg_from_type(self.topic_type)

        #  iterate through msg attributes according to topic field and write value
        r = msg
        fields =self.topic_field.split(".")[1:]
        for subfields in fields[:len(fields)-1]:
            r = getattr(r,subfields) 
        setattr(r, fields[len(fields)-1], val)
        self.publisher.publish(msg)
        
        if self.cur_idx < self.max_idx:
            self.cur_idx += 1
        else:
            self.cur_idx = 0