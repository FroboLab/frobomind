from python_qt_binding.QtGui import QLabel
import RosHelper

class RosLabel(QLabel):
    def __init__(self,parent,label_name,topic_name,topic_type,topic_field):
        super(RosLabel,self).__init__(label_name,parent)
        self.label_name = label_name
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_field = topic_field
        self.subscriber = RosHelper.create_subscriber_from_type(self.topic_name,self.topic_type,self.onmsg)
        self.setText("N/A")
        
    def onmsg(self,msg):
        # loop msg with topic field and
        r = msg
        for subfields in self.topic_field.split(".")[1:]:
            r = getattr(r,subfields) 
        self.setText(str(r))
    