from python_qt_binding.QtGui import QGroupBox,QGridLayout,QLabel
import RosHelper

class RosMsgView(QGroupBox):
    def __init__(self,parent,name,topic_name,topic_type,topic_fields):
        super(RosMsgView,self).__init__(name,parent)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_fields = topic_fields
        self.ref_dict = {}
        self.layout = QGridLayout()
        self.__populate_self()
        self.subscriber = RosHelper.create_subscriber_from_type(self.topic_name,self.topic_type,self.onmsg)
        
    def __populate_self(self):
        i = 1
        if len(self.topic_fields) == 0:
            fields = RosHelper.get_all_msg_fields(self.topic_type)
        else:  
            fields = self.topic_fields
            
        for field in fields:
            self.ref_dict[field] = QLabel("None",self)
            self.layout.addWidget(QLabel(field,self),i,1)
            self.layout.addWidget(self.ref_dict[field],i,2)
            i+=1
        self.setLayout(self.layout)
    
    def onmsg(self,msg):
        # loop msg with topic field and
        for topic_field in self.topic_fields:
            r = msg
            lbl = self.ref_dict[topic_field]
            for subfields in topic_field.split(".")[1:]:
                r = getattr(r,subfields) 
            lbl.setText(str(r))
        