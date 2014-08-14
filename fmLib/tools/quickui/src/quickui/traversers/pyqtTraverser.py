
import Traverser
from python_qt_binding.QtGui import QFrame,QPushButton,QLabel,QWidget,QVBoxLayout,QHBoxLayout,QGroupBox,QGridLayout,QScrollArea
from quickui.pyqtElements.RosToggleButton import RosToggleButton 
from quickui.pyqtElements.RosLabel import RosLabel
from quickui.pyqtElements.RosSlider import RosSlider
from quickui.pyqtElements.RosPlot import RosPlot
from quickui.pyqtElements.RosMsgView import RosMsgView

class pyqtTraverse(Traverser.Traverser):
    def __init__(self):
        self.parent = None
        self.nesting_stack = []
    
    def beginGui(self,obj):
        self.parent = QScrollArea()
        self.frame = QFrame(self.parent)
        if obj.layout == "vertical":
            self.tl = QVBoxLayout()
        else:
            self.tl = QHBoxLayout()
            
        self.__increase_nesting_level(self.frame,self.tl)
    
    def endGui(self,obj):
        
        parent,layout = self.__get_immediate_parent()
        self.frame.setLayout(layout)
        self.parent.setWidget(self.frame)
        self.parent.show()
        
    def beginGroup(self,obj):
        parent,layout = self.__get_immediate_parent()
        panel = QGroupBox(obj.name,parent)
        if obj.layout == "grid":
            l = QGridLayout()
        elif obj.layout == "vertical":
            l = QVBoxLayout()
        else:
            l = QHBoxLayout()
        
        self.__increase_nesting_level(panel, l)
    
    def endGroup(self,obj):
        parent,layout = self.__get_immediate_parent()
        parent.setLayout(layout)
       
        self.__decrease_nesting_level()
       
        p1,l1 = self.__get_immediate_parent()
        l1.addWidget(parent)
        
    
    def beginRosLabel(self,obj):
        pm,lm = self.__get_immediate_parent()
        
        fr = QFrame(pm)
        layout = QGridLayout()
        nlb = QLabel(obj.label_name + ":",fr)
        nlb.setToolTip(obj.topic_name)
        layout.addWidget(nlb,1,1)
        layout.addWidget(RosLabel(fr,obj.label_name,obj.topic_name,obj.topic_type,obj.topic_field),1,2)
        fr.setLayout(layout)
        
        lm.addWidget(fr)
    
    def endRosLabel(self,obj):
        pass
    
    def beginRosToggleButton(self,obj):
        pm,lm = self.__get_immediate_parent()
        
        btn = RosToggleButton(pm,
                              obj.btn_name,
                              obj.topic_name,
                              obj.topic_type,
                              obj.topic_field,
                              obj.toggle_dict)
        
        lm.addWidget(btn)
        pass
    
    def endRosToggleButton(self,obj):
        pass
    
    def beginRosSlider(self,obj):
        pm,lm = self.__get_immediate_parent()
        slider = RosSlider(pm,obj.label_name,
                              obj.topic_name,
                              obj.topic_type,
                              obj.topic_field,
                              obj.min_max_tuple,
                              obj.default
                              )
        
        lm.addWidget(slider)
        pass
    
    def endRosSlider(self,obj):
        pass
    
    def beginRosPlot(self,obj):
        pm,lm = self.__get_immediate_parent()
        
        plot = RosPlot(pm,obj.label_name,
                          obj.topic_name,
                          obj.topic_type,
                          obj.topic_field,
                          obj.buffer_size
                        )
        lm.addWidget(plot)
    
    def endRosPlot(self,obj):
        pass
    
    def beginRosMsgView(self,obj):
        pm,lm = self.__get_immediate_parent()
        
        fr = RosMsgView(pm, obj.grp_name, obj.topic_name, obj.topic_type, obj.topic_fields)
        
        lm.addWidget(fr)
        pass
    
    def endRosMsgView(self,obj):
        pass
    
    def __get_immediate_parent(self):
        l = len(self.nesting_stack)
        return self.nesting_stack[l-1]
    
    def __increase_nesting_level(self,parent,container):
        self.nesting_stack.append((parent,container))
        
    def __decrease_nesting_level(self):
        self.nesting_stack.pop()
    
