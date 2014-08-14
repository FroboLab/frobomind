
import Traverser

class pyqtTraverse(Traverser.Traverser):
    def __init__(self):
        self.parent = None
        self.nesting_stack = []
        self.str = ""
        
    def beginGui(self,obj):
        #self.parent = QScrollArea()
        #self.frame = QFrame(self.parent)
        #if obj.layout == "vertical":
        #    self.tl = QVBoxLayout()
        #else:
        #    self.tl = QHBoxLayout()
        str += """
        <div data-role="page" data-theme="c" id="{0}">
            <div data-role="header" data-theme="c">
                <h1>{0}</h1>
            </div>
            <div data-role="content" data-theme="c">
        """.format(obj.name)
            
        self.__increase_nesting_level(self.frame,self.tl)
    
    def endGui(self,obj):
        
        #parent,layout = self.__get_immediate_parent()
        #self.frame.setLayout(layout)
        #self.parent.setWidget(self.frame)
        #self.parent.show()
        self.str += """
            </div> <!--content -->
        </div> <!-- page -->
        """
        print self.str
        
    def beginGroup(self,obj):
        
        self.str += """
        <div data-role="collapsible">
            <h3>{0}</h3>
        
        """
        
        self.__increase_nesting_level(panel, l)
    
    def endGroup(self,obj):
        self.str += """
        </div> <!-- collapsible -->
        """
    
    def beginRosLabel(self,obj):
        self.str += """
            <p id="{0}">N/A</p>
        """.format(obj.name)
        
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
    
