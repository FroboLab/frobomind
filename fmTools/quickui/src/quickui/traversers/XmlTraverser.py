
import Traverser

from lxml import etree

class XmlTraverse(Traverser.Traverser):
    def __init__(self):
        self.level = 1
        self.top = None
        
    def beginGui(self,obj):
        #print "Begin Gui: ", obj.name
        self.top = etree.Element("Gui", attrib={"name":obj.name})
        self.__increase_nesting_level(self.top)
        
    def endGui(self,obj):
        pass
        
    def beginGroup(self,obj):
        parent = self.__get_immediate_parent()
        grp = etree.Element("Group", attrib={"name":obj.name})
        parent.append(grp)
        self.__increase_nesting_level(grp)
        
    def endGroup(self,obj):
        pass
    
    def beginRosLabel(self,obj):
        parent = self.__get_immediate_parent()
        kv = {}
        kv["name"] = obj.label_name
        kv["topic"] = obj.topic_name
        kv["field"] = obj.topic_field
        lbl = etree.Element("RosLabel", attrib=kv, nsmap=None, **_extra)
        #print s,obj.label_name," -- ",obj.topic_name," -- ",obj.topic_field
        parent.append(lbl)
        
    def endRosLabel(self,obj):
        pass
    
    def beginRosToggleButton(self,obj):
        #s = self.__prepend_level()
        #print s,obj.btn_name," -- ",obj.topic_name," -- ",obj.topic_field
        parent = self.__get_immediate_parent()
        kv = {}
        kv["name"] = obj.btn_name
        kv["topic"] = obj.topic_name
        kv["field"] = obj.topic_field
        #lbl = etree.Element("RosLabel", attrib=kv, nsmap=None, **_extra)
        #print s,obj.label_name," -- ",obj.topic_name," -- ",obj.topic_field
        #parent.append(lbl)
        
    def endRosToggleButton(self,obj):
        pass
    
    def beginRosSlider(self,obj):
        s = self.__prepend_level()
        print s,obj.label_name," -- ",obj.topic_name," -- ",obj.topic_field
        pass
    
    def endRosSlider(self,obj):
        pass

    def beginRosPlot(self,obj):
        s = self.__prepend_level()
        print s,"Plot", obj.label_name," -- ",obj.topic_name," -- ",obj.topic_field
        pass
    
    def endRosPlot(self,obj):
        pass
    
    def __get_immediate_parent(self):
        l = len(self.nesting_stack)
        return self.nesting_stack[l-1]
    
    def __increase_nesting_level(self,parent):
        self.nesting_stack.append(parent)
        
    def __decrease_nesting_level(self):
        self.nesting_stack.pop()      