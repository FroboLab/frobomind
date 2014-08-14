
import Traverser

class StringTraverse(Traverser.Traverser):
    def __init__(self):
        self.level = 1
        
    def beginGui(self,obj):
        print "Begin Gui: ", obj.name
        
    def endGui(self,obj):
        print "End Gui: ", obj.name
        
    def beginGroup(self,obj):
        s = self.__prepend_level()
        
        print s,"Begin Group: "  , obj.name
        self.level += 1
        
    def endGroup(self,obj):
        self.level -= 1
        print self.__prepend_level(),"End Group: ",obj.name
        
    def beginRosLabel(self,obj):
        s = self.__prepend_level()
        print s,obj.label_name," -- ",obj.topic_name," -- ",obj.topic_field
        
    def endRosLabel(self,obj):
        pass
    
    def beginRosToggleButton(self,obj):
        s = self.__prepend_level()
        print s,obj.btn_name," -- ",obj.topic_name," -- ",obj.topic_field
    
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
    
    def beginRosMsgView(self,obj):
        s = self.__prepend_level()
        print s,"Msg", obj.grp_name," -- ",obj.topic_name," -- ",obj.topic_fields
    
    def endRosMsgView(self,obj):
        pass
    
    def __prepend_level(self):
        s = ""
        for i in range(0,self.level):
            s += "\t"
            
        return s
        