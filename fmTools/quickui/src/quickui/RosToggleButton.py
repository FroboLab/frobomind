
class RosToggleButton():
    
    def __init__(self,btn_name,topic_name,topic_type,topic_field,toggle_list):
        self.btn_name= btn_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.topic_field = topic_field
        self.toggle_dict = toggle_list

        
    def traverse(self,traverser):
        traverser.beginRosToggleButton(self)
        traverser.endRosToggleButton(self)
    
    def to_str(self,level):

        s = ""
        for i in range(0,level):
            s+= "\t"
        s += "RosLabel: {0} - \"{1}\" - \"{2}\" ".format(self.btn_name,self.topic_name,self.topic_field)
        return s