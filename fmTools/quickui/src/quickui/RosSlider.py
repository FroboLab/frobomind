class RosSlider():
    
    def __init__(self,label_name,topic_name,topic_type,topic_field,min_max_tuple,default):
        self.label_name= label_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.topic_field = topic_field
        self.min_max_tuple = min_max_tuple
        self.default= default

    def traverse(self,traverser):
        traverser.beginRosSlider(self)
        traverser.endRosSlider(self)
    
    def to_str(self,level):

        s = ""
        for i in range(0,level):
            s+= "\t"
        s += "Ros: {0} - \"{1}\" - \"{2}\" ".format(self.label_name,self.topic_name,self.topic_field)
        return s