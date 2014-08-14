class RosPlot():
    
    def __init__(self,label_name,topic_name,topic_type,topic_field,buffer_size):
        self.label_name= label_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.topic_field = topic_field
        self.buffer_size = buffer_size

    def traverse(self,traverser):
        traverser.beginRosPlot(self)
        traverser.endRosPlot(self)
    
    def to_str(self,level):

        s = ""
        for i in range(0,level):
            s+= "\t"
        s += "Ros: {0} - \"{1}\" - \"{2}\" ".format(self.label_name,self.topic_name,self.topic_field)
        return s