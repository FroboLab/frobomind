class RosMsgView():
    
    def __init__(self,grp_name,topic_name,topic_type,topic_fields):
        self.grp_name= grp_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.topic_fields = topic_fields

    def traverse(self,traverser):
        traverser.beginRosMsgView(self)
        traverser.endRosMsgView(self)
    
    def to_str(self,level):

        s = ""
        for i in range(0,level):
            s+= "\t"
        s += "RosMsg: {0} - \"{1}\" - \"{2}\" ".format(self.grp_name,self.topic_name,self.topic_field)
        return s