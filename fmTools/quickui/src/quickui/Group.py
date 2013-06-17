

class Group():
    
    def __init__(self,group_name,layout):
        self.name = group_name
        self.layout = layout
        self.components = []
        
    def add(self,comp):
        self.components.append(comp)
        
    def traverse(self,traverser):
        traverser.beginGroup(self)
        
        for c in self.components:
            c.traverse(traverser)
        traverser.endGroup(self)
    
    def to_str(self,level):
        
        s=""
        for i in range(0,level):
           s +="\t" 
        level += 1
        s += "Group: " + self.name + "\n"
        for c in self.components:
            s+= c.to_str(level) + "\n"
            
        level -= 1
            
        return s