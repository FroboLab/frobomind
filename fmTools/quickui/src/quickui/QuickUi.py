
from python_qt_binding.QtGui import QApplication 
import sys
from .RosLabel import RosLabel
from .Group import Group
from .Gui import Gui
from .RosToggleButton import RosToggleButton
from .RosSlider import RosSlider
from .RosPlot import RosPlot
from traversers.pyqtTraverser import pyqtTraverse
import rospy
from quickui.traversers.StringTraverser import StringTraverse
from .RosMsgView import RosMsgView

__top = None

def run(gui,debug=False):
    """
    @param gui: The gui to render and execute
    """
    
    if debug:
        str_traverse = StringTraverse()
        gui.traverse(str_traverse)
        
    rospy.init_node("guiname")
    code_traverse = pyqtTraverse()
    
    app = QApplication(sys.argv)
    
    gui.traverse(code_traverse)
    sys.exit(app.exec_())
    

def gui(gui_name,layout,*components):
    """
    @param gui_name: The name of the gui to display 
    @param components: list of components to be included in the Gui can be of type 
    \'group\',\'ros_label\',\'ros_plot\',\'ros_slider\',\'ros_btn\'
    
    @return: Returns the toplevel gui which can be runned by the \ref run function
    """
    global __top
    if __top is None:
        __top = Gui(gui_name,layout,components)
    else:
        print "<<< error populating model: gui can only appear once"
    
    return __top
      
def group(group_name,layout,*components):
    """
    @param group_name: The name of the group to display in the GUI
    @param components: a list of components which can be of type 
    ros_label, ros_plot, ros_slider, ros_btn
    @return returns a component to be included in either a subgroup or toplevel gui
    """
    grp = Group(group_name,layout)
    
    for c in components:
        grp.add(c)

    return grp

def iterate(component,args,index_list):
    """ iterate allows you the generate multiple components in one go based on a substitution list.
    
    This function works by substituting the first two arguments in args with the values provided in the index_list,
    a new component is created for each entry in the index list. 
    
    @note remember to expand the list of components returned by this function, use *
    
    @example:
        gui("test","vertical",*iterate(ros_label, ( "lbl_name_{0}", "/topic_name{0}", "std_msgs/Int8", ".data" ), [0, 1, 2, 3] ) )
    
    @param component: the function reference to the component to iterate over, e.g ros_label
    @param args: the arguments that the component should be called with.
    @param index_list: the list to iterate over and substitute strings for the first two arguments in args
    """
    lbls = []
    ag = list(args)
    for i in index_list:
        if isinstance(i,(list,tuple)):
            ag[0] = args[0].format(*i)
            ag[1] = args[1].format(*i)
        else:
            ag[0] = args[0].format(i)
            ag[1] = args[1].format(i)
        
        lbls.append( component(*ag))
    return lbls

def ros_label(label_name,topic_name,topic_type,topic_field):
    """ Insert a label displaying a value of a field in a message published.
    
    @param label_name: The name of the label displayed in the GUI
    @type  label_name: str
    @param topic_name: The name of the topic in the Ros Graph
    @type  topic_name: str
    @param topic_type: The type of the topic fully qualified e.g. std_msgs/String
    @type  topic_type: str  
    @param topic_field: The field within the message to display e.g \".header.stamp\"
    @type  topic_field: str
    """
    return RosLabel(label_name,topic_name,topic_type,topic_field)

def ros_msg_view(grp_name,topic_name,topic_type,topic_fields):
    """Displays multiple fields in a message grouped as a single component
    @param label_name: The name of the msg displayed in the GUI
    @type  label_name: str
    @param topic_name: The name of the topic in the Ros Graph
    @type  topic_name: str
    @param topic_type: The type of the topic fully qualified e.g. std_msgs/String
    @type  topic_type: str  
    @param topic_fields: A list of fields within the message to 
    display e.g [".header.stamp",".twist.linear.x"], leave empty inorder to include all fields
    @type  topic_field: [str,...]
    """
    return RosMsgView(grp_name,topic_name,topic_type,topic_fields)

def ros_btn(btn_name,topic_name,topic_type, topic_field,toggle_dict):
    """ Insert a button which toggles between n values in toggle_dict
    @param btn_name: The name of the label displayed in the GUI
    @type  btn_name: str
    @param topic_name: The name of the topic in the Ros Graph
    @type  topic_name: str
    @param topic_type: The type of the topic fully qualified e.g. std_msgs/String
    @type  topic_type: str  
    @param topic_field: The field within the message to display e.g \".header.stamp\"
    @type  topic_field: str
    @param toggle_dict: The dictionary containing the string to display in the button as key and the value to publish as value.
    @type  toggle_dict: dict{"On":0,"Off":1}        
    """
    return RosToggleButton(btn_name,topic_name,topic_type,topic_field,toggle_dict)
    
def ros_slider(label_name,topic_name,topic_type,topic_field,min_max_tuple,default=None):
    """ Pubslihes a value as read from the slider linearly between min and max
    @param label_name: The name of the label displayed in the GUI
    @type  label_name: str
    @param topic_name: The name of the topic in the Ros Graph
    @type  topic_name: str
    @param topic_type: The type of the topic fully qualified e.g. std_msgs/String
    @type  topic_type: str  
    @param topic_field: The field within the message to display e.g \".header.stamp\"
    @type  topic_field: str
    @param min_max_tuple: A tuple containing the minimum value and maximum value e.g (0,10)
    @type  min_max_tuple: (min,max)        
    """
    return RosSlider(label_name,topic_name,topic_type,topic_field,min_max_tuple,default)

def ros_plot(label_name,topic_name,topic_type,topic_field,buffer_size):
    """ Plots the specified field within the message
    @param label_name: The name of the label displayed in the GUI
    @type  label_name: str
    @param topic_name: The name of the topic in the Ros Graph
    @type  topic_name: str
    @param topic_type: The type of the topic fully qualified e.g. std_msgs/String
    @type  topic_type: str  
    @param topic_field: The field within the message to display e.g \".header.stamp\"
    @type  topic_field: str
    @param buffer_size: The number of messages to keep in the plot buffers
    @type  buffer_size: int
    """
    return RosPlot(label_name,topic_name,topic_type,topic_field,buffer_size)
    