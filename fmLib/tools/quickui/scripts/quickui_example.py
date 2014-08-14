#
# If you want to create your own gui add a dependency to your package.xml and CMakelist of this package
# then the rest should work
#

from quickui.QuickUi import *

#
# A gui is the root of the quick ui and only appears once
#
m = gui("TestExampleGui","vertical",
        # Groups:
        # use groups to group labels, sliders, buttons and plots together
        group("The basics","vertical",
              # Labels:
              # A ros_label is a label which display the text "lbl" along
              # with the value of the specified field in the specified topic
              ros_label("Published by button","/ros_topic_pub","std_msgs/String",".data"),
              # Buttons:
              # Ros button allows you create a button which toggles between given values on each click.
              # the value toggled is then published to the selected topic.
              ros_btn("button_name_in_gui","/ros_topic_pub","std_msgs/String",".data",{"On":"CMD_SET_ON","Off":"CMD_SET_OFF"}),
              #
              # ros_slider allows one to manipulate a single value in a published topic
              ros_slider("Encoder","/example_topic","std_msgs/Float64",".data",(-5.0,5.0)),
              #
              # Lets add a plot also just to visualize one of the sliders
              ros_plot("Encoder","/example_topic","std_msgs/Float64",".data",30),
              
              #
              # iterate is a neat language construct that allows you to create several components of the same type 
              # this is useful if the only difference between the elements are their name or topic
              group("Sliders","horizontal",
                    *iterate( ros_slider, ("Enc{0}","/example_topic{0}","std_msgs/Float64",".data",(-5.0,5.0) ), range(0,4) )
                    ),
              ),
        group("Advanced creation","horizontal",
              #
              # Besides integer ranges, strings can also be used and we can have 
              # different entries here we want the label to be EncA,EncB... 
              # but the topics are /example_topic0,/example_topic1
              *iterate( ros_label, ("Enc{0}","/example_topic{1}","std_msgs/Float64",".data"), [["A",0], ["B",1],["C",2],["D",3]] )           
              )
        )
#
# if debug=True a string representation of the gui is printed to the console 
# 
run(m,debug=True)