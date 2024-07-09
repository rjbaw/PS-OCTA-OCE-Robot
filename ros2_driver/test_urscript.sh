#!/bin/sh
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[-0.2, -0.3, 0.8, 1, 1, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[0.2, 0.3, 0.8, 0, 0, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'

ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[-0.3, -0.3, 0.3, 0, 0, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[0.2, 0.3, 0.8, 0, 0, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[-0.2, -0.3, 0.8, 1, 1, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[0.5, 0.5, 0.5, 0, 0, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def my_prog():

  set_digital_out(1, True)

  movej(p[-0.5, -0.5, 0.5, 1, 1, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

end"}'
