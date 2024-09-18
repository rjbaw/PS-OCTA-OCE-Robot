#!/bin/sh
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
"def program():
  global check = \"Made it\"
  while(True):
   freedrive_mode()
  end
end"}'
