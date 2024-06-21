msg_format = 'def program():\n global pose_translate = p[%01.4f,%01.4f,0,0,0,%01.4f]\n global b1 = read_output_float_register(1)\n global b2 = read_output_float_register(2)\n global b3 = read_output_float_register(3)\n global b4 = read_output_float_register(4)\n global b5 = read_output_float_register(5)\n global b6 = read_output_float_register(6)\n global home_pose = p[b1,b2,b3,b4,b5,b6]\n global pose_wrt_base = pose_trans(home_pose, pose_translate)\n movel(pose_wrt_base, a=%01.4f, v=%01.4f)\nend\n'

msg = sprintf(msg_format, [dX,dY,dR,acc,vel])
