msg_format = 'def program():\n global pose_wrt_tool = p[%01.4f,%01.4f,%01.4f,%01.4f,%01.4f,%01.4f]\n global pose_wrt_base = pose_trans(get_forward_kin(), pose_wrt_tool)\n movel(pose_wrt_base, a=%01.2f, v=%01.2f)\nend\n';
%msg_format = 'def program():\n global pose_wrt_tool = p[%01.4f,%01.4f,%01.4f,%01.4f,%01.4f,%01.4f]\n global angle_axis = rotvec2rpy([pose_wrt_tool[3], pose_wrt_tool[4], pose_wrt_tool[5]])\n pose_wrt_tool[3] = angle_axis[3]\n pose_wrt_tool[4] = angle_axis[4]\n pose_wrt_tool[5] = angle_axis[5]\n global pose_wrt_base = pose_trans(get_forward_kin(), pose_wrt_tool)\n movel(pose_wrt_base, a=%01.2f, v=%01.2f)\nend\n';

dx=0;dy=0;
%dRx=0;dRy=-dR;dRz=0;
dRx=-dR*0.75;dRy=0;dRz=0;


msg = sprintf(msg_format, [dx,dy,dz,dRx,dRy,dRz,acc,vel])
