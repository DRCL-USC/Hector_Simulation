function q_des = foot_to_joint(foot,q,Rotm,eul,feedback_ratio,sidesign)
 % this function mapping the desired foot position to joint angle
 % sidesign 1 stands for right leg
 % sidesign -1 stands for left leg

 %eul=rotm2eul(Rotm,'XYZ');
 q_des=zeros(5,1);
 
 
 % hip_yaw fixed to 0
 q_des(1)=0; 

 
 % hip_roll, hip_pitch, knee calculated with simple geometry 
 hip_roll=[0;-0.047*sidesign;-0.1265]+[0.0465;-0.015*sidesign;-0.0705];
 foot_des=Rotm.'*foot; % convert coordinates to body frame
 foot_des_to_hip_roll=foot_des-hip_roll;
 %hip_roll_to_hip_pitch=[-0.06;-0.018*sidesign;0];
 %hip_pitch_to_knee=[0;0;-0.22];
 %knee_to_ankle=[0;-0.0025*sidesign;-0.22];
 distance_3D=((foot_des_to_hip_roll(1)+0.06)^2+foot_des_to_hip_roll(2)^2+foot_des_to_hip_roll(3)^2)^0.5;
 distance_2D_yOz=(foot_des_to_hip_roll(2)^2+foot_des_to_hip_roll(3)^2)^0.5;
 distance_horizontal=0.0205;
 distance_vertical=(distance_2D_yOz^2-distance_horizontal^2)^0.5;
 distance_2D_xOz=(distance_3D^2-distance_horizontal^2)^0.5;
 %
 q_des(2)=asin(foot_des_to_hip_roll(2)/distance_2D_yOz)+asin(distance_horizontal*sidesign/distance_2D_yOz);
 q_des(3)=acos(distance_2D_xOz/2/0.22)-acos(distance_vertical/distance_2D_xOz)*(foot_des_to_hip_roll(1)+0.06)/abs((foot_des_to_hip_roll(1)+0.06));
 q_des(4)=2*asin(distance_2D_xOz/2/0.22)-pi;



 % ankle calculated to keep the foot horizontal
 %q_des(5)=0-q(3)-q(4);
 pitch_feedback=1;
 feedback_limit=0.05*pi;
 q_des(5)=0-q_des(3)-q_des(4)-feedback_ratio*max(min((1-pitch_feedback)*eul(2),feedback_limit),-feedback_limit);
 %q_des(5)=pi/4;



end