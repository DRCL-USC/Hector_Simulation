function tau=Swing_PD(uin)

%% Swing Foot Control %%

%test=0; % used for debug
%test=test+1 % check this output to locate break point

global  dt_MPC gait i_gait acc_t i_MPC_var dt_MPC_vec stand_position current_height
t=uin(1);
xdes=uin(2:13); % desired SRBD states
x=uin(14:25); % actual SRBD states
q=uin(26:35); % joint states
qd=uin(36:45); % joint velocity
fpfv=uin(46:57); % foot position and velocity

hip=uin(58:63); % hip location

RPY=x(1:3); % Roll Pitch Yaw
x_act=x(4:6); % CoM position
w_act=x(7:9); % Angular velocity
v_act=x(10:12); % CoM velocity


% Feedback term coefficient
% (rough approximation, actual COM is lower than xdes(6))
% K_step=1*(0.1*xdes(6))^0.5;
K_step = 0.03;

dt = dt_MPC;
gaitcycle = dt_MPC*10;

vx_des = xdes(10); % CoM desired vel
vy_des = xdes(11);
vx_act = v_act(1); % CoM actual vel
vy_act = v_act(2);
wz_act=w_act(3);

% desired lift height of each foot:
lift_height = 0.12;
% current foot states
fpR = fpfv(1:3); % foot pos
fpL = fpfv(7:9);
fvR = fpfv(4:6); % foot vel
fvL = fpfv(10:12);

%% Gait scheduler
stage = floor((i_MPC_var-1)/5);
ii = (stage)*5+1;
t_cycle = t-acc_t(ii);
swing_schedule = zeros(2,1);
if gait == 1 || gait == 3 % walking running
    if i_gait == 1
        swing_schedule(1) = 1;
        swing_schedule(2) = 0;
    else
        swing_schedule(1) = 0;
        swing_schedule(2) = 1;
    end
elseif gait == 2
    if i_gait == 1 %hopping
        swing_schedule(1) = 0;
        swing_schedule(2) = 0;
    else
        swing_schedule(1) = 1;
        swing_schedule(2) = 1;
    end
end

%% transition between two gait freq. (for adaptive frequency walking):
delta_t = gaitcycle/2;
if i_MPC_var>5 %delta_t2 is needed to have even transition between different gaits
    delta_t2 = dt_MPC_vec(i_MPC_var-5)*5;
    trans_quot = dt_MPC_vec(i_MPC_var-5)^1/dt_MPC_vec(i_MPC_var)^1;
    dt2 = dt_MPC_vec(i_MPC_var-5);
else
    delta_t2 =delta_t;
    trans_quot = 1;
    dt2 = dt;
end

if t_cycle<=delta_t
    t_halfcycle=t_cycle;
else
    t_halfcycle=t_cycle-delta_t;
end

if (t_halfcycle-delta_t<1e-4&&t_halfcycle-delta_t>-1e-4)||(t_halfcycle<1e-4&&t_halfcycle>-1e-4)
    swing_schedule = zeros(2,1);
end

if t<=0.002
    stand_position=[0;-0.1;0;0;0.1;0];
    current_height=x_act(3);
end

if fpR(3)<=0.002&&swing_schedule(1)==0
    stand_position(1:3,1)=fpR;
end
if fpL(3)<=0.002&&swing_schedule(2)==0
    stand_position(4:6,1)=fpL;
end

if abs(t_halfcycle-delta_t)<2e-3
    current_height=x_act(3);
end
tolerance=0.000; % to minimize impact
fzdes = tolerance+0.04-(lift_height/(gaitcycle/4)^2)*t_cycle*(t_cycle-gaitcycle/2);
fzend=tolerance+0.04;

%% foot placement policy:
Rotm = eul2rotm(flip(RPY'));
eul = RPY;
eul=[0,0,1;0,1,0;1,0,0]*eul;
width = -0.011; % add up to stand width, wider stance helps with stability
x_com=-0.01; % actual COM is a little behind x_act(4)
p_hip_R_w = (hip(1:3) + Rotm*[x_com;-width;0]);
p_hip_L_w = (hip(4:6) + Rotm*[x_com;width;0]);
r=0.047+width;
% herustic policy:
% ref: Li, Junheng, and Quan Nguyen. "Dynamic Walking of Bipedal Robots
% on Uneven Stepping Stones via Adaptive-Frequency MPC." IEEE Control
% Systems Letters 7 (2023): 1279-1284.
fx_end_R = p_hip_R_w(1)+(delta_t+delta_t2)/2*(vx_act+wz_act*r*cos(eul(3)))/2+K_step*(vx_act-vx_des);
fy_end_R = p_hip_R_w(2)+(delta_t+delta_t2)/2*(vy_act+wz_act*r*sin(eul(3)))/2+K_step*(vy_act-vy_des);
fx_end_L = p_hip_L_w(1)+(delta_t+delta_t2)/2*(vx_act-wz_act*r*cos(eul(3)))/2+K_step*(vx_act-vx_des);
fy_end_L = p_hip_L_w(2)+(delta_t+delta_t2)/2*(vy_act-wz_act*r*sin(eul(3)))/2+K_step*(vy_act-vy_des);

fx_des_R=(t_halfcycle/delta_t)*(fx_end_R-stand_position(1,1))+stand_position(1,1);
fy_des_R=(t_halfcycle/delta_t)*(fy_end_R-stand_position(2,1))+stand_position(2,1);
fx_des_L=(t_halfcycle/delta_t)*(fx_end_L-stand_position(4,1))+stand_position(4,1);
fy_des_L=(t_halfcycle/delta_t)*(fy_end_L-stand_position(5,1))+stand_position(5,1);

if swing_schedule(1)==0
    fx_des_R=fpR(1,1);
    fy_des_R=fpR(2,1);
end
if swing_schedule(2)==0
    fx_des_L=fpL(1,1);
    fy_des_L=fpL(2,1);
end

fpL_des = [fx_des_L;fy_des_L;fzdes];%-x_act;
fpR_des = [fx_des_R;fy_des_R;fzdes];%-x_act;
fpL_end = [fx_end_L;fy_end_L;fzend];
fpR_end = [fx_end_R;fy_end_R;fzend];

%% Joint PD
% function foot_to_joint() mapping the desired foot position to joint angle
% hip_yaw fixed to 0
% hip_roll, hip_pitch, knee calculated with simple geometry
% ankle calculated to keep the foot horizontal

% sidesign 1 and swing_schedule(1) stand for right leg
% sidesign -1 and swing_schedule(2) stand for left leg
feedback_ratio=t_halfcycle/delta_t;
height_feedback=0;
feedback_limit=0.03;
x_act(3)=current_height+min(height_feedback*(xdes(6)-current_height),feedback_limit);
qR_des = foot_to_joint(fpR_des-x_act(1:3),q(1:5,1),Rotm,eul,feedback_ratio,1);
qL_des = foot_to_joint(fpL_des-x_act(1:3),q(6:10,1),Rotm,eul,feedback_ratio,-1);

Kp = diag([100 100 100 100 5]); Kd = diag([2 2 2 2 0.1]);

tau1 = (Kp*(qR_des - q(1:5)) + Kd*(0 - qd(1:5))).*swing_schedule(1);
tau2 = (Kp*(qL_des - q(6:10)) + Kd*(0 - qd(6:10))).*swing_schedule(2);

tau = [tau1;tau2];

test=[fpR_des;fpL_des];
tau=[tau;test];
end