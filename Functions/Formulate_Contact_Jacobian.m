function [Contact_Jacobian,Rotm_foot]=Formulate_Contact_Jacobian
% Written by Han on 20220719
% Calculate contact Jacobian for bipedal robot, only run on first start
% Jc=[dr;do]/dq (o is orientation of foot)(dr, do in world frame)
% tau=Jc'*GRFM (GRFM in world frame)
% q is defined as [hip_yaw;hip_roll;hip_pitch;knee;ankle]


% Input
q=sym('q',[10,1]);
q_R=q(1:5,1);
q_L=q(6:10);

R=sym('r',[9,1]); % input R as vector
R=[reshape(R,[3,3]),zeros(3,1);zeros(1,3),1];




% Body


% Right leg configuration
link=repmat({zeros(3,1)},5,1);
link{1}=[0;-0.047;-0.1265];
link{2}=[0.0465;-0.015;-0.0705];
link{3}=[-0.06;-0.018;0];
link{4}=[0;0;-0.22];
link{5}=[0;-0.0025;-0.22];

foot=[0;0;-0.04;1];

% Translation matrices
T_R=repmat({eye(4)},5,1);
T_L=repmat({eye(4)},5,1);
for i=1:5
    T_R{i}(1:3,4)=link{i};
    T_L{i}(1:3,4)=[1;-1;1].*link{i};
end

% Rotation matrices
%R_R=repmat({eye(4)},5,1);
%R_L=repmat({eye(4)},5,1);
R_R{1}=[cos(q_R(1)),-sin(q_R(1)),0,0;sin(q_R(1)),cos(q_R(1)),0,0;0,0,1,0;0,0,0,1];
R_R{2}=[1,0,0,0;0,cos(q_R(2)),-sin(q_R(2)),0;0,sin(q_R(2)),cos(q_R(2)),0;0,0,0,1];
R_L{1}=[cos(q_L(1)),-sin(q_L(1)),0,0;sin(q_L(1)),cos(q_L(1)),0,0;0,0,1,0;0,0,0,1];
R_L{2}=[1,0,0,0;0,cos(q_L(2)),-sin(q_L(2)),0;0,sin(q_L(2)),cos(q_L(2)),0;0,0,0,1];
for i=3:5
    R_R{i}=[cos(q_R(i)),0,sin(q_R(i)),0;0,1,0,0;-sin(q_R(i)),0,cos(q_R(i)),0;0,0,0,1];
    R_L{i}=[cos(q_L(i)),0,sin(q_L(i)),0;0,1,0,0;-sin(q_L(i)),0,cos(q_L(i)),0;0,0,0,1];
end

r_R=foot;
r_L=foot;
o_R{1}=R(1:3,1:3);
o_L{1}=R(1:3,1:3);
for i=5:-1:1
    r_R=T_R{i}*R_R{i}*r_R;
    r_L=T_L{i}*R_L{i}*r_L;
    o_R{7-i}=o_R{6-i}*R_R{6-i}(1:3,1:3);
    o_L{7-i}=o_L{6-i}*R_L{6-i}(1:3,1:3);
end
r_R=R*r_R;
r_L=R*r_L;


% Jc=[dr;do]/dq
Jr_R=jacobian(r_R(1:3,1),q_R);
Jr_L=jacobian(r_L(1:3,1),q_L);

%Jo_R=o_R{6}*[0,1,0,0,0;0,0,1,1,1;1,0,0,0,0];
%Jo_L=o_L{6}*[0,1,0,0,0;0,0,1,1,1;1,0,0,0,0];
Jo_R=[o_R{1}*[0;0;1],o_R{2}*[1;0;0],o_R{3}*[0;1;0],o_R{4}*[0;1;0],o_R{5}*[0;1;0]];
Jo_L=[o_L{1}*[0;0;1],o_L{2}*[1;0;0],o_L{3}*[0;1;0],o_L{4}*[0;1;0],o_L{5}*[0;1;0]];

Jc=[Jr_R;Jr_L;Jo_R;Jo_L]; % right force, left force, right moment, left moment 
Contact_Jacobian=matlabFunction(Jc); % size=[12,5]
Rotm_foot=matlabFunction([o_R{6};o_L{6}]); % additional output, foot orientation, size=[6,3]



end