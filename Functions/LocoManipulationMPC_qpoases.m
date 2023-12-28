%% Force-and-moment-based MPC for Dynamic Loco-manipulation

% Associated with manuscript: 
% "Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research"

% CITE AS: 
% @article{li2023dynamic,
%  title={Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research},
%  author={Li, Junheng and Ma, Junchao and Kolt, Omar and Shah, Manas and Nguyen, Quan},
%  journal={arXiv preprint arXiv:2312.11868},
%  year={2023}
% }

function u = LocoManipulationMPC_qpoases(uin)
tic
%% MPC Parameters
global i_MPC_var dt_MPC_vec gait x_traj_IC Contact_Jacobian Rotm_foot addArm m_o
k = i_MPC_var; % current horizon
h = 10; % prediction horizons
g = 9.81; % gravity

%% QPOASES Parameters
import casadi.*
numVar = 12*h;
numCons = 34*h;

%% Making definition consistent with MPC
% input definition
xdes=uin(1:12); % desired states [eul, p, omega, v]'
x=uin(13:24); % current states [eul, p, omega, v]'
q=uin(25:34); % leg joint angles [q_L, q_R]'
foot=uin(35:46); % contact position and velocity [p_L, v_L, p_R, v_R]'
t = uin(end); % newly added simulation clock for contact schedule

eul = x(1:3);
eul_des = xdes(1:3);
R = eul2rotm(flip(eul'));

% enforce "2*pi=0" relation for turning
yaw_correction=0;
while yaw_correction==0
    if eul_des(3,1)-eul(3,1)>pi
        eul(3,1)=eul(3,1)+2*pi;
    elseif eul_des(3,1)-eul(3,1)<-pi
        eul(3,1)=eul(3,1)-2*pi;
    else
        yaw_correction=1;
    end
end

% rearrange MPC states
xdes = [xdes;1];

% handling payload
if addArm
    m_o = Variable_Mass(t); % adding variable payload mass
    r_o = [0.2; 0; 0.15]; % define object CoM Location relative to trunk CoM
    xdes(2) = -deg2rad(m_o*3)+0.0001; % lean back when handling load, offsetting small pitch to avoid matrix sigularity
end

x = [x;1];

%% Assigning desired trajectory for CoM and Foot locations
if k == 1
    x_traj = x_traj_IC;
end
x_traj = Calc_x_traj(xdes,x,h,k) % desired trajectory
foot_traj =[foot(1:3);foot(7:9)]; % assume foot under hip
% alternatively, can use function Calc_foot_traj_3Dwalking for more
% accurate foot position esitmate, may present some bugs for 3D locomotion

%% Contact Schedules:
% gait schedules:
if gait == 1
    sigma_m = gaitSchedule(k, 1);
elseif gait == 0
    sigma_m = ones(10*h,1);
end
% payload contact schedules:
sigma_o = payloadContact(t);

%% Robot simplified dynamics physical properties 
mu = 0.5; %friction coefficient
if addArm
    m = 16.4 + m_o; % included arms for humanoid 
    Ib = diag([0.932, 0.9420, 0.0711]); % SRBD MoI (included body, arms, hips, and thighs)

else
    m = 5.75 + 2*(0.835+0.764+1.613+0.12+0.08); % mass (inclulded body, hips, and thighs)
    Ib = diag([0.5413, 0.5200, 0.0691]); % SRBD MoI (included body, hips, and thighs)
    % Ib = diag([0.064, 0.057, 0.016]);
end
Fmax = 500; Fmin = 0; % force limits

%% Forward Kinematics (for joint torque constraints)
RR=reshape(R,[9,1]);
Jc=Contact_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
contact_mapping=[blkdiag(Jc(1:3,:)',Jc(4:6,:)'),blkdiag(Jc(7:9,:)',Jc(10:12,:)')]; % torque=contact_mapping*u

%% State-space A & B matrices  (dynamics are done in world frame)
% foot rotation wrt. world frame
R_foot=Rotm_foot(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
R_foot_R=R_foot(1:3,:);
R_foot_L=R_foot(4:6,:);

I = R*Ib*R'; % MoI in world frame

B=repmat({zeros(13,12)},h,1);
A_hat=repmat({zeros(13,13)},h,1);

% continuous A & B:
for i = 1 : h
S_R = [sin(x_traj(2,i))*sin(x_traj(3,i)), cos(x_traj(3,i)), 0;
       sin(x_traj(2,i))*cos(x_traj(3,i)), -sin(x_traj(3,i)), 0;
       cos(x_traj(2,i)), 0, 1];

Ac=[zeros(3,3), zeros(3,3), S_R\eye(3), zeros(3,3), zeros(3,1);
    zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,1);
    zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), I\(sigma_o*skew(r_o)*m_o*[0;0;g]);
    zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3),(m_o-m)*[0;0;g]/m;
    zeros(1,13)];

Bc=[zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
    zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
    I\skew(-x_traj(4:6,i) + foot_traj(1:3,1)), I\skew(-x_traj(4:6,i) + foot_traj(4:6,1)), I\eye(3), I\eye(3);
    eye(3)/m, eye(3)/m, zeros(3),zeros(3);
    zeros(1,12)];

% discretization:
B{i}=Bc*dt_MPC_vec(i+k-1);
A_hat{i}=eye(13)+Ac*dt_MPC_vec(i+k-1);

end

%% QP_MPC derivation:
% translating MPC to a condensed QP problem:
% ref: Jerez, Juan L., Eric C. Kerrigan, and George A. Constantinides. 
% "A condensed and sparse QP formulation for predictive control." 
% 2011 50th IEEE Conference on Decision and Control and European Control 
% Conference. IEEE, 2011.

y = reshape(x_traj,[13*h 1]);
Aqp=repmat({zeros(13,13)},h,1);
%Aqp
Aqp{1}=A_hat{1};
for i=2:h
    Aqp{i}=Aqp{i-1}*A_hat{i};
end
Aqp=cell2mat(Aqp);
%Bqp
Bqp=repmat({zeros(13,12)},h,h);
for i=1:h
    Bqp{i,i}=B{i};
    for j=1:h-1
        Bqp{i,j}=A_hat{i}^(i-j)*B{j};
    end
end
for i=1:h-1
    for j=i+1:h
        Bqp{i,j}=zeros(13,12);
    end
end
Bqp=cell2mat(Bqp);

%% MPC Weights: (tune these according to your task)
% state tracking objective:
% L1 = [850 600 250 ,  300 300 350 ,  1 1 1 ,  1 1 1,  0]; % [eul, p, omega, v, g]
L1 = [3000 1000 1000 ,  1000 2000 7000 ,  1 1 1 ,  1 1 1,  0]; % [eul, p, omega, v, g]
% control input minimization:
alpha=[1 1 1, 1 1 1, 1 1 1, 1 1 1]*10^(-6); % humanoid

L10 = repmat(L1,1,h);
L=diag(L10);
alpha10 = repmat(alpha,1,h);
K = diag(alpha10);
% MPC->QP math:
Hd = 2*(Bqp'*L*Bqp+K);
fd = 2*Bqp'*L*(Aqp*x-y);  

%% MPC Constraints:
% please refer to the QPOASES constraint format: lbA <= A*u <= ubA
A = DM(numCons,numVar); 
bigNum = 1e6;
smallNum = -bigNum;
lba = zeros(numCons,1);
uba = lba;

%friction constraint:
A_mu = [1,0,-mu,zeros(1,9);
    0,1,-mu,zeros(1,9); 
    1,0,mu,zeros(1,9); 
    0,1,mu,zeros(1,9);
    zeros(1,3),1,0,-mu,zeros(1,6);
    zeros(1,3),0,1,-mu,zeros(1,6); 
    zeros(1,3),1,0,mu,zeros(1,6); 
    zeros(1,3),0,1,mu,zeros(1,6)];
A_mu_h = blkdiag(A_mu,A_mu,A_mu,A_mu,A_mu,A_mu,A_mu,A_mu,A_mu,A_mu);
lba_mu = repmat([smallNum;smallNum;0;0; smallNum;smallNum;0;0],h,1);
uba_mu = repmat([0;0;bigNum;bigNum; 0;0;bigNum;bigNum],h,1);

%force limit constraint:
A_f = [0,0,1,zeros(1,9); zeros(1,3),0,0,1,zeros(1,6)];
A_f_h = blkdiag(A_f,A_f,A_f,A_f,A_f,A_f,A_f,A_f,A_f,A_f);
lba_force = repmat([Fmin; Fmin],h,1); 
uba_force = repmat([Fmax; Fmax],h,1);

% Line foot constraints :
% ref: Ding, Yanran, et al. "Orientation-Aware Model Predictive Control 
% with Footstep Adaptation for Dynamic Humanoid Walking." 2022 IEEE-RAS 
% 21st International Conference on Humanoid Robots (Humanoids). IEEE, 2022.

lt = 0.09-0.01; lh = 0.06-0.02;% line foot lengths (conservative)
A_LF1=[-lh*[0,0,1]*R_foot_R',zeros(1,3),[0,1,0]*R_foot_R',zeros(1,3);
    -lt*[0,0,1]*R_foot_R',zeros(1,3),-[0,1,0]*R_foot_R',zeros(1,3);
    zeros(1,3),-lh*[0,0,1]*R_foot_L',zeros(1,3),[0,1,0]*R_foot_L';
    zeros(1,3),-lt*[0,0,1]*R_foot_L',zeros(1,3),-[0,1,0]*R_foot_L'];
A_LF2 = 1*[ [0, lt, -mu*lt]*R_foot_R', zeros(1,3), [0, -mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, lt, -mu*lt]*R_foot_L', zeros(1,3), [0, -mu, -1]*R_foot_L';
    [0, -lt, -mu*lt]*R_foot_R', zeros(1,3), [0, -mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, -lt, -mu*lt]*R_foot_L', zeros(1,3), [0, -mu, -1]*R_foot_L';
    [0, lh, -mu*lh]*R_foot_R', zeros(1,3), [0, mu, 1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, lh, -mu*lh]*R_foot_L', zeros(1,3), [0, mu, 1]*R_foot_L';
    [0, -lh, -mu*lh]*R_foot_R', zeros(1,3), [0, mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, -lh, -mu*lh]*R_foot_L', zeros(1,3), [0, mu, -1]*R_foot_L'];
uba_LF = repmat(zeros(12,1),h,1); 
lba_LF = repmat(ones(12,1)*smallNum,h,1);
A_LF = [A_LF1;A_LF2]; 
A_LF_h = blkdiag(A_LF,A_LF,A_LF,A_LF,A_LF,A_LF,A_LF,A_LF,A_LF,A_LF);

% motor torque limit
A_tau = contact_mapping;
A_tau_h = blkdiag(A_tau,A_tau,A_tau,A_tau,A_tau,A_tau,A_tau,A_tau,A_tau,A_tau);

uba_tau = repmat([33.5;33.5;33.5;51;33.5; 33.5;33.5;33.5;51;33.5],h,1).*sigma_m;
lba_tau = -uba_tau.*sigma_m;

% foot moment Mx=0: 
Moment_selection=[1,0,0];
A_M = [zeros(1,3),zeros(1,3),Moment_selection*R_foot_R',zeros(1,3);
    zeros(1,3),zeros(1,3),zeros(1,3),Moment_selection*R_foot_L'];
A_M_h = blkdiag(A_M,A_M,A_M,A_M,A_M,A_M,A_M,A_M,A_M,A_M);
uba_M = zeros(2*h,1);
lba_M = uba_M;

% Constraint Aggregation:
A(1:80,:) = A_mu_h; % row 1-80
A(81:100,:) = A_f_h; % row 81-100
A(101:220,:) = A_LF_h; % row 101-220
A(221:320,:) = A_tau_h; % row 221-320
A(321:340,:) = A_M_h; % row 321-340

lba = [lba_mu; lba_force; lba_LF; lba_tau; lba_M];
uba = [uba_mu; uba_force; uba_LF; uba_tau; uba_M];

%% QPOASES setup:
Hsize = size(Hd);
fsize = size(fd);
H = DM(Hsize);
H = DM(Hd);
g = DM(fsize);
g = DM(fd);

qp = struct;
qp.h = H.sparsity();
qp.a = A.sparsity();
% qp.setOptions( setToReliable() );
% print_options()
opts = struct('enableEqualities', 1, 'printLevel', 'low',...
    'enableFullLITests',0);
S = conic('S','qpoases',qp,opts);
% disp(S)
r = S('h', H, 'g', g, 'a', A, 'lba',lba, 'uba', uba);

% solve!:
tic
x_opt = r.x;
% disp(x_opt(1:12));
disp(' ');
disp(['MPC Time Step:',num2str(k)]);
disp('QPOASES-MPC Solve Time:');
et = toc

GRFM = full(x_opt);
GRFM = GRFM(1:12);
% u=-contact_mapping*GRFM(1:12); 
u=[GRFM(1:12);et];
disp('QPOASES-MPC Function Total Time:');
toc
end


%% functions %%

% TODO: implement this more accurate foot prediction based on gait
function foot_traj = Calc_foot_traj_3Dwalking(xdes,x_traj,foot,h,k,R)
global dt_MPC_vec 
i_MPC_gait = rem(k,h);

foot_traj = zeros(6,h);
%foot prediction: walking gait
if 1 <= i_MPC_gait && i_MPC_gait <= 5 % stance sequence: R-L-R
    current_R = foot(1:3); % current right foot - anchor foot
    next_L = current_R + xdes(10)*dt_MPC_vec(k)*h/2;
    next_R = next_L + xdes(10)*dt_MPC_vec(k+5)*h/2;
    %phase 1: right foot anchoring (6-i_MPC_gait)
    for i = 1:6-i_MPC_gait
        foot_traj(4:6,i) = R*current_R;
    end
    %phase 2: next left foot anchoring (5)
    for i = 7-i_MPC_gait:11-i_MPC_gait
        foot_traj(1:3,i) = R*next_L;
    end
    %phase 3: next right foot anchoring (i_MPC-gait-1)
    if i_MPC_gait>1
        for i = 12-i_MPC_gait:h
            foot_traj(4:6,i) = R*next_R;
        end
    end
else % stance sequence: L-R-L
    if i_MPC_gait == 0; i_MPC_gait = 10; end
    i_MPC_gait = i_MPC_gait - h/2;
    current_L = foot(1:3); % current left foot - anchor foot
    next_R = current_L + n*xdes(10)*dt_MPC_vec(k)*h/2;
    next_L = next_R + n*xdes(10)*dt_MPC_vec(k+5)*h/2;
    %phase 1: left foot anchoring (6-i_MPC_gait)
    for i = 1:6-i_MPC_gait
        foot_traj(1:3,i) = R*current_L;
    end
    %phase 2: next right foot anchoring (5)
    for i = 7-i_MPC_gait:11-i_MPC_gait
        foot_traj(4:6,i) = R*next_R;
    end
    %phase 3: next left foot anchoring (i_MPC-gait-1)
    if i_MPC_gait>1
        for i = 12-i_MPC_gait:h
            foot_traj(1:3,i) = R*next_L;
        end
    end
end

end


function x_traj = Calc_x_traj(xdes,x,h,k)
global dt_MPC_vec 
 for i = 0:h-1
     for j = 1:6
         if xdes(6+j) == 0
             x_traj(j,i+1) = xdes(j) + xdes(6+j)*sum(dt_MPC_vec(k:k+i));
         else

             x_traj(j,i+1) = x(j) + xdes(6+j)*sum(dt_MPC_vec(k:k+i));
         end
         x_traj(6+j,i+1) = xdes(6+j);
     end
         x_traj(13,i+1) = xdes(13);
 end
end

function sigma_m = gaitSchedule(i, gait)
h = 10;
k = rem(i,h);
if k == 0; k = 10; end
% walking 
Rm = [0;0;0;0;0; 1;1;1;1;1];
Lm = [1;1;1;1;1; 0;0;0;0;0];

j = [2-k,3-k,4-k,5-k,6-k,...
    7-k,8-k,9-k,10-k,11-k];
j(j<=0) = j(j<=0)+h;

IOI{j(1)} =  Rm; IOI{j(2)} =  Rm; IOI{j(3)} =  Rm; IOI{j(4)} =  Rm; IOI{j(5)} =  Rm;
IOI{j(6)} =  Lm; IOI{j(7)} =  Lm; IOI{j(8)} =  Lm; IOI{j(9)} =  Lm; IOI{j(10)} = Lm;

sigma_m = ...%gait matrix for 10 horizons;
    vertcat(IOI{1},IOI{2},IOI{3},IOI{4},IOI{5},IOI{6},IOI{7},IOI{8},IOI{9},IOI{10});
end

function sigma_o = payloadContact(t)
    sigma_o = 1*(t>0);
end

function A= skew(v)
 A=[0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ]; 
end