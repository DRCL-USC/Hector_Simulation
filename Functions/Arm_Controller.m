function tau_arm = Arm_Controller(uin)

%% Arm Control %%

% arm control mode
herusticSwing = 1; % otherwise fixed arm

global addArm 
addArm = 1; %turn on addArm for MPC, otherwise Biped properties

q_arm = uin(1:6); % [arm 1, arm 2]
qd_arm = uin(7:12);
q_legs = uin(13:22); % [leg 1, leg 2]

% desired joint position:
q_arm_des = zeros(6,1);
if herusticSwing % arm swing depend the opposite side leg movement
    for i = 0:1
        q_arm_des(3*i + 2) = q_legs(8 - 5*i) - pi/4 ;
        % q_arm_des(3*i + 2) = q_legs(5*i + 3) - pi/4 ;
        q_arm_des(3*i + 1) = 0;
        q_arm_des(3*i + 3) = -pi/2;
    end
else % fixed arm
    q_arm_des = [0;pi/6; -pi/2; 0;pi/6; -pi/2];
end

% PD controller:
Kp = diag([50 50 50 50 50 50]);
Kd = diag([2 2 2 2 2 2]);
tau_arm = Kp*(q_arm_des - q_arm) + Kd*(0 - qd_arm);

end