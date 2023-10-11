function out = JacobianMapping(uin)
% This function maps MPC output (GRF&M) to joint torque (Actuation) %

x = uin(1:12); % SRBD states
q = uin(13:22); % Joint states
u = uin(23:34); % MPC output
eul = x(1:3);

% R = Rz(eul(3))*Ry(eul(2))*Rx(eul(1)); % rotation matrix
R = eul2rotm(flip(eul'));
RR=reshape(R,[9,1]);

global Contact_Jacobian
Jc=Contact_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));

% torque=contact_mapping*u
contact_mapping=[blkdiag(Jc(1:3,:)',Jc(4:6,:)'),blkdiag(Jc(7:9,:)',Jc(10:12,:)')]; 
out = contact_mapping*(-u); % joint torque

end