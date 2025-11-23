function thetadot_ref = fcn(vx_ref, vy_ref, theta1, theta2)
% u(1) = vx_ref      [m/s]
% u(2) = vy_ref      [m/s]
% u(3) = theta1      [rad]
% u(4) = theta2      [rad]
%
% thetadot_ref1 [rad/s]
% thetadot_ref  [rad/s]



% Link lengths [m]
L1 = 1903.392e-3;
L2 = 1351.485e-3;


max_joint_vel1 = 0.5;  % [rad/s] joint 1 limit (set to your value)
max_joint_vel2 = 0.5;  % [rad/s] joint 2 limit (set to your value)

% --- Jacobian ---
J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
J12 = -L2 * sin(theta1 + theta2);
J21 =  L1 * cos(theta1) + L2 * cos(theta1 + theta2);
J22 =  L2 * cos(theta1 + theta2);

J = [J11, J12;
     J21, J22];

% --- desired Cartesian velocity vector ---
xydot_ref = [vx_ref; vy_ref];

% --- joint velocity reference (IK with pseudoinverse) ---
thetadot_ref_vec = pinv(J) * xydot_ref;

thetadot_ref1 = thetadot_ref_vec(1);
thetadot_ref2 = thetadot_ref_vec(2);

% --- clip to joint velocity limits ---
if thetadot_ref1 >  max_joint_vel1
    thetadot_ref1 =  max_joint_vel1;
elseif thetadot_ref1 < -max_joint_vel1
    thetadot_ref1 = -max_joint_vel1;
end

if thetadot_ref2 >  max_joint_vel2
    thetadot_ref2 =  max_joint_vel2;
elseif thetadot_ref2 < -max_joint_vel2
    thetadot_ref2 = -max_joint_vel2;
end

% --- output vector ---
thetadot_ref = [thetadot_ref1; thetadot_ref2];
end
