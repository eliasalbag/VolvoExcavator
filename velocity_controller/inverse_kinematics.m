function thetadot_ref = fcn(vx_ref, vy_ref, theta1, theta2)
% Safe version for Simulink — works even when inputs become vectors

vx_ref = vx_ref(1);


% Link lengths [m]
L1 = 1903.392e-3;
L2 = 1351.485e-3;

max_joint_vel1 = 0.1;  
max_joint_vel2 = 0.1;

% --- Jacobian ---
J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
J12 = -L2 * sin(theta1 + theta2);
J21 =  L1 * cos(theta1) + L2 * cos(theta1 + theta2);
J22 =  L2 * cos(theta1 + theta2);

J = [J11 J12; 
     J21 J22];

% --- desired Cartesian velocity vector (2×1) ---
xydot_ref = [vx_ref; vy_ref];



% --- compute damped least squares inverse (2x2 case) ---
lambda = 0.05;   % damping, tune (0.01..0.2) as needed
JJt = J * J';
inv_term = inv(JJt + lambda^2 * eye(2));
J_dls = J' * inv_term;

thetadot = J_dls * xydot_ref;

% --- clip to joint velocity limits ---
thetadot1 = max(min(thetadot(1), max_joint_vel1), -max_joint_vel1);
thetadot2 = max(min(thetadot(2), max_joint_vel2), -max_joint_vel2);

% --- output vector (2×1) ---
thetadot_ref = [thetadot1; thetadot2];
end
