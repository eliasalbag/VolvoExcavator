L_AE = 27e-2;
L_AF = 97e-2;
L_EF = 106.8e-2;

L_BG = 98e-2;
L_BH = 30e-2;
L_GH = 109.3e-2;

theta_eaq = 61.3;
theta_fab = 14.55;
theta_abg = 36.46;
theta_hbc = 147.69;

Bucket = 14168;


% calulation of thetaA
numA = L_AE^2 + L_AF^2 - L_EF^2;
denA = 2*L_AE*L_AF;
theta_a = rad2deg(acos(numA/denA)) - theta_eaq - theta_fab


% calculation of thetaB
numB = L_BG^2 + L_BH^2 - L_GH^2;
denB = 2*L_BG*L_BH;
theta_b = rad2deg(acos(numB/denB)) - theta_hbc - theta_abg


% calculation of thetaC
%inåt motsvarar mindre sensorutslag
theta_c = -(theta_a + theta_b) - 180


% 
% newTheta_a = 32.746
% 
% 
% LB = 1876 *10^-3;
% LN = 762 *10^-3;
% L_BN = 1180 * 10^-3;
% theta_NB_vertical = rad2deg(acos((LB-LN)/L_BN))
% newtheta_b = -(180-(90-newTheta_a + theta_NB_vertical))
% 
% Length_floor_CD = 361 *10^-3;
% Height_C = 596 *10^-3;
% Height_tip = 78*10^-3;
% newtheta_c = -(theta_NB_vertical + rad2deg(atan(Length_floor_CD/(Height_C - Height_tip))))