function J = compute_3T2R_J(q, Tr0E, T_N_E, phiconv_W_E, I_EElink, reci, handle_f, handle_J,len)
%Task jacobian for 3T3R- or 3T2R-formulation [Schappler2019]
%Input:
%q double [n x 1]
%   current joint angles
%Tr0E double [4 x 4]
%   desired end-effector pose
% T_N_E double [4 x 4]
%   static transformation EE-flange -> tool tip
% phiconv_W_E int
%   euler convention world -> EE
% I_EElink
%   number of the joint to be positioned
% reci bool
%   true: use reciprocal euler-angles for orientation error (3T2R)
%   false: 3T3R
% handle_f function handle
%   direct kinematics
% handle_J function handle
%   geometric jacobian
% len double
%   optional kinematic length within handle_f and handle_J
%Output:
% J double [6 x n]
%   task jacobian
%

%#codegen
%$cgargs {zeros(8,1),zeros(3,4),zeros(1,1),zeros(4,4),uint8(2),uint8(0), true}

%% Translation
if nargin==8
    J0_i_tmp = handle_J(q, I_EElink);
else
    J0_i_tmp = handle_J(q, I_EElink,len);
end
J0_i_trans = J0_i_tmp(1:3,:);
dPhit_dq = J0_i_trans;

%% Rotation
R_0_E_x = Tr0E(1:3,1:3);
if reci
  [~,phiconv_delta] = euler_angle_properties(phiconv_W_E);
else
  phiconv_delta = phiconv_W_E;
end

J0_i_rot = J0_i_tmp(4:6,:);
if nargin==8
    T_0_N_q = handle_f(q, I_EElink);
else
    T_0_N_q = handle_f(q, I_EElink,len);
end

R_0_E_q = T_0_N_q(1:3,1:3) * T_N_E(1:3,1:3);
R_Ex_Eq = R_0_E_x' * R_0_E_q;

% dR_0_E/dq
b11=R_0_E_q(1,1);b12=R_0_E_q(1,2);b13=R_0_E_q(1,3);
b21=R_0_E_q(2,1);b22=R_0_E_q(2,2);b23=R_0_E_q(2,3);
b31=R_0_E_q(3,1);b32=R_0_E_q(3,2);b33=R_0_E_q(3,3);
premult_matrix = [0 b31 -b21; -b31 0 b11; b21 -b11 0; 0 b32 -b22; -b32 0 b12; b22 -b12 0; 0 b33 -b23; -b33 0 b13; b23 -b13 0;];
dRb_0E_dq = premult_matrix * J0_i_rot;

a11=R_0_E_x(1,1);a12=R_0_E_x(2,1);a13=R_0_E_x(3,1);
a21=R_0_E_x(1,2);a22=R_0_E_x(2,2);a23=R_0_E_x(3,2);
a31=R_0_E_x(1,3);a32=R_0_E_x(2,3);a33=R_0_E_x(3,3);
dPi_dRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

ddeltaR_dRb = eul_diff_rotmat(R_Ex_Eq,phiconv_delta);

dPhir_dq = ddeltaR_dRb * dPi_dRb2 * dRb_0E_dq;

if reci
    J = [dPhit_dq; dPhir_dq(2:3,:)];
else
    J = [dPhit_dq; dPhir_dq];
end

end

