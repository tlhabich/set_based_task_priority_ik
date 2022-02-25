function sigma_tilde = compute_3T2R_sigma_tilde(q, Tr0E, T_N_E, phiconv_W_E, I_EElink, reci, handle_f, len)
%Task error for 3T3R- or 3T2R-formulation
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
% len double
%   optional kinematic length within handle_f
%Output:
% sigma_tilde [5 x 1] or [6 x 1]
%   task error (position and orientation)

if nargin==7
    T_0_N_q=handle_f(q,I_EElink);
else
    T_0_N_q = handle_f(q, I_EElink,len);
end
T_0_E_q = T_0_N_q * T_N_E;

%% Translation
r_0_E_x = zeros(3,1);
r_0_E_x = Tr0E(1:3,4); 
r_0_E_q = T_0_E_q(1:3,4); 
Phix = r_0_E_q - r_0_E_x;

%% Rotation
R_0_E_x = Tr0E(1:3,1:3); 
if reci
  % reciprocal euler angles
  [~,phiconv_delta] = euler_angle_properties(phiconv_W_E);
else
  phiconv_delta = phiconv_W_E;
end
R_0_E_q = T_0_E_q(1:3,1:3);

% Difference rotation
R_Ex_Eq = R_0_E_x' * R_0_E_q;
phiR = r2eul(R_Ex_Eq, phiconv_delta);


if reci
    sigma_tilde = [Phix; phiR(2:3,1)]; 
else
    sigma_tilde = [Phix; phiR];
end

end