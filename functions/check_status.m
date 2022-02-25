function [status] = check_status(sigma_dot,sigma_star,valid_set)
%Check whether set-based task is active or inactive with extended tangent
%cone to valid_set (current task velocity sigma_dot and task value in NEXT iteration
%sigma_star are relevant)
%This is a modification of the extended tangent cone from [Moe2016]
%Input:
%double sigma_dot: 
%   Task velocity
%double sigma_star: 
%   Task value in next iteration (after numerical integration of q_dot)
%double [1 x 2] valid_set: 
%   Task lower and upper boundary
%Output:
%bool status: 
%   set-based task status -> Is sigma_dot element of the extended tangent cone? 
sigma_min=valid_set(1);
sigma_max=valid_set(2);
if sigma_min<sigma_star && sigma_star<sigma_max
    %task value is within valid-set in the next iteration
    status=true;
elseif sigma_star<=sigma_min && (round(sigma_dot,5)==0 || sigma_dot>0)
    %task value in the next iteration is outside the valid-set, but task
    %velocity points to the interior of the valid-set
    status=true;
elseif sigma_star>=sigma_max && (round(sigma_dot,5)==0 || sigma_dot<0) 
    %task value in the next iteration is outside the valid-set, but task
    %velocity points to the interior of the valid-set
    status=true;
else
    status=false;
end
end

