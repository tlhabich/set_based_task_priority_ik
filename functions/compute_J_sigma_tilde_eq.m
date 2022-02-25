function [J,sigma_tilde,dim_task] = compute_J_sigma_tilde_eq(task_type,task_value,task_param,fkine_handle,J_geom_handle,q,len)
%Compute task jacobian and task error (equality)
%Input:
%task_type double
%   x,y,z position task: 1
%   euclidean distance (scalar) position task: 2
%   5-D (3T2R) position task: 3
%   6-D position task: 4
%   2-D orientation (2R) task: 5
%task_value double [4 x 4]
%   desired pose value for "3T", "eucl_dist", "3T2R", "3T3R", "2R": T_0_i [4 x 4]
%task_param double
%   param for "3T", "eucl_dist", "3T2R, "3T3R", "2R": number of the joint to be positioned
%fkine function handle
%   direct kinematics to all joints i 
%J_geom function handle
%   geometric jacobian to all joints i
%q double [n x 1]
%   current joint angles
%Output:
%J double [6 x n]
%   task jacobian (for simulink with maximum size of 6 lines -> see dim_task!)
%sigma_tilde double [6 x 1]
%   task error (for simulink with maximum size of 6 lines -> see dim_task!)
%dim_task double
%   J(1:dim_task,:) is desired task jacobian 
%   sigma_tilde(1:dim_task,:) is desired task error
    T_0_i=zeros(4);
    J_g=zeros(6,task_param);
    J=zeros(6,task_param);
    sigma_tilde=zeros(6,1);
    dim_task=0;
    p_ref=task_value(1:3,4);
    if nargin==6
        T_0_i=fkine_handle(q,task_param);
        J_g=J_geom_handle(q,task_param);
    else
        T_0_i=fkine_handle(q,task_param,len);
        J_g=J_geom_handle(q,task_param,len);
    end
    p_i=zeros(3,1);
    p_i=T_0_i(1:3,4);
    if task_type==1
        % 3T
        dim_task=3;
        sigma_tilde(1:dim_task,:)=p_ref-p_i;
        J(1:dim_task,:)=J_g(1:3,:);
    elseif task_type==2
        % eucl_dist
        dim_task=1;
        sigma=norm(p_ref-p_i);
        sigma_tilde(1:dim_task,:)=0-sigma;  % desired distance = 0
        J_g_pos=J_g(1:3,:);
        J(1:dim_task,:)=-(1/sigma)*(p_ref-p_i)'*J_g_pos;
        if size(J,2)<length(q)
            % change size of the calculated jacobian J
            % to match the dimension of q
            J_tmp=zeros(1, length(q));
            J_tmp(:,1:size(J,2))=J;
            J=J_tmp;
        end
    elseif task_type==3
        % 3T2R
        dim_task=5;
        if nargin==6
            sigma_tilde=-compute_3T2R_sigma_tilde(q,task_value(1:3,:),eye(4),uint8(2), task_param, true, fkine_handle);
            J=compute_3T2R_J(q, task_value(1:3,:), eye(4), uint8(2), task_param, true, fkine_handle, J_geom_handle);
        else
            sigma_tilde=-compute_3T2R_sigma_tilde(q,task_value(1:3,:),eye(4),uint8(2), task_param, true, fkine_handle,len);
            J=compute_3T2R_J(q, task_value(1:3,:), eye(4), uint8(2), task_param, true, fkine_handle, J_geom_handle,len);
        end
    elseif task_type==4
        % 3T3R
        dim_task=6;
        if nargin==6
            sigma_tilde=-compute_3T2R_sigma_tilde(q,task_value(1:3,:),eye(4),uint8(2), task_param, false, fkine_handle);
            J=compute_3T2R_J(q, task_value(1:3,:), eye(4), uint8(2), task_param, false, fkine_handle, J_geom_handle);
        else
            sigma_tilde=-compute_3T2R_sigma_tilde(q,task_value(1:3,:),eye(4),uint8(2), task_param, false, fkine_handle,len);
            J=compute_3T2R_J(q, task_value(1:3,:), eye(4), uint8(2), task_param, false, fkine_handle, J_geom_handle,len);
        end
    elseif task_type==5
        % 2R
        dim_task=2;
        if nargin==6
            sigma_tilde_3T2R=-compute_3T2R_sigma_tilde(q,task_value(1:3,:),eye(4),uint8(2), task_param, true, fkine_handle);
            J_3T2R=compute_3T2R_J(q, task_value(1:3,:), eye(4), uint8(2), task_param, true, fkine_handle, J_geom_handle);
        else
            sigma_tilde_3T2R=-compute_3T2R_sigma_tilde(q,task_value(1:3,:),eye(4),uint8(2), task_param, true, fkine_handle,len);
            J_3T2R=compute_3T2R_J(q, task_value(1:3,:), eye(4), uint8(2), task_param, true, fkine_handle, J_geom_handle,len);
        end
        sigma_tilde=sigma_tilde_3T2R(4:5);
        J=J_3T2R(4:5,:);

    end
end

