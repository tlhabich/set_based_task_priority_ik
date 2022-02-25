function [J,sigma] = compute_J_sigma_set(task_type,task_value,task_param,fkine_handle,J_geom_handle,q,len)
%Compute task jacobian (set-based)
%Input:
%task_type double
%   euclidean distance positioning or collision avoidance task: 1
%   joint limit task: 2
%   joint limit task with hyperbolical function: 3
%   singularity avoidance task (condition number of geometric jacobian): 4
%   singularity avoidance task (condition number of 3T jacobian): 5
%task_value double [.. x 1]
%   desired value for "eucl_dist": 3-dimensional position
%   desired value for "joint_lim": not necessary
%   desired value for "cond_J_geom", "cond_J_pos": number of joints whose jacobians 
%       are to be checked by means of multiplying condition numbers 
%       (not necessary if condition number of single jacobian should be checked) 
%task_param double
%   param for "eucl_dist": number of the joint to be positioned
%   param for "joint_lim": number of the joint
%   param for "cond_J_geom", "cond_J_pos": 
%       0 -> jacobians of several joints are to be checked (see task_value)
%       >0 -> number of single joint whose jacobian is to be checked
%fkine function handle
%   direct kinematics to all joints i 
%J_geom function handle
%   geometric jacobian to all joints i
%q double [n x 1]
%   current joint angles
% len double
%   optional kinematic length within handle_f and handle_J
%Output:
%J double [1 x n]
%   task jacobian
%sigma double
%   task value
    if nargin==6
        T_0_i=fkine_handle(q,task_param);
        J_g=J_geom_handle(q,task_param);
    else
        T_0_i=fkine_handle(q,task_param,len);
        J_g=J_geom_handle(q,task_param,len);
    end
    if task_type==1 
        % other set-based task types can be implemented here 
        % see "Antonelli2014 - Underwater Robots" for set-based
        % task definitions
        p_ref=task_value;      
        p_i=T_0_i(1:3,4);
        sigma=norm(p_ref-p_i);
        J_g_pos=J_g(1:3,:);
        J=-(1/sigma)*(p_ref-p_i)'*J_g_pos;
        if size(J,2)<length(q)
            % change size of the calculated jacobian J
            % to match the dimension of q
            J_tmp=zeros(1, length(q));
            J_tmp(:,1:size(J,2))=J;
            J=J_tmp;
        end 
    elseif task_type==2
        sigma=q(task_param);
        J=zeros(1,size(q,1));
        J(1,task_param)=1;
    elseif task_type==3
        % every joint has the same threshold for activation
        % every joint has the same q_min and q_max
        
        q_min = ones(length(q),1) * task_value(1);
        qthr_min = ones(length(q),1) * task_value(2);
        qthr_max = ones(length(q),1) * task_value(3);
        q_max = ones(length(q),1) * task_value(4);
        
        [sigma, J] = invkin_optimcrit_limits2(q, ...
                                              [q_min,q_max], ...
                                              [qthr_min,qthr_max]);
        if sigma ~= 0
            sigma = sigma
            J = J
        end
    elseif task_type==4
        J = NaN(1,length(q));
        dq=1e-6;
        if task_param>0 % condition number of single jacobian
            sigma=cond(J_g);
        else % sigma = product of condtion numbers of choosen jacobians/number of jacobians
            sigma=1;
        end
        for i = 1:length(q) % difference quotient for every joint
          q_test = q; % current configuration
          q_test(i) = q_test(i) + dq; % minimal increment
          if task_param > 0 % condition number of single jacobian
              if nargin==6
                  sigma_test=cond(J_geom_handle(q_test,task_param));% condition q_i+dq
              else
                  sigma_test=cond(J_geom_handle(q_test,task_param,len));% condition q_i+dq
              end
              dsigma=sigma_test-sigma;
              J(1,i)=dsigma/dq;      
          else % sigma = product of condtion numbers of choosen jacobians/number of jacobians
              sigma_test=1;
              for j = 1:length(task_value)
                  if i==1 % sigma must be calculated only once
                      if nargin==6
                          sigma=sigma*cond(J_geom_handle(q,task_value(j)));
                      else
                          sigma=sigma*cond(J_geom_handle(q,task_value(j),len));
                      end
                    if j==length(task_value) % normalize once
                        sigma=sigma/length(task_value);
                    end
                  end
                  if nargin==6
                      sigma_test=sigma_test*cond(J_geom_handle(q_test,task_value(j)));                  
                  else
                      sigma_test=sigma_test*cond(J_geom_handle(q_test,task_value(j),len));                  
                  end   
              end
              dsigma=sigma_test/length(task_value)-sigma; %sigma already normalized
              J(1,i)=dsigma/dq;
          end   
        end
    elseif task_type==5
        J = NaN(1,length(q));
        dq=1e-6;
        if task_param>0 % condition number of single jacobian
            J_pos=J_g(1:3,:);
            sigma=cond(J_pos);
        else % sigma = product of condtion numbers of choosen jacobians/number of jacobians
            sigma=1;
        end
        for i = 1:length(q) % difference quotient for every joint
          q_test = q; % current configuration
          q_test(i) = q_test(i) + dq; % minimal increment
          if task_param > 0 % condition number of single jacobian
              if nargin==6
                  J_pos_test=J_geom_handle(q_test,task_param);                
              else
                  J_pos_test=J_geom_handle(q_test,task_param,len);                  
              end
              J_pos_test=J_pos_test(1:3,:);
              sigma_test=cond(J_pos_test); % condition q_i+dq
              dsigma=sigma_test-sigma;
              J(1,i)=dsigma/dq;      
          else % sigma = product of condtion numbers of choosen jacobians/number of jacobians
              sigma_test=1;
              for j = 1:length(task_value)
                  if i==1 % sigma must be calculated only once
                      if nargin==6
                          J_pos=J_geom_handle(q,task_value(j));                
                      else
                          J_pos=J_geom_handle(q,task_value(j),len);                  
                      end     
                    J_pos=J_pos(1:3,:);  
                    sigma=sigma*cond(J_pos);
                    if j==length(task_value) % normalize once
                        sigma=sigma/length(task_value);
                    end
                  end
                  if nargin==6
                      J_pos_test=J_geom_handle(q_test,task_value(j));                
                  else
                      J_pos_test=J_geom_handle(q_test,task_value(j),len);                 
                  end
                  J_pos_test=J_pos_test(1:3,:);
                  sigma_test=sigma_test*cond(J_pos_test);
              end
              dsigma=sigma_test/length(task_value)-sigma; %sigma already normalized
              J(1,i)=dsigma/dq;
          end   
        end
    end          
end

