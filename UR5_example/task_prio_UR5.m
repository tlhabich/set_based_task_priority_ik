%% UR5 Example 1 (without FOV set-based task)
% Section 7.3 in [Moe2016]
close all;
clear;
clc;

% DoF UR5
n_UR5=6;
% maximum iterations
max_iterations=3000;
% maximum delta_q per iteration
max_delta_q=3*pi/180;
% maximum time step (numerical integration delta_q=q_dot*delta_t)
delta_t=1e-2;

% Set-based task sigma_a (collision avoidance)
p_o1=[0.4;-0.25;-0.33];
r1=0.18;
C_a=[r1,inf];

% Set-based task sigma_b (collision avoidance)
p_o2=[0.4;0.15;-0.33];
r2=0.15;
C_b=[r2,inf];

% Equality task sigma_1 (position control)
p_w1=[0.486;-0.066;-0.25];
p_w2=[0.32;0.37;-0.25];

% Active modes table with descending priority (2^j lines)
% Table 6 [Moe2016]
% Active=1
% Inactive=0
modes=[...
    0,0;
    1,0;
    0,1;
    1,1];

% 3T2R equality
% Equality tasks types and additional parameters
eq_tasks=struct('types',[3],... % 3T2R position
    'params',[n_UR5]); % at joint #n_UR5 (end effector)
% Equality tasks desired values
T_0_w1=eye(4);
T_0_w1(1:3,4)=p_w1;
eq_tasks_values=T_0_w1;
% Equality tasks gains as cell array
eq_tasks_gains=.3*eye(5);

% Set-based tasks struct obstacle avoidance
set_tasks=struct(...
    'j',2,... % number of set-based tasks
    'C',[C_a;C_b],... % valid sets
    'types', [1,1],... % type of set-based task
    'params',[n_UR5,n_UR5]); % at joint #n_UR5 (end effector)
% Set-based tasks values as cell array
set_tasks_values=[p_o1,p_o2];

% Preallocation
q = [1.7809;0.4115;0.4475;0.2984;-0.2144;0.0000];
p_start=[0.3;-0.5;-0.4];
fkine = str2func(sprintf('trans_0_E_UR5'));
J_geom = str2func(sprintf('jac_g_UR5'));
history_EE_pos=NaN(3,max_iterations);
history_sigma_a=NaN(1,max_iterations);
history_sigma_b=NaN(1,max_iterations);
history_sigma_tilde1=NaN(size(eq_tasks_gains,1),max_iterations);
history_sigma_tilde2=NaN(size(eq_tasks_gains,1),max_iterations);
history_mode=NaN(1,max_iterations);
J_aug_set=NaN(set_tasks.j,n_UR5); % maximum possible size
line_J_aug_set=0;
line_J_aug=0;
%max dimension of all equality tasks
max_dim_eq=size(eq_tasks_gains,1);
%dimension of all equality tasks
dim_eq=size(eq_tasks_gains,1);

J_eq=NaN(max_dim_eq,n_UR5,size(eq_tasks.types,2)); % maximum possible size
sigma_tilde=NaN(max_dim_eq,size(eq_tasks.types,2)); % maximum possible size
J_aug=NaN(dim_eq+set_tasks.j,n_UR5); % maximum possible size

% Task Prio
for iter = 1:max_iterations % ik iterations
    % compute all jacobians for equality tasks (one time per iteration)
    for k_current = 1:size(eq_tasks_values,3)
        [J,sigma_tilde_out,dim_task] = compute_J_sigma_tilde_eq(eq_tasks.types(k_current),...
            eq_tasks_values(:,:,k_current),eq_tasks.params(k_current),fkine,J_geom,q);
        sigma_tilde(1:dim_task,k_current)=sigma_tilde_out(1:dim_task,:);
        J_eq(1:dim_task,:,k_current)=J(1:dim_task,:);
    end
    for i_mode = 1:size(modes,1) % check mode during every iteration
        mode=modes(i_mode,:); % line in modes table
        line_J_aug_set=0;
        for j_current = 1:set_tasks.j % build augmented jacobian for current mode
            if mode(j_current)==1 % for active set-based task only
                [J,] = compute_J_sigma_set(set_tasks.types(j_current),set_tasks_values(:,j_current),...
                    set_tasks.params(j_current),fkine,J_geom,q);
                line_J_aug_set=line_J_aug_set+1;
                J_aug_set(line_J_aug_set,:)=J;
            end
        end
        
        % compute IK step for current mode
        q_dot=zeros(n_UR5,1);
        line_J_aug=0;
        if line_J_aug_set~=0
            J_aug(1:line_J_aug_set,:)=J_aug_set(1:line_J_aug_set,:);
            line_J_aug=line_J_aug_set;
        end
        for k_current = 1:size(eq_tasks_values,3)
            if line_J_aug_set==0 && k_current==1
                % all set-based tasks inactive
                % N=eye(DoF) for FIRST equality task
                N=eye(n_UR5);
            else
                % compute null space projector N=eye-pinv(J_aug)*J_aug
                N=eye(n_UR5)-pinv(J_aug(1:line_J_aug,:))*J_aug(1:line_J_aug,:);
            end
            % add joint velocity from equality task #k_current
            q_dot=q_dot+N*pinv(J_eq(1:size(eq_tasks_gains(:,:,k_current),1),:,k_current))*eq_tasks_gains(:,:,k_current)*sigma_tilde(1:size(eq_tasks_gains(:,:,k_current),1),k_current);
            % build J_aug for equality task #k_current+1
            J_aug(line_J_aug+1:line_J_aug+size(eq_tasks_gains(:,:,k_current),1),:)=J_eq(1:size(eq_tasks_gains(:,:,k_current),1),:,k_current);
            line_J_aug=line_J_aug+size(eq_tasks_gains(:,:,k_current),1);
        end
        
        % numerical integration of q_dot
        % maximum delta_q with choosen time step
        delta_q=q_dot*delta_t;
        max_delta_q_current=max(abs(delta_q));
        if max_delta_q_current>max_delta_q
            % scale
            delta_q=delta_q*max_delta_q/max_delta_q_current;
        end
        
        % check set-based tasks
        set_based_flag=1;
        for j_current = 1:set_tasks.j
            % current iteration
            [J,~]=compute_J_sigma_set(set_tasks.types(j_current),set_tasks_values(:,j_current),...
                set_tasks.params(j_current),fkine,J_geom,q);
            % next iteration (q+delta_q)
            [~,sigma_star]=compute_J_sigma_set(set_tasks.types(j_current),set_tasks_values(:,j_current),...
                set_tasks.params(j_current),fkine,J_geom,q+delta_q);
            current_status=check_status(J*q_dot,sigma_star,set_tasks.C(j_current,:));
            if current_status==false
                set_based_flag=0;
                break;
            end
        end
        
        % all set-based task velocitys are element of the extended tangent
        % cone and, therefore, q+delta_q is accepted
        if set_based_flag
            % update q
            q=q+delta_q;
            break;
        end       
    end
    
    history_mode(iter)=i_mode;
    T_0_i=fkine(q,eq_tasks.params(1));
    history_EE_pos(:,iter)=T_0_i(1:3,4);
    history_sigma_a(iter)=norm(p_o1-T_0_i(1:3,4));
    history_sigma_b(iter)=norm(p_o2-T_0_i(1:3,4));
%     J_geom_b=J_geom(q,n_UR5);
%     history_sigma_b(iter)=cond(J_geom1(1:3,:))*cond(J_geom2(1:3,:))/2;
%     history_sigma_b(iter)=cond(J_geom_b(1:3,:));
    history_sigma_tilde1(:,iter)=sigma_tilde(1:size(eq_tasks_gains(:,:,1),1),1);
    
    % Switch to second goal
    if norm(sigma_tilde(1:size(eq_tasks_gains(:,:,1),1),1))<0.02
            T_0_w2=eye(4);
            T_0_w2(1:3,4)=p_w2;
            eq_tasks_values=(T_0_w2);
    end
end

%% Plot results
imesblau   = [0 80 155 ]/255; 
imesorange = [231 123 41 ]/255; 
imesgruen  = [200 211 23 ]/255;
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'DefaultLineLineWidth',2);
set(groot,'defaultAxesFontSize',14)
set(gcf,'PaperPositionMode','auto');


if eq_tasks.types(1)==4 || eq_tasks.types(1)==3
    % Position error
    figure;
    plot((1:max_iterations)*delta_t,history_sigma_tilde1(1,:))
    hold on
    plot((1:max_iterations)*delta_t,history_sigma_tilde1(2,:))
    plot((1:max_iterations)*delta_t,history_sigma_tilde1(3,:))
    legend("$\tilde{\sigma}_{1x}$","$\tilde{\sigma}_{1y}$","$\tilde{\sigma}_{1z}$")
    ylabel("Position error in m")
    xlabel("Time in s")
    title("Equality tasks")
    grid on;
    % Orientation error
    figure;
    plot((1:max_iterations)*delta_t,history_sigma_tilde1(4,:))
    hold on
    plot((1:max_iterations)*delta_t,history_sigma_tilde1(5,:))
    if eq_tasks.types(1)==4
        plot((1:max_iterations)*delta_t,history_sigma_tilde1( 6,:))
        legend("$\tilde{\sigma}_{1\alpha_1}$","$\tilde{\sigma}_{1\alpha_2}$","$\tilde{\sigma}_{1\alpha_3}$")
    else
        legend("$\tilde{\sigma}_{1\alpha_2}$","$\tilde{\sigma}_{1\alpha_3}$")
    end
    ylabel("Orientation error in rad")
    xlabel("Time in s")
    title("Equality tasks")
    grid on;
    %exportgraphics(gcf,'.\plots\orientation_error.png','Resolution',500)  
end

% EE trajectory
figure;
[x_s1,y_s1,z_s1]=sphere;
x_s1=r1*x_s1;
y_s1=r1*y_s1;
z_s1=r1*z_s1;
surf(x_s1+p_o1(1),y_s1+p_o1(2),z_s1+p_o1(3),"FaceAlpha",0.25);
hold on;
[x_s2,y_s2,z_s2]=sphere;
x_s2=r2*x_s2;
y_s2=r2*y_s2;
z_s2=r2*z_s2;
surf(x_s2+p_o2(1),y_s2+p_o2(2),z_s2+p_o2(3),"FaceAlpha",0.25);
axis equal;
grid on;
title('EE trajectory')
plot3(history_EE_pos(1,:),history_EE_pos(2,:),history_EE_pos(3,:),'LineWidth',2,'Color',imesorange);
plot3(p_start(1),p_start(2),p_start(3),'o','MarkerSize',10,'Color',imesgruen,'MarkerFaceColor',imesgruen);
plot3(p_w1(1),p_w1(2),p_w1(3),'o','MarkerSize',10,'Color',imesgruen,'MarkerFaceColor',imesgruen);
plot3(p_w2(1),p_w2(2),p_w2(3),'o','MarkerSize',10,'Color',imesgruen,'MarkerFaceColor',imesgruen);
xlabel("$x$ in m")
ylabel("$y$ in m")
zlabel("$z$ in m")
trplot(T_0_w2,'color',imesgruen,'length',0.2);
trplot(fkine(q,n_UR5),'color',imesorange,'length',0.2);
%exportgraphics(gcf,'.\plots\ee_trajectory.png','Resolution',500)


% Collision avoidance
figure;
yline(r1,'--','$r_1$','Interpreter','Latex');
hold on
yline(r2,'--','$r_2$','Interpreter','Latex');
pa=plot((1:max_iterations)*delta_t,history_sigma_a,'Color',imesorange);
pb=plot((1:max_iterations)*delta_t,history_sigma_b,'Color',imesblau);
ylabel("Distance in m")
xlabel("Time in s")
grid on;
title("Set-based tasks")
%legend('$\sigma_a$','$\sigma_{a,min}$','$\sigma_b$','$\sigma_{b,min}$')
legend([pa pb],{'$\sigma_a$','$\sigma_b$'});
%exportgraphics(gcf,'.\plots\set_based.png','Resolution',500)

% Modes
figure;
plot((1:max_iterations)*delta_t,history_mode)
ylabel("Mode")
xlabel('Time in s')
grid on;
title("Active Mode")
%exportgraphics(gcf,'.\plots\mode_history.png','Resolution',500)