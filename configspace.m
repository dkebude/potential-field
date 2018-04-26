function [alpha, beta, alpha_goal, beta_goal, c_space] = configspace(axes_hl, goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, obs_radius, plotFlag)

cla(axes_hl, 'reset')
%% angle intervals
d_alpha = 1.8;
d_beta = 3.6;

alpha = ((0:d_alpha:180)*pi)/180;
beta = ((0:d_beta:360)*pi)/180;
[alpha_goal, beta_goal] = inverseKinematics(goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2);

%% convert to configuration space
c_space = zeros(length(alpha), length(beta));
for i = 1:length(alpha)
    for j = 1:length(beta)
        c_space(i,j) = checkCollision(alpha(i), beta(j), robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, obs_radius);
    end
end

%% display configuration space
if(plotFlag == 1)
    axes(axes_hl)
    [alpha_grid, beta_grid] = meshgrid(alpha*180/pi, beta*180/pi);
    meshc(alpha_grid, beta_grid, c_space);
    view(2);
    drawnow;
end