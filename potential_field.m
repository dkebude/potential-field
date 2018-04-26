function potential_field(init_pos_x, init_pos_y, goal_pos_x, goal_pos_y, L1, L2, robot_base_x, robot_base_y, obs_pos_x, obs_pos_y, obs_radius)

%% area
figure;
xborder=[0,100];
yborder=[0,100];
line([xborder(1),xborder(2),xborder(2),xborder(1)],[yborder(1),yborder(1),yborder(2),yborder(2)]);
hold on;

%% draw the robot
pos_init=[init_pos_x, init_pos_y];
pos_goal=[goal_pos_x, goal_pos_y];
x_ee = pos_init(1);
y_ee = pos_init(2);
[alpha_init, beta_init] = inverseKinematics(x_ee, y_ee, robot_base_x, robot_base_y, L1, L2);
[alpha_goal, beta_goal] = inverseKinematics(pos_goal(1), pos_goal(2), robot_base_x, robot_base_y, L1, L2);
[x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(alpha_init, beta_init, robot_base_x, robot_base_y, L1, L2);
x_link1_init = [robot_base_x, x_elbow];
y_link1_init = [robot_base_y, y_elbow];
x_link2_init = [x_elbow, x_ee];
y_link2_init = [y_elbow, y_ee];

hl = line(x_link1_init, y_link1_init);
set(hl, 'Color', 'k');
plot(x_link1_init(2), y_link1_init(2), 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
hl = line(x_link2_init, y_link2_init);
set(hl, 'Color', 'k');
plot(x_link2_init(2), y_link2_init(2), 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

%% plot initial and goal location of the robot
plot(pos_init(1),pos_init(2),'rd', 'MarkerFaceColor', 'r')
plot(pos_goal(1),pos_goal(2),'rd', 'MarkerFaceColor', 'r')

%% draw the circle
pos_obs=[obs_pos_x,obs_pos_y];
radius=obs_radius;
xfirst=pos_obs(1)+radius*cos(0);
yfirst=pos_obs(2)+radius*sin(0);

for i=0:10:360
    teta=(i*pi)/180;
    xc=pos_obs(1)+(radius*cos(teta));
    yc=pos_obs(2)+(radius*sin(teta));
    line([xfirst,xc],[yfirst,yc]);
    xfirst = xc;
    yfirst = yc;
end

grid on;
drawnow;
hold off;

%%%%% convert to configuration space
%% angle intervals
d_alpha = 1.8;
d_beta = 3.6;

alpha = ((0:d_alpha:180)*pi)/180;
beta = ((0:d_beta:360)*pi)/180;

%% convert to configuration space
c_space = zeros(length(alpha), length(beta));
for i = 1:length(alpha)
    for j = 1:length(beta)
        c_space(i,j) = checkCollision(alpha(i), beta(j), robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, radius);
    end
end

%% display configuration space
[alpha_grid, beta_grid] = meshgrid(alpha*180/pi, beta*180/pi);
figure;
meshc(alpha_grid, beta_grid, c_space);
view(2);
drawnow;

%% define potential field gains
att_gain = 100;
rep_gain = 1;

%% define radius of influence
rho_0 = 10;

%% Initialize potentials and forces
U_att = zeros(length(alpha), length(beta));
U_rep = zeros(length(alpha), length(beta));
F_att_alpha_grid = zeros(length(alpha), length(beta));
F_att_beta_grid = zeros(length(alpha), length(beta));
F_rep_alpha_grid = zeros(length(alpha), length(beta));
F_rep_beta_grid = zeros(length(alpha), length(beta));

%% Calculate potentials and forces for the whole grid
for a_i = 1:length(alpha)
    for b_i = 1:length(beta)
        a = alpha(a_i);
        b = beta(b_i);
        U_att(a_i, b_i) = (1/2)*att_gain*((a-alpha_goal).^2+(b-beta_goal).^2);
        F_att_alpha_grid(a_i, b_i) = -att_gain*(a-alpha_goal);
        F_att_beta_grid(a_i, b_i) = -att_gain*(b-beta_goal);
        rho = 15;

        if a_i <= 50
            a_interval = (a_i-(a_i-1)):(a_i+50);
        elseif a_i >= length(alpha)-50
            a_interval = (a_i-50):(a_i+(length(alpha)-a_i-1));
        else
            a_interval = a_i-50:a_i+50;
        end

        if b_i <= 50
            b_interval = (b_i-(b_i-1)):(b_i+50);
        elseif b_i >= length(beta)-50
            b_interval = (b_i-50):(b_i+((length(beta)-b_i)-1));
        else
            b_interval = b_i-50:b_i+50;
        end

        for i = a_interval
            for j = b_interval
                if(c_space(i,j) == 1)
                    a_obs = alpha(i);
                    b_obs = beta(j);
                    dist = sqrt((a-a_obs+1e-2).^2+(b-b_obs+1e-2)^2);
                else
                    dist = rho;
                end
                if(dist < rho)
                    rho = dist;
                    del_rho_a = (a-a_obs+1e-2)/dist;
                    del_rho_b = (b-b_obs+1e-2)/dist;
                end
            end
        end

        if rho <= rho_0
            U_rep(a_i, b_i) = (1/2)*rep_gain*(1/rho - 1/rho_0)^2;
            F_rep_alpha_grid(a_i, b_i) = rep_gain*(1/rho - 1/rho_0)*((1/rho)^2)*del_rho_a;
            F_rep_beta_grid(a_i, b_i) = rep_gain*(1/rho - 1/rho_0)*((1/rho)^2)*del_rho_b;
            if(U_rep(a_i, b_i) > 750)
                U_rep(a_i, b_i) = 750;
            end
        else
            U_rep(a_i, b_i) = 0;
            F_rep_alpha_grid(a_i, b_i) = 0;
            F_rep_beta_grid(a_i, b_i) = 0;
        end
    end
end

%% Total potential and its visualization
U_tot = U_att + U_rep;

figure;
subplot(2,2,1);
meshc(alpha_grid, beta_grid, U_att);
title('Attractive Potential');
subplot(2,2,2);
meshc(alpha_grid, beta_grid, U_rep);
title('Repulsive Potential');
subplot(2,2,[3,4]);
meshc(alpha_grid, beta_grid, U_tot);
title('Total Potential');
drawnow;

%% Calculate total forces
F_tot_alpha = F_att_alpha_grid + F_rep_alpha_grid;
F_tot_beta = F_att_beta_grid + F_rep_beta_grid;

%% Plot initial workspace in cartesian domain and configuration domain
figure;
subplot(2,1,1);
line([xborder(1),xborder(2),xborder(2),xborder(1)],[yborder(1),yborder(1),yborder(2),yborder(2)]);
hold on;
grid on;
plot(pos_init(1),pos_init(2),'rd', 'MarkerFaceColor', 'r')
plot(pos_goal(1),pos_goal(2),'rd', 'MarkerFaceColor', 'r')
xfirst=pos_obs(1)+radius*cos(0);
yfirst=pos_obs(2)+radius*sin(0);
for j=0:10:360
    teta=(j*pi)/180;
    xc=pos_obs(1)+(radius*cos(teta));
    yc=pos_obs(2)+(radius*sin(teta));
    line([xfirst,xc],[yfirst,yc]);
    xfirst = xc;
    yfirst = yc;
end
hl1 = line(x_link1_init, y_link1_init);
set(hl1, 'Color', 'k');
hl2 = plot(x_link1_init(2), y_link1_init(2), 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
hl3 = line(x_link2_init, y_link2_init);
set(hl3, 'Color', 'k');
hl4 = plot(x_link2_init(2), y_link2_init(2), 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

subplot(2,1,2);
alpha_init_index = find(abs(alpha-alpha_init) == min(abs(alpha-alpha_init)));
beta_init_index = find(abs(beta - beta_init) == min(abs(beta - beta_init)));
alpha_goal_index = find(abs(alpha - alpha_goal) == min(abs(alpha - alpha_goal)));
beta_goal_index = find(abs(beta - beta_goal) == min(abs(beta - beta_goal)));
c_space(alpha_init_index, beta_init_index) = 1.5;
hl5 = meshc(alpha_grid, beta_grid, c_space);
view(2);

%% Initialize everything that is necessary for path planning
pos_0 = [alpha_init, beta_init];
i = 1;
path = pos_0;
delta = 0.01;
w_steps = 10;
w_check = 10;

%% Plan and carry out
while(((x_ee-pos_goal(1))^2+(y_ee-pos_goal(2))^2)>1)
    alpha_index = find(abs(alpha-path(i,1)) == min(abs(alpha-path(i,1))));
    beta_index = find(abs(beta-path(i,2)) == min(abs(beta-path(i,2))));
    F_norm = sqrt(F_tot_alpha(alpha_index, beta_index)^2+F_tot_beta(alpha_index,beta_index)^2);
    F_grad = [F_tot_alpha(alpha_index,beta_index)/F_norm, F_tot_beta(alpha_index,beta_index)/F_norm];
        
    if(i>w_check)
        if(sqrt(sum((path(i,:)-path(i-w_check,:)).^2))<delta)
            w_path = wavefront_planner(c_space, alpha, beta, alpha_index, beta_index, alpha_goal_index, beta_goal_index, w_steps);
            path(i+1:i+length(w_path),:) = [alpha(w_path(1,:))', beta(w_path(2,:))'];
            for j = 1:length(w_path)
                a = path(i+j,1);
                b = path(i+j,2);
                [x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(a, b, robot_base_x, robot_base_y, L1, L2);
                x_link1 = [robot_base_x, x_elbow];
                y_link1 = [robot_base_y, y_elbow];
                x_link2 = [x_elbow, x_ee];
                y_link2 = [y_elbow, y_ee];    

                set(hl1, 'XData', x_link1, 'YData', y_link1);
                set(hl2, 'XData', x_link1(2), 'YData', y_link1(2));
                set(hl3, 'XData', x_link2, 'YData', y_link2);
                set(hl4, 'XData', x_link2(2), 'YData', y_link2(2));

                c_space(alpha_index,beta_index) = 1.5;
                set(hl5, 'ZData', c_space);

                drawnow;    
            end
            i = i+length(w_path);
        else
            path(i+1,:) = path(i,:) + delta*F_grad;
            a = path(i,1);
            b = path(i,2);
            [x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(a, b, robot_base_x, robot_base_y, L1, L2);
            x_link1 = [robot_base_x, x_elbow];
            y_link1 = [robot_base_y, y_elbow];
            x_link2 = [x_elbow, x_ee];
            y_link2 = [y_elbow, y_ee];    
            i = i + 1;
        
            set(hl1, 'XData', x_link1, 'YData', y_link1);
            set(hl2, 'XData', x_link1(2), 'YData', y_link1(2));
            set(hl3, 'XData', x_link2, 'YData', y_link2);
            set(hl4, 'XData', x_link2(2), 'YData', y_link2(2));

            c_space(alpha_index,beta_index) = 1.5;
            set(hl5, 'ZData', c_space);

            drawnow;
        end
    else
        path(i+1,:) = path(i,:) + delta*F_grad;
        a = path(i,1);
        b = path(i,2);
        [x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(a, b, robot_base_x, robot_base_y, L1, L2);
        x_link1 = [robot_base_x, x_elbow];
        y_link1 = [robot_base_y, y_elbow];
        x_link2 = [x_elbow, x_ee];
        y_link2 = [y_elbow, y_ee];    
        i = i + 1;
        
        set(hl1, 'XData', x_link1, 'YData', y_link1);
        set(hl2, 'XData', x_link1(2), 'YData', y_link1(2));
        set(hl3, 'XData', x_link2, 'YData', y_link2);
        set(hl4, 'XData', x_link2(2), 'YData', y_link2(2));

        c_space(alpha_index,beta_index) = 1.5;
        set(hl5, 'ZData', c_space);

        drawnow;
    end
end


end

