function motion(axes_hl1, axes_hl2, init_pos_x, init_pos_y, goal_pos_x, goal_pos_y, L1, L2, robot_base_x, robot_base_y, obs_pos_x, obs_pos_y, obs_radius, alpha, beta, c_space, F_tot_alpha, F_tot_beta)

    cla(axes_hl1, 'reset')
    cla(axes_hl2, 'reset')

%% Plot initial workspace in cartesian domain and configuration domain
    xborder=[0,100];
    yborder=[0,100];
    [alpha_init, beta_init] = inverseKinematics(init_pos_x, init_pos_y, robot_base_x, robot_base_y, L1, L2);
    [alpha_goal, beta_goal] = inverseKinematics(goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2);
    pos_init = [init_pos_x, init_pos_y];
    pos_goal = [goal_pos_x, goal_pos_y];
    pos_obs=[obs_pos_x,obs_pos_y];
    radius = obs_radius;
    [x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(alpha_init, beta_init, robot_base_x, robot_base_y, L1, L2);
    x_link1_init = [robot_base_x, x_elbow];
    y_link1_init = [robot_base_y, y_elbow];
    x_link2_init = [x_elbow, x_ee];
    y_link2_init = [y_elbow, y_ee];

    axes(axes_hl1)
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

    axes(axes_hl2)
    alpha_init_index = find(abs(alpha-alpha_init) == min(abs(alpha-alpha_init)));
    beta_init_index = find(abs(beta - beta_init) == min(abs(beta - beta_init)));
    alpha_goal_index = find(abs(alpha - alpha_goal) == min(abs(alpha - alpha_goal)));
    beta_goal_index = find(abs(beta - beta_goal) == min(abs(beta - beta_goal)));
    c_space(alpha_init_index, beta_init_index) = 1.5;
    [alpha_grid, beta_grid] = meshgrid(alpha*180/pi, beta*180/pi);
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