function workspace(axes_hl, init_pos_x, init_pos_y, goal_pos_x, goal_pos_y, L1, L2, robot_base_x, robot_base_y, obs_pos_x, obs_pos_y, obs_radius, plotFlag)

    cla(axes_hl, 'reset')
    %% area
    xborder=[0,100];
    yborder=[0,100];
    
    %% draw the robot
    pos_init=[init_pos_x, init_pos_y];
    pos_goal=[goal_pos_x, goal_pos_y];
    x_ee = pos_init(1);
    y_ee = pos_init(2);
    [alpha_init, beta_init] = inverseKinematics(x_ee, y_ee, robot_base_x, robot_base_y, L1, L2);
    [x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(alpha_init, beta_init, robot_base_x, robot_base_y, L1, L2);
    x_link1_init = [robot_base_x, x_elbow];
    y_link1_init = [robot_base_y, y_elbow];
    x_link2_init = [x_elbow, x_ee];
    y_link2_init = [y_elbow, y_ee];

    if(plotFlag == 1)
        axes(axes_hl);
        line([xborder(1),xborder(2),xborder(2),xborder(1)],[yborder(1),yborder(1),yborder(2),yborder(2)]);
        hold on;

        line(x_link1_init, y_link1_init, 'Color', 'k');
        plot(x_link1_init(2), y_link1_init(2), 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        line(x_link2_init, y_link2_init, 'Color', 'k');
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
    end
    
end