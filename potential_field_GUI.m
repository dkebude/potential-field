function potential_field_GUI

    handles.f = figure('Visible','off','Position',[50,50,1300,600]);
    handles.htextinitpos_x  = uicontrol('Style','text','String','X coord. of Initial Pos.',...
                 'Position',[1140,580,150,15]);
    handles.heditinitpos_x  = uicontrol('Style','edit','String','20',...
                 'Position',[1140,560,150,20]);
    handles.htextinitpos_y  = uicontrol('Style','text','String','Y coord. of Initial Pos.',...
                 'Position',[1140,540,150,15]);
    handles.heditinitpos_y  = uicontrol('Style','edit','String','70',...
                 'Position',[1140,520,150,20]);
    handles.htextgoalpos_x  = uicontrol('Style','text','String','X coord. of Goal Pos.',...
                 'Position',[1140,500,150,15]);
    handles.heditgoalpos_x  = uicontrol('Style','edit','String','60',...
                 'Position',[1140,480,150,20]);
    handles.htextgoalpos_y  = uicontrol('Style','text','String','Y coord. of Goal Pos.',...
                 'Position',[1140,460,150,15]);
    handles.heditgoalpos_y  = uicontrol('Style','edit','String','40',...
                 'Position',[1140,440,150,20]);
    handles.htextobspos_x  = uicontrol('Style','text','String','X coord. of Obstacle Pos.',...
                 'Position',[1140,420,150,15]);
    handles.heditobspos_x  = uicontrol('Style','edit','String','40',...
                 'Position',[1140,400,150,20]);
    handles.htextobspos_y  = uicontrol('Style','text','String','Y coord. of Obstacle Pos.',...
                 'Position',[1140,380,150,15]);
    handles.heditobspos_y  = uicontrol('Style','edit','String','60',...
                 'Position',[1140,360,150,20]);
    handles.htextobsradius = uicontrol('Style','text','String','Radius of Obstacle',...
                 'Position',[1140,340,150,15]);
    handles.heditobsradius = uicontrol('Style','edit','String','10',...
                 'Position',[1140,320,150,20]);
    handles.htextL1  = uicontrol('Style','text','String','Link Length 1',...
                 'Position',[1140,300,150,15]);
    handles.heditL1  = uicontrol('Style','edit','String','46.1',...
                 'Position',[1140,280,150,20]);
    handles.htextL2  = uicontrol('Style','text','String','Link Length 2',...
                 'Position',[1140,260,150,15]);
    handles.heditL2  = uicontrol('Style','edit','String','26.9',...
                 'Position',[1140,240,150,20]);
    handles.htextbasepos_x  = uicontrol('Style','text','String','X coord. of Robot Base',...
                 'Position',[1140,220,150,15]);
    handles.heditbasepos_x  = uicontrol('Style','edit','String','20',...
                 'Position',[1140,200,150,20]);
    handles.htextbasepos_y  = uicontrol('Style','text','String','Y coord. of Robot Base',...
                 'Position',[1140,180,150,15]);
    handles.heditbasepos_y  = uicontrol('Style','edit','String','0',...
                 'Position',[1140,160,150,20]);
    handles.ha1 = axes('Units','pixels','Position',[40,60,520,520]);
    handles.ha2 = axes('Units','pixels','Position',[600,60,520,520]);
    handles.hworkspace    = uicontrol('Style','pushbutton',...
                 'String','Generate Workspace','Position',[1140,120,150,25],...
                 'Callback',{@workspace_Callback, handles});
    handles.hconfigspace    = uicontrol('Style','pushbutton',...
                 'String','Go to Config. Space','Position',[1140,90,150,25],...
                 'Callback',{@configspace_Callback, handles});
    handles.hpotential    = uicontrol('Style','pushbutton',...
                 'String','Show Potential Field','Position',[1140,60,150,25],...
                 'Callback',{@potential_Callback, handles});
    handles.hmotion = uicontrol('Style','pushbutton',...
                 'String','Plan Motion','Position',[1140,30,150,25],...
                 'Callback', {@motion_Callback, handles});
    
    init_pos_x=str2double(get(handles.heditinitpos_x,'string'));
    init_pos_y=str2double(get(handles.heditinitpos_y,'string'));
    goal_pos_x=str2double(get(handles.heditgoalpos_x,'string'));
    goal_pos_y=str2double(get(handles.heditgoalpos_y,'string'));
    L1=str2double(get(handles.heditL1,'string'));
    L2=str2double(get(handles.heditL2,'string'));
    robot_base_x=str2double(get(handles.heditbasepos_x,'string'));
    robot_base_y=str2double(get(handles.heditbasepos_y,'string'));
    obs_pos_x=str2double(get(handles.heditobspos_x,'string'));
    obs_pos_y=str2double(get(handles.heditobspos_y,'string'));
    obs_radius=str2double(get(handles.heditobsradius,'string'));
    
    axes_hl = handles.ha1;
    plotFlag = 1;
    workspace(axes_hl, init_pos_x, init_pos_y, goal_pos_x, goal_pos_y, L1, L2, robot_base_x, robot_base_y, obs_pos_x, obs_pos_y, obs_radius, plotFlag)
    axes_hl = handles.ha2;
    plotFlag = 1;
    configspace(axes_hl, goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, obs_radius, plotFlag);
    
    handles.f.Visible = 'on';
end

function workspace_Callback(source, eventdata, handles)
    init_pos_x=str2double(get(handles.heditinitpos_x,'string'));
    init_pos_y=str2double(get(handles.heditinitpos_y,'string'));
    goal_pos_x=str2double(get(handles.heditgoalpos_x,'string'));
    goal_pos_y=str2double(get(handles.heditgoalpos_y,'string'));
    L1=str2double(get(handles.heditL1,'string'));
    L2=str2double(get(handles.heditL2,'string'));
    robot_base_x=str2double(get(handles.heditbasepos_x,'string'));
    robot_base_y=str2double(get(handles.heditbasepos_y,'string'));
    obs_pos_x=str2double(get(handles.heditobspos_x,'string'));
    obs_pos_y=str2double(get(handles.heditobspos_y,'string'));
    obs_radius=str2double(get(handles.heditobsradius,'string'));
    axes_hl = handles.ha1;
    plotFlag = 1;
    workspace(axes_hl, init_pos_x, init_pos_y, goal_pos_x, goal_pos_y, L1, L2, robot_base_x, robot_base_y, obs_pos_x, obs_pos_y, obs_radius, plotFlag)
end

function configspace_Callback(source, eventdata, handles)
    L1=str2double(get(handles.heditL1,'string'));
    L2=str2double(get(handles.heditL2,'string'));
    robot_base_x=str2double(get(handles.heditbasepos_x,'string'));
    robot_base_y=str2double(get(handles.heditbasepos_y,'string'));
    obs_pos_x=str2double(get(handles.heditobspos_x,'string'));
    obs_pos_y=str2double(get(handles.heditobspos_y,'string'));
    obs_radius=str2double(get(handles.heditobsradius,'string'));
    goal_pos_x=str2double(get(handles.heditgoalpos_x,'string'));
    goal_pos_y=str2double(get(handles.heditgoalpos_y,'string'));
    axes_hl = handles.ha2;
    plotFlag = 1;
    configspace(axes_hl, goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, obs_radius, plotFlag);
end

function potential_Callback(source, eventdata, handles)
    L1=str2double(get(handles.heditL1,'string'));
    L2=str2double(get(handles.heditL2,'string'));
    robot_base_x=str2double(get(handles.heditbasepos_x,'string'));
    robot_base_y=str2double(get(handles.heditbasepos_y,'string'));
    obs_pos_x=str2double(get(handles.heditobspos_x,'string'));
    obs_pos_y=str2double(get(handles.heditobspos_y,'string'));
    obs_radius=str2double(get(handles.heditobsradius,'string'));
    goal_pos_x=str2double(get(handles.heditgoalpos_x,'string'));
    goal_pos_y=str2double(get(handles.heditgoalpos_y,'string'));
    axes_hl = handles.ha2;
    plotFlag = 0;
    [alpha, beta, alpha_goal, beta_goal, c_space] = configspace(axes_hl, goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, obs_radius, plotFlag);
    plotFlag = 1;
    potential(axes_hl, alpha, beta, alpha_goal, beta_goal, c_space, plotFlag);
    set(gcf, 'PaperPositionMode', 'auto');
    print(gcf,'C:/Users/dkebude/Documents/Lectures/MECH_544_Robotics/Project_2/Figures/potential.png','-dpng','-r300');
end

function motion_Callback(source, eventdata, handles)
    init_pos_x=str2double(get(handles.heditinitpos_x,'string'));
    init_pos_y=str2double(get(handles.heditinitpos_y,'string'));
    goal_pos_x=str2double(get(handles.heditgoalpos_x,'string'));
    goal_pos_y=str2double(get(handles.heditgoalpos_y,'string'));
    L1=str2double(get(handles.heditL1,'string'));
    L2=str2double(get(handles.heditL2,'string'));
    robot_base_x=str2double(get(handles.heditbasepos_x,'string'));
    robot_base_y=str2double(get(handles.heditbasepos_y,'string'));
    obs_pos_x=str2double(get(handles.heditobspos_x,'string'));
    obs_pos_y=str2double(get(handles.heditobspos_y,'string'));
    obs_radius=str2double(get(handles.heditobsradius,'string'));
    axes_hl1 = handles.ha1;
    axes_hl2 = handles.ha2;
    plotFlag = 0;
    [alpha, beta, alpha_goal, beta_goal, c_space] = configspace(axes_hl2, goal_pos_x, goal_pos_y, robot_base_x, robot_base_y, L1, L2, obs_pos_x, obs_pos_y, obs_radius, plotFlag);
    [F_tot_alpha, F_tot_beta] = potential(axes_hl2, alpha, beta, alpha_goal, beta_goal, c_space, plotFlag);
    motion(axes_hl1, axes_hl2, init_pos_x, init_pos_y, goal_pos_x, goal_pos_y, L1, L2, robot_base_x, robot_base_y, obs_pos_x, obs_pos_y, obs_radius, alpha, beta, c_space, F_tot_alpha, F_tot_beta)
end