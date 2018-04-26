function [x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(alpha, beta, robot_base_x, robot_base_y, L1, L2)

x_elbow = robot_base_x + L1*cos(alpha); %x1
y_elbow = robot_base_y + L1*sin(alpha); %y1

x_ee = x_elbow + L2*cos(alpha+beta); %x2
y_ee = y_elbow + L2*sin(alpha+beta); %y2

end

