function [alpha, beta] = inverseKinematics(x_ee, y_ee, robot_base_x, robot_base_y, L1, L2)

cos_beta = ((x_ee-robot_base_x)^2+(y_ee-robot_base_y)^2-L1^2-L2^2)/(2*L1*L2);
sin_beta = sqrt(1-cos_beta^2);

beta = atan2(sin_beta, cos_beta);
if(beta < 0)
    beta = beta + 2*pi;
elseif(beta > 2*pi)
    beta = beta - 2*pi;
end

alpha = atan2((y_ee-robot_base_y), (x_ee-robot_base_x))-atan2((L2*sin(beta)),L1+L2*cos(beta)); 

end

