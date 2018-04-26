function collision = checkCollision(alpha, beta, robot_base_x, robot_base_y, L1, L2, x_obs, y_obs, r)

[x_ee, y_ee, x_elbow, y_elbow] = forwardKinematics(alpha, beta, robot_base_x, robot_base_y, L1, L2);

x1 = x_elbow;
y1 = y_elbow;
x2 = x_ee;
y2 = y_ee;
x3 = x_obs;
y3 = y_obs;

a = (x2 - x1)^2 + (y2 - y1)^2;
b = 2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
c = x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
u = ((x3-x1)*(x2-x1)+(y3-y1)*(y2-y1))/((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

check = b*b - 4*a*c;

xInBorders = (0 <= x_ee) && (x_ee <= 100) && (0 <= x_elbow) && (x_elbow <= 100);
yInBorders = (0 <= y_ee) && (y_ee <= 100) && (0 <= y_elbow) && (y_elbow <= 100);
aBorder = (alpha <= 0) || (alpha >= pi);
bBorder = (beta <= 0) || (beta >= 2*pi);

if (0 <= u && u <= 1 && check >= 0) || (~xInBorders || ~yInBorders || aBorder || bBorder )    
    collision = 1;
else
    collision = 0;
end
end

