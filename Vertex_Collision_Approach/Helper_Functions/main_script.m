% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0;        % Inclination
dw = -70/3;               % Deviation in degrees from theta1 limit

obstacle = create_obstacle(l, w, alpha, beta, phi);
[theta1, theta2] = detectAngularCollision(3, obstacle);
[blue_angle, magenta_angle] = safespace(r, obstacle);
theta2collisionrange(magenta_angle, dw, obstacle); % enter theta1 limit point argument, and deviation in degrees
