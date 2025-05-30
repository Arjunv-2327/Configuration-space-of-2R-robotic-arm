% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination


% Robot link parameters
L_1 = 3; % Length of link 1
L_2 = 3; % Length of link 2
dw = 12*3; % Deviation from the safe limit for theta_1


% Obstacle creation
obstacle = create_obstacle(l, w, alpha, beta, phi);


% Collision limits for link 1
[theta1, theta2] = detectAngularCollision(L_1, obstacle);


% Safe region, both links do not collide - entire robot is safe
[blue_angle, magenta_angle] = safespace(L_1, L_2, obstacle);


% Get the range of unsafe range of theta_2 after deviating link 1
theta2collisionrange(magenta_angle, dw, obstacle, L_2); 
