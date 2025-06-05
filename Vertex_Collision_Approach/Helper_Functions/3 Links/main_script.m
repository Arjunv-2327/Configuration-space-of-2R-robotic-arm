% Rectangle parameters
l = 3.2;               % Length
w = 8.6;               % Width
alpha = 1;          % Center X
beta = -4.9;            % Center Y
phi = pi/2.77;        % Inclination
dw = 5;              % Deviation in degrees

% Robot Link lengths
L_1 = 8;  % length of link 1
L_2 = 6;  % length of link 2
L_3 = 3;  % length of link 3

% Obstacle creation
obstacle = create_obstacle(l, w, alpha, beta, phi);

% Safe region, none of the 3 links collides - entire robot is safe
[angle_degs, Sol] = new_3_links(L_1, L_2, L_3, obstacle);

% Collision limits for link 1
%[theta1, theta2] = detectAngularCollision(L_1, obstacle);

% Safe region, both links do not collide - entire robot is safe
%[blue_angle, magenta_angle] = safespace(L_1, L_2, obstacle);

% Get the range of unsafe range of theta_2 after deviating link 1
%theta2collisionrange(magenta_angle, dw, obstacle, L_1, L_2);   


