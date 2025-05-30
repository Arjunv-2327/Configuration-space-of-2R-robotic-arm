% --- Set 15 ---
l = 1.17; w = 0.843; alpha = 2.191; beta = 0.914; phi = 2.347202;
L_1 = 2.56; L_2 = 3.03;




dw = 55;

% Obstacle creation
obstacle = create_obstacle(l, w, alpha, beta, phi);


% Collision limits for link 1
%[theta1, theta2] = detectAngularCollision(L_1, obstacle);


% Safe region, both links do not collide - entire robot is safe
[blue_angle, magenta_angle] = safespace(L_1, L_2, obstacle);


% Get the range of unsafe range of theta_2 after deviating link 1
theta2collisionrange(magenta_angle, dw, obstacle, L_1, L_2); 
