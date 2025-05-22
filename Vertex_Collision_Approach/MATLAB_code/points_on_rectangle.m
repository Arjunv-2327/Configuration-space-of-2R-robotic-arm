load('obstacle_vertices.mat');  % This will load the variable `vertices` back into your workspace
r = 3;
FPX = 2.7263;
FPY = 1.2519;
dw = 0.1; % deviation in radians of link 1 from limit position (anticlockwise)
arg = atan2(FPY, FPX); % FPX AND FPY are x and y coords of limit point on circumference of circle centres at origin and r3 for link1
p_x = r*cos(arg + dw); %(x-coordinate on circle after w deviation)
p_y = r*sin(arg + dw); %(y-coordinate on circle after w deviation)

% Obstacle Vertices labelled
V1_X = vertices(1,1);
V2_X = vertices(2,1);
V3_X = vertices(3,1);
V4_X = vertices(4,1);
V1_Y = vertices(1,2);
V2_Y = vertices(2,2);
V3_Y = vertices(3,2);
V4_Y = vertices(4,2);

% Slopes and intercepts for edges of rectangle:
m1 = (V4_Y-V3_Y)/(V4_X-V3_X);
m2 = -1/m1;
c1 = V3_Y - m1*V3_X;
c2 = V2_Y - m2*V2_X;


% Define the coefficients for the first quadratic in x1
a1 = 1 + m1^2;
b1 = -2*p_x + 2*m1*(c1 - p_y);
d1 = p_x^2 + c1^2 + p_y^2 - r^2;

% Solve the first equation for x1
coeffs1 = [a1, b1, d1];
roots1 = roots(coeffs1);

% Define the coefficients for the second quadratic in x2
a2 = 1 + m2^2;
b2 = -2*p_x + 2*m2*(c2 - p_y);
d2 = p_x^2 + c2^2 + p_y^2 - r^2;

% Solve the second equation for x2
coeffs2 = [a2, b2, d2];
roots2 = roots(coeffs2);

% Display both roots of x1 and x2
coords = zeros(4, 2);  % Preallocate 4x2 array

for i = 1:length(roots1)
    x1 = roots1(i);
    x2 = roots2(i);

    y1 = m1 * x1 + c1;
    y2 = m2 * x2 + c2;

    coords(2*i - 1, :) = [x1, y1];  % Store first point
    coords(2*i, :)     = [x2, y2];  % Store second point

    %fprintf('Root %d:\n', i);
    %fprintf('x1 = %.6f, y1 = %.6f\n', x1, y1);
    %fprintf('x2 = %.6f, y2 = %.6f\n\n', x2, y2);
end
%save('coordinates.mat', 'coords');
%disp(coords)

% Load coords if not already in workspace
%load('coordinates.mat');  % This loads 'coords'

% Initialize an empty array for real coordinates
coord_real = [];

% Loop through each row of coords
for i = 1:size(coords, 1)
    x = coords(i, 1);
    y = coords(i, 2);
    
    if isreal(x) && isreal(y)
        coord_real = [coord_real; x, y];  % Append if both parts are real
    end
end

% Save to new .mat file
%save('coordinates_real.mat', 'coord_real');
%load('coordinates_real.mat');
disp(coord_real);  % Show real coordinates


% Circle of radius 3 centered at origin
r = 3;
theta = linspace(0, 2*pi, 300);
circle_x = r * cos(theta);
circle_y = r * sin(theta);

% Plot the circle
figure;
plot(circle_x, circle_y, 'k-', 'LineWidth', 2);
hold on;
axis equal;
xlabel('X'); ylabel('Y');
title('Circle and Inclined Rectangle');
grid on;

% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination (radians)

% Compute midpoints of length edges
M_x = alpha - l*cos(phi)/2;
N_x = alpha + l*cos(phi)/2;
M_y = beta - l*sin(phi)/2;
N_y = beta + l*sin(phi)/2;

% Compute vertices of the rectangle
vertices = [
    M_x - w*sin(phi)/2, M_y + w*cos(phi)/2;  % Vertex 1
    N_x - w*sin(phi)/2, N_y + w*cos(phi)/2;  % Vertex 2
    M_x + w*sin(phi)/2, M_y - w*cos(phi)/2;  % Vertex 3
    N_x + w*sin(phi)/2, N_y - w*cos(phi)/2   % Vertex 4
];

% Define faces to draw filled rectangle using 2 triangles
faces = [1 2 4; 1 4 3];

% Draw rectangle using patch
patch('Faces', faces, ...
      'Vertices', vertices, ...
      'FaceColor', [1 0.6 0], ...       % Orange fill
      'EdgeColor', 'k', ...
      'LineWidth', 1.5);

% Load and plot the real coordinates
%load('coordinates_real.mat');  % This loads 'coord_real'

% Plot the real coordinate points in yellow
plot(coord_real(:,1), coord_real(:,2), 'yo', ...
    'MarkerFaceColor', 'yellow', 'MarkerSize', 8);

%legend('Circle of Radius 3', 'Inclined Rectangle', 'Rectangle Vertices', 'Real Root Points');








