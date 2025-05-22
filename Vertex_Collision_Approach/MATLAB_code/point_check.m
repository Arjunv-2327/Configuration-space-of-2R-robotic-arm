% Rectangle parameters
l = 1.2;               % Length
w = 0.6;               % Width
alpha = -0.9;          % Center X
beta = 0.9;            % Center Y
phi = 0.820305;        % Inclination (radians)

% Compute midpoints along the length
M_x = alpha - l*cos(phi)/2;
N_x = alpha + l*cos(phi)/2;
M_y = beta - l*sin(phi)/2;
N_y = beta + l*sin(phi)/2;

% Compute rectangle vertices
vertices = [
    M_x - w*sin(phi)/2, M_y + w*cos(phi)/2;
    N_x - w*sin(phi)/2, N_y + w*cos(phi)/2;
    M_x + w*sin(phi)/2, M_y - w*cos(phi)/2;
    N_x + w*sin(phi)/2, N_y - w*cos(phi)/2
];

faces = [1 2 4; 1 4 3];

% Plot the rectangle
figure;
patch('Faces', faces, 'Vertices', vertices, ...
      'FaceColor', [1 0.65 0], 'EdgeColor', [1 0.65 0]);
hold on;

% Root values
x1 = 0.658960;
y1 = 2.994923;
x2 = -0.455195;
y2 = 1.305609;

% All 4 combinations
x_vals = [x1, x2, x1, x2];
y_vals = [y1, y2, y2, y1];
labels = {
    'P1: (x1, y1)';
    'P2: (x2, y2)';
    'P3: (x1, y2)';
    'P4: (x2, y1)'
};

% Plot and annotate all points
scatter(x_vals, y_vals, 80, 'y', 'filled');
for i = 1:length(x_vals)
    text(x_vals(i) + 0.2, y_vals(i), ...
        sprintf('%s\n(%.3f, %.3f)', labels{i}, x_vals(i), y_vals(i)), ...
        'FontSize', 9, 'Color', 'black');
end

% Axis settings
axis equal;
xlim([-10, 10]);
ylim([-10, 10]);
title('All (x, y) Combinations from Roots with Rectangle');
