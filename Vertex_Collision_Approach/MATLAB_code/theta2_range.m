% Base circle parameters
r3 = 3;
angle_start = 24.6648;     % Starting angle in degrees
num_points = 10;
angle_step = 5;            % Step size in degrees

% Generate angles
angles_deg = angle_start + (0:num_points-1) * angle_step;
angles_rad = deg2rad(angles_deg);

% Circle centered at origin
theta = linspace(0, 2*pi, 300);
circle_x = r3 * cos(theta);
circle_y = r3 * sin(theta);

% Store point coordinates
points = zeros(num_points, 2);

% Load obstacle vertices
load('obstacle_vertices.mat');  % loads 'vertices' (4x2 matrix)
V4 = vertices(4, :);            % Extract vertex V4

% Create figure
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
title('Focus on P2 Circle Intersections');

% Plot main circle at origin
plot(circle_x, circle_y, 'k-', 'LineWidth', 2);

% Plot obstacle rectangle
patch('Vertices', vertices, ...
      'Faces', [1 2 4 3], ...
      'FaceColor', [1 0.6 0], ...
      'EdgeColor', 'k', ...
      'LineWidth', 1.5);

% Compute all 10 points (just store them)
for i = 1:num_points
    px = r3 * cos(angles_rad(i));
    py = r3 * sin(angles_rad(i));
    points(i, :) = [px, py];
end

% Extract P2 only (here p2 is the 4th circle)
P2 = points(4, :);

% Plot P2 and its vector
plot(P2(1), P2(2), 'bo', 'MarkerFaceColor', 'b');
plot([0 P2(1)], [0 P2(2)], 'b-', 'LineWidth', 1.2);
text(P2(1) + 0.1, P2(2), 'P2');

% Plot dotted circle around P2
circle_dotted_x = P2(1) + r3 * cos(theta);
circle_dotted_y = P2(2) + r3 * sin(theta);
plot(circle_dotted_x, circle_dotted_y, 'b--', 'LineWidth', 1);

% ---- INTERSECTION LOGIC ----

% Rectangle edges as segments
edges = [
    vertices(1,:); vertices(2,:);
    vertices(2,:); vertices(4,:);
    vertices(4,:); vertices(3,:);
    vertices(3,:); vertices(1,:)
];

intersections = [];
for i = 1:4
    A = edges(2*i-1,:);
    B = edges(2*i,:);
    
    % Parametrize edge
    t_vals = linspace(0, 1, 1000);
    seg_pts = A + (B - A) .* t_vals';
    
    % Check distance from center P2 to each point
    dists = vecnorm(seg_pts - P2, 2, 2);
    mask = abs(dists - r3) < 1e-3;
    
    match_pts = seg_pts(mask, :);
    
    if ~isempty(match_pts)
        % Only take the first and last matching point per edge
        intersections = [intersections; match_pts(1,:)]; 
        if size(match_pts,1) > 1
            intersections = [intersections; match_pts(end,:)]; 
        end
    end
end

% Filter unique intersections
intersections = unique(round(intersections, 6), 'rows');

% Plot and return found points
if size(intersections,1) >= 2
    plot(intersections(:,1), intersections(:,2), 'ko', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    fprintf('Intersection points for P2 circle:\n');
    disp(intersections(1:2,:));
else
    warning('Less than 2 intersection points found for P2 circle.');
end

% --------- Compute acute angles ∠OP2R1 and ∠OP2R2 ---------
if size(intersections,1) >= 2
    O = [0 0];
    P = P2;
    R1 = intersections(1,:);
    R2 = intersections(2,:);
    
    % Vectors
    OP = P - O;
    PR1 = R1 - P;
    PR2 = R2 - P;
    
    % Compute angles
    angle_OPR1 = acosd(dot(OP, PR1) / (norm(OP) * norm(PR1)));
    angle_OPR2 = acosd(dot(OP, PR2) / (norm(OP) * norm(PR2)));
    
    % Convert to acute if needed
    if angle_OPR1 > 90
        angle_OPR1 = 180 - angle_OPR1;
    end
    if angle_OPR2 > 90
        angle_OPR2 = 180 - angle_OPR2;
    end

    % Display results
    fprintf('Acute angle ∠OP2R1 = %.4f degrees\n', angle_OPR1);
    fprintf('Acute angle ∠OP2R2 = %.4f degrees\n', angle_OPR2);
end
