%Base circle parameters
r3 = 3;
angle_start = 24.66;     % Argument of limit point on main circle in degrees
dw = 12;                 % Angle deviation from angle_start (keep dw less than or maximum near 15 degrees)

% Compute the target angle
angle_deg = angle_start + dw;
angle_rad = deg2rad(angle_deg);

% Circle centered at origin
theta = linspace(0, 2*pi, 300);
circle_x = r3 * cos(theta);
circle_y = r3 * sin(theta);

% Load obstacle vertices
load('obstacle_vertices.mat');
% loads 'vertices' (4x2 matrix)
V4 = vertices(4, :);            % Extract vertex V4

% Compute P2 position
P2 = [r3 * cos(angle_rad), r3 * sin(angle_rad)];

% Create figure
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
%title('Focus on P2 Circle Intersections');

% Plot main circle at origin
plot(circle_x, circle_y, 'k-', 'LineWidth', 2);

% Plot obstacle rectangle
patch('Vertices', vertices, ...
      'Faces', [1 2 4 3], ...
      'FaceColor', [1 0.6 0], ...
      'EdgeColor', 'k', ...
      'LineWidth', 1.5);

% Plot P2
plot(P2(1), P2(2), 'bo', 'MarkerFaceColor', 'b');
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
    
    if size(match_pts,1) >= 3
        % Select the point in the middle of the first and last
        mid_idx = round((1 + size(match_pts,1)) / 2);
        intersections = [intersections; match_pts(mid_idx,:)];
    elseif size(match_pts,1) == 2
        intersections = [intersections; match_pts(1,:)];
    elseif size(match_pts,1) == 1
        intersections = [intersections; match_pts];
    end
end

% Filter unique intersections
intersections = unique(round(intersections, 6), 'rows');

% Plot and return found points
if size(intersections,1) >= 2
    plot(intersections(:,1), intersections(:,2), 'ko', ...
        'MarkerSize', 7.5, 'MarkerFaceColor', 'k');
    
    fprintf('Intersection points for P2 circle:\n');
    disp(intersections(1:2,:));

    % Draw magenta lines from each intersection to P2
    for i = 1:2
        plot([P2(1) intersections(i,1)], [P2(2) intersections(i,2)], 'm-', 'LineWidth', 1.5);
    end

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

    %Restrictde range of theta2
    fprintf('Restricted theta2 range = [%.4f, %.4f] degrees\n', min(angle_OPR2, angle_OPR1), max(angle_OPR2, angle_OPR1));
end
