function theta2collisionrange(initial_angle, dw, obstacle, r, r3)
    % Calculate P2 position based on initial angle and deviation dw
    angle_deg = initial_angle + dw;
    angle_rad = deg2rad(angle_deg);

    % Plot parameters
    theta = linspace(0, 2*pi, 300);
    circle_x = r * cos(theta);
    circle_y = r * sin(theta);

    vertices = obstacle.vertices;
    P2 = [r * cos(angle_rad), r * sin(angle_rad)];

    % ------------------- Plot Setup -------------------
    figure; hold on; axis equal off;
    plot(circle_x, circle_y, 'k-', 'LineWidth', 2);
    patch('Vertices', vertices, 'Faces', obstacle.faces, ...
          'FaceColor', [1 0.6 0], 'EdgeColor', 'k', 'LineWidth', 1.5);
    plot(P2(1), P2(2), 'bo', 'MarkerFaceColor', 'b');
    text(P2(1) + 0.1, P2(2), 'P2');
    plot(P2(1) + r3*cos(theta), P2(2) + r3*sin(theta), 'b--', 'LineWidth', 1);

    % ------------------ Edge Sampling ------------------
    % Define edges of rectangle obstacle (assuming 4 edges)
    edges = [
        vertices(1,:); vertices(2,:);
        vertices(2,:); vertices(4,:);
        vertices(4,:); vertices(3,:);
        vertices(3,:); vertices(1,:)
    ];

    sample_pts = [];
    edge_owner = [];

    % Sample points along each edge (1000 points per edge)
    for i = 1:4
        A = edges(2*i-1,:);
        B = edges(2*i,:);
        t_vals = linspace(0, 1, 1000)';
        edge_pts = A + (B - A) .* t_vals;
        sample_pts = [sample_pts; edge_pts];
        edge_owner = [edge_owner; repmat(i, length(t_vals), 1)];
    end

    % Distances from P2 to each sampled point on edges
    dists = vecnorm(sample_pts - P2, 2, 2);

    % Rectangle polygon coordinates for inpolygon checks
    rect_x = vertices([1 2 4 3 1], 1);
    rect_y = vertices([1 2 4 3 1], 2);

    % ------------------ Find points approximately at distance r3 ------------------
    near_mask = abs(dists - r3) < 1e-3;
    near_pts = sample_pts(near_mask, :);
    near_edges = edge_owner(near_mask);

    R_actual = [];  % Initialize valid ray endpoints storage

    % For each near point, check if the segment from P2 to near_pt crosses inside polygon
    for i = 1:size(near_pts, 1)
        P_int = near_pts(i, :);
        edge_id = near_edges(i);

        % Parametric points on segment P2->P_int (excluding endpoints)
        t = linspace(0, 1, 100)';
        seg = P2 + (P_int - P2) .* t;
        seg = seg(2:end-1, :);

        % Check if segment points lie inside polygon obstacle
        inside = inpolygon(seg(:,1), seg(:,2), rect_x, rect_y);

        if any(inside)
            % Segment crosses inside polygon → use nearest vertex on that edge instead
            v1 = edges(2*edge_id - 1, :);
            v2 = edges(2*edge_id, :);
            d1 = norm(v1 - P2);
            d2 = norm(v2 - P2);

            nearest_vertex = v1;
            if d2 < d1
                nearest_vertex = v2;
            end

            dir_vec = nearest_vertex - P2;
            unit_vec = dir_vec / norm(dir_vec);
            endpoint = P2 + r3 * unit_vec;

            plot([P2(1), endpoint(1)], [P2(2), endpoint(2)], 'm-', 'LineWidth', 1.5);
            plot(nearest_vertex(1), nearest_vertex(2), 'ko', 'MarkerSize', 7.5, 'MarkerFaceColor', 'k');

            % Append this endpoint if not already present
            if isempty(R_actual) || all(vecnorm(R_actual - endpoint, 2, 2) > 1e-6)
                R_actual = [R_actual; endpoint];
            end
        else
            % Clean ray — segment does not intersect polygon interior
            plot([P2(1), P_int(1)], [P2(2), P_int(2)], 'm-', 'LineWidth', 1.5);
            plot(P_int(1), P_int(2), 'ko', 'MarkerSize', 7.5, 'MarkerFaceColor', 'k');

            if isempty(R_actual) || all(vecnorm(R_actual - P_int, 2, 2) > 1e-6)
                R_actual = [R_actual; P_int];
            end
        end
    end

    % ------------------ Handle case when no points found at r3 distance ------------------
    if isempty(R_actual)
        if all(dists > r3)
            warning('No points within r3 — skipping ray generation.');
            return;
        elseif all(dists < r3)
            % All points inside r3 radius, so take rays to vertices spanning max angle
            vectors = vertices - P2;
            num_vertices = size(vectors, 1);

            max_angle = -inf;
            idx_pair = [0, 0];

            for i = 1:num_vertices
                for j = i+1:num_vertices
                    v1 = vectors(i, :);
                    v2 = vectors(j, :);
                    dot_val = dot(v1, v2) / (norm(v1) * norm(v2));
                    dot_val = max(min(dot_val, 1), -1);
                    angle = acosd(dot_val);
                    if angle > max_angle
                        max_angle = angle;
                        idx_pair = [i, j];
                    end
                end
            end

            v1 = vertices(idx_pair(1), :);
            v2 = vertices(idx_pair(2), :);

            dir1 = v1 - P2;
            dir2 = v2 - P2;

            end1 = P2 + r3 * dir1 / norm(dir1);
            end2 = P2 + r3 * dir2 / norm(dir2);

            plot([P2(1), end1(1)], [P2(2), end1(2)], 'm-', 'LineWidth', 1.5);
            plot([P2(1), end2(1)], [P2(2), end2(2)], 'm-', 'LineWidth', 1.5);
            plot(v1(1), v1(2), 'ko', 'MarkerSize', 7.5, 'MarkerFaceColor', 'k');
            plot(v2(1), v2(2), 'ko', 'MarkerSize', 7.5, 'MarkerFaceColor', 'k');

            R_actual = [end1; end2];
        end
    end

    % ------------------ Ensure we have at least two distinct rays ------------------
    if size(R_actual, 1) < 2
        warning('Fewer than 2 valid rays found — cannot compute angular range.');
        return;
    end

    % Check if rays are distinct enough (if not, try to find a second ray)
    if norm(R_actual(1,:) - R_actual(2,:)) < 1e-6
        warning('Two rays are identical; angular range not computable.');
        return;
    end

    % ------------------ Compute angular range from P2 ------------------
    O = [0 0];  % Origin
    OP = P2 - O;
    R1 = R_actual(1,:);
    R2 = R_actual(2,:);
    PR1 = R1 - P2;
    PR2 = R2 - P2;

    angle_OPR1 = acosd(dot(OP, PR1) / (norm(OP) * norm(PR1)));
    angle_OPR2 = acosd(dot(OP, PR2) / (norm(OP) * norm(PR2)));

    % Normalize angles to [0,90] for range
    if angle_OPR1 > 90
        angle_OPR1 = 180 - angle_OPR1;
    end
    if angle_OPR2 > 90
        angle_OPR2 = 180 - angle_OPR2;
    end

    theta_min = min(angle_OPR1, angle_OPR2);
    theta_max = max(angle_OPR1, angle_OPR2);

    % ------------------ Final Output ------------------
    fprintf('Unfeasible theta2 range based on final magenta rays is [%.4f°, %.4f°]\n', theta_min, theta_max);
end
