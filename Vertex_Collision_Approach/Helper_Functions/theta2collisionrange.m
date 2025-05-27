function theta2collisionrange(initial_angle, dw, obstacle)
    % Compute P2 angle (where we moved theta1 limit by dw)
    angle_deg = initial_angle + dw;
    angle_rad = deg2rad(angle_deg);

    r3 = 3;  % Base circle radius
    theta = linspace(0, 2*pi, 300);
    circle_x = r3 * cos(theta);
    circle_y = r3 * sin(theta);

    vertices = obstacle.vertices;
    P2 = [r3 * cos(angle_rad), r3 * sin(angle_rad)];

    % ------------------- Plot Setup -------------------
    figure;
    hold on;
    axis equal off;

    plot(circle_x, circle_y, 'k-', 'LineWidth', 2);
    patch('Vertices', vertices, ...
          'Faces', obstacle.faces, ...
          'FaceColor', [1 0.6 0], ...
          'EdgeColor', 'k', ...
          'LineWidth', 1.5);
    plot(P2(1), P2(2), 'bo', 'MarkerFaceColor', 'b');
    text(P2(1) + 0.1, P2(2), 'P2');
    plot(P2(1) + r3*cos(theta), P2(2) + r3*sin(theta), 'b--', 'LineWidth', 1);

    % ------------------ Intersection Search ------------------
    edges = [
        vertices(1,:); vertices(2,:);
        vertices(2,:); vertices(4,:);
        vertices(4,:); vertices(3,:);
        vertices(3,:); vertices(1,:)
    ];

    intersections = [];
    edge_indices = [];

    for i = 1:4
        A = edges(2*i-1,:);
        B = edges(2*i,:);
        t_vals = linspace(0, 1, 1000);
        seg_pts = A + (B - A) .* t_vals';

        dists = vecnorm(seg_pts - P2, 2, 2);
        mask = abs(dists - r3) < 1e-3;
        match_pts = seg_pts(mask, :);

        if size(match_pts,1) >= 3
            mid_idx = round((1 + size(match_pts,1)) / 2);
            intersections = [intersections; match_pts(mid_idx,:)];
            edge_indices = [edge_indices; i];
        elseif size(match_pts,1) >= 1
            intersections = [intersections; match_pts(1,:)];
            edge_indices = [edge_indices; i];
        end
    end

    intersections = unique(round(intersections, 6), 'rows');
    R_actual = [];

    if size(intersections,1) >= 2
        rect_x = vertices([1 2 4 3 1], 1);
        rect_y = vertices([1 2 4 3 1], 2);

        for i = 1:2
            P_int = intersections(i,:);
            t = linspace(0, 1, 15)';
            sample_pts = P2 + t .* (P_int - P2);
            sample_pts = sample_pts(2:end-1, :);
            inside = inpolygon(sample_pts(:,1), sample_pts(:,2), rect_x, rect_y);

            if any(inside)
                edge_id = edge_indices(i);
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

                % Draw grazing magenta ray and black dot at vertex
                plot([P2(1) endpoint(1)], [P2(2) endpoint(2)], 'm-', 'LineWidth', 1.5);
                plot(nearest_vertex(1), nearest_vertex(2), 'ko', 'MarkerSize', 7.5, 'MarkerFaceColor', 'k');
                R_actual = [R_actual; endpoint];
            else
                % Draw direct magenta line and black dot at intersection
                plot([P2(1) P_int(1)], [P2(2) P_int(2)], 'm-', 'LineWidth', 1.5);
                plot(P_int(1), P_int(2), 'ko', 'MarkerSize', 7.5, 'MarkerFaceColor', 'k');
                R_actual = [R_actual; P_int];
            end
        end
    else
        warning('Less than 2 intersection points found for P2 circle.');
        return;
    end

    % ------------------ Compute Acute Angles ------------------
    O = [0 0];
    OP = P2 - O;
    R1 = R_actual(1,:);
    R2 = R_actual(2,:);

    PR1 = R1 - P2;
    PR2 = R2 - P2;

    angle_OPR1 = acosd(dot(OP, PR1) / (norm(OP) * norm(PR1)));
    angle_OPR2 = acosd(dot(OP, PR2) / (norm(OP) * norm(PR2)));

    if angle_OPR1 > 90
        angle_OPR1 = 180 - angle_OPR1;
    end
    if angle_OPR2 > 90
        angle_OPR2 = 180 - angle_OPR2;
    end

    theta_min = min(angle_OPR1, angle_OPR2);
    theta_max = max(angle_OPR1, angle_OPR2);

    % ------------------ Final Output ------------------
    fprintf('Unfeasible range of theta_2 measured wrt link_1 is [%.4f°, %.4f°]\n', theta_min, theta_max);
end
