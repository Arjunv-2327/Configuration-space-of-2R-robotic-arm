function [angle_degs, P_circle, P_centres] = three_links(l1, l2, l3, obstacle)
    V = obstacle.vertices;
    nV = size(V,1);
    candidates = [];

    for i = 1:4
        j = mod(i, nV) + 1;
        v1 = V(i, :); v2 = V(j, :);
        verts = [v1; v2];

        for vi = 1:2
            alpha = verts(vi,1);
            beta  = verts(vi,2);

            centers = centre_on_link1_circle(l1, l2 + l3, alpha, beta);
            if isempty(centers), continue; end

            for jj = 1:size(centers,1)
                h = centers(jj,1); k = centers(jj,2);
                dir_vec = [alpha - h, beta - k];
                if norm(dir_vec) == 0, continue; end
                unit_dir = dir_vec / norm(dir_vec);
                px = alpha - l3 * unit_dir(1);
                py = beta - l3 * unit_dir(2);

                if abs(norm([px - h, py - k]) - l2) > 1e-5, continue; end

                % Circle intersection check
                theta = linspace(0, 2*pi, 10000);
                x_circ = px + l3 * cos(theta);
                y_circ = py + l3 * sin(theta);
                if sum(inpolygon(x_circ, y_circ, V(:,1), V(:,2))) > 10
                    continue;
                end

                candidates = [candidates; h, k, px, py];
            end
        end
    end

    % Try plotting only feasible configs (after link sampling test)
    P_centres = [];
    P_circle = [];
    angle_degs = [];
    count = 0;

    figure; hold on; axis equal; axis off;
    patch('Vertices', V, 'Faces', obstacle.faces, 'FaceColor', [1, 0.4, 0], ...
          'EdgeColor', 'k', 'LineWidth', 2.5, 'FaceAlpha', 0.85);

    theta_full = linspace(0, 2*pi, 1000);
    plot(l1*cos(theta_full), l1*sin(theta_full), 'black:', 'LineWidth', 2.0);
    scatter(0, 0, 100, 'r', 'filled'); % base joint

    for i = 1:size(candidates,1)
        if count >= 2, break; end

        h = candidates(i,1); k = candidates(i,2);
        px = candidates(i,3); py = candidates(i,4);
        dir = [px - h, py - k]; unit = dir / norm(dir);
        x3 = px + l3 * unit(1);
        y3 = py + l3 * unit(2);

        % Link collision checks
        t1 = linspace(0, 1, 10000);
        p1x = (1 - t1) * 0 + t1 * h;
        p1y = (1 - t1) * 0 + t1 * k;
        p2x = (1 - t1) * h + t1 * px;
        p2y = (1 - t1) * k + t1 * py;
        p3x = (1 - t1) * px + t1 * x3;
        p3y = (1 - t1) * py + t1 * y3;

        if sum(inpolygon(p1x, p1y, V(:,1), V(:,2))) > 10 || ...
           sum(inpolygon(p2x, p2y, V(:,1), V(:,2))) > 10 || ...
           sum(inpolygon(p3x, p3y, V(:,1), V(:,2))) > 10
            continue;
        end

        % Passed final link-bulk test
        count = count + 1;
        P_centres = [P_centres; h, k];
        P_circle = [P_circle; px, py];
        ang = atan2(k, h) * 180/pi;
        angle_degs = [angle_degs; mod(ang, 360)];

        % Plot circles as dotted
        plot(px + l3*cos(theta_full), py + l3*sin(theta_full), 'b:', 'LineWidth', 1.5);
        plot([0 h], [0 k], 'k', 'LineWidth', 3);         % l1 - black
        plot([h px], [k py], 'Color', [0.5 0.25 0], 'LineWidth', 3); % l2 - brown
        plot([px x3], [py y3], 'm', 'LineWidth', 3);     % l3 - pink

        % Joints and end-effector
        scatter([h, px], [k, py], 80, 'r', 'filled');
        scatter(x3, y3, 80, 'k', 'filled');
    end

    hold off;
end
