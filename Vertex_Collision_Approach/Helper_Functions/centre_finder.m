function centre_finder(l1, l2, l3, m, c, xmin, xmax)
    % Generate 5000 sample points on the line segment
    x_samples = linspace(xmin, xmax, 5000);
    y_samples = m * x_samples + c;
    
    best_centers = [];
    best_sample_idx = -1;

    % Iterate over all samples and check if circles intersect and satisfy l2 criteria
    for i = 1:length(x_samples)
        alpha = x_samples(i);
        beta = y_samples(i);
        
        centers = find_circle_center_nosymbolic(l1, l2 + l3, alpha, beta);

        if isempty(centers)
            continue;
        end

        % For each valid center, check if l2 constraint is satisfied
        for j = 1:size(centers,1)
            h = centers(j,1);
            k = centers(j,2);

            % Closest point from center (h,k) to line
            xp = (h + m*(k - c)) / (1 + m^2);
            yp = m * xp + c;

            % Clamp closest point to segment bounds
            if xp < xmin
                xp = xmin;
                yp = m * xp + c;
            elseif xp > xmax
                xp = xmax;
                yp = m * xp + c;
            end

            % Unit vector direction
            dir = [xp - h, yp - k];
            unit_dir = dir / norm(dir);

            % Compute point on the unknown circle (l3 units from closest point)
            px = xp - l3 * unit_dir(1);
            py = yp - l3 * unit_dir(2);

            % Check if distance from center (h,k) to point (px,py) is l2
            dist_l2 = norm([px - h, py - k]);

            if abs(dist_l2 - l2) < 1e-6  % good match
                best_centers = [best_centers; h, k, px, py, xp, yp];
                best_sample_idx = i;
            end
        end
    end

    if isempty(best_centers)
        disp('No valid centers found that satisfy all constraints.');
        return;
    end

    %fprintf('Found %d valid center(s).\n', size(best_centers,1));

    figure; hold on; axis equal; grid on;
    title('Valid Circle Centers (h, k)');
    xlabel('x'); ylabel('y');

    % Plot fixed origin circle
    theta = linspace(0, 2*pi, 500);
    plot(l1 * cos(theta), l1 * sin(theta), 'b', 'LineWidth', 1.2);
    scatter(0, 0, 100, 'r', 'filled');  % origin

    % Plot the line
    x_line = linspace(xmin, xmax, 500);
    y_line = m * x_line + c;
    plot(x_line, y_line, 'r', 'LineWidth', 1.2);

    % Colors
    color_l1 = 'k';
    color_l2 = [0.5, 0.25, 0];
    color_l3 = [1, 0.4, 0.7];

    for i = 1:size(best_centers,1)
        h = best_centers(i,1);
        k = best_centers(i,2);
        px = best_centers(i,3);
        py = best_centers(i,4);
        xp = best_centers(i,5);
        yp = best_centers(i,6);

        % Plot unknown circle
        x_circ = h + l2 * cos(theta);
        y_circ = k + l2 * sin(theta);
        plot(x_circ, y_circ, '--', 'Color', color_l2, 'LineWidth', 1.2);

        % Plot center and segments
        scatter(h, k, 100, 'r', 'filled');
        text(h, k, sprintf('  (h=%.2f, k=%.2f)', h, k), 'Color', 'g');

        % l1 segment
        plot([0, h], [0, k], '-', 'LineWidth', 2.5, 'Color', color_l1);
        text((0+h)/2, (0+k)/2, 'l1', 'FontSize', 10, 'Color', color_l1, 'FontWeight', 'bold');

        % l2 segment
        plot([h, px], [k, py], '-', 'LineWidth', 2.5, 'Color', color_l2);
        text((h+px)/2, (k+py)/2, 'l2', 'FontSize', 10, 'Color', color_l2, 'FontWeight', 'bold');

        % l3 segment
        plot([px, xp], [py, yp], '-', 'LineWidth', 2.5, 'Color', color_l3);
        text((px+xp)/2, (py+yp)/2, 'l3', 'FontSize', 10, 'Color', color_l3, 'FontWeight', 'bold');

        % Points
        scatter(px, py, 100, 'r', 'filled');
        scatter(xp, yp, 60, 'ko', 'filled');
    end
end

function centers = find_circle_center_nosymbolic(l1, r2, xp, yp)
    x1 = 0; y1 = 0;
    x2 = xp; y2 = yp;
    d = sqrt((x2 - x1)^2 + (y2 - y1)^2);

    if d > l1 + r2 || d < abs(l1 - r2)
        centers = [];
        return;
    end

    a = (l1^2 - r2^2 + d^2) / (2*d);
    x3 = x1 + a * (x2 - x1) / d;
    y3 = y1 + a * (y2 - y1) / d;

    h = sqrt(l1^2 - a^2);
    rx = -(y2 - y1) * (h / d);
    ry =  (x2 - x1) * (h / d);

    centers = [x3 + rx, y3 + ry; 
               x3 - rx, y3 - ry];
end
