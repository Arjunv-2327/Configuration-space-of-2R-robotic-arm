function centre_finder(l1, l2, l3, alpha, beta, m, c, xmin, xmax)
    % Compute possible centers (h,k) of the unknown circle
    centers = find_circle_center_nosymbolic(l1, l2, l3, alpha, beta);
    
    if isempty(centers)
        disp('No valid centers found. Circles do not intersect.');
        return;
    end

    fprintf('Possible centers (h,k):\n');
    disp(centers);

    figure; hold on; axis equal; grid on;
    title('Possible Circle Centers (h, k)');
    xlabel('x'); ylabel('y');

    % Plot the constraint circle (center at origin, radius = l1)
    theta = linspace(0, 2*pi, 500);
    plot(l1 * cos(theta), l1 * sin(theta), 'b', 'LineWidth', 1.2);
    text(0, 0, '  Origin (0,0)', 'Color', 'b');

    % Thick red dot at origin
    scatter(0, 0, 100, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');

    % Plot the line y = mx + c (bounded between xmin and xmax)
    x_line = linspace(xmin, xmax, 500);
    plot(x_line, m * x_line + c, 'r', 'LineWidth', 1.2);
    text(xmin, m*xmin + c, '  Line y = mx + c', 'Color', 'r');

    % Plot the known point on the line (alpha, beta)
    scatter(alpha, beta, 60, 'ko', 'filled');
    text(alpha, beta, '  Closest Point', 'Color', 'k');

    % Define segment colors
    color_l1 = 'k';              % black for origin to (h,k)
    color_l2 = [0.5, 0.25, 0];   % brown for (h,k) to (px, py)
    color_l3 = [1, 0.4, 0.7];    % pink for (px, py) to (alpha, beta)

    for i = 1:size(centers,1)
        h = centers(i,1);
        k = centers(i,2);

        % Compute angular position from origin to (h, k)
        theta_angle = atan2(k, h);
        if theta_angle < 0
            theta_angle = theta_angle + 2*pi; % Convert to [0, 2*pi)
        end
        fprintf('Angular position of l1 segment %d: %.4f radians (%.2f°)\n', i, theta_angle, rad2deg(theta_angle));

        % Unit vector from center (h,k) to closest point (alpha,beta)
        dir = [alpha - h, beta - k];
        unit_dir = dir / norm(dir);

        % Calculate point on unknown circle (distance l3 away from closest point along line)
        px = alpha - l3 * unit_dir(1);
        py = beta - l3 * unit_dir(2);

        % Thick red dot at center (h,k)
        scatter(h, k, 100, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
        text(h, k, sprintf('  (h=%.2f, k=%.2f)', h, k), 'Color', 'g');

        % Plot segments and labels
        plot([0, h], [0, k], '-', 'LineWidth', 6, 'Color', color_l1);
        text((0+h)/2, (0+k)/2, 'l1', 'FontSize', 10, 'Color', color_l1, 'FontWeight', 'bold');

        plot([h, px], [k, py], '-', 'LineWidth', 4, 'Color', color_l2);
        text((h+px)/2, (k+py)/2, 'l2', 'FontSize', 10, 'Color', color_l2, 'FontWeight', 'bold');

        plot([px, alpha], [py, beta], '-', 'LineWidth', 2, 'Color', color_l3);
        text((px+alpha)/2, (py+beta)/2, 'l3', 'FontSize', 10, 'Color', color_l3, 'FontWeight', 'bold');

        % Thick red dot at point on circle
        scatter(px, py, 100, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');

        % Dashed unknown circle
        x_circle = h + l2 * cos(theta);
        y_circle = k + l2 * sin(theta);
        plot(x_circle, y_circle, '--', 'Color', color_l2, 'LineWidth', 1.2);
    end
end

function centers = find_circle_center_nosymbolic(l1, l2, l3, xp, yp)
    % Radii
    r1 = l1;
    r2 = l2 + l3;

    % Centers of circles
    x1 = 0; y1 = 0;     % origin circle
    x2 = xp; y2 = yp;   % circle centered at known point on line

    % Distance between centers
    d = sqrt((x2 - x1)^2 + (y2 - y1)^2);

    % Check intersection validity
    if d > r1 + r2 || d < abs(r1 - r2)
        centers = []; % no intersection
        return;
    end

    % Distance from first center to point P2 along line joining centers
    a = (r1^2 - r2^2 + d^2) / (2*d);

    % Coordinates of point P2
    x3 = x1 + a * (x2 - x1) / d;
    y3 = y1 + a * (y2 - y1) / d;

    % Height from P2 to intersection points
    h = sqrt(r1^2 - a^2);

    % Offsets perpendicular to line between centers
    rx = -(y2 - y1) * (h / d);
    ry =  (x2 - x1) * (h / d);

    % Two intersection points - possible centers (h,k)
    centers = [x3 + rx, y3 + ry; 
               x3 - rx, y3 - ry];
end
