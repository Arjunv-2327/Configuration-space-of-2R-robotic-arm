function [theta1, theta2] = detectAngularCollision(r, obstacle)
    figure;
    hold on;
    axis equal off;
    axis([-r-1 r+1 -r-1 r+1]);  % Set proper scale

    % Draw base circle
    theta = linspace(0, 2*pi, 300);
    plot(r * cos(theta), r * sin(theta), 'k-', 'LineWidth', 2);

    % Draw obstacle as solid orange rectangle
    fill(obstacle.vertices([1 2 4 3 1],1), obstacle.vertices([1 2 4 3 1],2), ...
         [1 0.6 0], 'EdgeColor', 'k', 'LineWidth', 1.5);

    % Collision angle detection
    distances = sqrt(sum(obstacle.vertices.^2, 2));
    inside_idx = find(distances <= r);
    outside_idx = find(distances > r);

    normalize_angle = @(x, y) mod(atan2(y, x), 2*pi);

    if length(outside_idx) <= 1
        angles_rad = arrayfun(@(i) normalize_angle(obstacle.vertices(i,1), obstacle.vertices(i,2)), 1:4);
        max_diff = 0; idx1 = 0; idx2 = 0;
        for i = 1:3
            for j = i+1:4
                diff = abs(angles_rad(i) - angles_rad(j));
                diff = min(diff, 2*pi - diff);
                if diff > max_diff
                    max_diff = diff;
                    idx1 = i;
                    idx2 = j;
                end
            end
        end
        points = [obstacle.vertices(idx1,:); obstacle.vertices(idx2,:)];
    else
        lines = [1 2; 1 3; 2 4; 3 4];
        intersection_pts = [];
        for k = 1:size(lines,1)
            P1 = obstacle.vertices(lines(k,1),:);
            P2 = obstacle.vertices(lines(k,2),:);
            d = P2 - P1; f = P1;
            a = dot(d,d); b = 2 * dot(f,d); c = dot(f,f) - r^2;
            discriminant = b^2 - 4*a*c;

            if discriminant >= 0
                discriminant = sqrt(discriminant);
                t1 = (-b - discriminant) / (2*a);
                t2 = (-b + discriminant) / (2*a);
                if t1 >= 0 && t1 <= 1
                    intersection_pts(end+1,:) = P1 + t1*d;
                end
                if t2 >= 0 && t2 <= 1
                    intersection_pts(end+1,:) = P1 + t2*d;
                end
            end
        end

        all_pts = [obstacle.vertices(inside_idx,:); intersection_pts];
        angles_rad = arrayfun(@(i) normalize_angle(all_pts(i,1), all_pts(i,2)), 1:size(all_pts,1));
        max_diff = 0; idx1 = 0; idx2 = 0;
        for i = 1:length(angles_rad)-1
            for j = i+1:length(angles_rad)
                diff = abs(angles_rad(i) - angles_rad(j));
                diff = min(diff, 2*pi - diff);
                if diff > max_diff
                    max_diff = diff;
                    idx1 = i;
                    idx2 = j;
                end
            end
        end
        points = [all_pts(idx1,:); all_pts(idx2,:)];
    end

    % Output limiting angles
    theta1 = normalize_angle(points(1,1), points(1,2));
    theta2 = normalize_angle(points(2,1), points(2,2));

    % Plot limiting radii up to circle boundary
    plot([0 r*cos(theta1)], [0 r*sin(theta1)], 'r--', 'LineWidth', 1.5);
    plot([0 r*cos(theta2)], [0 r*sin(theta2)], 'r--', 'LineWidth', 1.5);


    % Print result
    %fprintf('Limiting angles (in radians):\n');
    fprintf('  θ₁ = %.4f deg\n', theta1*180/pi);
    fprintf('  θ₂ = %.4f deg\n', theta2*180/pi);
end
