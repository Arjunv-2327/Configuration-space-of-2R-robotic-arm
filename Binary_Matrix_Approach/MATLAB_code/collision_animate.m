clear;
close all;

l = 1; % obstacle length
w = 1; % obstacle width
a = 0; % x coordinate of obstacle centre (alpha)
b = 0; % y coordinate of obstacle centre (beta)

d = 2;
dim = 360 / d;

all_M = cell(1, 21);
index = 1;

for k = 0:0.25:5
    obstacle.vertices = [
        a-l/2 b+w/2;  % vertex 1
        a+l/2 b+w/2;  % vertex 2
        a+l/2 b-w/2;  % vertex 3
        a-l/2 b-w/2   % vertex 4
    ];
    obstacle.vertices(:,1) = obstacle.vertices(:,1) + k;
    obstacle.faces = [1 2 3; 1 3 4];

    M = zeros(dim + 1, dim + 1);

    parfor i_idx = 1:(dim + 1)
        i = (i_idx - 1) * d;
        temp_row = zeros(1, dim + 1);

        for j_idx = 1:(dim + 1)
            j = (j_idx - 1) * d;

            rc = RobotLinksN([i j]);
            [collision_output, ~, ~, ~] = CollisionCheck1(rc, obstacle);

            if collision_output
                temp_row(j_idx) = 1;
            end
        end
        M(i_idx, :) = temp_row;
    end

    all_M{index} = M;
    index = index + 1;
end

% Animation code for saving and siplaying
v = VideoWriter('cspace_animation8.mp4', 'MPEG-4');
v.FrameRate = 10; % 10 fps means 30 frames = 3 seconds
open(v);

figure;
colormap('gray');

hold_duration = 30; % Number of frames to repeat (e.g., 30 frames @10 fps = 3 sec)

for idx = 1:length(all_M)  % Forward direction
    imagesc((0:2:360), (0:2:360), ~all_M{idx}'); % axis with actual angles
    axis equal tight;
    title(sprintf('C-space at k = %.2f', (idx - 1) * 0.25));
    xlabel('theta1 (joint 1 angle)');
    ylabel('theta2 (joint 2 angle)');
    colorbar;
    set(gca, 'YDir', 'normal');  % Ensure y-axis is bottom to top

    frame = getframe(gcf);

    for rep = 1:hold_duration
        writeVideo(v, frame);
    end
end

close(v);
disp('Slow animation saved as cspace_animation8.mp4');

