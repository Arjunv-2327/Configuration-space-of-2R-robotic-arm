clear;
close all;

cs_start1 = 25;
cs_start2 = 200;

% just displaying the manipulator
rr = RobotLinksN ([cs_start1 cs_start2]);


% plot start location
p = patch (rr);
p.FaceColor = 'green';
p.EdgeColor = 'none';


hold on;

% for representing start joints, just for figure
u = 2; %total number of links
for i = 1 : u-1
    xx = (rr.vertices((i-1)*4+2,1)+rr.vertices((i-1)*4+3,1))/2 ;
    yy = (rr.vertices((i-1)*4+2,2)+rr.vertices((i-1)*4+3,2))/2 ;
    circles(xx,yy,0.2,'facecolor','black','edgecolor','black');
end

% for now we construct a rectangular obstacle with the normal orientation,
% both sides are parallel to respective coordiate axes

% ## a ranges from -4.4 to 1.6 and b ranges from -3 to 3

l = 1; % obstacle length
w = 1; % obstacle width
a = -0.4; % x coordinate of obstacle centre (alpha)
b = 3; % y coordinate of obstacle centre (beta)

% These arrays define the vertices and faces of the obstacles as a patch
obstacle.vertices = [
    a-l/2 b+w/2;  % vertex 1
    a+l/2 b+w/2;  % vertex 2
    a+l/2 b-w/2;  % vertex 3
    a-l/2 b-w/2   % vertex 4
];

obstacle.faces = [1 2 3;1 3 4];  % Connect these vertices in order to form a rectangle
obs = patch(obstacle);

obs.FaceColor = [255 165 0]/255;   % Orange
obs.EdgeColor = [255 165 0]/255;   % Orange

% for representing fixed base, just for figure
circles(-1.5,0.0,0.4,'facecolor',[0.7 0.7 0.7],'edgecolor','black');
circles(-1.5,0.0,0.3,'facecolor',[0.7 0.7 0.7],'edgecolor','black');

hold off;

axis equal;
% axis (sz*[-1 1 -1 1]);

set(gca,'xtick',[])
set(gca,'xticklabel',[])
set(gca,'ytick',[])
set(gca,'yticklabel',[])
set(gca, 'xlim', [-10 10]);
set(gca, 'ylim', [-9 9]);
box on;
set(gca,'linewidth',1);


% C-space <- true()
% for C-space   for i = 1 : d: 360; for j = 1: d: 360 -> [i, j] ->
% robotlinksN -> collisioncheck1 -> if collison 

% Define matrix size
d = 2;
dim = 360 / d;
M = false(dim + 1, dim + 1);  % Preallocate binary matrix

% Looping the matrix indices instaed of angles (both are inter-related)
% Relation between angle and index is;; index = angle/d + 1
% i_idx means the row number for M
%j_idx means the column number for M
%i_idx, j_idx indicate positions of elements in M matrix (row, column)

parfor i_idx = 1:(dim + 1)
    i = (i_idx - 1) * d;

    % Create temporary row (entire row in matrix M)
    temp_row = false(1, dim + 1);

    for j_idx = 1:(dim + 1)
        j = (j_idx - 1) * d;

        rc = RobotLinksN([i j]);
        [collision_output, ~, ~, ~] = CollisionCheck1(rc, obstacle);

        if collision_output
            temp_row(j_idx) = true;
        end
    end

    % Safe write: each worker writes one full row
    M(i_idx, :) = temp_row;
end


%%Plotting functions%%
matrix = M; % Replace this with your actual matrix

% Create a figure for the plot
figure;

% Loop through the matrix and plot the points
hold on;
for row = 1:size(matrix, 1)
    for col = 1:size(matrix, 2)
        x = 2 * (row - 1);
        y = 2 * (col - 1);
        
        if matrix(row, col) == 1
            plot(x, y, 'ko'); % Black point for 1
        
        end
    end
end

% Set axis limits
axis([0 (180*d) 0 (180*d)]);

% Set grid and axis labels
grid on;
xlabel('theta1-1st link angle');
ylabel('theta2-2nd link angle wrt first');

% Set axis to equal scaling
axis equal;

hold off;






            





