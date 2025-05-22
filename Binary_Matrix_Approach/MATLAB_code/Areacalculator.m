clear;
close all;

%cs_start1 = 120;
%cs_start2 = 90;

% just displaying the manipulator
%rr = RobotLinksN ([cs_start1 cs_start2]);


% plot start location
%p = patch (rr);
%p.FaceColor = 'green';
%p.EdgeColor = 'none';


%hold on;

% for representing start joints, just for figure
%for i = 1 : 1
   % xx = (rr.vertices((i-1)*4+2,1)+rr.vertices((i-1)*4+3,1))/2 ;
   % yy = (rr.vertices((i-1)*4+2,2)+rr.vertices((i-1)*4+3,2))/2 ;
   % circles(xx,yy,0.2,'facecolor','black','edgecolor','black');
%end

% These arrays define the vertices and faces of the obstacle as a patch
d = 2;  % Step size in degrees
dim = 360 / d;

% Initialize an empty list to store the sums


% Loop over obstacle vertical positions (k)
for k = 0:0.25:5
    %  Define the moving obstacle 
    obstacle.vertices = [
        -1,  1 + k;   % vertex 1
         1,  1 + k;   % vertex 2
         1, -1 + k;   % vertex 3
        -1, -1 + k    % vertex 4
    ];
    obstacle.faces = [1 2 3; 1 3 4];

    % Initialize C-space collision matrix 
    M = false(dim + 1, dim + 1);  % 181x181 matrix for 0 to 360 deg in steps of 2

    %  Loop over joint angles 
    for i = 0:d:360
        i_idx = i / d + 1;

        for j = 0:d:360
            j_idx = j / d + 1;

            rc = RobotLinksN([i j]);  % Robot configuration
            [collision_output, ~, ~, ~] = CollisionCheck1(rc, obstacle);

            if collision_output
                M(i_idx, j_idx) = true;
            end
        end
    end
    sumList = [];
    
    % Append the sum of the current M matrix to the list
    sumList = [sumList, sum(M(:))];
end

disp(sumList)







            





