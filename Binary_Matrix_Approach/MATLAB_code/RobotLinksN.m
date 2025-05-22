function fv = RobotLinksN(cspace_coords)
% Parameters that define robot dimensions
w = 0.5; % width of each link
l = 3;   % length of each link

% Initialize starting position
cx = 0;
cy = 0;

% Define the first link's vertices and faces
link.vertices = [cx-l/2 cy-w/2; cx+l/2 cy-w/2; cx+l/2 cy+w/2; cx-l/2 cy+w/2];
link.faces = [1 2 3; 1 3 4];

% Initialize transformation
link = transformFV(link, cspace_coords(1), [cx-l/2 cy]);   % canging axis of rotation to base midpoint

% Initialize the combined fv structure for the entire manipulator
fv.vertices = link.vertices;
fv.faces = link.faces;

% Loop through each degree of freedom and add links
faces_length = length(link.vertices);
prev_link = link;


for n = 2:length(cspace_coords)
    % Create a new link based on the previous link's endpoint
    new_link = prev_link;

    % Update each vertex of the new link
    for i = 1:length(prev_link.vertices)
        new_link.vertices(i,1) = prev_link.vertices(i,1) + l * cosd(sum(cspace_coords(1:n-1)));
        new_link.vertices(i,2) = prev_link.vertices(i,2) + l * sind(sum(cspace_coords(1:n-1)));
    end

    % Apply transformation to new link
    new_link = transformFV(new_link, cspace_coords(n), [(new_link.vertices(1,1)+new_link.vertices(4,1))/2, (new_link.vertices(1,2)+new_link.vertices(4,2))/2]);

    % Append the new link's vertices and faces to the full fv structure
    fv.vertices = [fv.vertices; new_link.vertices];
    fv.faces = [fv.faces; new_link.faces + faces_length];

    % Update total faces and previous link for next iteration
    faces_length = faces_length + length(link.vertices);
    prev_link = new_link;
end

