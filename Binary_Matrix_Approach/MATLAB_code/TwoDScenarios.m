clear;
close all;

% figure;

cs_start1 = 0;
cs_start2 = 30;
cs_start3 = 40;
cs_start4 = 20;


% placement for start
rr = RobotLinksN ([cs_start1 cs_start2 cs_start3 cs_start4]);

% plot start location
p = patch (rr);
p.FaceColor = 'green';
p.EdgeColor = 'none';


hold on;

% for representing start joints, just for figure
for i = 1 : 3
    xx = (rr.vertices((i-1)*4+2,1)+rr.vertices((i-1)*4+3,1))/2 ;
    yy = (rr.vertices((i-1)*4+2,2)+rr.vertices((i-1)*4+3,2))/2 ;
    circles(xx,yy,0.2,'facecolor','black','edgecolor','black');
end

cs_goal1 = 180;
cs_goal2 = 50;
cs_goal3 = 20;
cs_goal4 = 40;

% placement for goal
rr1 = RobotLinksN ([cs_goal1 cs_goal2 cs_goal3 cs_goal4]);

%plot goal location
p = patch (rr1);
p.FaceColor = 'red';
p.EdgeColor = 'none';

% for representing goal joints, just for figure
for i = 1 : 3
    xx = (rr1.vertices((i-1)*4+2,1)+rr1.vertices((i-1)*4+3,1))/2 ;
    yy = (rr1.vertices((i-1)*4+2,2)+rr1.vertices((i-1)*4+3,2))/2 ;
    circles(xx,yy,0.2,'facecolor','black','edgecolor','black');
end


% These arrays define the vertices and faces of the obstacles as a patch
obstacle.vertices =   [5 1.9; 7 2; 7.5 4; 6 4;...
    -5 0.5; -2 2; -5 4; -6 1;...
    -1 -1; 1 -1; 2 -3; -1 -2];


tmp = [];
for i = 1 :  length(obstacle.vertices)/4
    tmp = [tmp; [4*(i-1)+1 4*(i-1)+2  4*(i-1)+3;4*(i-1)+1 4*(i-1)+3  4*(i-1)+4]];
end
obstacle.faces = tmp;
% counter = 1;
% for i = 1 : length(obstacle.faces)
%     text(obstacle.vertices(counter,1), obstacle.vertices(counter,2), num2str(i), 'VerticalAlignment','bottom','HorizontalAlignment','right','FontSize',10,'FontWeight','bold' ,'Color', 'r');
%     counter = counter + 3;
% end

obs = patch(obstacle);

obs.FaceColor = [80 22 22]/255;
obs.EdgeColor = [80 22 22]/255;

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

%random configuration
rr = RobotLinksN([340 40 50 60]);
% get the pixels corresponding to the robot placement
%collision checker
[collision, ~, ~, ~] = CollisionCheck1(rr, obstacle);

if collision
    p = patch (rr);
    p.FaceColor = 'blue';
    p.EdgeColor = 'none';
end

% C-space <- true()
% for C-space   for i = 1 : d: 360; for j = 1: d: 360 -> [i, j] ->
% robotlinksN -> collisioncheck1 -> if collison 



