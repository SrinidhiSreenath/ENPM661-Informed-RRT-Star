tic

clear
clc

x = 250;
y = 150;
%% Defining the obstacles using half plane.
% An obstacle is defined by the half plane region which in turn is defined
% by combination of line equations. Each line equation is defined by the
% standard for "ax+by+c=0"

% Array "A" represents the constants "a" in the linear equation with respect to an obstacle.
% Array "B" represents the constants "b" in the linear equation with respect to an obstacle.
% Array "C" represents the constants "c" in the linear equation with respect to an obstacle.

% Obstacle 1 (Square)

A1 = [-1 1 0 0]; 
B1 = [0 0 -1 1];
C1 = [55 -105 67.5 -112.5];

% Obstacle 2

xy = [120 55; 145 14; 168 14; 158 51; 165 89; 188 51];

% Part 1
m1 = (xy(2,2) - xy(1,2))/(xy(2,1) - xy(1,1)); % slope of the line
c1 = xy(1,2) - m1*xy(1,1);                    % y-intercept of the line

m2 = (xy(3,2) - xy(2,2))/(xy(3,1) - xy(2,1));
c2 = xy(2,2) - m2*xy(2,1);

m3 = (xy(4,2) - xy(3,2))/(xy(4,1) - xy(3,1));
c3 = xy(3,2) - m3*xy(3,1);

m4 = (xy(1,2) - xy(4,2))/(xy(1,1) - xy(4,1));
c4 = xy(4,2) - m4*xy(4,1);

A2 = [m1 m2 -m3 -m4];
B2 = [-1 -1 1 1];
C2 = [c1 c2 -c3 -c4];  

%Part 2
m5 = (xy(6,2) - xy(5,2))/(xy(6,1) - xy(5,1));
c5 = xy(5,2) - m5*xy(5,1);

m6 = (xy(5,2) - xy(4,2))/(xy(5,1) - xy(4,1));
c6 = xy(4,2) - m6*xy(4,1);

m7 = (xy(4,2) - xy(3,2))/(xy(4,1) - xy(3,1));
c7 = xy(3,2) - m7*xy(3,1);

m8 = (xy(3,2) - xy(6,2))/(xy(3,1) - xy(6,1));
c8 = xy(6,2) - m8*xy(6,1);

A3 = [-m5 -m6 m7 m8];
B3 = [1 1 -1 -1];
C3 = [-c5 -c6 c7 c8];

%% A* Search
tic
xys = [5 5];         %Start Node
xyg = [225 140];     %Goal Node

s = insideObs(xys(1),xys(2),A1,A2,A3,B1,B2,B3,C1,C2,C3);
g = insideObs(xyg(1),xyg(2),A1,A2,A3,B1,B2,B3,C1,C2,C3);

if s==1 || g==1
    fprintf('Invalid start and/or goal nodes\n');       % Either start and/or goal node not in C_free
    return
end

RRT = rrtpathplan(xys, xyg, x, y,A1,A2,A3,B1,B2,B3,C1,C2,C3);
toc
%% Finding shortest path after Informed - RRT*
test = size(RRT.nodes,1);
path = RRT.nodes(test,:);

index = 0;

while index ~=1
[tf, index]=ismember(RRT.parent(test,:),RRT.nodes,'rows');
path(end+1,:) = RRT.nodes(index,:);
test = index;
end


%% Plotting the graph. 

xsquare = [55 105 105 55];
ysquare = [67.5 67.5 112.5 112.5];

xpol = [120 158 165 188 168 145];
ypol = [55 51 89 51 14 14];

v = VideoWriter('RRT1.avi');
open(v);

hold on;
rectangle('Position',[165 105 30 30],'FaceColor','m','Curvature',[1 1])
fill(xsquare,ysquare,'m');
fill(xpol,ypol,'m');
title('RRT Search')
axis([1 x 1 y])
set(gcf, 'Position', get(0, 'Screensize'));
text(xys(1),xys(2),'\leftarrow Start');
text(xyg(1),xyg(2),'\leftarrow Goal');

for i=1:1:size(RRT.nodes,1)
scatter(RRT.nodes(i,1),RRT.nodes(i,2),'filled','b');
line([RRT.nodes(i,1) RRT.parent(i,1)], [RRT.nodes(i,2) RRT.parent(i,2)], 'Linewidth',2,'Color','k')
pause(0.001);
frame = getframe;
writeVideo(v,frame);
end


for j=size(path,1):-1:1
    xy=path(j,:);
        
    scatter(xy(1),xy(2),'filled','r');
    hold on;
    grid on
    
    if j~=1
    pa = path(j-1,:);
    line([xy(1) pa(1)], [xy(2) pa(2)], 'Linewidth',3,'Color','r')
    end
    
    pause(0.01) 
 frame = getframe;
writeVideo(v,frame);
end

close(v);








