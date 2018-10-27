function [costtocome,costtogo] = pathCost( goal, current, parent)

d = sqrt((parent(1) - current(1))^2 + (parent(2) - current(2))^2); 

if d>1
    costtocome = 14;
else
    costtocome = 10;
end

dx = abs(goal(1) - current(1));
dy = abs(goal(2) - current(2));

costtogo = 10*(dx+dy) - 6*min(dx,dy);

end