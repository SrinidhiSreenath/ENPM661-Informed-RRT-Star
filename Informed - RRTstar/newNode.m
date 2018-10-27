function newn = newNode(snode, gnode, goal, cmin, cbest ,x, y, inclination)

if goal 
    if cbest >  sqrt(cbest^2 - cmin^2)
        a = cbest/2;
        b = sqrt(cbest^2 - cmin^2)/2;
    else
        b = cbest/2;
        a = sqrt(cbest^2 - cmin^2)/2;
    end
    
    theta = -pi/2 + pi*rand;
    max = round((a*b)/(sqrt((b*cos(theta))^2 + (a*sin(theta))^2)));

    r = randi(max);
  
    if mod(r,2) == 0
    xpoint = snode(1) + r*cos(theta + inclination);
    ypoint = snode(2) + r*sin(theta + inclination);
    else
    xpoint = gnode(1) - r*cos(theta + inclination);
    ypoint = gnode(2) - r*sin(theta + inclination);
    end
        
    newn = [round(xpoint) round(ypoint)];
else
    newn = [randi(x) randi(y)];    
end
    

end