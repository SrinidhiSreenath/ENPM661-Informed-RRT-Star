function RRT = rrtpathplan(start_node, goal_node, x, y,A1,A2,A3,B1,B2,B3,C1,C2,C3)

RRT.nodes(1,:) = start_node;
RRT.parent(1,:) = start_node;

maxIterations = 10000;
iter = 1;
delta = 7;

while iter < maxIterations 
    
    new_node = [randi(x) randi(y)];
    
    s = insideObs(new_node(1),new_node(2),A1,A2,A3,B1,B2,B3,C1,C2,C3);
   
    if s == 1
        iter = iter+1;
        continue
    end
    
    dmin = 10000000;
    for i = 1:1:size(RRT.nodes,1)
        d = dist(new_node,RRT.nodes(i,:));
        if d < dmin
            dmin = d;
            var = i;
        end
    end
        
    if dmin<=delta
        update_node = new_node;
    else 
        update_node = updatedNode(RRT.nodes(var,:),new_node,delta);
        dmin = dist(update_node,RRT.nodes(var,:));
    end
    
     valid = checkEdge(RRT.nodes(var,:),update_node,dmin,A1,A2,A3,B1,B2,B3,C1,C2,C3);
        if valid == 1
            iter = iter+1;
            continue
        end
        
    update_node = round(update_node); 
        
    RRT.nodes(end+1,:) = update_node;
    si = size(RRT.nodes,1);
    RRT.parent(si,:) = RRT.nodes(var,:);
           
    goal = reachedGoal(update_node,goal_node);
    
    if goal
        fprintf('Found');
        break
    end
    
    iter = iter+1; 
    clear s valid
    

end

end