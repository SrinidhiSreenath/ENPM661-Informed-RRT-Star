function [goal] = reachedGoal(C,G)

threshold = sqrt((C(1) - G(1))^2 + (C(2) - G(2))^2);

if threshold < 2
    goal = true;
else 
    goal = false;
end


end