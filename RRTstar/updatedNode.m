function update_node = updatedNode(node1, node2,delta)

v = node2 - node1;
u = v/norm(v);

update_node = node1 + delta*u;

end

