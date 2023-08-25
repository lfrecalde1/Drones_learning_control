function [learning, l1_memories, l2_memories, l3_memories, l4_memories, e1_memories, e2_memories, e3_memories, e4_memories] = learning_control(vd , v, A, B, l1_memories, l2_memories, l3_memories, l4_memories, e1_memories, e2_memories, e3_memories, e4_memories)
xe = 2*tanh(vd-v);
% Learning algorithm
l1_k = -B*l1_memories + A*e1_memories;
l2_k = -B*l2_memories + A*e2_memories;
l3_k = -B*l3_memories + A*e3_memories;
l4_k = -B*l4_memories + A*e4_memories;

% Vector of learning parameters
learning = [l1_k;...
    l2_k;...
    l3_k;...
    l4_k];

% Update values memories learning 1
[l1_memories, l2_memories, l3_memories, l4_memories] = update_l_memories(learning, l1_memories, l2_memories, l3_memories, l4_memories);
[e1_memories, e2_memories, e3_memories, e4_memories] = update_e_memories(xe,e1_memories, e2_memories, e3_memories, e4_memories);

end

function [l1_memories, l2_memories, l3_memories, l4_memories] = update_l_memories(l, l1_memories, l2_memories, l3_memories, l4_memories)
for k = length(l1_memories):-1:2
    
    l1_memories(k) = l1_memories(k-1);
    l2_memories(k) = l2_memories(k-1);
    l3_memories(k) = l3_memories(k-1);
    l4_memories(k) = l4_memories(k-1);
end
l1_memories(1) = l(1);
l2_memories(1) = l(2);
l3_memories(1) = l(3);
l4_memories(1) = l(4);
end

function [e1_memories, e2_memories, e3_memories, e4_memories] = update_e_memories(xe, e1_memories, e2_memories, e3_memories, e4_memories)
for k = length(e1_memories):-1:2
    
    e1_memories(k) = e1_memories(k-1);
    e2_memories(k) = e2_memories(k-1);
    e3_memories(k) = e3_memories(k-1);
    e4_memories(k) = e4_memories(k-1);
end
e1_memories(1) = xe(1);
e2_memories(1) = xe(2);
e3_memories(1) = xe(3);
e4_memories(1) = xe(4);
end