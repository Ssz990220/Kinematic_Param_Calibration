function Ts = gen_cubic_position(Ts_origin, dis_hole, n_holes_each_line, edge_length)
%GEN_CUBIC_POSITION Summary of this function goes here
%   Detailed explanation goes here
holes_coor = linspace(1,n_holes_each_line,n_holes_each_line) * dis_hole;
Ts_local = repmat(eye(4),1,1, n_holes_each_line^2 * 3);
Ts = zeros(size(Ts_local));
n_holes_each_face = n_holes_each_line ^ 2;
n_holes = n_holes_each_face * 3;

%% Face x-y
z = edge_length;
for i = 1:n_holes_each_line
    x = holes_coor(i);
    for j = 1:n_holes_each_line
        y = holes_coor(j);
        Ts_local(1:3,1:3,(i-1)*n_holes_each_line+j) = rotz(45);
        Ts_local(1:3,4,(i-1)*n_holes_each_line+j) = [x y z]';
    end
end

%% Face x-z
y = 0;
for i = 1:n_holes_each_line
    x = holes_coor(i);
    for j = 1:n_holes_each_line
        z = holes_coor(j);
        Ts_local(1:3,1:3,(i-1)*n_holes_each_line+j + n_holes_each_face) = rotz(90);
        Ts_local(1:3,4,(i-1)*n_holes_each_line+j + n_holes_each_face) = [x y z]';
    end
end

%% Face y-z
x = 0;
for i = 1:n_holes_each_line
    y = holes_coor(i);
    for j = 1:n_holes_each_line
        z = holes_coor(j);
        Ts_local(1:3,4,(i-1)*n_holes_each_line +j + n_holes_each_face * 2) = [x y z]';
    end
end

%% Ts output
for i = 1:n_holes
    Ts(:,:,i) = Ts_origin * Ts_local(:,:,i);
end

end

