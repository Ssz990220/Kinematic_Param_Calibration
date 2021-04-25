function O = get_O_multi_balls(qs_masked, p_measure_masked, n_balls, n_measures_ball, type)
%GET_O Summary of this function goes here
%   Detailed explanation goes here
global robot_poe
M = size(qs_masked, 1);
n_points = size(qs_masked,1);
x_measure = zeros([3,n_points]);
if type == 2
    J = zeros(3,6*robot_poe.n_dof + 6,n_points);
elseif type == 1
    J = zeros(3,6*robot_poe.n_dof,n_points);
elseif type == 3
    J = zeros(3,7*robot_poe.n_dof,n_points);
end
for i = 1:n_points
    T = robot_poe.fkine(qs_masked(i,:));
    x_coor4 = T*[p_measure_masked(:,i);1];
    x_measure(:,i) = x_coor4(1:3);
    J_full = robot_poe.get_J(qs_masked(i,:), type);
    J(:,:,i) = [-skew(x_measure(:,i)),eye(3)]*J_full;
end

Delta_x = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,1);
if type == 1
    G = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,6*robot_poe.n_dof);
elseif type == 2
    G = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,6*robot_poe.n_dof + 6);
elseif type == 3
    G = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,7*robot_poe.n_dof);
end
m_n = n_measures_ball*(n_measures_ball-1)/2;
for m = 1:n_balls
        base_idx = 0;
        for i = 1:n_measures_ball
            for j = i+1 : n_measures_ball
                Delta_x((m-1)*m_n+ base_idx + j - i) = - norm(x_measure(:,(m-1)*n_measures_ball + i) - x_measure(:,(m-1)*n_measures_ball +j))^2;
    %             Delta_x((m-1)*m_n +base_idx + j - i) = norm(x_true(:,i) - x_true(:,j))- norm(x_measure(:,i) - x_measure(:,j));
                G(((m-1)*m_n+base_idx + j - i),:) = 2 * (x_measure(:,(m-1)*n_measures_ball +i) - x_measure(:,(m-1)*n_measures_ball +j))'...
                    *(J(:,:,(m-1)*n_measures_ball +i)-J(:,:,(m-1)*n_measures_ball +j));
    %             G(((m-1)*m_n +base_idx + j - i),:) = (x_measure(:,i)-x_measure(:,j))'*(J(:,:,i)-J(:,:,j))/norm(x_measure(:,i)-x_measure(:,j));
            end
            base_idx = base_idx + n_measures_ball - i;
        end
end
% info_matrix = G.'*G;
% e = eig(info_matrix);
s = svd(G);
O = nthroot(prod(s),length(s))/M;
end

