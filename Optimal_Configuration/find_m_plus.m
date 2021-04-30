function m_plus_index= find_m_plus(qs, mask, p_measure, n_balls, n_measure_each_ball, type, robot_poe)
%FIND_M_PLUS Summary of this function goes here
%   Detailed explanation goes here
n = size(qs, 1);
no_mask = setdiff(1:n, mask);
Os = [];
for i = 1:length(no_mask)
    mask_local = [mask;no_mask(i)];
    qs_mask_local = qs(mask_local,:);
    p_measure_local = p_measure(:,mask_local);
    O_local =  get_O_multi_balls(qs_mask_local, p_measure_local, n_balls, length(mask_local), type, robot_poe);
    Os = [Os, O_local];
end
[~, max_m_index] = max(Os);
m_plus_index = no_mask(max_m_index);
end

