function m_minus_index= find_m_minus(qs, mask, p_measure, n_balls, n_measure_each_ball, type, robot_poe)
%FIND_M_PLUS Summary of this function goes here
%   Detailed explanation goes here
Os = [];
for i = 1:length(mask)
    mask_local = mask;
    mask_local(i) = [];
    qs_mask_local = qs(mask_local,:);
    p_measure_local = p_measure(:,mask_local);
    O_local =  get_O_multi_balls(qs_mask_local, p_measure_local, n_balls, length(mask_local), type, robot_poe);
    Os = [Os, O_local];
end
[~, m_index] = max(Os);
m_minus_index = m_index;
end

