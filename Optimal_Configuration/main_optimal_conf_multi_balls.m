function qs_mask = main_optimal_conf_multi_balls(qs, p_measure, m ,l, p, q, n_balls, n_measure_each_ball, type)
%MAIN_OPTIMAL_CONF Summary of this function goes here
%   Detailed explanation goes here
n = size(qs, 1);
mask = randsample(n, l);
k = l;
j = 0;
if length(n_measure_each_ball) == 1
    n_measure_each_ball = repmat(n_measure_each_ball,n_balls,1);
end
Best_O = get_O_multi_balls(qs(mask,:),p_measure(:,mask),n_balls, length(mask), type);
while j<= q
    while k < m
        skip = false;
        while 1
            if ~skip
                mask_old = mask;
                m_plus_index = find_m_plus(qs, mask, p_measure, n_balls, length(mask), type);
                mask = [mask;m_plus_index];
                k = k + 1;
            end
            m_minus_index = find_m_minus(qs, mask, p_measure, n_balls, length(mask), type);
            mask_km1 = mask;
            mask_km1(m_minus_index) = [];
            O_km1 = get_O_multi_balls(qs(mask_km1,:), p_measure(:,mask_km1), n_balls, length(mask_km1), type);
            O_k_old = get_O_multi_balls(qs(mask_old,:), p_measure(:,mask_old), n_balls, length(mask_old), type);
            if O_km1 > O_k_old
                break
            else
                mask = mask_km1;
                k = k - 1;
                if k > l
                    skip = true;
                else
                    skip = false;
                end
            end
        end
    end
    O_k = get_O_multi_balls(qs(mask,:), p_measure(:,mask), n_balls, length(mask), type);
    if O_k > Best_O
        Best_O = O_k;
        qs_mask = mask;
    end
    rand_p = randsample(k,k-p);
    mask = mask(rand_p);
    k = k-p;
    j = j+1;
end
        
end

