function [norm1,norm2] = plot_error_1(Ball_Pos,err2,real,figure_title,is_graph,index)
    Pos = zeros(size(Ball_Pos{1}));
    batch_size = size(Ball_Pos,2);
    sample_size = size(Ball_Pos{1},2);
    test = zeros(1,batch_size);
    for k = 1:size(Pos,2)
        error_count = 0;
        for j = 1:3
            for i = 1:batch_size
                test(1,i) = Ball_Pos{i}(j,k);
            end
            [A,TF] = rmoutliers(test);
            if any(TF)
%                 disp(['batch ',num2str(i),' outliers at dim ',num2str(j),' on number ',num2str(k)])
%                 disp(test(TF))
                error_count = error_count+length(test(TF));
            end
            Pos(j,k) = mean(A);
        end
%         disp(['error percentage for number ' num2str(k),': ', num2str(error_count/3/batch_size)])
    end
    [err,err2_avg,distance_matrix] = check_error(Pos,real);
    norm1 = sum(abs(err2_avg),2)/sample_size;
    norm2 = norm(sum(abs(err2_avg),2)/sample_size);

    if is_graph
        err2_avg = zeros(3,sample_size);
        for i = 1:batch_size
            err2_avg = err2_avg + err2{i};
        end
        err2_avg = err2_avg/batch_size;

        for i = 1:batch_size
            plot(1:sample_size,sqrt(sum(err2{i}.^2,1)),'--')
            hold on
        end
        plot(1:sample_size,sqrt(sum(err2_avg.^2,1)),'LineWidth',1.5)
        plot(1:sample_size,norm2*ones(1,sample_size),'--r','LineWidth',3)
        axis([0 9 0 1])
        xlabel('hole numbers');ylabel('norm error/mm')
        title(figure_title)
        legend('batch 1','batch 2','batch 3','batch 4','average','error level')
    end

    
end