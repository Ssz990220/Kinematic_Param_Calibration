function [norm1,norm2] = plot_error(Ball_Pos,err2,real,figure_title,is_graph)
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

    if is_graph
        title(figure_title)
        subplot(4,1,1)
        title(figure_title)
        err2_avg = zeros(3,sample_size);
        for i = 1:batch_size
            err2_avg = err2_avg + err2{i};
        end
        err2_avg = err2_avg/batch_size;

    %     figure()
    %     titles = {'x','y','z'};
    %     for i = 1:3
    %         subplot(4,1,i)
    %         for number = 1:batch_size
    %             plot(abs(err2{number}(i,:)))
    %             xlabel('numbers');ylabel([titles{i},' error/mm'])
    %     %         labels{number} = (['sample', num2str(number)]);
    %             hold on
    %             plot(abs(err2_avg(i,:)),'LineWidth',1.5)
    %             axis([0 9 0 1])
    %         end
    %     %     legend(labels)
    %     end

        for i = 1:batch_size
            plot(sqrt(sum(err2{i}.^2,1)))
            hold on
        end
        plot(sqrt(sum(err2_avg.^2,1)),'LineWidth',1.5)
        axis([0 9 0 0.8])
        xlabel('numbers');ylabel('norm error/mm')
        titles = {'x','y','z'};
        for i = 1:3
            subplot(4,1,i+1)
            title(figure_title)
            for number = 1:batch_size
                plot(Ball_Pos{number}(i,:)-mean(Ball_Pos{number}(i,:))-Pos(i,:)+mean(Pos(i,:)))
                xlabel('numbers');ylabel([titles{i},' error/mm'])
                axis([0 9 -0.3 0.3])
        %         labels{number} = (['sample', num2str(number)]);
                hold on
            end
        %     legend(labels)
        end
    title(figure_title)
    end

    [err,err2,distance_matrix] = check_error(Pos,real);
    norm1 = sum(abs(err2),2)/sample_size;
    norm2 = norm(sum(abs(err2),2)/8);
end