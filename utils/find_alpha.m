function alpha = find_alpha(f,g,x,d,alp0)
    a = 0;
    b = inf;
    alpha = alp0;
    iter = 100;
    while iter
        lhs = f(x)+0.8*alpha*transpose(g(x))*d;
        exp = f(x+alpha*d);
        rhs = f(x)+0.2*alpha*transpose(g(x))*d;
        if(exp<=rhs)
            if(exp>=lhs)
                break;
            else
                a = alpha;
                if isinf(b)
                    alpha = 2*alpha;
                else
                    alpha = (a+b)/2;
                end
            end
        else
            b = alpha;
            alpha = (a+b)/2;
        end
    iter = iter-1;
    end
end