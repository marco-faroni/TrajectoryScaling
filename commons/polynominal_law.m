function [Y] = polynominal_law(t,Ttot)
    cf_a=6;
    cf_b=15;
    cf_c=10;
    Y=[ones(1,length(t));
       zeros(2,length(t))];
    for i=1:length(t)
        t_now=t(i);
        if t_now<=Ttot    
            Y(1,i) = cf_a/Ttot^5*t_now.^5  -   cf_b/Ttot^4*t_now.^4 +   cf_c/Ttot^3*t_now.^3;
            Y(2,i) = 5*cf_a/Ttot^5*t_now.^4  - 4*cf_b/Ttot^4*t_now.^3 + 3*cf_c/Ttot^3*t_now.^2; 
            Y(3,i) = 20*cf_a/Ttot^5*t_now.^3  -12*cf_b/Ttot^4*t_now.^2 + 6*cf_c/Ttot^3*t_now;
        end
    end
end
        