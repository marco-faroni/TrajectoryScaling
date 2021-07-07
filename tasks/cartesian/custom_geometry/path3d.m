function [xyz]=path3d(s,lengths,fcn)
    xyz=zeros(3,length(s));
    cum_lengths=cumsum(lengths);
    for i=1:length(s)
        s_i=s(i);
        i_seg=find(s_i<=cum_lengths,1);
        if isempty(i_seg)
            s_i=cum_lengths(end);
            i_seg=length(cum_lengths);
        end
        if i_seg==1
            covered_seg=s_i;
        else
            covered_seg=s_i-sum(lengths(1:(i_seg-1)));
        end
        try
            xyz(:,i)=fcn{i_seg}(covered_seg);
        catch
            s_i;
        end
    end
end

