% Giving an n-by-m matrix S, it returns an m-vector which elements are the
% norms 2 of every column of S

function nrm = mycolnorm(S)
    nrm = sqrt(sum(S.^2,1));
    