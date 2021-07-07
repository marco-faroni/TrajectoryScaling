function dJdQ = DJDQ(q,qp,J)
  dJdQ = zeros(6);
  for i=1:size(dJdQ,1)
    row = 0*qp;
    row(i)=1;
    for j = 1:size(dJdQ,2)
      col = 0*qp;
      col(j)=1;
      fcn = @(q) row'*J(q)*col; %J_ANAL(q,M,Ta,i,j);
      fcn_vec = @(q) vectorizedFcn(fcn,q);
      dJdQ(i,j) = mygradient(fcn_vec,q,'forward',1e-6)*qp;
    end
  end