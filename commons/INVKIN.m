function q = INVKIN(x,q0,M)
  persistent options
  if isempty(options)
    options = optimoptions('fsolve','TolFun',1e-12,'TolX',1e-12,'display','off');
  end
  
  EQN = @(q) FKIN(q,M)-x;
  q = fsolve(EQN,q0,options);
end