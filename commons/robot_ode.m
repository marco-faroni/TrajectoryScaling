function ode_f = robot_odefcn_noacc(tau,q,dq,dyn_fcn)
    [H,C,G,Fs,Fv] = dyn_fcn(q,dq);
    ode_f = [ dq; H\(tau-(C+G+Fs+Fv)) ];
end