function [q,dq,ev_int,ep]=integrate_ddq(model,robot,st,ev_int,ep_old,q_ref,dq_ref,q,dq,u_real)
    df=robot.df;    
    st_PI=1e-3;
    Kpos=100;
    Kp=5*251;
    Ki=5*15.6;
    ev_sum=ev_int;
    ep=ep_old;
    
    if strcmp(model,'dynamic')
      ep = q_ref - q; 
      for i_PI=1:fix(st/st_PI)
        % velocity control every 1 ms
        qp_des = dq_ref;
        ev = qp_des - dq + Kpos*ep;
        ev_sum = ev_sum + ev;
        [H,C,G,Fs,Fv] = robot.invdynamics(q,dq); 
        tau = H*(u_real(1:df) + Kp*ev + Ki*ev_sum) + C + G + Fv + Fs;
        
        if max(tau>robot.tau_max*1.01)||max(tau<robot.tau_min*1.01)
          warning('Saturazione di coppia'); 
        end
        tau_sat = tau;
        tau_sat(tau>=robot.tau_max) = robot.tau_max(tau>=robot.tau_max); % saturazione coppia
        tau_sat(tau<=robot.tau_min) = robot.tau_min(tau<=robot.tau_min);  % saturazione coppia
        
        % antiwindup
        if max(tau>=robot.tau_max)||max(tau<=robot.tau_min)
            ev_sum = ev_sum - ev;
        end

        f = @(t,y) robot_ode(tau_sat,y(1:6),y(7:12),robot.invdynamics);
        qNEW = ode3(f,0:0.05*st_PI:st_PI,[q;dq]);
        q = qNEW(end,1:df)'; % unpack q0 in q and dq
        dq = qNEW(end,df+1:end)'; % unpack q0 in q and dq            
      end
%       ep = q_ref - q; 

    elseif strcmp(model,'kinematic') % aggiorno stato con modello cinematico
        q=q_ref;
        dq=dq_ref;
    end
