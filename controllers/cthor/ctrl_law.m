%% control law for method 'cthor'

function [ddq_ctrl,ds,ds_ref,ctrl]=ctrl_law(ctrl,robot,task,s,q0,sc_old,s_ref_old)

    st=ctrl.st;
    Np=ctrl.Np;
    index_p=ctrl.index_p;
    t=s;
    sc=sc_old;
    Nref=ctrl.horizon;
    Ns=Np;
    df=robot.df;
    m=task.size;
    
    % compute first prediction
    if ctrl.is_initialized==0
        for i_ord = 1:ctrl.order
          ctrl.prediction(:,:,i_ord) = reshape(ctrl.L(1:df*Np,:,i_ord)*q0+ctrl.F(1:df*Np,:,i_ord)*zeros(df*Np,1),df,Np);
        end
    end
    
    ctrl.xreal = FKIN(q0(1:ctrl.order:end),robot.fwdkin,ctrl.xreal);

    t_vec = t + st*index_p(1:Np); % vettore degli istanti di predizione sulla traiettoria
    endTRJ = t_vec>task.total_time;
    t_vec(t_vec>task.total_time) = task.total_time;
    
    x_des = task.trj_fcn(t);  
    xd_des = task.dtrj_fcn(t_vec);
    xd_des = xd_des + ctrl.Kclik*max(sc(1),0.5)*repmat((x_des - ctrl.xreal),1,Np);% + 1*sc(1)*eye(m)*repmat((Xd_des_old - Xd_real),1,Nc); % mettere o no *sc(1)? 
    xd_des_KE=zeros(Np*m,1);
    for ij=1:Np
        xd_des_KE((ij-1)*m+1:ij*m)=robot.Ta(ctrl.xreal)*xd_des(:,ij);
    end
%     xd_des_KE = xd_des(:);

    Jdiag=zeros(Np*m,Np*df);
    for i_jac = 1:Np
        Jdiag( (i_jac-1)*m+1:i_jac*m, (i_jac-1)*df+1:i_jac*df ) = robot.jacobian(ctrl.prediction(:,i_jac,1));
    end
    JF = Jdiag*ctrl.F(:,:,2);

    s_ref=1;
    Heq = [JF, -diag(xd_des_KE)*ctrl.PD; zeros(Np,Np*df), eye(Np)];
    H = Heq'*ctrl.Wsc*Heq + ctrl.lambdaU*ctrl.Hu;
    f = -[-Jdiag*ctrl.L(1:df*Np,:,2)*q0; s_ref.*ones(Np,1)]'*ctrl.Wsc*Heq;
    HH = (H+H')/2;

 
  
    if ctrl.TORQ_CONST==1 % vincoli di coppia
        for idN = 1:Np
          [Hi,Ci,Gi,Fsi,Fvi] = robot.invdynamics(ctrl.prediction(:,idN,1),ctrl.prediction(:,idN,2));
          Bi = Ci+Gi+Fsi+Fvi;
          Hmat((idN-1)*df+1:idN*df,(idN-1)*df+1:idN*df) = Hi;
          Bmat((idN-1)*df+1:idN*df,:) = Bi;
        end
        for idT = idN+1:Np
          Hmat = horzcat(Hmat,0*Hi);
        end    

        Atq  =  [Hmat, zeros(df*Np,Ns)];
        BUtq =  0.98*ctrl.Tau_max - Bmat;
        BLtq =  0.98*ctrl.Tau_min - Bmat;

        if min([BUtq, -BLtq]) <= 0
          warning('Il vincolo di coppia non permette accelerazione nulla: possibile non esistenza di soluzione')
        end
    
    else
        Atq  = [];
        BUtq = [];
        BLtq = [];
    end
  
    Aeq = [];
    Beq = [];  
    BBNDU = ctrl.BU1 + ctrl.b2*q0;
    BBNDL = ctrl.BL1 + ctrl.b2*q0;
    ATOT  = [Aeq; ctrl.Abnd;  Atq];
    BLTOT = [Beq; BBNDL; BLtq];
    BUTOT = [Beq; BBNDU; BUtq];  

    for idx = 1:Np
      if endTRJ(idx)==1
          ctrl.ub(end-Np+idx) = 1.001;
          ctrl.lb(end-Np+idx) = 0.999;
      end
    end

    % Risoluzione QP
    tic
    if ctrl.is_initialized==0 % QPOASES al primo ciclo richiede sintassi leggermente diversa
      [ctrl.QP,U,ctrl.fval,ctrl.exitflag,ctrl.num_iters] = qpOASES_sequence( 'i',HH,f',ATOT,ctrl.lb,ctrl.ub,BLTOT,BUTOT,ctrl.optqpOASES );
    else
      try
        [U,ctrl.fval,ctrl.exitflag,ctrl.num_iters] = qpOASES_sequence( 'm',ctrl.QP,HH,f',ATOT,ctrl.lb,ctrl.ub,BLTOT,BUTOT,ctrl.optqpOASES );
      catch
        [ctrl.QP,U,ctrl.fval,ctrl.exitflag,ctrl.num_iters] = qpOASES_sequence( 'i',HH,f',ATOT,ctrl.lb,ctrl.ub,BLTOT,BUTOT,ctrl.optqpOASES );
      end
    end
    ctrl.cycle_time = toc;
    % spacchetto U = [u', sc']'
    u = U(1:df*Np);
    sc = U(df*Np+1:end);  
    
    % Calcola previsione di q, qd, qdd nei prossimi Np istanti
    for i_ord = 1:ctrl.order
      ctrl.prediction(:,:,i_ord) = reshape(ctrl.L(1:df*Np,:,i_ord)*q0+ctrl.F(1:df*Np,:,i_ord)*u,df,Np);
      %qpred(:,:,i_ord) = reshape(L(:,:,i_ord)*q0+F(:,:,i_ord)*u,df,Np);
    end

    ctrl.is_initialized=1;

    ddq_ctrl=u(1:df);
    ds=sc(1);
    ds_ref=s_ref;
  
end

