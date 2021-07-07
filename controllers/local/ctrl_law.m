%% control law for method 'thor'

function [ddq_ctrl,ds,ds_ref,ctrl]=ctrl_law(ctrl,robot,task,s,q0,sc_old,s_ref_old)

    st=ctrl.st;
    Np=ctrl.Np;
    index_p=ctrl.index_p;
    t=s;
    sc=sc_old;
    Nref=ctrl.horizon;
    Ns=Np;
    df=robot.df;
    
    % compute first prediction
    if ctrl.is_initialized==0
        for i_ord = 1:ctrl.order
          ctrl.prediction(:,:,i_ord) = reshape(ctrl.L(1:df*Np,:,i_ord)*q0+ctrl.F(1:df*Np,:,i_ord)*zeros(df*Np,1),df,Np);
        end
    end

    t_vec = t + st*index_p(1:Np); % vettore degli istanti di predizione sulla traiettoria
    qdes = task.trj_fcn(t);   % posizione desiderata ( da usare nella "CLIK")
    qd_des = task.dtrj_fcn(t_vec); % riferimento di velocita
    qd_des = qd_des(:);

    % calcolo matrici modello dinamico al passo attuale
    if ctrl.TORQ_CONST==1
        [Hmat1,Cmat1,Gmat1,Fsmat1,Fvmat1] = robot.invdynamics(q0(1:ctrl.order:end),q0(2:ctrl.order:end));
        Bmat1 = Cmat1+Gmat1+Fsmat1+Fvmat1;
    end
    
    % Modifica riferimento s_ref
  tic
  if ctrl.MODIFY_SREF
    Q_P  = qd_des(1:df);
    Q_PP = task.ddtrj_fcn(t_vec(1));
    
    Tref = t_vec(1)+(0:sc*st:sc*st*Nref);
    Q_PPmax = max( abs( task.ddtrj_fcn(Tref ) ),[],2); % prendo i massimi valori di accelerazione nella finestre considerata
    
    s_ref_vel = min( (robot.dq_max)./abs(Q_P) ); % velocit�
    s_ref_acc = min( sqrt( (robot.ddq_max)./abs(Q_PPmax) ) ); % accelerazione
    
    if ctrl.TORQ_CONST % coppia
      
      % stack con s_ref docuto a coppia (FIFO)
      if ctrl.fifo_iter>Nref
        ctrl.fifo_iter = 1;
      end
              
%       if ctrl.is_initialized==0
%         tauDES = Hmat1*Q_PP+Bmat1;
%         [maxTorq,id_MAX3] = max(abs(tauDES)./robot.tau_max);
%         if maxTorq>1
%           if sign(tauDES(id_MAX3))>=0
%             s_ref_torq = sqrt( (robot.tau_max(id_MAX3)- Bmat1(id_MAX3)) / (Hmat1(id_MAX3,:)*Q_PP) );
%           else
%             s_ref_torq = sqrt( (robot.tau_min(id_MAX3)- Bmat1(id_MAX3)) / (Hmat1(id_MAX3,:)*Q_PP) );
%           end
%         else
%           s_ref_torq = 1;
%         end
%         ctrl.s_ref_tor_vec = s_ref_torq*ones(1,Nref); % al primo ciclo inizializzo lo stack con il valore iniziale
%        else
     if ctrl.is_initialized==0
        ctrl.s_ref_tor_vec = ones(Nref,1);   
     end
     
        q_Tref = task.trj_fcn(Tref(end));
        qp_Tref = s_ref_old*task.dtrj_fcn(Tref(end));
        qpp_Tref = task.ddtrj_fcn(Tref(end));
                
        [HmatTref,CmatTref,GmatTref,FsmatTref,FvmatTref] = robot.invdynamics(q_Tref,s_ref_old*qp_Tref);
        BmatTref = CmatTref+GmatTref+FsmatTref+FvmatTref;
        
        tau_Tref = HmatTref*qpp_Tref + BmatTref;
        [maxTorq,id_MAX3] = max(abs(tau_Tref)./robot.tau_max);
        if maxTorq>1
          if sign(tau_Tref(id_MAX3))>=0
            %s_ref_torq = sqrt( (tau_max(id_MAX3)- BmatTref(id_MAX3)) / (HmatTref(id_MAX3,:)*qpp_Tref) );
              aa = HmatTref(id_MAX3,:)*qpp_Tref + CmatTref(id_MAX3);
              bb = FvmatTref(id_MAX3);
              cc = GmatTref(id_MAX3) + FsmatTref(id_MAX3,:) - 0.95*robot.tau_max(id_MAX3);
              s12 = roots([aa bb cc]);
              if min(isreal(s12))==0
                  display('complex root')
              end
              s_ref_torq = max(s12);  
          else
              aa = HmatTref(id_MAX3,:)*qpp_Tref + CmatTref(id_MAX3);
              bb = FvmatTref(id_MAX3,:);
              cc = GmatTref(id_MAX3) + FsmatTref(id_MAX3,:) - 0.95*robot.tau_min(id_MAX3);
              s12 = roots([aa bb cc]);
              if min(isreal(s12))==0
                  display('complex root')
              end
              s_ref_torq = max(s12); 
              %s_ref_torq = sqrt( (tau_min(id_MAX3)- BmatTref(id_MAX3)) / (HmatTref(id_MAX3,:)*qpp_Tref) );
          end
        else
          s_ref_torq = 1;
        end
        ctrl.s_ref_tor_vec(ctrl.fifo_iter) = s_ref_torq; % aggiorno lo stack con valore corrente
%       end
      s_ref_torq = min(ctrl.s_ref_tor_vec);
      ctrl.fifo_iter = ctrl.fifo_iter+1;    
    else
        s_ref_torq = 1;
    end
    
    s_ref_u = min( [1; s_ref_vel; s_ref_acc; s_ref_torq] )';
    % LPF su s_ref
    af = st/(0.15*Nref*st + st);
    s_ref = (1-af)*s_ref_old+af*s_ref_u;
    
    if isreal(s_ref)==0||s_ref<0
      warning(['s_ref = ' num2str(s_ref')]);
    end
  else
      s_ref = 1;
  end
  additional_time=toc;

  if ctrl.TORQ_CONST % vincoli di coppia
    for idN = 1:Np
      if idN==1 && ctrl.MODIFY_SREF % se ho gi� calcolato H e B le riutilizzo
        Hi = Hmat1;
        Bi = Bmat1;
      else
          [Hi,Ci,Gi,Fsi,Fvi] = robot.invdynamics(ctrl.prediction(:,idN,1),ctrl.prediction(:,idN,2));
          Bi = Ci+Gi+Fsi+Fvi;
      end
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

  %% DA QUI
  
% Imposto QP
  Heq = [ctrl.F(:,:,2)+[ctrl.Kclik*ctrl.F(1:df,:,1);0*ctrl.F(df+1:end,:,1)], -diag(qd_des)*ctrl.PD; zeros(Ns,Np*df), eye(Ns)];  % CLIK fuori da scaling (OK)
  H = Heq'*ctrl.Wsc*Heq  + ctrl.lambdaU*ctrl.Hu; % lambdaU peso su acceleraz
  f = -[ -(ctrl.L(:,:,2) + [ ctrl.Kclik*ctrl.L(1:df,:,1);0*ctrl.L(df+1:end,:,1)] )*q0 + [ctrl.Kclik*qdes; zeros(df*(Np-1),1)] ; s_ref.*ones(Ns,1) ]'*ctrl.Wsc*ctrl.Hp*Heq; % CLIK fuori da scaling (OK)
  %  f = -[ -(L(:,:,2) + [ Kclik*( (L(1:df,:,1)-PQ) ); 0*L(df+1:end,:,1)] )*q0 + [Kclik*(qdes-qreal); zeros(df*(Np-1),1)] ; s_ref.*ones(Ns,1) ]'*Wsc*Hp*Heq; % CLIK fuori da scaling (OK)

  HH = (H+H')/2;
  
  Aeq = [];
  Beq = [];  
  BBNDU = ctrl.BU1 + ctrl.b2*q0;
  BBNDL = ctrl.BL1 + ctrl.b2*q0;
  ATOT  = [Aeq; ctrl.Abnd;  Atq];
  BLTOT = [Beq; BBNDL; BLtq];
  BUTOT = [Beq; BBNDU; BUtq];  
  
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
  time_cycle=toc;
  ctrl.cycle_time = time_cycle;
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

