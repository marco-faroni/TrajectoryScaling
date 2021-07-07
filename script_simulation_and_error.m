ccc

addpath('./common_marco');

% settings
robot_name='ur10';
robot=import_robot(robot_name);
model='dynamic'; % kinematic dynamic
st=8e-3; % [s]
addpath(['./robots/', robot_name]);

% load trajectories
folder_path='./traiettorie_TSS_corrado/traiettorie_filtro_locale';
addpath(folder_path);
trjs=[dir([folder_path,'/*ref.mat']);];

table_results=[];

% for itrj=1:length(trjs)
%     load([trjs(itrj).folder,filesep,trjs(itrj).name]);
%     mydata.t=data.time';
%     mydata.q=data.q';
%     mydata.qp=data.qp';
%     mydata.qpp=data.qpp';
%     mydata.nominalTime=data.nominalTime;
%     mydata.s_mean=data.scaling;
%     if trjs(itrj).name(4)=='A'
%         mydata.case=1;
%     else
%         mydata.case=2;
%     end
%     save([trjs(itrj).name(1:end-4) '_marco.mat'],'mydata')
% end

for itrj=1:length(trjs)
    
    temp=load([trjs(itrj).folder,filesep,trjs(itrj).name]);
    trj=temp.mydata;
    task_space='joint'; % cartesian joint
    path='sine'; % sine
    timinglaw='settetratti'; % settetratti polynomial
    time_tot=trj.nominalTime;
    task=import_task(task_space, path, timinglaw, 3); % l'ultimo parametro non influisce sulla simulazione
    
    % initialize
    q=trj.q(:,1);
    dq=trj.qp(:,1);
    ev_int=zeros(robot.df,1);
    ep=zeros(robot.df,1);
    
    % log
    t_log=[];
    u_log=[];
    qref_log=[];
    q_log=[];
    dq_log=[];
    tau_log=[];
    
    for i=1:length(trj.t)
        t_real=trj.t(i);
        q_ref=trj.q(:,i);
        dq_ref=trj.qp(:,i);
        ddq_ctrl=trj.qpp(:,i);
        
        [q,dq,ev_int,ep]=integrate_ddq(model,robot,st,ev_int,ep,q_ref,dq_ref,q,dq,ddq_ctrl);
        %  q=q+st*dq+ddq_ctrl*st^2*0.5;
        %  dq=dq+ddq_ctrl*st;
        
        try
            [Hmat1,Cmat1,Gmat1,Fsmat1,Fvmat1] = robot.invdynamics(q_ref,dq_ref);
            Bmat1 = Cmat1+Gmat1+Fsmat1+Fvmat1;
            tau = Hmat1*ddq_ctrl(1:robot.df) + Bmat1;
        catch
            tau=[];
        end
        
        % log
        t_log=[t_log,t_real];
        u_log=[u_log, ddq_ctrl];
        qref_log=[qref_log,q_ref];
        q_log=[q_log,q];
        dq_log=[dq_log,dq];
        tau_log=[tau_log,tau];
    end
    
    if 1
        compute_error;
    else
        error_mean=0;
        error_max=0;
    end

    table_results=[table_results; trj.case time_tot error_max error_mean]
end
    
rmpath(['./robots/', robot_name]);
rmpath(folder_path);

