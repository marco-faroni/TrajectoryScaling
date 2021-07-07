%% script for the comparison of different scaling methods
% Author: Marco Faroni
% Organization: CNR-STIIMA
% Date: 18/09/2019

ccc

% settings
robot_name='ur10';
method='clocal'; % choose among: thor cthor local clocal
task_space='cartesian'; % choose among: cartesian joint
path='custom_geometry'; % choose among: sine circle custom_geometry
timinglaw='settetratti'; % choose among: settetratti polynomial
model='kinematic'; % choose among: kinematic dynamic

%
table_results=[];
comput_times=[];
if strcmp(task_space,'joint')
    time_vectors=[0.29 0.35 0.44 0.6 0.9 1.23 2]; % correspond to different nominal execution time
elseif strcmp(task_space,'cartesian')
    time_vectors=[0.7 0.9 1.2 2]; % correspond to different nominal execution time
end

horizon_vectors=[0.2]; % length of the predictive horizons in seconds

for itime=1:length(time_vectors)
 
for ihor=1:length(horizon_vectors)
    
time_tot=time_vectors(itime);
st=8e-3; % sampling time in seconds
Np=5; % number of steps in OCP (if applicable)
horizon=fix(horizon_vectors(ihor)/st); % [number of sampling periods]

%
addpath('./commons');
robot=import_robot(robot_name);
task=import_task(task_space, path, timinglaw, time_tot);
ctrl=init_controller(method, robot, task, st, horizon, Np);
addpath(['./robots/', robot_name]);
addpath(['./controllers/', method]);
order=ctrl.order;
time_tot=task.total_time;

% initial state
if strcmp(task_space,'joint')
    q0 = [task.trj_fcn(0)'; task.dtrj_fcn(0)'];
elseif strcmp(task_space,'cartesian')
    if strcmp(path,'circle') 
        qmean = [0,-0.3,1,-1,0,3]';  
    elseif strcmp(path,'sine') 
        qmean = [0,-0.3,1,-1,0,0]';
    elseif strcmp(path,'custom_geometry') 
        qmean = [0,-0.3,1,-1,0,3]';  
    end
    tmp = INVKIN(task.trj_fcn(0),qmean,robot.fwdkin);
    q0 = [tmp';zeros(1,robot.df)]; % initial configuration
    xreal=task.trj_fcn(0);
    ctrl.xreal=xreal;
end
q0 = q0(:);


% init variables
q = q0(1:order:end);
dq = q0(2:order:end);
ddq_ctrl = zeros(robot.df,1);
iter = 0;
ds = 1;
ds_ref=1;
s = 0;
t_real = 0;
ev_int=zeros(robot.df,1);
ep=zeros(robot.df,1);

% log variables
u_log = [];
qref_log = [];
ds_log = [];
tau_log = [];
t_log = [];
s_log = [];
dsref_log = [];
q_log = [];
dq_log = [];
x_log = [];
cycle_log=[];

% simulation
while and(s-time_tot <= 0.5,t_real(end)<20*time_tot) % stop 1 s after the end of the trajectory
    iter=iter+1;
    t_real = st*iter;   % actual time
    s = s + ds(1)*st;   % scaled time
    [ddq_ctrl,ds,ds_ref,ctrl]=ctrl_law(ctrl,robot,task,s,q0,ds,ds_ref);
    
    % integrate planner reference
    q0(1:order:end)=q0(1:order:end)+st*q0(2:order:end)+0.5*st*st*ddq_ctrl;
    q0(2:order:end)=q0(2:order:end)+st*ddq_ctrl;
    % integrate robot model
    [q,dq,ev_int,ep]=integrate_ddq(model,robot,st,ev_int,ep,q0(1:order:end),q0(2:order:end),q,dq,ddq_ctrl);
    
    
    try
        xreal = FKIN(q,robot.fwdkin,xreal);
    catch
        xreal=[];
    end
    try
        [Hmat1,Cmat1,Gmat1,Fsmat1,Fvmat1] = robot.invdynamics(q0(1:order:end),q0(2:order:end));
        Bmat1 = Cmat1+Gmat1+Fsmat1+Fvmat1;
        tau = Hmat1*ddq_ctrl(1:robot.df) + Bmat1;
    catch
        tau=[];
    end
    
    % waitbar
    if ~mod(iter,20)
        waitbar(s/time_tot)
    end  
    
    % log
    t_log=[t_log,t_real];
    u_log=[u_log, ddq_ctrl];
    ds_log=[ds_log, ds];
    s_log=[s_log, s];
    qref_log=[qref_log,q0(1:order:end)];
    q_log=[q_log,q];
    dq_log=[dq_log,dq];
    x_log = [x_log, xreal];
    tau_log=[tau_log,tau];
    cycle_log=[cycle_log,ctrl.cycle_time];
    gamma=task.ldm(s_log);
    dsref_log=[dsref_log,ds_ref];
        
end

if 1
    compute_error;    
else
    error_mean=0;
    error_max=0;
end

display(['Mean scaling=' num2str(mean(ds_log(1:end-fix(1/st))))])

table_results=[table_results; time_tot error_max error_mean, mean(ds_log(1:end-fix(0.5/st)))];
comput_times=[comput_times; max(cycle_log(2:end)) mean(cycle_log) std(cycle_log)]

end

x_log_cells{itime}=x_log;
q_log_cells{itime}=q_log;
dq_log_cells{itime}=dq_log;

end

save('TAM_tests_for_reviewMMT_st8ms', 'table_results', 'task', 'x_log_cells', 'q_log_cells', 'dq_log_cells')

rmpath(['./robots/', robot_name]);
rmpath(['./controllers/', method]);


