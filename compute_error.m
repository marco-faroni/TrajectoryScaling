%% Compute the path error with respect to a given geometrical curve

error_mean=[];
error_max=[];

if strcmp(task_space,'joint')
  trj_log=q_log;
elseif strcmp(task_space,'cartesian')
  trj_log=x_log;
end

step=0.01*st;
try
    s_log;
    FAST=1;
catch
    FAST=0;
end
    
if strcmp(task_space,'joint')

    tic
    error = zeros(1,size(trj_log,2));
    if FAST
        for i = 1:length(error)
          s_i = s_log(i);
          s_vec_i = max(min(s_i+(-10*st:step:10*st),time_tot),0);
          path_nominal = task.trj_fcn( s_vec_i );
          error_norm = mycolnorm( ( path_nominal - repmat(trj_log(:,i,1),1,size(path_nominal,2)) ) );
          [error(i), index_error(i)] = min( error_norm );
        end
    else
        path_nominal = task.trj_fcn( 0:step:time_tot );
        for i = 1:length(error)
          error_norm = mycolnorm( ( path_nominal - repmat(trj_log(:,i,1),1,size(path_nominal,2)) ) );
          [error(i), index_error(i)] = min( error_norm );
        end
    end
    try
%         id_end=find(s_log<time_tot-1e-4,1,'last')
        error_from_goal=mycolnorm( repmat( task.trj_fcn( time_tot ),1,size(trj_log,2) ) - trj_log(:,:,1)  );
        id_end= find(error_from_goal>1e-4,1,'last');    
    catch
        try
            id_end=find(trj.t<(trj.nominalTime/trj.s_mean),1,'last');
        catch
            id_end= find(error>5e-4,1,'last');
        end
    end
    if isempty(id_end)
        id_end=length(error);
    end
    error_mean = mean(error(1:id_end));
    [error_max, index_error_max] = max(error(1:id_end)); 
    display(['Maximum error = ' num2str(error_max) ' at iteration ' num2str(index_error_max)]);
    display(['Mean error= ' num2str(error_mean)]);
    toc
    
    figure
    subplot(2,1,1)
    plot(t_log,error)
    plotLine(id_end*st,1)
    subplot(2,1,2)
    plot(trj_log')
    legend("1","2","3","4","5","6")
    hold on
    plot(task.trj_fcn(0:st:time_tot)')
    pause(0.1)

elseif strcmp(task_space,'cartesian')
    tic
    err_lin = zeros(1,size(trj_log,2));
    for i = 1:length(err_lin)
      s_i = s_log(i);
      s_i_vec = max(min(s_i+(-10*st:step:10*st),time_tot),0);
      path_nominal = [eye(3), zeros(3)]*task.trj_fcn( s_i_vec );
      norm_e = mycolnorm( ( path_nominal - repmat(trj_log(1:3,i,1),1,size(path_nominal,2)) ) );
      [err_lin(i), idx_err(i)] = min( norm_e );
    end
    try
%         id_end=find(s_log<time_tot-1e-4,1,'last');
        error_from_goal=mycolnorm( repmat( [eye(3), zeros(3)]*task.trj_fcn( time_tot ),1,size(trj_log,2) ) - trj_log(1:3,:,1)  );
        id_end= find(error_from_goal>1e-4,1,'last');
    catch
        id_end= find(err_lin>5e-4,1,'last');
    end
    if isempty(id_end)
        id_end=length(err_lin);
    end
    error_mean_lin = mean(err_lin(1:id_end));
    [error_max_lin, index_error_max] = max(err_lin); 
    display(['Maximum error = ' num2str(error_max_lin) ' at iteration ' num2str(index_error_max)]);
    display(['Mean error= ' num2str(error_mean_lin)]);    
    toc    
    figure
    subplot(2,1,1)
    plot(t_log,err_lin)
    plotLine(id_end*st,1)
    subplot(2,1,2)
    plot(trj_log')
    legend("1","2","3","4","5","6")
    hold on
    plot(task.trj_fcn(0:st:time_tot)')
    pause(0.1)
    
    tic
    err_ang = zeros(1,size(trj_log,2));
    for i = 1:length(err_ang)
      s_i = s_log(i);
      s_i_vec = max(min(s_i+(-10*st:step:10*st),time_tot),0);
      path_nominal = [zeros(3), eye(3)]*task.trj_fcn( s_i_vec );
      norm_e = mycolnorm( ( path_nominal - repmat(trj_log(4:6,i,1),1,size(path_nominal,2)) ) );
      [err_ang(i), idx_err(i)] = min( norm_e );
    end
%     try
%         id_end=find(s_log<time_tot-1e-4,'last')
%     catch
%         id_end= find(err_ang>1e-4,1,'last');
%     end
%     if isempty(id_end)
%         id_end=length(err_ang);
%     end
    error_mean_ang = mean(err_ang(1:id_end));
    [error_max_ang, index_error_max] = max(err_ang); 
    display(['Maximum error = ' num2str(error_max_ang) ' at iteration ' num2str(index_error_max)]);
    display(['Mean error= ' num2str(error_mean_ang)]);
    toc
    figure
    subplot(2,1,1)
    plot(t_log,err_ang)
    plotLine(id_end*st,1)
    subplot(2,1,2)
    plot(trj_log')
    legend("1","2","3","4","5","6")
    hold on
    plot(task.trj_fcn(0:st:time_tot)')
    pause(0.1)
    
    error_max=[error_max_lin, error_max_ang];
    error_mean=[error_mean_lin, error_mean_ang];
end