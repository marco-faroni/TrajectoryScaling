function [ TASK ] = import_task(task_space, path, timinglaw, Ttot)
    % import a structure TASK based on input
    addpath('./tasks/');
    select_timing_law;
    rmpath('./tasks/');
    addpath(['./tasks/', task_space, '/', path]);
    taskdata;
%     rmpath(['./tasks/', task_space, '/', path]);
    TASK.trj_fcn=trj_fcn;
    TASK.dtrj_fcn=dtrj_fcn;
    TASK.ddtrj_fcn=ddtrj_fcn;
    TASK.ldm=ldm;
    TASK.ldm_d=ldm_d;
    TASK.ldm_dd=ldm_dd;
    TASK.total_time=Ttot;
    TASK.size=size(dtrj_fcn(0),1);
end
