function [ CONTROLLER ] = init_controller( name , robot, task, st, horizon, Np)
    % import a structure CONTROLLER based on name
    addpath(['./controllers/', name]);
    controllerdata;
    CONTROLLER=ws2struct();
    rmpath(['./controllers/', name]);
end

