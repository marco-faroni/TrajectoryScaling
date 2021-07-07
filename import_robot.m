function [ ROBOT ] = import_robot( name )
    % import a structure ROBOT based on name
    addpath(['./robots/', name]);
    robotdata;
    ROBOT=ws2struct();
    rmpath(['./robots/', name]);
end

