%% select the law of motion

if strcmp(timinglaw,'tretratti') 
    ldm    = @(t)  [1 0 0]*tretratti_vec(t,Ttot,0,1,0.25,0.25);
    ldm_d  = @(t)  [0 1 0]*tretratti_vec(t,Ttot,0,1,0.25,0.25);
    ldm_dd = @(t)  [0 0 1]*tretratti_vec(t,Ttot,0,1,0.25,0.25);
elseif strcmp(timinglaw,'cicloidale')
    ldm    = @(t)  [1 0 0]*mycicloidale(t,Ttot,0,1);
    ldm_d  = @(t)  [0 1 0]*mycicloidale(t,Ttot,0,1);
    ldm_dd = @(t)  [0 0 1]*mycicloidale(t,Ttot,0,1);
elseif strcmp(timinglaw,'polynomial')
    ldm    = @(t) [1 0 0]*polynominal_law(t,Ttot);
    ldm_d  = @(t) [0 1 0]*polynominal_law(t,Ttot);
    ldm_dd = @(t) [0 0 1]*polynominal_law(t,Ttot);
elseif strcmp(timinglaw,'settetratti')
    jerk_ref = 1; 
    acc_ref = 0.4; 
    vel_ref = 0.4;
    pos_ref = 1;

    %kk = 2.02; % 2s
    %kk = 1.52; % 2.5s
    %kk = 1.23; % 3s
    %kk = 1.04; % 3.5s
    %kk = 0.9; % 4s
    %kk = 0.712; % 5.0s
    %kk = 0.646; % 5.5s
    %kk = 0.504; % 7.0s
    %kk = 0.391; % 9.0s

    %         kk = 0.504; % PROVE TME: 0.29 0.35 0.44 0.6 0.9 2 % PROVE CEP: 0.39 0.5 0.645 0.9 1.5
    kk=Ttot;
    jerk_ref = 3*kk; 
    acc_ref = 0.4*kk^2; 
    vel_ref = 0.4*kk;
    pos_ref = 1;      
    [Ttot,DT] = calcola_T_7tratti(jerk_ref,acc_ref,vel_ref,pos_ref);
    Ttot
    ldm    = @(t)  [1 0 0]*mySetteTratti(t,jerk_ref,acc_ref,vel_ref,pos_ref);
    ldm_d  = @(t)  [0 1 0]*mySetteTratti(t,jerk_ref,acc_ref,vel_ref,pos_ref);
    ldm_dd = @(t)  [0 0 1]*mySetteTratti(t,jerk_ref,acc_ref,vel_ref,pos_ref);
end 
