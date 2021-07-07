df=6;
xC =  0.6;
yC =  0.8;
zC =  0.4;
R  =  0.2;
DX =  -0.5;

 R0 = [ 0            0             1;
        0            1             0;
       -1            0             0];
Phi0 = 0; Theta0 = pi/2; Psi0 = 0;    % R2ZYZ(R0,1);    

w = 4*pi;
trj_fcn =  @(t) [ xC+DX*ldm(t); 
               yC+0*ldm(t); 
               zC + R*sin(w*ldm(t));
               Phi0+0*ldm(t);
               Theta0+0*ldm(t);
               Psi0+0*ldm(t) ];

dtrj_fcn = @(t) [ DX*ldm_d(t);
               0*ldm(t);
               w*R*cos(w*ldm(t)).*ldm_d(t);
               0*ldm(t);
               0*ldm(t);
               0*ldm(t) ];  

ddtrj_fcn = @(t) [ DX*ldm_dd(t);
               0*ldm(t);
               -w^2*R*sin(w*ldm(t)).*(ldm_d(t).^2) + w*R*cos(w*ldm(t)).*ldm_dd(t);
               0*ldm(t);
               0*ldm(t);
               0*ldm(t) ];  
