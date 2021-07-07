df=6;
xC = 1;
yC = 0.0;
zC = 0.4;
R  = 0.2; 
cicli = 1;

R0 = [ 0            0             1;
    -0.5*sqrt(2)  -0.5*sqrt(2)  0;
     0.5*sqrt(2)  -0.5*sqrt(2)  0];
Phi0 = 0; Theta0 = pi/2; Psi0 = -0.75*pi;    % R2ZYZ(R0,1);

trj_fcn =   @(t) [ xC+0*ldm(t); 
               yC + R*cos(2*pi*cicli*ldm(t)); 
               zC + R*sin(2*pi*cicli*ldm(t));
               Phi0+0*ldm(t);
               Theta0+0*ldm(t);
               Psi0+0*ldm(t) ];

dtrj_fcn = @(t) [   0*ldm(t);
               -2*pi*cicli*R*sin(2*pi*cicli*ldm(t)).*ldm_d(t);
                2*pi*cicli*R*cos(2*pi*cicli*ldm(t)).*ldm_d(t);
                0*ldm(t);
                0*ldm(t);
                0*ldm(t) ]; 

ddtrj_fcn = @(t) [  0*ldm(t);
                -(2*pi*cicli)^2*R*cos(2*pi*cicli*ldm(t)).*(ldm_d(t).^2) - 2*pi*cicli*R*sin(2*pi*cicli*ldm(t)).*ldm_dd(t);
                -(2*pi*cicli)^2*R*sin(2*pi*cicli*ldm(t)).*(ldm_d(t).^2) + 2*pi*cicli*R*cos(2*pi*cicli*ldm(t)).*ldm_dd(t);
                0*ldm(t);
                0*ldm(t);
                0*ldm(t) ];  
    