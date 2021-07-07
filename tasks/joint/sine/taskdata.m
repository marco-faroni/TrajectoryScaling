df=6;
qstart = [  0,   -2,   0, 0,    0,   0.5]';
Amp = [0.3 0.6 0.7 0.65 0.75 0.8]'; % TME - % CEP +-
%   Amp = [ 1.0, 0.5, 1.0, 1.5, 3.14, 3.0]';
% Amp = [ 1.0, 0.5, 1.0, 1.5, 2.5, 4.7]';
%   Amp=0.5*ones(df,1);
omega = 1*[2*pi 2*pi 2*pi 2*pi 2*pi 2*pi]'; % TME 4pi % CEP 2pi 3pi
phi = [0 0 0 0 0 0]';

trj_fcn   = @(t) repmat(qstart,1,size(t,2)) + repmat(Amp,1,size(t,2)).*sin(repmat(omega,1,length(t)).*repmat(ldm(t),df,1)+repmat(phi,1,length(t)));
dtrj_fcn  = @(t) repmat(Amp,1,size(t,2)).*repmat(omega,1,length(t)).*cos(repmat(omega,1,length(t)).*repmat(ldm(t),df,1)+repmat(phi,1,length(t))).*repmat(ldm_d(t),df,1);
ddtrj_fcn = @(t) repmat(Amp,1,size(t,2)).*repmat(omega,1,length(t)).*cos(repmat(omega,1,length(t)).*repmat(ldm(t),df,1)+repmat(phi,1,length(t))).*repmat(ldm_dd(t),df,1) - repmat(Amp,1,size(t,2)).*repmat(omega,1,length(t)).^2.*sin(repmat(omega,1,length(t)).*repmat(ldm(t),df,1)+repmat(phi,1,length(t))).*(repmat(ldm_d(t),df,1)).^2;
