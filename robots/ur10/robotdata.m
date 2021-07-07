% data of ur10 robot
name='ur10';
ur10 = importrobot('ur10.urdf');
ur10.DataFormat = 'column'; 
ur10.Gravity = [0 0 -9.8];
df = 6;
jacobian = @(q) [zeros(3), eye(3); eye(3), zeros(3)]*geometricJacobian(ur10,q,'ur10_tool0');  % NB: [Jangular; Jlinear]  
fwdkin = @(q) getTransform(ur10,q,'ur10_tool0');    
Ta = @(x) [eye(3), zeros(3); 0 0 0, 0, -sin(x(4)), cos(x(4))*sin(x(5)); 0 0 0, 0, cos(x(4)), sin(x(4))*sin(x(5)); 0 0 0, 1 0 cos(x(5))];
dJdQ = @(q,qp) DJDQ(q,qp,jacobian);

q_max  = 10*[3, 3, 3, 3, 3, 3]'; %[rad] 
dq_max = [2, 2, 3, 3, 3, 3]'; %[rad/s]
ddq_max = [5 5 10 10 10 10]';
tau_max = [200, 200, 100, 50, 50, 50]';
q_min   = -q_max;
dq_min  = -dq_max;
ddq_min = -ddq_max; 
tau_min  = -tau_max;
load friction_parameters_TYPE2
invdynamics = @(q,qp) dynamics_matrix_ur10_HCBF( ur10, friction_parameters, q, qp,2 );

clear ur10
clear friction_parameters