%% function per il calcolo delle matrici della dinamica per il robot UR10
% la dinamica inversa � calcolata nella forma tau = H(q)*qpp + B(q,qp) dove
% H � la matrice di inerzia e B tiene conto di Coriolis, azioni centrifughe
% e attriti.
% Input:
%   robot: oggetto robot generato tramite importrobot(urdf)
%   friction parameters: vettore 1x6 dei parametri di attrito viscoso e
%                        statico
%   q: configurazione robot vettore length=6
%   dq: velocit� giunti robot length=6
% Output:
%   H: inertia matrix 6x6
%   C: Coriolis matrix
%   G: gravity vector
%   Fs: static friction matrix
%   Fv: viscous friction matrix

function [H, C, G, Fs, Fv] = dynamics_matrix_ur10_HCBF( robot, friction_parameters, q, dq, TYPE)
    if size(q,2)~=1
        q = q';
    end
    if size(dq,2)~=1
        dq = dq';
    end
        segno = sign(dq);
    if TYPE==1
        H = massMatrix(robot,q);
        C = velocityProduct(robot,q,dq);
        G = gravityTorque(robot,q);
        Fs = segno.*friction_parameters(:,1);
        Fv = dq.*friction_parameters(:,2);
    elseif TYPE==2
        [H,G] = myrne1(q,0*dq,0*q);
        [~,C] = myrne1(q,dq,0*q);
        C = C - G;
        Fs = segno.*friction_parameters(:,1);
        Fv = dq.*friction_parameters(:,2);
    end
    H(end) = 10*H(end);
end