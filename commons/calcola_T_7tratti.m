% Input:
%   jerk_ref = valore massimo di jerk della legge di moto
%    acc_ref  = valore massimo di accelerazione della legge di moto
%    vel_ref  = valore massimo di velocità della legge di moto
%    pos_ref = valore finale posizione (=1)
%  Output:
%    Ttot = tempo totale della traiettoria di riferimento
%    DT = vettore contenente la durata di ogni tratto della legge di moto a 7 tratti

function [Ttot,DT] = calcola_T_7tratti(jerk_ref,acc_ref,vel_ref,pos_ref)
  vel_end = 0;

pos_0 = 0;
vel_0 = 0;
acc_0 = 0;
jerk_0 = jerk_ref;

a = acc_0;
v = vel_0;
p = 0;
j = jerk_ref;


if vel_ref*jerk_ref >= acc_ref^2
        t1 = acc_ref/jerk_ref;
        t2 = t1+vel_ref/acc_ref -2*t1;
else
    t1 = sqrt(vel_ref/jerk_ref);
    t2 = 0;
end
t3 = t1;
t5 = t3;
t7 = t3;  
t6 = t2;

t4 = (pos_ref-pos_0)/vel_ref - (t2+t1+t3);
acc_lim = acc_ref;
    vel_lim = vel_ref;
    
if t4 < 0
if abs(pos_ref-pos_0) >= abs(2*acc_ref^3/jerk_ref^2)
  t1 = acc_ref/jerk_ref;
  t2 = t1/2 + sqrt((t1/2)^2+(pos_ref-pos_0)/acc_ref) -2*t1;
else
  t1 = ((pos_ref-pos_0)/(2*jerk_ref))^1/3;
  t2 = 0;
end

  t3 = t1;
  t5 = t3;
  t7 = t3;
  t6 = t2;

  acc_lim = jerk_ref*t1;
  vel_lim = (t1+t2)*acc_lim;

  t4 = 0;
end

DT = [t1,t2,t3,t4,t5,t6,t7];
Ttot = t1+t2+t3+t4+t5+t6+t7;