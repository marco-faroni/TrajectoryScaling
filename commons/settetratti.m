%ccc


Ts = 0.001;% st;

Tend = 3;



% jerk_ref = 20.0;
% acc_ref = 1.50;
% vel_ref = 1;
% pos_ref = 1;
% vel_end = 0;

jerk_ref = 500;
acc_ref = 40;
vel_ref = 4;
pos_ref = 1;
vel_end = 0;



pos_0 = 0;
vel_0 = 0;
acc_0 = 0;
jerk_0 = jerk_ref;

t = 0;
a = acc_0;
v = vel_0;
p = 0;
Dp = 0;
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

Ttot = t1+t2+t3+t4+t5+t6+t7;
Ta = t1+t2+t3;
Td = t5+t6+t7;




for idx = 1:fix(Ttot/Ts)
  
  t = (idx-1)*Ts;  
  if (t>=0) && (t<t1)
      state = 1;
      fine = 0;
  end
  if (t>=t1) && (t<t1+t2)
      state = 2;
  end
  if (t>=t1+t2) && (t<t1+t2+t3)
      state = 3;
  end
  if (t>=t1+t2+t3) && (t<t1+t2+t3+t4)
      state = 4;
  end
  if (t>=t1+t2+t3+t4) && (t<t1+t2+t3+t4+t5)
      state = 5;
  end
  if (t>=t1+t2+t3+t4+t5) && (t<t1+t2+t3+t4+t5+t6)
      state = 6;
  end
  if (t>=t1+t2+t3+t4+t5+t6) && (t<t1+t2+t3+t4+t5+t6+t7)
      state = 7;
  end
  if (t>=t1+t2+t3+t4+t5+t6+t7) && (t<t1+t2+t3+t4+t5+t6+t7+Tend)
      state = 8;
  end
  if (t>=t1+t2+t3+t4+t5+t6+t7+Tend) || (t<0)
      state = 9;
      fine = 1;
  end
  
  switch state
      case 1
          j = +jerk_ref;
          a = +j*t;
          v = vel_0 + 1/2*j*t^2;
          p = pos_0 + vel_0*t + 1/2*acc_0*t^2 + 1/6*j*t^3;
      case 2
          j = 0;
          a = acc_lim;
          v = vel_0 + a*(t-t1/2);
          p = pos_0 + vel_0*t + 1/6*a*(3*t^2-3*t*t1+t1^2);
      case 3
          j = -jerk_ref;
          a = -j*(Ta-t);
          v = vel_lim + 1/2*j*(2*t1+t2-t)^2;
          p = pos_0 + 1/2*(vel_lim+vel_0)*Ta + vel_lim*(t-Ta)...
              - 1/6*j*(Ta-t)^3;
      case 4
          j = 0;
          a = 0;
          v = vel_lim;
          p = pos_0 + 1/2*(vel_lim+vel_0)*Ta+vel_lim*(t-Ta);
      case 5
          j = -jerk_ref;
          a = +j*(t-Ttot+Td);
          v = vel_lim + 1/2*j*(t-Ttot+Td)^2;
          p = pos_ref - 1/2*(vel_lim-vel_0)*Td + vel_lim*(t-Ttot+Td)...
              + 1/6*j*(t-Ttot+Td)^3;
      case 6
          j = 0;
          a = -acc_lim;
          v = vel_lim + a*(t-Ttot+Td-t5/2);
          p = pos_ref - 1/2*Td*(vel_lim+vel_end) + vel_lim*(t-Ttot+Td)...
              + 1/6*a*( 3*(t-Ttot+Td)^2 -3*t5*(t-Ttot+Td) +t5^2);
      case 7
          j = +jerk_ref;
          a = -j*(Ttot-t);
          v = vel_end + 1/2*j*(Ttot-t)^2;
          p = pos_ref - vel_end*(Ttot-t) - 1/6*j*(Ttot-t)^3;
      case 8
          j = 0;
          a = 0;
          v = vel_end;
          p = p;
      case 9
          j = 0;
          a = 0;
          v = vel_end;
          p = p;
  end
  
  P(idx) = p;
  V(idx) = v;
  A(idx) = a;
  J(idx) = j;
  
end
  
  
  