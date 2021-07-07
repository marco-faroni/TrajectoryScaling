function [x,xp,xpp]=tretratti(tt,T,S0,dS,l1,l3)
%
% legge di moto TreTratti (acc.costante)
%
% t tempo per cui calcolare la legge
% T tempo di azionamento
% S0 posizione iniziale
% dS ampiezza movimento
% l1 lambda1 (durata 1^ intervallo/T  0<l1<1)
% l3 lambda3 (durata 3^ intervallo/T  0<l3<1)
% 
% si assume Vini=Vfin=0
%
V=dS/T*2/(2-l1-l3);
A=dS/T^2*2/(l1*(2-l1-l3));
D=dS/T^2*2/(l3*(2-l1-l3));

T1=T*l1;
T2=T*(1-l1-l3);
Ta=T1+T2;

% inizializzo x xp xpp
x = 0*tt;
xp = 0*tt;
xpp = 0*tt;

for i=1:length(tt)
  
  if tt(i)<T1
     x(i)=1/2*A*tt(i)^2+S0;
     xp(i)=A*tt(i);
     xpp(i)=A;
  else
     if tt(i)<Ta
        x(i)=1/2*A*T1^2+V*(tt(i)-T1)+S0;
        xp(i)=V;
        xpp(i)=0;
     else
        x(i)=1/2*A*T1^2+V*T2+V*(tt(i)-Ta)-1/2*D*(tt(i)-Ta)^2+S0;
        xp(i)=V-D*(tt(i)-Ta);
        xpp(i)=-D;
     end
  end
  
end         