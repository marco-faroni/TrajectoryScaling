% Calcola le matrici L e F per la previsione delle uscite y = L*x0 + F*u di
% un sistema lineare xd = Ax + Bu; y = Cx dato un vettore dei tempi "time"
% e un vettore "h" delle ampiezze degli intervalli di tempo relativi alle
% azioni di comando u.
% -> time: vettore dei tempi a cui eseguire la previsione;
% -> h: vettore di ampiezze dei tempi relativi agli istati di tempo delle
% azioni di comando u (Nota: length(h) può essere minore di length(time);
% -> A,B,C: matrici del sistema dinamico;
% -> expA,IexpAB: matlab functions calcolate precedentemente


function [F,L]=FLmatrix(time,h,A,B,C,expA,IexpAB)

cumh=cumsum(h);
Nu=length(h);
order = fix(size(B,1)/size(B,2));
input = fix(size(B,2));

%F=zeros(length(time)*input,Nu,order);
L=[];
% L=zeros(length(time)*input,order*input,order);
F=[];

for idx_r=1:length(time)
  for idx_order=1:order
    Lid(:,:,idx_order)=C*A^(idx_order-1)*expA( time(idx_r) );
  end
  L = vertcat(L,Lid);
end

for idx_r=1:length(time)
  Fidc = [];
  for idx_c=1:Nu
    if (time(idx_r)>=cumh(idx_c))
      t1=h(idx_c);
      t2=time(idx_r)-cumh(idx_c);
    elseif idx_c==1
      t1=time(idx_r);
      t2=0;
    else
      t1=time(idx_r)-cumh(idx_c-1);
      t2=0;
    end   
    
    if t1<0
      for idx_order=1:order
        Fidx(:,:,idx_order)=zeros(input);
      end
    elseif t2==0
      Fidx(:,:,1)=C*(IexpAB(t1));
      for idx_order=2:order
        Fidx(:,:,idx_order)=C*A^(idx_order-2)*(expA(t1)*B);
      end
    else
      for idx_order=1:order
        Fidx(:,:,idx_order)=C*A^(idx_order-1)*expA(t2)*(IexpAB(t1));
      end    
    end
    Fidc = horzcat(Fidc,Fidx);
    
  end
  
  F = vertcat(F,Fidc);

end

% F = flip(F,3);  % fa cagare, sistema tutta la function
% L = flip(L,3);
% 
end