function x = FKIN(q,M,x0)
  T = M(q);
  if nargin==3
    [PhiThetaPsi1] = R2ZYZ(T(1:3,1:3),1);
    [PhiThetaPsi2] = R2ZYZ(T(1:3,1:3),2);
    if sum(abs(x0(4:6)-PhiThetaPsi1)) <= sum(abs(x0(4:6)-PhiThetaPsi2))
      PhiThetaPsi = PhiThetaPsi1;
    else
      PhiThetaPsi = PhiThetaPsi2;
    end
  elseif nargin==2
    [PhiThetaPsi] = R2ZYZ(T(1:3,1:3),1);
  end
  x = [T(1:3,4); PhiThetaPsi];

end