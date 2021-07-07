function [PhiThetaPsi] = R2ZYZ(R,sol)
  
  if sol == 1
    Phi = atan2(R(2,3),R(1,3));
    Theta = atan2( sqrt(R(1,3)^2+R(2,3)^2) , R(3,3) );
    Psi = atan2(R(3,2), -R(3,1));
  else
    Phi = atan2(-R(2,3),-R(1,3));
    Theta = atan2( -sqrt(R(1,3)^2+R(2,3)^2) , R(3,3) );
    Psi = atan2(-R(3,2), -R(3,1));
  end
  PhiThetaPsi = [ Phi; Theta; Psi ];

end