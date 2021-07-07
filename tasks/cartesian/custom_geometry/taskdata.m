%% create path with bevels 
% ccc
% % some path
% points = [ 1, 0.15, 0.4;
%            1, 0.15, 0.48;
%            1, 0.08, 0.4;
%            1, 0.08, 0.55;
%            1,-0.08, 0.55;
%            1,-0.08, 0.48;
%            1,-0.15, 0.48;
%            1,-0.15, 0.32;
%            1,-0.08, 0.32;
%            1,-0.08, 0.25;
%            1, 0.08, 0.25;
%            1, 0.08, 0.32;
%            1, 0.15, 0.32 ]';
       
% CNR
points = [1,-0.1,0.4]' + [0,0.16,0.2;
                       0,0,0.2;
                       0,0,0;
                       0,0.2,0;
                       0,0.2,0.25;
                       0,0.4,-0.05;
                       0,0.4,0.2;
                       0,0.55,0.2;
                       0,0.55,0.09;
                       0,0.4,0.09;
                       0,0.55,0 ]';   % 0,0.55,0 
                   
radius_vec= [0.08,0.08,0.02,0.02,0.02,0.02,0.05,0.05,0.01];

figure(999)
plot3(points(1,:),points(2,:),points(3,:));
xlabel("x")
ylabel("y")
zlabel("z")
axis equal
hold on

segments = diff(points');  
lengths = mycolnorm(segments');
versors = segments./lengths';

lengths_smoothed=zeros(1,length(lengths)*2-1); % lengths of the segment-and-circle path
lengths_smoothed(1:2:end)=lengths(1:end);

path_fcn=cell(length(lengths_smoothed),1);
dpath_fcn=cell(length(lengths_smoothed),1);
ddpath_fcn=cell(length(lengths_smoothed),1);

radius3old=0;

for i_seg=1:length(lengths)-1

    point_cross = points(:,i_seg+1);
    versor1=versors(i_seg,:);
    versor2=versors(i_seg+1,:);
    radius=radius_vec(i_seg);

    start_point=point_cross-radius*versor1';
    end_point=point_cross+radius*versor2';

    s_prod = versor1*versor2';
    if s_prod~=0 % acute or obtuse
        if s_prod<0
            angle_inner=acos(abs(s_prod));
            angle=pi-angle_inner;
        else
            angle=acos(abs(s_prod));
            angle_inner=pi-angle;
        end
        radius2=radius/sin(angle_inner/2);
        radius3=radius2*cos(angle_inner/2);
        bisec_dir=( (start_point+end_point)*0.5-point_cross )'/norm((start_point+end_point)*0.5-point_cross);
        center=point_cross+radius2*bisec_dir';
        start_point=point_cross-radius3*versor1';
    else % rect
        angle=pi/2;
        center=start_point+radius*versor2';
        radius3=radius;
    end
    base_v1=(start_point-center)'/norm(start_point-center); 
    base_v2=versor1;
    
    lengths_smoothed(2*i_seg-1)=lengths_smoothed(2*i_seg-1)-radius3;
    lengths_smoothed(2*i_seg+1)=lengths_smoothed(2*i_seg+1)-radius3;
    
    lengths_smoothed(i_seg*2)=angle*radius;
    
    path_fcn{i_seg*2-1}=@(s) points(:,i_seg)+versor1'*(s+radius3old);
    path_fcn{i_seg*2}=@(s) center+radius*cos(s/radius)*base_v1'+radius*sin(s/radius)*base_v2';
    dpath_fcn{i_seg*2-1}=@(s) versor1';
    dpath_fcn{i_seg*2}=@(s) -sin(s/radius)*base_v1'+cos(s/radius)*base_v2';
    ddpath_fcn{i_seg*2-1}=@(s) 0*versor1';
    ddpath_fcn{i_seg*2}=@(s) -1/radius*cos(s/radius)*base_v1'-1/radius*sin(s/radius)*base_v2';

%     theta=linspace(0,pi/2,100);
%     bevel_points=zeros(3,length(theta));
%     for idx=1:length(theta)
%         theta_i=theta(idx);
%         bevel_points(:,idx)=center+radius*cos(theta_i)*base_v1'+radius*sin(theta_i)*base_v2';
%     end
%     plot3(bevel_points(1,:),bevel_points(2,:),bevel_points(3,:))

    radius3old=radius3;

end

path_fcn{1}=@(s) points(:,1)+versors(1,:)'*s;
path_fcn{end}=@(s) points(:,end-1)+versors(end,:)'*(s+radius3);
dpath_fcn{end}=@(s) versors(end,:)';
ddpath_fcn{end}=@(s) 0*versors(end,:)';

% 
% zz=[];
% for i=1:length(lengths_smoothed)
%     s=linspace(0,lengths_smoothed(i),fix(lengths_smoothed(i)*200));
%     for j=1:length(s)
%         xx=path_fcn{i}(s(j));
%         zz=[zz, xx];
%     end
% end
% 
% figure
% plot(zz(2,:),zz(3,:),'.k')

%% path functions
% 
% ldm=@(t) [1 0 0]*tretratti_vec(t,10,0,sum(lengths_smoothed),0.2,0.2);
% ldm_d=@(t) [0 1 0]*tretratti_vec(t,10,0,sum(lengths_smoothed),0.2,0.2);
% ldm_dd=@(t) [0 0 1]*tretratti_vec(t,10,0,sum(lengths_smoothed),0.2,0.2);


length_tot=sum(lengths_smoothed);
Phi0 = 0; Theta0 = pi/2; Psi0 = -0.75*pi;    % orientation

trj_fcn =   @(t) [ path3d(length_tot*ldm(t),lengths_smoothed,path_fcn);
               Phi0+0*ldm(t);
               Theta0+0*ldm(t);
               Psi0+0*ldm(t) ];

dtrj_fcn = @(t) [ path3d(length_tot*ldm(t),lengths_smoothed,dpath_fcn).*(length_tot*ldm_d(t));
                0*ldm(t);
                0*ldm(t);
                0*ldm(t) ]; 

ddtrj_fcn = @(t) [ path3d(length_tot*ldm(t),lengths_smoothed,ddpath_fcn).*(length_tot*ldm_d(t)).^2 + path3d(ldm(t),lengths_smoothed,dpath_fcn).*(length_tot*ldm_dd(t));
                0*ldm(t);
                0*ldm(t);
                0*ldm(t) ];  
            
            
%% path functions for plots
% path_fcn_plot=@(s) [ path3d(length_tot*s,lengths_smoothed,path_fcn);
%                Phi0+0*s;
%                Theta0+0*s;
%                Psi0+0*s ];

 
% xx=trj_fcn(0:0.001:11);
% dxx=dtrj_fcn(0:0.001:11);
% ddxx=ddtrj_fcn(0:0.001:11);
% 
% figure
% plot3(xx(1,:),xx(2,:),xx(3,:));
% 
% figure
% plot(xx(1,:)')
% hold on
% plot(dxx(1,:)')
% plot(ddxx(1,:)')