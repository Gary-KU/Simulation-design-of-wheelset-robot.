function init_lqr_model()
here=fileparts(mfilename('fullpath'));
previous=pwd; restore=onCleanup(@()cd(previous)); %#ok<NASGU>
cd(here); p=lqr_model_parameters();
assert(all(isfinite([p.body_mass_kg p.body_length_m p.body_width_m p.body_height_m p.wheel_radius_m p.wheel_width_m p.plate_thickness_m])));
assert(all([p.body_mass_kg p.body_length_m p.body_width_m p.body_height_m p.wheel_radius_m p.wheel_width_m p.plate_thickness_m]>0));
lengths=p.length_min_m:0.01:p.length_max_m;
values=zeros(12,numel(lengths));
for k=1:numel(lengths)
 [A,B]=linear_plant(p,lengths(k));
 plant=c2d(ss(A,B,eye(6),zeros(6,2)),p.Ts);
 % Existing actuator coordinates implement u=+K*x, opposite analytical B.
 gain=dlqr(plant.A,plant.B,diag(p.Q),diag(p.R));
 values(:,k)=gain(:);
end
coefficients=zeros(12,4);
for j=1:12,coefficients(j,:)=polyfit(lengths,values(j,:),3);end
% Verify gain scheduling over the entire established leg length range.
largest=0;
for length=linspace(p.length_min_m,p.length_max_m,101)
 [A,B]=linear_plant(p,length);
 plant=c2d(ss(A,B,eye(6),zeros(6,2)),p.Ts);
 gain=reshape(coefficients*[length^3;length^2;length;1],2,6);
 radius=max(abs(eig(plant.A-plant.B*gain)));
 assert(radius<1,'LQR fit produced an unstable closed-loop linearization');
 largest=max(largest,radius);
end
assignin('base','lqr_model_config',p);
assignin('base','lqr_gain_coefficients',coefficients);
assignin('base','lqr_max_linear_pole_radius',largest);
end
function [A,B]=linear_plant(p,L0)
L=L0/2; r=p.wheel_radius_m; M=p.body_mass_kg/2;
mp=p.leg_mass_per_side_kg; mw=p.wheel_mass_per_side_kg;
E=[M+mp+mw+p.wheel_inertia_per_side/r^2, M*L0+mp*L, 0; ...
 M*L0+mp*L, p.leg_inertia_per_side+mp*L^2+M*L0^2, 0; ...
 0,0,p.body_pitch_inertia_per_side];
gravity=E\[0;(mp*L+M*L0)*p.gravity_m_s2;0];
actuation=E\[1/r,0;-1,1;0,1];
A=zeros(6); A(1,2)=1; A(3,4)=1; A(5,6)=1;
A([4 2 6],1)=gravity;
B=zeros(6,2); B([4 2 6],:)=actuation;
end
