function init_roll_lmpc()
% 自动初始化：模型启动时调用，日常调参只改两份 parameters 文件。
% MATLAB 当前目录优先于 addpath：防止旧 LMPC 目录的同名参数遮盖本版。
here=fileparts(mfilename('fullpath'));
previousDir=pwd;
restoreDir=onCleanup(@()cd(previousDir)); %#ok<NASGU>
cd(here);
p=lmpc_parameters();
assert(all(isfinite([p.body_mass_kg p.body_length_m p.body_height_m p.body_width_m])) && ...
 all([p.body_mass_kg p.body_length_m p.body_height_m p.body_width_m]>0));
mpcverbosity('off');
[controller,plant,K]=make_lmpc(p);
assignin('base','lmpc_config',p);
assignin('base','wheel_leg_lmpc',controller);
assignin('base','lmpc_plant',plant);
assignin('base','lmpc_lqr_gain',K);
% Terminal outputs must track chol(P)*reference too.
assignin('base','lmpc_reference',controller.Model.Plant.C*p.reference);
r=roll_parameters();
assert(any(r.enable==[0 1]) && isfinite(r.target_deg));
assert(all(isfinite([r.kp_n_rad r.kd_n_s_rad r.rate_filter_tau_s r.force_limit_n])));
assert(r.kp_n_rad>=0 && r.kd_n_s_rad>=0 && r.rate_filter_tau_s>0 && r.force_limit_n>0);
assignin('base','roll_config',r);
end

function [controller, plant, K] = make_lmpc(p)
% Linearization of leg_k_matrix's equations at upright and fixed length.
% Each controller represents one side (half body, one wheel).
% The source Simscape feedback uses +K*x; actuator coordinates are opposite
% the analytical T,Tp convention, hence the explicit minus sign on B.
L=p.leg_length_m/2; L0=p.leg_length_m; r=p.wheel_radius_m;
M=p.body_mass_kg/2; mp=p.leg_mass_kg/2; mw=p.wheel_mass_kg/2;
Iw=p.wheel_inertia/2; Ip=p.leg_inertia/2; Im=p.body_inertia/2;
E=[M+mp+mw+Iw/r^2, M*L0+mp*L, 0; ...
   M*L0+mp*L, Ip+mp*L^2+M*L0^2, 0; 0,0,Im];
gravity=E\[0;(mp*L+M*L0)*9.8;0];
actuation=-(E\[1/r,0;-1,1;0,1]);
A=zeros(6); A(1,2)=1; A(3,4)=1; A(5,6)=1;
A([4 2 6],1)=gravity;
B=zeros(6,2); B([4 2 6],:)=actuation;
plant=c2d(ss(A,B,eye(6),zeros(6,2)),p.Ts);
plant.StateName={'theta','dtheta','position','velocity','pitch','dpitch'};
plant.InputName={'wheel torque','virtual leg torque'};
Q=diag(p.Q); R=diag(p.R);
% Output-cost LQR Riccati terminal penalty follows MathWorks' MPC example.
[K,P]=lqry(plant,Q,R);
augmented=ss(plant.A,plant.B,[eye(6);chol(P)],zeros(12,2),p.Ts);
augmented=setmpcsignals(augmented,'MO',1:6,'UO',7:12);
controller=mpc(augmented,p.Ts,p.prediction_horizon,p.control_horizon);
controller.Weights.OV=[sqrt(p.Q),zeros(1,6)];
controller.Weights.MV=sqrt(p.R);
controller.Weights.MVRate=p.mv_rate_weight;
setterminal(controller,struct('Weight',[zeros(1,6),ones(1,6)]), ...
    struct('Weight',sqrt(p.R)));
setoutdist(controller,'model',ss(zeros(12,1)));
setEstimator(controller,[],eye(6)); % All six states are provided by Simscape.
for j=1:2
 controller.MV(j).Min=-p.torque_limit_nm(j);
 controller.MV(j).Max= p.torque_limit_nm(j);
 controller.MV(j).RateMin=-p.torque_step_nm(j);
 controller.MV(j).RateMax= p.torque_step_nm(j);
end
end
