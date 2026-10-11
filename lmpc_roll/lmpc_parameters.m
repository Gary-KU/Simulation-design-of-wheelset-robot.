function p = lmpc_parameters()
% LMPC 仿真调参入口。机身质量及轮/关节电机上限按用户指定值设置。
p.Ts = 0.005;
p.prediction_horizon = 20;
p.control_horizon = 20;
p.leg_length_m = 0.10; % Same as the source 3-D support controller.
p.wheel_radius_m = 0.02; % Source Simscape cylinder radius, NOT firmware 0.026.
% Whole robot values from leg_k_matrix, with wheel geometry corrected.
p.body_mass_kg = 1.45; % 用户提供：机身质量，不含仍沿用原模型参数的轮腿
p.body_length_m = 0.120;
p.body_height_m = 0.050;
p.body_width_m = 0.100; % 横向宽度，同时是左右腿安装间距
p.leg_mass_kg = 0.0248;
p.support_feedforward_n = (p.body_mass_kg+p.leg_mass_kg)*9.8/2; % 每腿承载机身与腿，轮重由地面承担
p.joint_torque_limit_nm = 0.68; % 用户指定：每个腿关节电机最大扭矩 N*m
p.support_kp_n_m = 100;
p.support_ki_n_m_s = 50; % 重力前馈已提供静态承载，积分只消除残差
p.support_kd_n_s_m = 20; % 机身垂向阻尼
p.wheel_mass_kg = 2*pi*0.02^2*0.02*5000;
p.wheel_inertia = p.wheel_mass_kg*p.wheel_radius_m^2/2;
p.leg_inertia = (9.66667e-7+7.9975e-6)*2;
p.body_inertia = p.body_mass_kg*(p.body_length_m^2+p.body_height_m^2)/12;
p.Q = [400 15 20 10 200 10]; % 腿角、腿角速度、位置、速度、俯仰、俯仰角速度
p.R = [1 0.5]; % 轮力矩、虚拟腿力矩的代价；越大越不愿意用力
p.mv_rate_weight = [0.02 0.02]; % 力矩变化惩罚；过大可能响应慢
p.torque_limit_nm = [0.03 0.12]; % Per side: [wheel, virtual leg].
p.torque_step_nm = [0.03 0.03]; % Per control sample, not Nm/s.
p.reference = zeros(6,1); % theta,dtheta,position,velocity,pitch,dpitch
end
