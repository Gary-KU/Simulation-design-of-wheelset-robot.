function r=roll_parameters()
% 横滚调参入口：只用于仿真，力修正作用在 VMC 前。
r.enable=1;
r.target_deg=0;
r.kp_n_rad=6;            % 每侧支撑力修正 / 横滚误差
r.kd_n_s_rad=5.5;        % 横滚角速度阻尼，优先抑制回正后的反复摆动
r.rate_filter_tau_s=0.002;
r.force_limit_n=2;       % 每侧最多加/减 2 N，左右修正之和为零
r.initial_roll_deg=0;    % 改成 +3 或 -3 可看侧倾恢复动画
end
