function p=lqr_model_parameters()
% Confirmed whole-robot and non-body mass. Wheel/leg split is estimated.
p.total_mass_kg=1.450;
p.non_body_mass_kg=0.350;
p.body_mass_kg=p.total_mass_kg-p.non_body_mass_kg;
p.body_length_m=0.120;
p.body_width_m=0.150;
p.body_height_m=0.050;
p.wheel_radius_m=0.026;
p.wheel_width_m=0.024;
p.plate_thickness_m=0.007;
p.plate_height_m=0.020;
p.upper_link_m=0.050;
p.lower_link_m=0.105;
p.hip_gap_m=0.060;
p.leg_length_m=0.100;
p.wheel_torque_limit_nm=0.030;
p.joint_torque_limit_nm=0.680;
p.Ts=0.005;
p.length_min_m=0.040;
p.length_max_m=0.140;
p.Q=[400 15 20 10 200 10];
p.R=[1 0.5];
p.support_kp=100;
p.support_ki=50;
p.support_kd=20;
p.gravity_m_s2=9.80665;
% Preserve the former wheel/leg mass ratio; normalize it to the stated 350 g.
% Replace this allocation with separately measured masses when available.
wheel_volume=pi*p.wheel_radius_m^2*p.wheel_width_m;
leg_volume=2*(p.upper_link_m+p.lower_link_m)*p.plate_height_m*p.plate_thickness_m;
reference_wheel_mass=5000*wheel_volume;
reference_leg_mass=800*leg_volume;
p.non_body_mass_scale=p.non_body_mass_kg/(2*(reference_wheel_mass+reference_leg_mass));
p.wheel_density_kg_m3=5000*p.non_body_mass_scale;
p.plate_density_kg_m3=800*p.non_body_mass_scale;
p.wheel_mass_per_side_kg=p.wheel_density_kg_m3*wheel_volume;
p.leg_mass_per_side_kg=p.plate_density_kg_m3*leg_volume;
p.wheel_inertia_per_side=p.wheel_mass_per_side_kg*p.wheel_radius_m^2/2;
mu=p.upper_link_m*p.plate_height_m*p.plate_thickness_m*p.plate_density_kg_m3;
ml=p.lower_link_m*p.plate_height_m*p.plate_thickness_m*p.plate_density_kg_m3;
p.leg_inertia_per_side=2*(mu*(p.upper_link_m^2+p.plate_height_m^2)/12+ml*(p.lower_link_m^2+p.plate_height_m^2)/12);
p.body_pitch_inertia_per_side=p.body_mass_kg*(p.body_length_m^2+p.body_height_m^2)/24;
p.support_feedforward_n=(p.body_mass_kg/2+p.leg_mass_per_side_kg)*p.gravity_m_s2;
assert(p.body_mass_kg>0);
assert(abs(p.body_mass_kg+2*p.wheel_mass_per_side_kg+2*p.leg_mass_per_side_kg-p.total_mass_kg)<1e-12);
% Outward assembly at 100 mm leg length, above the 25 mm-high ground top.
a=p.upper_link_m;b=p.lower_link_m;halfgap=p.hip_gap_m/2;len=p.leg_length_m;
alpha=fzero(@(v)a*sin(v)+sqrt(b*b-(halfgap+a*cos(v))^2)-len,[0 pi/2]);
p.phi1_initial_deg=(pi+alpha)*180/pi;
p.phi4_initial_deg=-alpha*180/pi;
p.passive_initial_deg=(atan2(-len+a*sin(alpha),-halfgap-a*cos(alpha))+alpha)*180/pi;
p.body_initial_height_m=0.025+p.wheel_radius_m+p.leg_length_m+0.010;
end
