function test_roll_lmpc(quick,combined,customCases)
if nargin<1,quick=false;end
if nargin<2,combined=false;end
if nargin<3,customCases={};end
here=fileparts(fileparts(mfilename('fullpath'))); addpath(here,'-begin');
output=fullfile(here,'results'); if ~isfolder(output),mkdir(output);end
previousDir=pwd; restoreDir=onCleanup(@()cd(previousDir)); %#ok<NASGU>
cd(output);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'.cache'), ...
 'CodeGenFolder',fullfile(here,'.cache','codegen'),'createDir',true);
mdl='Self_Model_3D_LMPC_Roll'; results=struct(); config=roll_parameters();
assert(~bdIsLoaded(mdl),'Close the roll model before running batch tests.');
cleanup=onCleanup(@()close_if_loaded(mdl)); %#ok<NASGU>
cases={'roll_off',3,0,0,0;'roll_pos',3,1,0,0;'roll_neg',-3,1,0,0; ...
 'nominal',0,1,0,0;'braking',0,1,0.15,0;'pitch_3deg',0,1,0,3};
if quick,cases={'nominal',0,1,0,0};end
if combined
 previous=load('roll_results.mat');
 assert(previous.validation_passed && isequal(previous.p,lmpc_parameters()) && isequal(previous.config,config));
 results.nominal=previous.results.nominal;
 cases={'combined_pos',3,1,0.15,3;'combined_neg',-3,1,0.15,-3};
end
if ~isempty(customCases),cases=customCases;end
for k=1:size(cases,1)
 load_system(fullfile(here,[mdl '.slx']));
 set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
 % Override blocks in memory only: delivered model keeps normal defaults.
 joint=Simulink.ID.getFullName([mdl ':12']);
 set_param(joint,'PositionTargetValue',num2str(cases{k,2}));
 set_param([mdl '/Roll PD support/Enable'],'Gain',num2str(cases{k,3}));
 set_param(Simulink.ID.getFullName([mdl ':9']),'PositionTargetValue',num2str(cases{k,5}));
 rp=get_param(Simulink.ID.getFullName([mdl ':208']),'PortHandles');
 set_param(rp.Outport(12),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','leg_length_left');
 set_param(rp.Outport(4),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','leg_length_right');
 for sid=[231 232 239 240]
  hp=get_param(Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid)),'PortHandles');
  set_param(hp.Outport,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',sprintf('joint_%d',sid));
 end
 if cases{k,4}~=0
  b=Simulink.ID.getFullName([mdl ':8']);
  set_param(b,'PxVelocityTargetSpecify','on','PxVelocityTargetPriority','High', ...
    'PxVelocityTargetValue',num2str(cases{k,4}),'PxVelocityTargetValueUnits','m/s');
 end
 tic; out=sim(mdl,'StopTime','5','CaptureErrors','on'); elapsed=toc;
 if ~isempty(out.ErrorMessage)
  save(['failed_' cases{k,1} '.mat'],'out');
  for nm={'leg_length_left','leg_length_right','lmpc_state_320','support_left_n','support_right_n'}
   sg=out.logsout.get(nm{1}).Values;
   fprintf('FAILED %s at t=%.6f last=',nm{1},sg.Time(end));disp(squeeze(sg.Data(end,:,:)));
  end
  error('%s',out.ErrorMessage);
 end
 data=struct(); names={'roll_rad','roll_rate_rad_s','roll_force_delta_n', ...
  'support_left_n','support_right_n','lmpc_state_320','lmpc_qp_320','lmpc_qp_328', ...
  'lmpc_torque_320','lmpc_torque_328','wheel_actual_320','wheel_actual_328', ...
  'leg_length_left','leg_length_right','joint_231','joint_232','joint_239','joint_240'};
 for j=1:numel(names)
  s=out.logsout.get(names{j}).Values; data.(names{j})=struct('t',s.Time,'x',squeeze(s.Data));
 end
 roll=data.roll_rad; rate=data.roll_rate_rad_s; x=data.lmpc_state_320.x;
 tail=roll.t>=4;
 data.roll_rms_last_s_deg=sqrt(mean(roll.x(tail).^2))*180/pi;
 data.roll_final_deg=roll.x(end)*180/pi;
 data.roll_peak_deg=max(abs(roll.x))*180/pi;
 data.roll_iae_deg_s=trapz(roll.t,abs(roll.x))*180/pi;
 data.roll_opposite_peak_deg=max(0,max(-sign(cases{k,2})*roll.x*180/pi));
 outside=find(abs(roll.x*180/pi)>0.2,1,'last');
 if isempty(outside),data.roll_settle_02deg_s=0;
 elseif outside==numel(roll.t),data.roll_settle_02deg_s=Inf;
 else,data.roll_settle_02deg_s=roll.t(outside+1);end
 significant=sign(roll.x(abs(roll.x*180/pi)>0.2));
 data.roll_crossings_02deg=sum(abs(diff(significant))>0);
 data.position_final_m=x(end,3); data.speed_final_m_s=x(end,4);
 data.pitch_final_deg=x(end,5)*180/pi;
 data.max_correction_n=max(abs(data.roll_force_delta_n.x));
 data.qp_ok=all(data.lmpc_qp_320.x(:)>0) && all(data.lmpc_qp_328.x(:)>0);
 data.leg_length_final_m=[data.leg_length_left.x(end),data.leg_length_right.x(end)];
 data.wheel_peak_nm=[max(abs(data.wheel_actual_320.x(:))),max(abs(data.wheel_actual_328.x(:)))];
 data.joint_peak_nm=[max(abs(data.joint_231.x)),max(abs(data.joint_232.x)), ...
  max(abs(data.joint_239.x)),max(abs(data.joint_240.x))];
 tailLength=data.leg_length_left.x(data.leg_length_left.t>=4);
 data.leg_tail_range_m=[min(tailLength),max(tailLength)];
 results.(cases{k,1})=data;
 fprintf('%s seconds=%.1f roll_final=%.4f peak=%.3f rms_tail=%.4f IAE=%.4f deltaMax=%.3f x=%.5f v=%.5f pitch=%.4f qp=%d\n', ...
  cases{k,1},elapsed,data.roll_final_deg,data.roll_peak_deg,data.roll_rms_last_s_deg, ...
  data.roll_iae_deg_s,data.max_correction_n,data.position_final_m,data.speed_final_m_s,data.pitch_final_deg,data.qp_ok);
 fprintf('leg lengths final = %.5f %.5f m\n',data.leg_length_final_m);
 fprintf('joint torque peaks Nm = ');disp(data.joint_peak_nm);
 fprintf('wheel torque peaks Nm = ');disp(data.wheel_peak_nm);
 fprintf('left leg last-second range m = ');disp(data.leg_tail_range_m);
 fprintf('roll opposite peak=%.4f deg, settle within 0.2deg=%.3fs, significant crossings=%d\n', ...
 data.roll_opposite_peak_deg,data.roll_settle_02deg_s,data.roll_crossings_02deg);
 save('roll_results.mat','results');
 close_system(mdl,0);
end
f=figure('Visible','off','Position',[100 100 1100 680]); tiledlayout(2,2);
labels=cases(:,1);
for j=1:4
 nexttile; hold on;
 for k=1:numel(labels)
  d=results.(labels{k});
  if j==1,s=d.roll_rad; v=s.x*180/pi; ttl='Roll (deg)';
  elseif j==2,s=d.roll_force_delta_n;v=s.x;ttl='Per-side force correction (N)';
  elseif j==3,s=d.lmpc_state_320;v=s.x(:,5)*180/pi;ttl='Pitch (deg)';
  else,s=d.lmpc_state_320;v=s.x(:,4);ttl='Forward speed (m/s)';end
  plot(s.t,v,'DisplayName',strrep(labels{k},'_',' '));
 end
 grid on; xlabel('Time (s)');ylabel(ttl); if j==1,legend('Location','best');end
end
exportgraphics(f,'roll_comparison.png');close(f);
for k=1:size(cases,1)
 if strcmp(labels{k},'roll_off'),continue;end
 d=results.(labels{k});
 assert(d.qp_ok && d.max_correction_n<=config.force_limit_n+1e-7);
 assert(d.roll_rms_last_s_deg<0.5 && abs(d.pitch_final_deg)<0.5 && abs(d.speed_final_m_s)<0.005);
 p=lmpc_parameters(); assert(all(abs(d.leg_length_final_m-p.leg_length_m)<0.005),'Support height did not recover');
 assert(all(d.wheel_peak_nm<=p.torque_limit_nm(1)+1e-7),'Wheel torque exceeded motor limit');
 assert(all(d.joint_peak_nm<=p.joint_torque_limit_nm+1e-7),'Joint torque exceeded motor limit');
 assert(diff(d.leg_tail_range_m)<0.002,'Support height is oscillating');
end
if ~quick && ~combined && isempty(customCases)
assert(results.roll_pos.roll_iae_deg_s<results.roll_off.roll_iae_deg_s,'Roll PD did not improve recovery');
end
if quick
 fprintf('PASS nominal support and balance.\n');
else
 fprintf('PASS roll recovery, forward balance, support height, correction bounds and MPC QP status.\n');
end
validation_passed=true; p=lmpc_parameters();
save('roll_results.mat','results','p','config','validation_passed');
end
function close_if_loaded(mdl)
if bdIsLoaded(mdl),close_system(mdl,0);end
end
