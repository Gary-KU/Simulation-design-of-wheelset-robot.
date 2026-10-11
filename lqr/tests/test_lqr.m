function results = test_lqr(quick)
%TEST_LQR Reproduce standing and +/-10 degree pitch recovery in Simscape.
% Overrides are in memory only; no source model is saved or replaced.
if nargin<1,quick=false;end
here=fileparts(fileparts(mfilename('fullpath')));
addpath(here,'-begin');
output=fullfile(here,'results');if ~isfolder(output),mkdir(output);end
previous=pwd;restore=onCleanup(@()cd(previous)); %#ok<NASGU>
cd(output);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'.cache'), ...
    'CodeGenFolder',fullfile(here,'.cache','codegen'),'createDir',true);
mdl='Self_Model_3D';
assert(~bdIsLoaded(mdl),'Close the LQR model before running batch tests.');
cleanup=onCleanup(@()close_if_loaded(mdl)); %#ok<NASGU>
cases={'standing',0,5;'pitch_pos_10',10,8;'pitch_neg_10',-10,8};
if quick,cases=cases(1,:);end
results=struct();
for k=1:size(cases,1)
    load_system(fullfile(here,[mdl '.slx']));
    set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off','SignalLogging','on');
    set_param(block(mdl,9),'PositionTargetValue',num2str(cases{k,2}));
    for sid=[231 232 239 240 277]
        port=get_param(block(mdl,sid),'PortHandles');
        set_param(port.Outport,'DataLogging','on','DataLoggingNameMode','Custom', ...
            'DataLoggingName',sprintf('motor_%d',sid));
    end
    right=[get_param(block(mdl,335),'Parent') '/Wheel motor torque limit'];
    port=get_param(right,'PortHandles');
    set_param(port.Outport,'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName','motor_right_wheel');
    tic;
    out=sim(mdl,'StopTime',num2str(cases{k,3}),'CaptureErrors','on', ...
        'ReturnWorkspaceOutputs','on');
    elapsed=toc;
    p=lqr_model_parameters();
    assert(isempty(out.ErrorMessage),out.ErrorMessage);
    names={'pitch','roll','thetaL','thetaR','lengthL','lengthR','velocity', ...
        'motor_231','motor_232','motor_239','motor_240','motor_277','motor_right_wheel'};
    d=struct();
    for j=1:numel(names)
        signal=out.logsout.get(names{j}).Values;
        d.(names{j})=struct('t',signal.Time,'x',double(signal.Data(:)));
        assert(all(isfinite(d.(names{j}).x)),'Non-finite logged signal: %s',names{j});
    end
    d.elapsed_wall_s=elapsed;
    d.wheel_peak_nm=max([abs(d.motor_277.x);abs(d.motor_right_wheel.x)]);
    d.joint_peak_nm=max([abs(d.motor_231.x);abs(d.motor_232.x);abs(d.motor_239.x);abs(d.motor_240.x)]);
    d.pitch_final_deg=d.pitch.x(end)*180/pi;
    d.speed_final_m_s=d.velocity.x(end);
    d.leg_length_final_m=[d.lengthL.x(end) d.lengthR.x(end)];
    d.pitch_peak_last_s_deg=max(abs(d.pitch.x(d.pitch.t>=cases{k,3}-1)))*180/pi;
    d.speed_peak_last_s_m_s=max(abs(d.velocity.x(d.velocity.t>=cases{k,3}-1)));
    d.recovered=d.pitch_peak_last_s_deg<1 && d.speed_peak_last_s_m_s<.005 && ...
        all(abs(d.leg_length_final_m-p.leg_length_m)<.005);
    results.(cases{k,1})=d;
    save('lqr_results.mat','results','p');
    assert(d.wheel_peak_nm<=p.wheel_torque_limit_nm+1e-12,'Wheel torque exceeded limit');
    assert(d.joint_peak_nm<=p.joint_torque_limit_nm+1e-12,'Joint torque exceeded limit');
    assert(d.recovered,'Recovery failed: %s',cases{k,1});
    fprintf('LQR %s: PASS pitch=%.6f deg speed=%.6f m/s wheel=%.6f Nm joint=%.6f Nm\n', ...
        cases{k,1},d.pitch_final_deg,d.speed_final_m_s,d.wheel_peak_nm,d.joint_peak_nm);
    close_system(mdl,0);
end
f=figure('Visible','off','Position',[100 100 1000 650]);
figureCleanup=onCleanup(@()close(f)); %#ok<NASGU>
tiledlayout(2,2);
for j=1:4
    nexttile;hold on;
    for k=1:size(cases,1)
        d=results.(cases{k,1});
        if j==1,s=d.pitch;v=s.x*180/pi;label='Pitch (deg)';
        elseif j==2,s=d.velocity;v=s.x;label='Forward speed (m/s)';
        elseif j==3,s=d.lengthL;v=s.x;label='Left leg length (m)';
        else,s=d.motor_277;v=s.x;label='Left wheel torque (Nm)';end
        plot(s.t,v,'DisplayName',strrep(cases{k,1},'_',' '));
    end
    grid on;xlabel('Time (s)');ylabel(label);
    if j==1,legend('Location','best');end
end
exportgraphics(f,'lqr_validation.png');
fprintf('PASS LQR standing, pitch recovery, support height and actuator limits.\n');
end
function path=block(mdl,sid)
path=Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid));
end
function close_if_loaded(mdl)
if bdIsLoaded(mdl),close_system(mdl,0);end
end
