function reports = run_validation(controller)
%RUN_VALIDATION Run the release tests using only repository files.
if nargin<1,controller='all';end
controller=validatestring(controller,{'lqr','lmpc_roll','all'});
environment=check_environment(controller);
root=fileparts(mfilename('fullpath'));
previous=pwd;restore=onCleanup(@()cd(previous));
assert(~bdIsLoaded('Self_Model_3D') && ~bdIsLoaded('Self_Model_3D_LMPC_Roll'), ...
    'Close both main models first; batch tests do not discard open model edits.');
reports=struct();
if any(strcmp(controller,{'lqr','all'}))
    here=fullfile(root,'lqr');addpath(here,fullfile(here,'tests'),'-begin');
    cd(here);
    data=test_lqr();
    summary=struct();names=fieldnames(data);
    for k=1:numel(names)
        d=data.(names{k});
        summary.(names{k})=struct('passed',d.recovered,'pitch_final_deg',d.pitch_final_deg, ...
            'speed_final_m_s',d.speed_final_m_s,'leg_length_final_m',d.leg_length_final_m, ...
            'wheel_peak_nm',d.wheel_peak_nm,'joint_peak_nm',d.joint_peak_nm, ...
            'pitch_peak_last_s_deg',d.pitch_peak_last_s_deg, ...
            'speed_peak_last_s_m_s',d.speed_peak_last_s_m_s);
    end
    reports.lqr=struct('environment',environment,'parameters',lqr_model_parameters(), ...
        'max_linear_pole_radius',evalin('base','lqr_max_linear_pole_radius'), ...
        'cases',summary,'passed',true);
    write_report(fullfile(here,'results','validation.json'),reports.lqr);
end
if any(strcmp(controller,{'lmpc_roll','all'}))
    here=fullfile(root,'lmpc_roll');addpath(here,fullfile(here,'tests'),'-begin');
    cd(here);
    cases={'nominal',0,1,0,0;'braking',0,1,0.15,0;'pitch_3deg',0,1,0,3; ...
        'combined_pos',3,1,0.15,3;'combined_neg',-3,1,0.15,-3};
    test_roll_lmpc(false,false,cases);
    saved=load(fullfile(here,'results','roll_results.mat'));
    summary=struct();names=fieldnames(saved.results);
    for k=1:numel(names)
        d=saved.results.(names{k});
        if startsWith(names{k},'combined_')
            assert(d.roll_opposite_peak_deg<0.3,'Excessive opposite-side roll overshoot');
            assert(d.roll_crossings_02deg<=1,'Repeated roll rocking beyond 0.2 degrees');
            assert(d.roll_settle_02deg_s<2,'Roll recovery exceeds 2 seconds');
        end
        summary.(names{k})=struct('passed',true,'roll_final_deg',d.roll_final_deg, ...
            'roll_rms_last_s_deg',d.roll_rms_last_s_deg,'pitch_final_deg',d.pitch_final_deg, ...
            'speed_final_m_s',d.speed_final_m_s,'leg_length_final_m',d.leg_length_final_m, ...
            'wheel_peak_nm',d.wheel_peak_nm,'joint_peak_nm',d.joint_peak_nm, ...
            'roll_opposite_peak_deg',d.roll_opposite_peak_deg, ...
            'roll_settle_02deg_s',d.roll_settle_02deg_s,'qp_ok',d.qp_ok);
    end
    reports.lmpc_roll=struct('environment',environment,'parameters',saved.p, ...
        'roll_parameters',saved.config,'cases',summary,'passed',true);
    write_report(fullfile(here,'results','validation.json'),reports.lmpc_roll);
end
fprintf('PASS release validation: %s.\n',controller);
end
function write_report(path,report)
fid=fopen(path,'w','n','UTF-8');assert(fid>=0,'Cannot write validation report');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
