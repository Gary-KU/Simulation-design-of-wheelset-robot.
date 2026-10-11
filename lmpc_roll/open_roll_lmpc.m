function open_roll_lmpc()
%OPEN_ROLL_LMPC Load parameters, compile the model and enable the 3-D viewer.
here=fileparts(mfilename('fullpath'));
addpath(here,'-begin');
cd(here);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'.cache'), ...
    'CodeGenFolder',fullfile(here,'.cache','codegen'),'createDir',true);
init_roll_lmpc();
mdl='Self_Model_3D_LMPC_Roll';
open_system(fullfile(here,[mdl '.slx']));
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','on');
set_param(mdl,'SimulationCommand','update');
fprintf('LMPC + roll model: %s\n',get_param(mdl,'FileName'));
end
