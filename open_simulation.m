function open_simulation(controller)
%OPEN_SIMULATION Open one controller from any current directory.
% Call this from the repository root, or first add the root to the path.
controller=validatestring(controller,{'lqr','lmpc_roll'});
check_environment(controller);
root=fileparts(mfilename('fullpath'));
selected=fullfile(root,controller);
addpath(root,selected,'-begin');
% Current-folder functions outrank addpath. Stay in the selected directory
% so later model callbacks also use this package when Run is clicked.
cd(selected);
if strcmp(controller,'lqr')
    open_lqr();
else
    open_roll_lmpc();
end
end
