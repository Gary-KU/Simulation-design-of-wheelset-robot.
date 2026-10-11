function report = check_environment(controller)
%CHECK_ENVIRONMENT Check installed products for the selected simulation.
if nargin<1,controller='all';end
controller=validatestring(controller,{'lqr','lmpc_roll','all'});
required={'MATLAB','Simulink','Simscape','Simscape Multibody','Control System Toolbox'};
if ~strcmp(controller,'lqr')
    required{end+1}='Model Predictive Control Toolbox';
end
installed=ver;
missing=setdiff(required,{installed.Name},'stable');
assert(isempty(missing),'wheelleg:MissingProducts', ...
    'Required MATLAB products are missing: %s',strjoin(missing,', '));
report=struct('matlab_release',version('-release'),'controller',controller, ...
    'required_products',{required},'platform',computer,'checked_at',char(datetime('now')));
fprintf('Environment: MATLAB %s / %s; %s products installed.\n', ...
    report.matlab_release,report.platform,controller);
% Availability of a product does not guarantee a license checkout. Model
% loading and simulation will report any checkout failures themselves.
end
