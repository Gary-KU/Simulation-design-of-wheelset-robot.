function test_roll_smooth()
% 当前机身质量：站立 + 左右侧倾、俯仰、前进速度组合扰动。
test_roll_lmpc(false,false,{'nominal',0,1,0,0; ...
 'combined_pos',3,1,0.15,3;'combined_neg',-3,1,0.15,-3});
here=fileparts(fileparts(mfilename('fullpath')));
v=load(fullfile(here,'results','roll_results.mat'));
for name={'combined_pos','combined_neg'}
 d=v.results.(name{1});
 assert(d.roll_opposite_peak_deg<0.3,'Excessive opposite-side overshoot');
 assert(d.roll_crossings_02deg<=1,'Repeated roll rocking beyond 0.2 degrees');
 assert(d.roll_settle_02deg_s<2,'Roll recovery takes more than 2 seconds');
end
fprintf('PASS smooth roll recovery.\n');
end
