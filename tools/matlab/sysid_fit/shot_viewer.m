function shot_viewer(logPath, outName, sz)
% shot_viewer  ログビューアを開いて，画面を PNG に書き出す(応募資料の図のため)
%
%   shot_viewer('logs/search/fast_0083.bin', 'viewer.png')

cfg = sf_config();
here = fileparts(mfilename('fullpath'));
repo = fullfile(here, '..', '..', '..');
addpath(fullfile(here, '..'));
addpath('M:/User/Desktop/school/club/log-viewer/toolbox');
if nargin < 3, sz = [1600 900]; end
mlv.add_profile('yuho', 'yuho_viewer_config', fullfile(here, '..'));
if ~isfile(logPath), logPath = fullfile(repo, logPath); end
fig = mouse_log_viewer(logPath, 'yuho');
fig.Position(3:4) = sz;
drawnow; pause(8); drawnow;
exportapp(fig, fullfile(cfg.figDir, outName));
close(fig);
end
