function cfg = sf_config()
% sf_config  同定のスクリプトの共通の設定
%
% ログは yuho-sysid の worktree(feature/system-identification)にある．

here = fileparts(mfilename('fullpath'));
cfg.logDir = fullfile(here, '..', '..', '..', '..', 'yuho-sysid', 'logs', 'sysid');
cfg.figDir = fullfile(here, '..', '..', '..', 'docs', 'mathworks_award', 'fig');
cfg.outDir = fullfile(here, 'out');
cfg.pwmMax = 4199;          % TIM2 の周期(duty = pwm / 4199)
cfg.dt     = 1e-3;          % ログの周期 [s]

% 比べるための，前の(Python で求めた)値．docs/yuho_model.md
cfg.prev.a = 0.561;  cfg.prev.V0 = 0.210;  cfg.prev.b = 0.32;
cfg.prev.bw = 9.20e-3;  cfg.prev.sigma = 3.02e-3;
cfg.prev.Ap = 0.32;  cfg.prev.Am = 0.216;  cfg.prev.D = -0.128;
cfg.prev.k0v = [0.3 0.5 0.75 1.0 1.5];
cfg.prev.k0  = [1.03 1.14 0.80 0.89 0.92];

% 図の色
cfg.c.blue   = [0.165 0.435 0.859];
cfg.c.orange = [0.851 0.510 0.169];
cfg.c.grey   = [0.60 0.60 0.60];
cfg.c.dark   = [0.27 0.27 0.27];

if ~exist(cfg.figDir, 'dir'), mkdir(cfg.figDir); end
if ~exist(cfg.outDir, 'dir'), mkdir(cfg.outDir); end
end
