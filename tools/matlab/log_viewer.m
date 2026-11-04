function log_viewer(path)
%LOG_VIEWER yuho のログ(SD の .bin、get_log.py が作った .csv)を見る GUI。
%   本体は配布版のログビューア(log-viewer リポジトリの mouse_log_viewer)で、ここは yuho の設定
%   (yuho_viewer_config.m)を 'yuho' という名前で登録して、それで開くだけ。画面の使い方は mouse_log_viewer の説明を見る。
%   一度これを使うと登録が残るので、その後は mouse_log_viewer を直接開いても yuho の設定になる
%   (画面の上の「設定」で選べる。最後に使った設定で開く)。
%
%   log_viewer                                   % 起動するとログを選ぶ画面が出る
%   log_viewer('logs/search/search_0006.bin')    % 開くログを決めて起動(リポジトリの一番上から見たパスでもよい)
%
%   先に配布版を入れておく: log-viewer の .mltbx を入れる(ダブルクリック)か、
%   addpath('M:/User/Desktop/school/club/log-viewer/toolbox') する。
if exist('mouse_log_viewer', 'file') ~= 2
    error('log_viewer:missing', ['mouse_log_viewer が見つからない。log-viewer の .mltbx を入れるか、', ...
        'addpath(''<log-viewer>/toolbox'') してください。']);
end
if nargin < 1
    path = [];
end
mlv.add_profile('yuho', 'yuho_viewer_config', fileparts(mfilename('fullpath')));
mouse_log_viewer(path, 'yuho');
end
