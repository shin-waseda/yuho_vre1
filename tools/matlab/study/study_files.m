function files = study_files(kind)
%STUDY_FILES 演習で使う既定のログ(右の車軸を締め直した後．2026-10-08 の探索のログ．速さ 300〜600mm/s)。
%   study_files()          同定に使うログ(search_0015〜0029)
%   study_files('check')   確かめ用のログ(search_0030〜0031．同定には使わない)
if nargin >= 1 && strcmp(kind, 'check')
    nums = 30:31;
else
    nums = 15:29;
end
files = {};
for n = nums
    f = fullfile(yc.root(), 'logs', 'search', sprintf('search_%04d.bin', n));
    if isfile(f), files{end+1} = f; end %#ok<AGROW>
end
end
