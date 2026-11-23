function S = rot_collect(pattern, segNo)
% rot_collect  旋回の区間(seg = segNo)を集め，長さをそろえた行列にする(足りない所は NaN)
%
%   S = rot_collect('e1t3_*', 3)
%
% S.Vd，S.v，S.w : N×M．S.D，S.w0 : 1×M．S.name，S.vNom(名前の速さ [m/s])，S.dir(+1 左 / −1 右)

cfg = sf_config();
d = dir(fullfile(cfg.logDir, [pattern '.csv']));
runs = {};
for i = 1:numel(d)
    [~, nm] = fileparts(d(i).name);
    L = sf_load(nm);
    idx = find(L.seg == segNo);
    if numel(idx) < 50 || isnan(L.vd0), continue; end
    idx = idx(1):idx(end);
    r.name = nm;
    r.Vd = L.Vd(idx);  r.v = L.v(idx);  r.w = L.w(idx);
    r.D = L.vd0;
    tok = regexp(nm, '_(\d+)mms_', 'tokens', 'once');
    r.vNom = str2double(tok{1}) / 1000;
    r.dir = sign(mean(r.w));
    runs{end+1} = r; %#ok<AGROW>
end
M = numel(runs);
N = max(cellfun(@(r) numel(r.Vd), runs));
S.Vd = nan(N, M);  S.v = S.Vd;  S.w = S.Vd;
for j = 1:M
    n = numel(runs{j}.Vd);
    S.Vd(1:n, j) = runs{j}.Vd;  S.v(1:n, j) = runs{j}.v;  S.w(1:n, j) = runs{j}.w;
end
S.D    = cellfun(@(r) r.D, runs);
S.w0   = S.w(1, :);
S.name = cellfun(@(r) r.name, runs, 'UniformOutput', false);
S.vNom = cellfun(@(r) r.vNom, runs);
S.dir  = cellfun(@(r) r.dir, runs);
S.len  = cellfun(@(r) numel(r.Vd), runs);
end
