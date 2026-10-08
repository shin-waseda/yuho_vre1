function [idx, t, txt] = find_events(L, name)
%FIND_EVENTS ev_text のある行を返す(name を付けたら、その名前のものだけ)。
%   [idx, t, txt] = yc.find_events(L, 'STEP')   idx: 行の番号(1 から), t: 時刻, txt: 文字(string)
idx = zeros(0, 1); t = zeros(0, 1); txt = strings(0, 1);
if ~isfield(L, 'ev_text')
    return
end
s = L.ev_text;
ok = s ~= "";
if nargin >= 2 && ~isempty(name)
    ok = ok & (extractBefore(s + " ", " ") == string(name));
end
idx = find(ok);
t = L.time_s(idx);
txt = s(idx);
end
