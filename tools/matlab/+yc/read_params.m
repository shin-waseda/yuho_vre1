function p = read_params(path)
%READ_PARAMS Core/Inc/params.h の #define を読み、数の値にして struct で返す。
%   他の名前を使った式も計算する(yuho_common.read_params と同じ)。数でない定義(配列など)は飛ばす。
%   p = yc.read_params();  p.SLALOM_V_MM_S
if nargin < 1
    path = fullfile(yc.root(), 'Core', 'Inc', 'params.h');
end
lines = splitlines(fileread(path, 'Encoding', 'UTF-8'));
raw = containers.Map();
for k = 1:numel(lines)
    tok = regexp(lines{k}, '^\s*#define\s+([A-Za-z_]\w*)\s+(.+?)\s*(?://.*)?$', 'tokens', 'once');
    if ~isempty(tok)
        raw(tok{1}) = tok{2};
    end
end

values = containers.Map();
names = keys(raw);
for k = 1:numel(names)
    try
        evaluate(names{k}, 0);
    catch
        % 数でない定義は飛ばす
    end
end

p = struct();
vn = keys(values);
for k = 1:numel(vn)
    p.(vn{k}) = values(vn{k});
end

    function val = evaluate(name, depth)
        if isKey(values, name)
            val = values(name);
            return
        end
        if ~isKey(raw, name) || depth > 20
            error('read_params:unknown', '%s', name);
        end
        expr = raw(name);
        expr = regexprep(expr, '\(\s*(?:float|uint32_t|int|uint16_t|uint8_t|int16_t)\s*\)', '');
        expr = regexprep(expr, '(\d+\.?\d*(?:[eE][-+]?\d+)?)[fFuUlL]+(?!\w)', '$1');
        % 名前を値に置き換える(数の中の e や 0x の x は前に文字があるので名前にならない)
        [s, e] = regexp(expr, '(?<![\w.])[A-Za-z_]\w*', 'start', 'end');
        out = '';
        last = 1;
        for j = 1:numel(s)
            v = evaluate(expr(s(j):e(j)), depth + 1);
            out = [out, expr(last:s(j)-1), '(', sprintf('%.17g', v), ')']; %#ok<AGROW>
            last = e(j) + 1;
        end
        expr = [out, expr(last:end)];
        if isempty(regexp(expr, '^[\d\s.eExXa-fA-F+\-*/()]+$', 'once'))
            error('read_params:expr', '%s', expr);
        end
        val = double(eval(expr));
        if ~isscalar(val) || ~isfinite(val)
            error('read_params:expr', '%s', expr);
        end
        values(name) = val;
    end
end
