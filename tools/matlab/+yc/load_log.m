function [L, names] = load_log(path)
%LOAD_LOG get_log.py が作った CSV か、SD の .bin(YLOG1 / YLOG2)を読む。
%   [L, names] = yc.load_log('logs/search/search_0008.bin')
%   L: 列名ごとの列ベクトル(double)の struct。イベントの列(ev)があれば L.ev_text(string)も付ける。
%   names: 列の順番(cellstr。ev_text も含む)
path = char(path);
[~, ~, ext] = fileparts(path);
if strcmpi(ext, '.bin')
    [cols, M] = parse_ylog(path);
    L = struct();
    for k = 1:numel(cols)
        L.(cols{k}) = M(:, k);
    end
    names = cols;
    texts = event_texts(cols, M);
    if ~isempty(texts)
        L.ev_text = texts;
        names{end+1} = 'ev_text';
    end
else
    opts = detectImportOptions(path, 'FileType', 'text', 'Delimiter', ',', ...
        'VariableNamingRule', 'preserve', 'TextType', 'string');
    names = opts.VariableNames;
    num = setdiff(names, {'ev_text'}, 'stable');
    opts = setvartype(opts, num, 'double');
    if any(strcmp(names, 'ev_text'))
        opts = setvartype(opts, 'ev_text', 'string');
    end
    T = readtable(path, opts);
    L = struct();
    for k = 1:numel(names)
        v = T.(names{k});
        if isstring(v)
            v(ismissing(v)) = "";
        end
        L.(names{k}) = v;
    end
end
end


function [names, M] = parse_ylog(path)
% get_log.parse_ylog と同じ。YLOG1 は最後の途中で切れた行を捨てる。
% YLOG2 は通し番号が飛んだ・ブロックが壊れていたら、そこで止める。
fid = fopen(path, 'r');
if fid < 0
    error('load_log:open', '%s を開けない', path);
end
data = fread(fid, inf, '*uint8')';
fclose(fid);
magic = char(data(1:min(5, end)));
nl = find(data == 10);
if strcmp(magic, 'YLOG1')
    if numel(nl) < 3
        error('load_log:ylog', 'broken YLOG1 header');
    end
    count = str2double(char(data(nl(1)+1:nl(2)-1)));
    names = strsplit(strtrim(char(data(nl(2)+1:nl(3)-1))), ',');
    if numel(names) ~= count
        error('load_log:ylog', 'header says %d columns but has %d names', count, numel(names));
    end
    body = data(nl(3)+1:end);
    n_rows = floor(numel(body) / (4 * count));
    v = typecast(body(1:4 * count * n_rows), 'single');
    M = double(reshape(v, count, n_rows)');
elseif strcmp(magic, 'YLOG2')
    HEADER = 512;
    MAGIC = uint32(hex2dec('4B4C4259'));  % "YBLK"
    nl = nl(nl <= HEADER);
    count = str2double(char(data(nl(1)+1:nl(2)-1)));
    names = strsplit(strtrim(char(data(nl(2)+1:nl(3)-1))), ',');
    block_bytes = str2double(strtrim(char(data(nl(3)+1:nl(4)-1))));
    if numel(names) ~= count
        error('load_log:ylog', 'header says %d columns but has %d names', count, numel(names));
    end
    chunks = {};
    pos = HEADER;  % 0 から数えた位置
    seq = 0;
    while pos + 16 <= numel(data)
        h = typecast(data(pos+1:pos+16), 'uint32');
        if h(1) ~= MAGIC || h(2) ~= seq || h(4) ~= count
            break  % 書かれていない所 (または壊れたブロック)
        end
        n_rows = double(h(3));
        nb = 4 * count * n_rows;
        if pos + 16 + nb > numel(data)
            break
        end
        v = typecast(data(pos+17:pos+16+nb), 'single');
        chunks{end+1} = double(reshape(v, count, n_rows)'); %#ok<AGROW>
        pos = pos + block_bytes;
        seq = seq + 1;
    end
    M = vertcat(chunks{:});
    if isempty(M)
        M = zeros(0, count);
    end
else
    error('load_log:ylog', 'not a YLOG1/YLOG2 file');
end
names = cellfun(@strtrim, names, 'UniformOutput', false);
end


function texts = event_texts(names, M)
% get_log.event_texts と同じ。"ev" の列があれば、行ごとに "名前 中身=値 ..." の文字にする(なければ空)。
texts = [];
iev = find(strcmp(names, 'ev'), 1);
if isempty(iev)
    return
end
tbl = load_event_table();
slots = {'ev_a', 'ev_b', 'ev_c', 'ev_d', 'ev_e'};
texts = strings(size(M, 1), 1);
codes = round(M(:, iev));
rows = find(codes ~= 0);
for r = rows'
    c = codes(r);
    if isKey(tbl, c)
        ent = tbl(c);
        name = ent{1};
        labels = ent{2};
    else
        name = sprintf('EV%d', c);
        labels = [slots; slots]';
    end
    s = name;
    for j = 1:size(labels, 1)
        col = find(strcmp(names, labels{j, 1}), 1);
        if ~isempty(col)
            s = [s, ' ', labels{j, 2}, '=', sprintf('%.6g', M(r, col))]; %#ok<AGROW>
        end
    end
    texts(r) = string(s);
end
end


function tbl = load_event_table()
% 機体の app/log_event.h の「LOG_EV_名前 = 番号, // a:名前 b:名前 ...」の行を読む。
% tbl: 番号 → {名前, {列, 中身の名前; ...}}
persistent cache cache_time
header = fullfile(yc.root(), 'Core', 'Inc', 'app', 'log_event.h');
tbl = containers.Map('KeyType', 'double', 'ValueType', 'any');
if ~isfile(header)
    return
end
d = dir(header);
if ~isempty(cache) && isequal(cache_time, d.datenum)
    tbl = cache;
    return
end
lines = splitlines(fileread(header, 'Encoding', 'UTF-8'));
for k = 1:numel(lines)
    tok = regexp(lines{k}, 'LOG_EV_(\w+)\s*=\s*(\d+)\s*,', 'tokens', 'once');
    if isempty(tok)
        continue
    end
    labels = cell(0, 2);
    c = strfind(lines{k}, '//');
    if ~isempty(c)
        parts = strsplit(strtrim(lines{k}(c(1)+2:end)));
        for j = 1:numel(parts)
            t = parts{j};
            if numel(t) > 2 && t(2) == ':' && any(t(1) == 'abcde')
                labels(end+1, :) = {['ev_', t(1)], t(3:end)}; %#ok<AGROW>
            end
        end
    end
    tbl(str2double(tok{2})) = {tok{1}, labels};
end
cache = tbl;
cache_time = d.datenum;
end
