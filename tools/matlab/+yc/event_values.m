function v = event_values(text)
%EVENT_VALUES 'STEP x=1 y=0 heading=0 ...' → struct('x', 1, 'y', 0, ...)
v = struct();
parts = strsplit(strtrim(char(text)));
for k = 2:numel(parts)
    kv = strsplit(parts{k}, '=');
    if numel(kv) == 2 && isvarname(kv{1})
        x = str2double(kv{2});
        if ~isnan(x) || strcmpi(kv{2}, 'nan')
            v.(kv{1}) = x;
        end
    end
end
end
