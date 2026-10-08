function r = root()
%ROOT yuho のリポジトリの一番上のフォルダ(このファイルは tools/matlab/+yc/root.m)
r = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end
