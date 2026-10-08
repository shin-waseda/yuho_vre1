function c = consts()
%CONSTS PC のツールで共通に使う定数(yuho_common.py と同じ値)
c.SECTION_MM = 180.0;
c.CONTROL_DT_S = 0.001;
c.PILLAR_HALF_MM = 6.0;        % 柱は 12mm 角
c.PROFILE_DECEL_MARGIN = 1.5;  % logic/control/velocity_profile.c と同じ
end
