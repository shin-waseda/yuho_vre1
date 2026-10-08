classdef TurnLog
    %TURNLOG SLALOM テストの1回分(turn_sim.py の TurnLog と同じ)。
    %   左に曲がった形(右なら左右を反転)にそろえて持つ。
    %   lg = yc.TurnLog('logs/slalom/turn_0001.bin', yc.read_params());

    properties
        path
        name
        kind        % 's90' / 'l90' / 'l180'
        right       % 右に曲がったか
        i_turn0
        i_turn1
        peak_omega  % [dps]
        t
        x           % 車輪+ジャイロの軌道(左に曲がった形) [mm]
        y
        theta       % 左に曲がった形での機体の向き [deg]
        omega       % [dps]
        v           % [mm/s]
        ad_in       % 内(曲がる側)の横の壁センサー
        ad_out
        ref_in
        ref_out
        th_in
        th_out
        win         % 曲がった後、止まるまで(出口の直進)の行の番号
        n_out       % 出口の外向き
        u_out       % 出口の向き
        exit_pt
    end

    methods
        function obj = TurnLog(path, params)
            obj.path = char(path);
            [~, obj.name] = fileparts(obj.path);
            L = yc.load_log(obj.path);
            need = {'time_s', 'dist', 'angle', 'omega_ref', 'ad_l', 'ad_r'};
            for k = 1:numel(need)
                if ~isfield(L, need{k})
                    error('TurnLog:column', '%s: ''%s'' の列がない(SLALOM テストのログではない?)', obj.name, need{k});
                end
            end
            t = L.time_s; ang = L.angle; om_ref = L.omega_ref;
            turning = find(abs(om_ref) > 0.5);
            if isempty(turning)
                error('TurnLog:noturn', '%s: 曲がっていない', obj.name);
            end
            obj.right = ang(end) < 0.0;
            s = 1.0 - 2.0 * obj.right;
            obj.kind = obj.detect_kind(om_ref, ang, params);
            obj.i_turn0 = turning(1);
            obj.i_turn1 = turning(end);
            obj.peak_omega = max(abs(om_ref));

            kd = yc.turn_kinds().(obj.kind);
            Ll = L;
            Ll.angle = s * ang;
            [obj.x, obj.y] = yc.reconstruct_path(Ll, kd.start(1), kd.start(2), 90.0);
            obj.t = t;
            obj.theta = 90.0 + s * (ang - ang(1));
            if isfield(L, 'gyro_z')
                gz = L.gyro_z;
            else
                gz = gradient(ang, t);
            end
            obj.omega = s * gz;
            if isfield(L, 'vl') && isfield(L, 'vr')
                obj.v = 0.5 * (L.vl + L.vr);
            else
                obj.v = gradient(L.dist, t);
            end
            % 左に曲がった形では、外 = 右
            if obj.right
                obj.ad_in = L.ad_r; obj.ad_out = L.ad_l;
                obj.ref_in = params.WALL_REF_R; obj.ref_out = params.WALL_REF_L;
                obj.th_in = params.WALL_TH_R; obj.th_out = params.WALL_TH_L;
            else
                obj.ad_in = L.ad_l; obj.ad_out = L.ad_r;
                obj.ref_in = params.WALL_REF_L; obj.ref_out = params.WALL_REF_R;
                obj.th_in = params.WALL_TH_L; obj.th_out = params.WALL_TH_R;
            end
            if isfield(L, 'target')
                target = L.target;
            else
                target = obj.v;
            end
            after = (obj.i_turn1 + 1:numel(t))';
            obj.win = after(target(after) > 1.0);

            obj.exit_pt = kd.exit;
            obj.u_out = kd.exit_dir;
            obj.n_out = [kd.exit_dir(2), -kd.exit_dir(1)];
        end

        function lat = kin_lateral(obj, idx)
            % 車輪とジャイロの軌道の、出口の線からの横のずれ(外が +)
            lat = (obj.x(idx) - obj.exit_pt(1)) * obj.n_out(1) + (obj.y(idx) - obj.exit_pt(2)) * obj.n_out(2);
        end

        function a = along_after_turn(obj, idx)
            a = (obj.x(idx) - obj.exit_pt(1)) * obj.u_out(1) + (obj.y(idx) - obj.exit_pt(2)) * obj.u_out(2);
        end

        function lat = sensor_lateral(obj, idx, slope)
            % 横の壁センサーで見た、出口の通路の真ん中からの横のずれ(外が +)。壁がない所は NaN。
            lat_out = (obj.ad_out(idx) - obj.ref_out) / slope;   % 外の壁に近いほど大きい
            lat_in = -(obj.ad_in(idx) - obj.ref_in) / slope;     % 内の壁に近いほど小さい
            a = lat_out; a(obj.ad_out(idx) <= obj.th_out) = NaN;
            b = lat_in;  b(obj.ad_in(idx) <= obj.th_in) = NaN;
            lat = mean([a(:), b(:)], 2, 'omitnan');  % 両方 NaN なら NaN
        end

        function out = slip_model(obj, Ks, C, idx)
            % ログの v と ω でスリップアングルを進めたときの、idx の行での横のずれ(外が +)。
            % Ks: K の配列。戻り値は numel(Ks) × numel(idx)。idx を省くと win。
            if nargin < 4
                idx = obj.win;
            end
            t = obj.t; v = obj.v;
            dt = [0; diff(t)];
            u = (v * 1e-3) .* deg2rad(obj.omega);  % β の目標 / K
            if C <= 0.0
                filt = u;
            else
                g = 1.0 - exp(-dt / C);
                filt = zeros(size(u));
                b = 0.0;
                for i = 1:numel(u)
                    b = b + g(i) * (u(i) - b);
                    filt(i) = b;
                end
            end
            th = deg2rad(obj.theta(:))';
            beta = Ks(:) * filt(:)';  % (nK, nT)
            % スリップがあるときと無いときの、1歩ごとの進み方の差(横の成分)
            step = (v(:) .* dt(:))';
            dlat = step .* ((cos(th - beta) - cos(th)) * obj.n_out(1) + (sin(th - beta) - sin(th)) * obj.n_out(2));
            cum = cumsum(dlat, 2);
            out = cum(:, idx);
        end
    end

    methods (Access = private)
        function kind = detect_kind(obj, om_ref, ang, params)
            tok = regexp(obj.name, '^(s90|l90|l180)[rl]', 'tokens', 'once');
            if ~isempty(tok)
                kind = tok{1};
                return
            end
            if abs(ang(end) - ang(1)) > 135.0
                kind = 'l180';
                return
            end
            peak = max(abs(om_ref));
            if abs(peak - params.SLALOM_OMEGA_DPS) <= abs(peak - params.FAST_LARGE90_OMEGA_DPS)
                kind = 's90';
            else
                kind = 'l90';
            end
        end
    end
end
