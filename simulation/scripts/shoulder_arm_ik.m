function shoulder_arm_ik()
%SHOULDER_ARM_IK  5-DOF analytical inverse kinematics solver with GUI.
%
% Solves for joint angles given a desired end-effector position and
% orientation (ZYX Euler angles). Enumerates up to 4 candidate solutions,
% filters by joint limits, and ranks by orientation match.

    %% ======================================================
    %% ====================  CONFIG  ========================
    %% ======================================================
    DH_offset = [0,   90,  0,  55.4;
                 90,  90,  0,  0;
                 -90, -90,  0,  283.5;
                 0,   90,  0,  0;
                 90,   0, 30,  257];

    joint_limits = [ -60, 180;
                      -30, 180;
                      -90,  90;
                        0, 150;
                      -90,  90];

    n_joints = 5;
    d1 = 55.4;  d3 = 283.5;  d5 = 257;  a5 = 30;
    p_shoulder = [0; 0; d1];

    mount_angle_deg = 0;
    R_body  = [0 0 1; 0 1 0; -1 0 0];
    ma      = deg2rad(mount_angle_deg);
    R_mount = [1 0 0; 0 cos(ma) -sin(ma); 0 sin(ma) cos(ma)];
    R_viz   = R_mount * R_body;

    %% ---- State variables ----
    all_solutions = [];
    sol_valid     = [];
    sol_ori_err   = [];
    sol_pos_err   = [];
    sol_ranked    = [];
    sel_idx       = 0;
    target_p      = zeros(3,1);
    target_R      = eye(3);

    % Home EE for default input values
    [T_home, ~, ~] = forward_kinematics(zeros(5,1));
    T_ee0 = T_home{5};
    p0    = T_ee0(1:3,4);
    R0    = T_ee0(1:3,1:3);
    [y0, pi0, r0] = rotmat_to_euler_zyx(R0);

    %% ======================================================
    %% ====================  GUI  ===========================
    %% ======================================================
    fig = figure('Name','Shoulder / Arm IK Solver', ...
                 'NumberTitle','off', ...
                 'Position',[80 80 1100 720], ...
                 'Color',[0.15 0.15 0.18]);

    ax = axes('Parent',fig,'Position',[0.32 0.10 0.63 0.85]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'X — Lateral (mm)');
    ylabel(ax,'Y — Anterior (mm)');
    zlabel(ax,'Z — Superior (mm)');
    title(ax,'Inverse Kinematics','Color','w');
    set(ax,'Color',[0.12 0.12 0.15], ...
           'XColor','w','YColor','w','ZColor','w', ...
           'GridColor',[0.4 0.4 0.4]);
    view(ax,135,25);
    lm = 700;
    xlim(ax,[-lm lm]); ylim(ax,[-lm lm]); zlim(ax,[-lm lm]);

    % Panel constants
    px  = 0.02;  pw = 0.28;
    cbg = [0.15 0.15 0.18];
    clb = [0.4 0.8 1.0];

    uicontrol('Parent',fig,'Style','text','String','IK Solver', ...
        'Units','normalized','Position',[px 0.94 pw 0.04], ...
        'FontSize',14,'FontWeight','bold', ...
        'ForegroundColor','w','BackgroundColor',cbg);

    % --- Position input ---
    uicontrol('Parent',fig,'Style','text','String','Position (mm)', ...
        'Units','normalized','Position',[px 0.895 pw 0.03], ...
        'FontSize',10,'ForegroundColor',clb,'BackgroundColor',cbg, ...
        'HorizontalAlignment','left');

    ed_px = make_input(fig,cbg, px,       0.86, 'X', sprintf('%.1f',p0(1)));
    ed_py = make_input(fig,cbg, px+0.095, 0.86, 'Y', sprintf('%.1f',p0(2)));
    ed_pz = make_input(fig,cbg, px+0.19,  0.86, 'Z', sprintf('%.1f',p0(3)));

    % --- Orientation input ---
    uicontrol('Parent',fig,'Style','text', ...
        'String','Orientation (deg, ZYX Euler)', ...
        'Units','normalized','Position',[px 0.82 pw 0.03], ...
        'FontSize',10,'ForegroundColor',clb,'BackgroundColor',cbg, ...
        'HorizontalAlignment','left');

    ed_yaw   = make_input(fig,cbg, px,       0.785, 'Y', sprintf('%.1f',y0));
    ed_pitch = make_input(fig,cbg, px+0.095, 0.785, 'P', sprintf('%.1f',pi0));
    ed_roll  = make_input(fig,cbg, px+0.19,  0.785, 'R', sprintf('%.1f',r0));

    % --- Buttons ---
    uicontrol('Parent',fig,'Style','pushbutton','String','Solve IK', ...
        'Units','normalized','Position',[px 0.735 pw*0.48 0.04], ...
        'FontSize',11,'FontWeight','bold','Callback',@solve_cb);

    uicontrol('Parent',fig,'Style','pushbutton','String','Test FK->IK', ...
        'Units','normalized','Position',[px+pw*0.52 0.735 pw*0.48 0.04], ...
        'FontSize',10,'Callback',@test_fk_cb);

    uicontrol('Parent',fig,'Style','text','String','Test q (deg):', ...
        'Units','normalized','Position',[px 0.70 0.08 0.025], ...
        'FontSize',9,'ForegroundColor',[0.6 0.6 0.6], ...
        'BackgroundColor',cbg,'HorizontalAlignment','left');
    ed_test = uicontrol('Parent',fig,'Style','edit', ...
        'String','30, 45, 0, 60, 0', ...
        'Units','normalized','Position',[px+0.08 0.70 pw-0.08 0.025], ...
        'FontSize',9);

    % --- Results table ---
    uicontrol('Parent',fig,'Style','text','String','Solutions', ...
        'Units','normalized','Position',[px 0.665 pw 0.025], ...
        'FontSize',11,'FontWeight','bold', ...
        'ForegroundColor','w','BackgroundColor',cbg, ...
        'HorizontalAlignment','left');

    tbl = uitable('Parent',fig, ...
        'Units','normalized','Position',[px 0.42 pw 0.24], ...
        'ColumnName',{'#','q1','q2','q3','q4','q5','OK','Err'}, ...
        'ColumnWidth',{22,38,38,38,38,38,24,40}, ...
        'RowName',{},'FontSize',9, ...
        'CellSelectionCallback',@table_sel_cb);

    % --- Info & status ---
    info_txt = uicontrol('Parent',fig,'Style','text', ...
        'String','Enter target and click Solve.', ...
        'Units','normalized','Position',[px 0.26 pw 0.14], ...
        'FontSize',10,'ForegroundColor',[0.3 1.0 0.5], ...
        'BackgroundColor',cbg,'HorizontalAlignment','left','Max',6);

    status_txt = uicontrol('Parent',fig,'Style','text','String','', ...
        'Units','normalized','Position',[px 0.20 pw 0.05], ...
        'FontSize',9,'ForegroundColor',[1 0.5 0.3], ...
        'BackgroundColor',cbg,'HorizontalAlignment','left','Max',2);

    ee_label = uicontrol('Parent',fig,'Style','text','String','EE: ---', ...
        'Units','normalized','Position',[0.32 0.01 0.45 0.04], ...
        'FontSize',11,'ForegroundColor',[0.3 1.0 0.5], ...
        'BackgroundColor',cbg,'HorizontalAlignment','center');

    %% ---- Initial draw ----
    draw_scene(zeros(5,1));

    %% ======================================================
    %% =================  CALLBACKS  ========================
    %% ======================================================
    function solve_cb(~,~)
        pv = [str2double(get(ed_px,'String'));
              str2double(get(ed_py,'String'));
              str2double(get(ed_pz,'String'))];
        ov = [str2double(get(ed_yaw,'String'));
              str2double(get(ed_pitch,'String'));
              str2double(get(ed_roll,'String'))];

        if any(isnan([pv; ov]))
            set(status_txt,'String','Invalid input — check numbers.');
            return;
        end

        target_p = pv;
        target_R = euler_zyx_to_rotmat(ov(1), ov(2), ov(3));

        [sols, emsg] = solve_ik(target_p, target_R);

        if isempty(sols)
            set(status_txt,'String',emsg);
            set(tbl,'Data',{});
            set(info_txt,'String','No solutions found.');
            all_solutions = [];  sel_idx = 0;
            draw_scene(zeros(5,1));
            return;
        end

        [ranked, valid, oerr, perr] = filter_and_rank(sols, target_R, target_p);
        all_solutions = sols;
        sol_valid     = valid;
        sol_ori_err   = oerr;
        sol_pos_err   = perr;
        sol_ranked    = ranked;

        populate_table(sols, ranked, valid, oerr);

        bst = find(valid(ranked), 1);
        if isempty(bst)
            sel_idx = ranked(1);
            set(status_txt,'String','All solutions violate joint limits!');
        else
            sel_idx = ranked(bst);
            set(status_txt,'String','');
        end

        update_info();
        draw_scene(all_solutions(sel_idx,:)');
    end

    function test_fk_cb(~,~)
        vals = parse_csv(get(ed_test,'String'));
        if numel(vals) ~= 5
            set(status_txt,'String','Enter 5 comma-separated angles.');
            return;
        end
        q = vals(:);
        [Ta, ~, ~] = forward_kinematics(q);
        Te = Ta{5};
        pe = Te(1:3,4);  Re = Te(1:3,1:3);
        [yy,pp,rr] = rotmat_to_euler_zyx(Re);

        set(ed_px,'String',sprintf('%.2f',pe(1)));
        set(ed_py,'String',sprintf('%.2f',pe(2)));
        set(ed_pz,'String',sprintf('%.2f',pe(3)));
        set(ed_yaw,  'String',sprintf('%.2f',yy));
        set(ed_pitch,'String',sprintf('%.2f',pp));
        set(ed_roll, 'String',sprintf('%.2f',rr));

        set(status_txt,'String',sprintf('FK target from q=[%s]', ...
            strjoin(arrayfun(@(x)sprintf('%.1f',x),vals, ...
            'UniformOutput',false),', ')));
    end

    function table_sel_cb(~,ev)
        if isempty(ev.Indices) || isempty(all_solutions) || isempty(sol_ranked)
            return;
        end
        row = ev.Indices(1);
        if row > numel(sol_ranked), return; end
        sel_idx = sol_ranked(row);
        update_info();
        draw_scene(all_solutions(sel_idx,:)');
    end

    function populate_table(sols, ranked, valid, oerr)
        cnames = {'A+','A-','B+','B-'};
        ns = size(sols,1);
        td = cell(ns, 8);
        for i = 1:ns
            ri = ranked(i);
            td{i,1} = cnames{ri};
            for j = 1:5
                td{i,j+1} = sprintf('%.1f', sols(ri,j));
            end
            if valid(ri)
                td{i,7} = 'Y';
                td{i,8} = sprintf('%.1f', oerr(ri));
            else
                td{i,7} = 'N';
                td{i,8} = '---';
            end
        end
        set(tbl,'Data',td);
    end

    function update_info()
        if sel_idx < 1 || isempty(all_solutions), return; end
        cnames = {'A+','A-','B+','B-'};
        nm = cnames{min(sel_idx,4)};
        q  = all_solutions(sel_idx,:);
        vld = sol_valid(sel_idx);
        warn = '';
        if sol_ori_err(sel_idx) > 15
            warn = sprintf('\nWarning: orientation error > 15 deg');
        end
        lines = { ...
            sprintf('Selected: %s %s', nm, iff(vld,'(valid)','(INVALID)')), ...
            sprintf('Pos error:  %.2f mm', sol_pos_err(sel_idx)), ...
            sprintf('Ori error:  %.1f deg', sol_ori_err(sel_idx)), ...
            sprintf('q = [%.1f, %.1f, %.1f, %.1f, %.1f]', q), ...
            warn};
        set(info_txt,'String',strjoin(lines,newline));
    end

    %% ======================================================
    %% ================  IK SOLVER  =========================
    %% ======================================================
    function [solutions, emsg] = solve_ik(pt, Rt)
        emsg = '';

        % Wrist center = elbow position (d4=0, a4=0)
        % p_ee = p_elbow + d5*z4 + a5*x5
        % z4 = Rt(:,3) since alpha5=0 → z5=z4
        % x5 = Rt(:,1) (x-axis of EE frame)
        p_wc = pt - d5*Rt(:,3) - a5*Rt(:,1);

        v    = p_wc - p_shoulder;
        dist = norm(v);

        if dist < 1e-6
            solutions = [];
            emsg = 'Target maps to shoulder center (singular).';
            return;
        end

        if abs(dist - d3) > 15
            solutions = [];
            emsg = sprintf(['Unreachable: elbow dist = %.1f mm, ' ...
                'required = %.1f mm (diff = %+.1f mm).\n' ...
                'Adjust orientation to bring elbow closer to ' ...
                'the %.1f mm shell.'], dist, d3, dist-d3, d3);
            return;
        end

        % z2 = unit vector from shoulder to elbow
        z2 = v / dist;

        % Shoulder solution A: q2 in [-90, 90]
        q1a = rad2deg(atan2(z2(2), z2(1)));
        q2a = rad2deg(asin(clamp(z2(3),-1,1)));

        % Shoulder solution B: flipped (q2 in [90, 270])
        q1b = wrap180(q1a + 180);
        q2b = wrap180(180 - q2a);

        % Elbow angle: cos(q4) = dot(z2, z4_target)
        z4t  = Rt(:,3);
        cq4  = clamp(dot(z2, z4t), -1, 1);
        q4p  = rad2deg(acos(cq4));
        q4n  = -q4p;

        solutions = zeros(4,5);
        pairs  = [q1a q2a; q1b q2b];
        elbows = [q4p, q4n];
        idx    = 0;

        for s = 1:2
            q1 = pairs(s,1);  q2 = pairs(s,2);
            R02 = compute_R02(q1, q2);
            % M = R_2_3 * R_3_4 * R_4_5 = Rz(q3-90)*Ry(q4)*Rz(q5+90)
            M = R02' * Rt;

            for e = 1:2
                q4 = elbows(e);
                [q3, q5] = extract_zyz(M, q4);
                idx = idx + 1;
                solutions(idx,:) = [q1, q2, q3, q4, q5];
            end
        end
    end

    %% ======== R_0_2 from q1, q2 ========
    % R_0_2 = Rz(q1)*Rx(90)*Rz(q2+90)*Rx(90)
    function R02 = compute_R02(q1d, q2d)
        c1 = cosd(q1d); s1 = sind(q1d);
        c2 = cosd(q2d); s2 = sind(q2d);
        R02 = [-c1*s2,  s1, c1*c2;
               -s1*s2, -c1, s1*c2;
                c2,      0,  s2  ];
    end

    %% ======== ZYZ Euler extraction ========
    % M = Rz(alpha)*Ry(beta)*Rz(gamma), beta = q4 known
    % q3 = rad2deg(alpha) + 90,  q5 = rad2deg(gamma) - 90
    function [q3, q5] = extract_zyz(M, q4d)
        sb = sind(q4d);

        if abs(sb) > 1e-6
            alpha = atan2(M(2,3)/sb, M(1,3)/sb);
            gamma = atan2(M(3,2)/sb, -M(3,1)/sb);
        else
            if cosd(q4d) > 0
                % beta ≈ 0: M ≈ Rz(alpha+gamma)
                ag = atan2(M(2,1), M(1,1));
            else
                % beta ≈ pi: M ≈ Rz(alpha)*Ry(pi)*Rz(gamma)
                ag = atan2(-M(2,1), -M(1,1));
            end
            alpha = ag/2;
            gamma = ag/2;
            if cosd(q4d) < 0
                gamma = -gamma;
            end
        end

        q3 = wrap180(rad2deg(alpha) + 90);
        q5 = wrap180(rad2deg(gamma) - 90);
    end

    %% ======================================================
    %% ==============  FILTER & RANK  =======================
    %% ======================================================
    function [ranked, valid, oerr, perr] = filter_and_rank(sols, Rt, pt)
        ns   = size(sols,1);
        valid = true(ns,1);
        oerr  = inf(ns,1);
        perr  = zeros(ns,1);

        for i = 1:ns
            q = sols(i,:)';

            % Joint-limit check (0.5° tolerance for numerical noise)
            for j = 1:5
                if q(j) < joint_limits(j,1)-0.5 || ...
                   q(j) > joint_limits(j,2)+0.5
                    valid(i) = false;
                end
            end

            % FK to get actual EE pose
            [Ta, ~, ~] = forward_kinematics(q);
            Te = Ta{5};
            Ra = Te(1:3,1:3);
            pa = Te(1:3,4);

            perr(i) = norm(pa - pt);

            % Geodesic orientation error
            Re = Ra' * Rt;
            ca = clamp((trace(Re)-1)/2, -1, 1);
            oerr(i) = rad2deg(acos(ca));
        end

        % Rank: valid solutions first, then by orientation error
        score = oerr;
        score(~valid) = score(~valid) + 1e4;
        [~, ranked] = sort(score);
    end

    %% ======================================================
    %% ===============  EULER CONVERSIONS  ==================
    %% ======================================================
    function R = euler_zyx_to_rotmat(yd, pd, rd)
        cy=cosd(yd); sy=sind(yd);
        cp=cosd(pd); sp=sind(pd);
        cr=cosd(rd); sr=sind(rd);
        R = [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr;
             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr;
             -sp,    cp*sr,             cp*cr           ];
    end

    function [y, p, r] = rotmat_to_euler_zyx(R)
        p = rad2deg(asin(clamp(-R(3,1),-1,1)));
        if abs(cosd(p)) > 1e-6
            y = rad2deg(atan2(R(2,1), R(1,1)));
            r = rad2deg(atan2(R(3,2), R(3,3)));
        else
            y = rad2deg(atan2(-R(1,2), R(2,2)));
            r = 0;
        end
    end

    %% ======================================================
    %% =============  FORWARD KINEMATICS  ===================
    %% ======================================================
    function [T_all, positions, mid_positions] = forward_kinematics(q_deg)
        DH = DH_offset;
        DH(:,1) = DH(:,1) + q_deg(:);

        T_all = cell(n_joints,1);
        T_cum = eye(4);
        positions     = zeros(3, n_joints+1);
        mid_positions = zeros(3, n_joints+1);

        for k = 1:n_joints
            theta = DH(k,1); alpha = DH(k,2);
            a_val = DH(k,3); d_val = DH(k,4);

            th = deg2rad(theta);
            ct = cos(th); st = sin(th);
            Rz_th = [ct -st 0 0; st ct 0 0; 0 0 1 0; 0 0 0 1];
            Tz_d  = [1 0 0 0; 0 1 0 0; 0 0 1 d_val; 0 0 0 1];
            Tx_a  = [1 0 0 a_val; 0 1 0 0; 0 0 1 0; 0 0 0 1];

            al = deg2rad(alpha);
            ca = cos(al); sa = sin(al);
            Rx_al = [1 0 0 0; 0 ca -sa 0; 0 sa ca 0; 0 0 0 1];

            T_mid = T_cum * (Rz_th * Tz_d);
            T_int = T_mid * Tx_a;
            T_cum = T_int * Rx_al;

            mid_positions(:,k+1) = T_mid(1:3,4);
            T_all{k}             = T_cum;
            positions(:,k+1)     = T_cum(1:3,4);
        end
    end

    %% ======================================================
    %% =================  DRAWING  ==========================
    %% ======================================================
    function draw_scene(q_deg)
        cla(ax);

        % Re-add lights (cla removes them)
        light('Parent',ax,'Position',[500 500 800]);
        light('Parent',ax,'Position',[-500 -500 400],'Style','infinite');

        [T_all, pos, mid_pos] = forward_kinematics(q_deg);

        % Apply visual rotation
        for k = 1:(n_joints+1)
            pos(:,k)     = R_viz * pos(:,k);
            mid_pos(:,k) = R_viz * mid_pos(:,k);
        end

        % Body context
        draw_body_context();

        % Target reference frame (dashed RGB arrows)
        if sel_idx > 0
            draw_target_arrows();
        end

        % Links
        lc = [0.5 0.5 0.6; 0.3 0.6 0.9; 0.3 0.6 0.9;
              0.8 0.5 0.2; 0.8 0.5 0.2];
        for k = 1:n_joints
            p0 = pos(:,k); pm = mid_pos(:,k+1); p1 = pos(:,k+1);
            if norm(pm-p0) > 1
                draw_link_cylinder(p0, pm, 8, lc(k,:));
            end
            if norm(p1-pm) > 1
                draw_link_cylinder(pm, p1, 8, lc(k,:)*0.8);
            end
        end

        % Joint spheres
        for k = 1:n_joints
            draw_sphere(pos(:,k+1), 12, [1 0.3 0.3]);
        end
        draw_sphere(pos(:,1), 14, [0.3 0.3 0.3]);

        % Joint z-axes (yellow)
        for k = 1:n_joints
            zv = R_viz * T_all{k}(1:3,3);
            o  = pos(:,k+1);
            quiver3(ax,o(1),o(2),o(3),zv(1)*40,zv(2)*40,zv(3)*40, ...
                'Color',[1 1 0],'LineWidth',1.5,'MaxHeadSize',0.5);
        end

        % End-effector frame (solid RGB arrows)
        Te = T_all{n_joints};
        ep = pos(:,n_joints+1);
        fl = 50;
        crgb = {'r','g','b'};
        for c = 1:3
            dv = R_viz * Te(1:3,c) * fl;
            quiver3(ax,ep(1),ep(2),ep(3),dv(1),dv(2),dv(3), ...
                'Color',crgb{c},'LineWidth',2.5, ...
                'MaxHeadSize',0.6,'AutoScale','off');
        end

        % Elbow marker
        if n_joints >= 4
            draw_sphere(pos(:,n_joints-1), 10, [0.2 1.0 0.4]);
        end

        % EE position label (FK coordinates)
        ee_fk = Te(1:3,4);
        set(ee_label,'String', ...
            sprintf('EE: [%.1f, %.1f, %.1f] mm', ee_fk(1),ee_fk(2),ee_fk(3)));

        drawnow;
    end

    function draw_target_arrows()
        pv = R_viz * target_p;
        fl = 60;
        cr = {[1 0.3 0.3],[0.3 1 0.3],[0.3 0.3 1]};
        for c = 1:3
            dv = R_viz * target_R(:,c) * fl;
            quiver3(ax,pv(1),pv(2),pv(3),dv(1),dv(2),dv(3), ...
                'Color',cr{c},'LineWidth',1.8,'LineStyle','--', ...
                'MaxHeadSize',0.8,'AutoScale','off');
        end
        draw_sphere(pv, 8, [1 1 1]);
    end

    %% ======== Body context (from FK file) ========
    function draw_body_context()
        tx = [-300 0]; ty = [-100 100]; tz = [55 -400];

        v = [tx(1) ty(1) tz(1); tx(2) ty(1) tz(1);
             tx(2) ty(2) tz(1); tx(1) ty(2) tz(1);
             tx(1) ty(1) tz(2); tx(2) ty(1) tz(2);
             tx(2) ty(2) tz(2); tx(1) ty(2) tz(2)];

        fd = {[1 2 3 4],[0.24 0.24 0.29]; [5 6 7 8],[0.18 0.18 0.22];
              [2 3 7 6],[0.32 0.38 0.50]; [1 4 8 5],[0.16 0.16 0.20];
              [1 2 6 5],[0.20 0.20 0.26]; [3 4 8 7],[0.26 0.26 0.32]};
        for f = 1:size(fd,1)
            patch(ax,'Vertices',v,'Faces',fd{f,1}, ...
                'FaceColor',fd{f,2},'EdgeColor',[0.4 0.4 0.4], ...
                'FaceAlpha',0.55,'FaceLighting','gouraud');
        end

        txm = mean(tx); tzm = mean(tz);
        quiver3(ax,txm,ty(2),tzm, 0,80,0, ...
            'Color',[1 0.85 0.2],'LineWidth',2.2, ...
            'MaxHeadSize',0.6,'AutoScale','off');
        text(ax,txm,ty(2)+90,tzm,'FRONT', ...
            'Color',[1 0.85 0.2],'FontSize',11,'FontWeight','bold');

        th_d = linspace(0,2*pi,40); r_d = 40;
        fill3(ax, zeros(size(th_d)), r_d*cos(th_d), r_d*sin(th_d), ...
            [0.45 0.55 0.78],'EdgeColor',[0.60 0.70 0.90],'FaceAlpha',0.60);
    end

    %% ======== Drawing helpers ========
    function draw_sphere(center, radius, color)
        [sx,sy,sz] = sphere(12);
        surf(ax, sx*radius+center(1), sy*radius+center(2), ...
             sz*radius+center(3), ...
             'FaceColor',color,'EdgeColor','none','FaceLighting','gouraud');
    end

    function draw_link_cylinder(p1, p2, radius, color)
        np = 12;
        vec = p2 - p1;
        len = norm(vec);
        if len < 1e-6, return; end

        [cx,cy,cz] = cylinder(radius, np);
        cz = cz * len;

        zh = vec/len;
        if abs(zh(3)-1) < 1e-6
            Rc = eye(3);
        elseif abs(zh(3)+1) < 1e-6
            Rc = diag([1,-1,-1]);
        else
            z0 = [0;0;1];
            vc = cross(z0,zh); s = norm(vc); c = dot(z0,zh);
            vx = [0 -vc(3) vc(2); vc(3) 0 -vc(1); -vc(2) vc(1) 0];
            Rc = eye(3) + vx + vx*vx*(1-c)/(s^2);
        end

        for j = 1:numel(cx)
            pt = Rc*[cx(j);cy(j);cz(j)] + p1;
            cx(j) = pt(1); cy(j) = pt(2); cz(j) = pt(3);
        end

        surf(ax,cx,cy,cz, ...
            'FaceColor',color,'EdgeColor','none', ...
            'FaceLighting','gouraud','AmbientStrength',0.5);
    end

    %% ======================================================
    %% ==================  UTILITIES  =======================
    %% ======================================================
    function a = wrap180(a)
        a = mod(a+180, 360) - 180;
    end

    function v = clamp(v, lo, hi)
        v = max(lo, min(hi, v));
    end

    function r = iff(cond, a, b)
        if cond, r = a; else, r = b; end
    end

    function vals = parse_csv(str)
        vals = str2double(strsplit(strtrim(str), {',',' '}));
        vals(isnan(vals)) = [];
    end

    function ed = make_input(fig, bg, x, y, label, default)
        uicontrol('Parent',fig,'Style','text','String',label, ...
            'Units','normalized','Position',[x y 0.02 0.025], ...
            'FontSize',10,'ForegroundColor','w','BackgroundColor',bg, ...
            'HorizontalAlignment','right');
        ed = uicontrol('Parent',fig,'Style','edit','String',default, ...
            'Units','normalized','Position',[x+0.025 y 0.065 0.025], ...
            'FontSize',10);
    end

end
