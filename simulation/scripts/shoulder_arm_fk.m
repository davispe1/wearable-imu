function shoulder_arm_fk()
% SHOULDER_ARM_FK  5-DOF Shoulder / Arm Forward Kinematics Visualizer
% with optional physiological (human arm) joint limit checking.
%
% Joint mapping (human upper limb):
%   theta1 = shoulder flexion/extension
%   theta2 = shoulder abduction/adduction
%   theta3 = shoulder internal/external rotation
%   theta4 = elbow flexion/extension
%   theta5 = forearm pronation/supination

    %% ======================================================
    %% ====================  CONFIG  ========================
    %% ======================================================
    % Columns: [theta_offset_deg, alpha_deg, a_mm, d_mm]
    DH_offset = [0,   90,  0,  55.4;   % Joint 1: shoulder flexion/extension
                 90,  90,  0,  0;      % Joint 2: shoulder abduction/adduction
                 -90, -90,  0,  283.5;  % Joint 3: shoulder internal/external rotation
                 0,   90,  0,  0;      % Joint 4: elbow flexion/extension
                 90,   0, 30,  257];   % Joint 5: forearm pronation/supination

    joint_limits = [ -60, 180;   % theta1: shoulder flexion(+)/extension(-)
                      -30, 180;  % theta2: shoulder abduction(+)/adduction(-)
                      -90,  90;  % theta3: shoulder internal(-)/external(+) rotation
                        0, 150;  % theta4: elbow flexion (0=straight, +=flexed)
                      -90,  90]; % theta5: forearm pronation(-)/supination(+)

    joint_names = {'\theta_1 (shoulder flex)','\theta_2 (shoulder abd)', ...
                   '\theta_3 (shoulder rot)','\theta_4 (elbow flex)', ...
                   '\theta_5 (forearm pron/sup)'};

    % --- VISUAL MOUNT ANGLE (degrees) ---
    % Rotates the entire arm rendering around the lateral axis (outward
    % from the torso). 0° = arm hangs straight down (posición de firme).
    % Does NOT affect forward kinematics — visualization only.
    mount_angle_deg = 0;

    %% ======================================================
    n_joints = size(DH_offset,1);
    q0 = zeros(n_joints,1);

    % --- Build visual rotation (does NOT enter FK) ---
    % Ry(90°): maps DH Z → lateral (+X world), DH X → inferior (-Z world)
    R_body = [0 0 1; 0 1 0; -1 0 0];
    % Rx(mount_angle): tilts around lateral axis
    ma = deg2rad(mount_angle_deg);
    R_mount = [1 0 0; 0 cos(ma) -sin(ma); 0 sin(ma) cos(ma)];
    R_viz = R_mount * R_body;

    %% ---- Build figure ----
    fig = figure('Name','Shoulder / Arm FK Visualizer', ...
                 'NumberTitle','off', ...
                 'Position',[100 100 1050 700], ...
                 'Color',[0.15 0.15 0.18]);

    ax = axes('Parent',fig, 'Position',[0.30 0.10 0.65 0.85]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'X — Lateral (mm)');
    ylabel(ax,'Y — Anterior (mm)');
    zlabel(ax,'Z — Superior (mm)');
    title(ax,'Shoulder / Arm FK','Color','w');
    set(ax,'Color',[0.12 0.12 0.15], ...
           'XColor','w','YColor','w','ZColor','w', ...
           'GridColor',[0.4 0.4 0.4]);
    view(ax, 135, 25);
    lim = 700;
    xlim(ax,[-lim lim]); ylim(ax,[-lim lim]); zlim(ax,[-lim lim]);

    light('Parent',ax,'Position',[500 500 800]);
    light('Parent',ax,'Position',[-500 -500 400],'Style','infinite');

    %% ---- Sliders panel ----
    sliders    = gobjects(n_joints,1);
    val_labels = gobjects(n_joints,1);
    name_labels= gobjects(n_joints,1);
    panel_x = 0.02;
    panel_w = 0.25;

    uicontrol('Parent',fig,'Style','text', ...
              'String','Joint Angles', ...
              'Units','normalized', ...
              'Position',[panel_x 0.94 panel_w 0.04], ...
              'FontSize',14,'FontWeight','bold', ...
              'ForegroundColor','w', ...
              'BackgroundColor',[0.15 0.15 0.18]);

    for i = 1:n_joints
        y_base = 0.89 - (i-1)*0.16;

        name_labels(i) = uicontrol('Parent',fig,'Style','text', ...
                  'String',joint_names{i}, ...
                  'Units','normalized', ...
                  'Position',[panel_x y_base panel_w*0.65 0.035], ...
                  'FontSize',10, ...
                  'ForegroundColor',[0.4 0.8 1.0], ...
                  'BackgroundColor',[0.15 0.15 0.18], ...
                  'HorizontalAlignment','left');

        val_labels(i) = uicontrol('Parent',fig,'Style','text', ...
                  'String',sprintf('%+.1f°', q0(i)), ...
                  'Units','normalized', ...
                  'Position',[panel_x+panel_w*0.65 y_base panel_w*0.35 0.035], ...
                  'FontSize',11, ...
                  'ForegroundColor','w', ...
                  'BackgroundColor',[0.15 0.15 0.18], ...
                  'HorizontalAlignment','right');

        sliders(i) = uicontrol('Parent',fig,'Style','slider', ...
                  'Units','normalized', ...
                  'Position',[panel_x y_base-0.045 panel_w 0.035], ...
                  'Min',-180,'Max',180,'Value',q0(i), ...
                  'SliderStep',[1/360, 10/360], ...
                  'Tag', num2str(i), ...
                  'Callback',@slider_cb);
    end

    show_limits = false;
    uicontrol('Parent',fig,'Style','checkbox', ...
              'String','Show Human Joint Limits', ...
              'Units','normalized', ...
              'Position',[panel_x 0.13 panel_w 0.04], ...
              'FontSize',10.5, ...
              'ForegroundColor','w', ...
              'BackgroundColor',[0.15 0.15 0.18], ...
              'Value',0, ...
              'Callback',@toggle_limits_cb);

    uicontrol('Parent',fig,'Style','pushbutton', ...
              'String','Reset to Home', ...
              'Units','normalized', ...
              'Position',[panel_x 0.02 panel_w 0.06], ...
              'FontSize',11, ...
              'Callback',@reset_cb);

    ee_label = uicontrol('Parent',fig,'Style','text', ...
              'String','EE: [0, 0, 0]', ...
              'Units','normalized', ...
              'Position',[0.30 0.01 0.45 0.04], ...
              'FontSize',11, ...
              'ForegroundColor',[0.3 1.0 0.5], ...
              'BackgroundColor',[0.15 0.15 0.18], ...
              'HorizontalAlignment','center');

    %% ---- Initial draw ----
    update_limit_visuals(q0);
    draw_robot(q0);

    %% ======== CALLBACKS ========
    function slider_cb(~, ~)
        q = get_angles();
        update_limit_visuals(q);
        draw_robot(q);
    end

    function toggle_limits_cb(src, ~)
        show_limits = logical(get(src,'Value'));
        update_limit_visuals(get_angles());
    end

    function reset_cb(~,~)
        for j = 1:n_joints
            set(sliders(j),'Value',0);
        end
        q = get_angles();
        update_limit_visuals(q);
        draw_robot(q);
    end

    function q = get_angles()
        q = zeros(n_joints,1);
        for j = 1:n_joints
            q(j) = get(sliders(j),'Value');
        end
    end

    function update_limit_visuals(q)
        for j = 1:n_joints
            val_labels(j).String = sprintf('%+.1f°', q(j));
            if show_limits && (q(j) < joint_limits(j,1) || q(j) > joint_limits(j,2))
                val_labels(j).ForegroundColor = [1 0.3 0.3];
                name_labels(j).ForegroundColor = [1 0.5 0.5];
            else
                val_labels(j).ForegroundColor = [1 1 1];
                name_labels(j).ForegroundColor = [0.4 0.8 1.0];
            end
        end
    end

    %% ======== FORWARD KINEMATICS (untouched) ========
    function [T_all, positions, mid_positions] = forward_kinematics(q_deg)
        DH = DH_offset;
        DH(:,1) = DH(:,1) + q_deg(:);

        T_all = cell(n_joints,1);
        T_cum = eye(4);
        positions = zeros(3, n_joints+1);
        mid_positions = zeros(3, n_joints+1);
        positions(:,1) = [0;0;0];
        mid_positions(:,1) = [0;0;0];

        for k = 1:n_joints
            theta = DH(k,1); alpha = DH(k,2); a = DH(k,3); d = DH(k,4);

            th = deg2rad(theta);
            ct = cos(th); st = sin(th);
            Rz_theta = [ct -st 0 0; st ct 0 0; 0 0 1 0; 0 0 0 1];
            Tz_d     = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
            Tx_a     = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];

            al = deg2rad(alpha);
            ca = cos(al); sa = sin(al);
            Rx_alpha = [1 0 0 0; 0 ca -sa 0; 0 sa ca 0; 0 0 0 1];

            T_mid = T_cum * (Rz_theta * Tz_d);
            T_inter = T_mid * Tx_a;
            T_cum = T_inter * Rx_alpha;

            mid_positions(:, k+1) = T_mid(1:3, 4);
            T_all{k} = T_cum;
            positions(:, k+1) = T_cum(1:3, 4);
        end
    end

    %% ======== DRAWING ========
    function draw_robot(q_deg)
        cla(ax);

        [T_all, pos, mid_pos] = forward_kinematics(q_deg);

        % --- Apply visual rotation (does NOT change FK) ---
        for k = 1:(n_joints+1)
            pos(:,k)     = R_viz * pos(:,k);
            mid_pos(:,k) = R_viz * mid_pos(:,k);
        end

        % --- Body context ---
        draw_body_context(ax);

        % ---- Draw links ----
        link_colors = [0.5 0.5 0.6; 0.3 0.6 0.9; 0.3 0.6 0.9; 0.8 0.5 0.2; 0.8 0.5 0.2];

        for k = 1:n_joints
            p0 = pos(:, k);
            p_mid = mid_pos(:, k+1);
            p1 = pos(:, k+1);

            if norm(p_mid - p0) > 1
                draw_link_cylinder(ax, p0, p_mid, 8, link_colors(k,:));
            end
            if norm(p1 - p_mid) > 1
                draw_link_cylinder(ax, p_mid, p1, 8, link_colors(k,:)*0.8);
            end
        end

        % ---- Joints ----
        for k = 1:n_joints
            draw_sphere(ax, pos(:,k+1), 12, [1 0.3 0.3]);
        end
        draw_sphere(ax, pos(:,1), 14, [0.3 0.3 0.3]);

        % ---- Joint z-axes (visual rotation applied) ----
        for k = 1:n_joints
            z_fk = T_all{k}(1:3, 3);
            z_vis = R_viz * z_fk;
            o = pos(:, k+1);
            quiver3(ax, o(1),o(2),o(3), z_vis(1)*40,z_vis(2)*40,z_vis(3)*40, ...
                    'Color',[1 1 0],'LineWidth',1.5,'MaxHeadSize',0.5);
        end

        % ---- End-effector frame (visual rotation applied) ----
        T_ee = T_all{n_joints};
        ep = pos(:, n_joints+1);
        frame_len = 50;
        colors_rgb = {'r','g','b'};
        for c = 1:3
            dv = R_viz * T_ee(1:3, c) * frame_len;
            quiver3(ax, ep(1),ep(2),ep(3), dv(1),dv(2),dv(3), ...
                    'Color',colors_rgb{c},'LineWidth',2.5, ...
                    'MaxHeadSize',0.6,'AutoScale','off');
        end

        % ---- Wrist center ----
        if n_joints >= 4
            draw_sphere(ax, pos(:,n_joints-1), 10, [0.2 1.0 0.4]);
        end

        % ---- EE label (FK coordinates, not visual) ----
        ee_fk = T_ee(1:3,4);
        ee_label.String = sprintf('EE Position: [%.1f, %.1f, %.1f] mm', ...
                                   ee_fk(1), ee_fk(2), ee_fk(3));
        drawnow;
    end

    %% ======== BODY CONTEXT ========
    function draw_body_context(ax)
        % World frame after R_viz:
        %   +X = Lateral (outward from torso, same as 55.4mm direction)
        %   +Y = Anterior (front of body)
        %   +Z = Superior (up)
        % Shoulder origin sits at (0,0,0).
        % Torso extends medially (-X) and inferiorly (-Z).

        tx = [-300, 0];       % X: body center ← shoulder
        ty = [-100, 100];     % Y: posterior ← anterior
        tz = [55, -400];      % Z: above shoulder top → below

        v = [tx(1) ty(1) tz(1);  tx(2) ty(1) tz(1);
             tx(2) ty(2) tz(1);  tx(1) ty(2) tz(1);
             tx(1) ty(1) tz(2);  tx(2) ty(1) tz(2);
             tx(2) ty(2) tz(2);  tx(1) ty(2) tz(2)];

        face_defs = {[1 2 3 4], [0.24 0.24 0.29];   % top
                     [5 6 7 8], [0.18 0.18 0.22];   % bottom
                     [2 3 7 6], [0.32 0.38 0.50];   % anterior (+Y, brighter)
                     [1 4 8 5], [0.16 0.16 0.20];   % posterior
                     [1 2 6 5], [0.20 0.20 0.26];   % medial (-X)
                     [3 4 8 7], [0.26 0.26 0.32]};  % lateral (shoulder side)

        for f = 1:size(face_defs,1)
            patch(ax,'Vertices',v,'Faces',face_defs{f,1}, ...
                  'FaceColor',face_defs{f,2},'EdgeColor',[0.4 0.4 0.4], ...
                  'FaceAlpha',0.55,'FaceLighting','gouraud');
        end

        % "FRONT" arrow on anterior face (+Y)
        tx_mid = mean(tx);
        tz_mid = mean(tz);
        quiver3(ax, tx_mid, ty(2), tz_mid, 0, 80, 0, ...
                'Color',[1.0 0.85 0.2],'LineWidth',2.2, ...
                'MaxHeadSize',0.6,'AutoScale','off');
        text(ax, tx_mid, ty(2)+90, tz_mid, 'FRONT', ...
             'Color',[1.0 0.85 0.2],'FontSize',11,'FontWeight','bold');

        % Shoulder disc — vertical, in the YZ plane (flat against torso wall)
        th_d = linspace(0, 2*pi, 40);
        r_d  = 40;
        fill3(ax, zeros(size(th_d)), r_d*cos(th_d), r_d*sin(th_d), ...
              [0.45 0.55 0.78],'EdgeColor',[0.60 0.70 0.90],'FaceAlpha',0.60);
    end

    %% ======== DRAWING HELPERS ========
    function draw_sphere(ax, center, radius, color)
        [sx, sy, sz] = sphere(12);
        surf(ax, sx*radius+center(1), sy*radius+center(2), sz*radius+center(3), ...
             'FaceColor',color,'EdgeColor','none','FaceLighting','gouraud');
    end

    function draw_link_cylinder(ax, p1, p2, radius, color)
        n_pts = 12;
        vec = p2 - p1;
        len = norm(vec);
        if len < 1e-6, return; end

        [cx, cy, cz] = cylinder(radius, n_pts);
        cz = cz * len;

        z_hat = vec / len;
        if abs(z_hat(3) - 1) < 1e-6
            R = eye(3);
        elseif abs(z_hat(3) + 1) < 1e-6
            R = diag([1, -1, -1]);
        else
            z0 = [0;0;1];
            v = cross(z0, z_hat);
            s = norm(v);
            c = dot(z0, z_hat);
            vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
            R = eye(3) + vx + vx*vx * (1-c)/(s^2);
        end

        for j = 1:numel(cx)
            pt = R * [cx(j); cy(j); cz(j)] + p1;
            cx(j) = pt(1); cy(j) = pt(2); cz(j) = pt(3);
        end

        surf(ax, cx, cy, cz, ...
             'FaceColor',color,'EdgeColor','none', ...
             'FaceLighting','gouraud','AmbientStrength',0.5);
    end

end
