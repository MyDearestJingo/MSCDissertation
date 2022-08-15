%% This MatLab code provide EKF function for estimating the pose and twist of an
%% in 6-D space

clear all;

%% Args
ekf = true; 
% ekf = false;

viz = true;
% viz = false;

addpath("./msqt");
addpath("./tools");


%% Define camera pose
% pose of camera_link_optical in the form as 
%   [position xyz, orientation (in MatLab quat order wxyz)]
camLinkOptical = [
    -0.04995862329776529
    0.1999999004306916
    0.15000000697127847
    1.1035039458341135e-10
    3.5048330370116335e-05
    1.3357013288769306e-06
    0.9999999993849154
    ]';
camOpticalJoint = [-pi/2, 0, -pi/2];  % writen in file rrbot.xcrob

%% Data loading and pre-process
estimation = loadData('data/dope_estm_rec.csv');
groundtruth = loadData('data/object_odom_rec.csv');

% ! IMPORTANT !
% ! the base frame of object in Gazebo model is different in DOPE 
% !   representation.
for i = 1:size(estimation,1)
    % convert estimation from camera frame to world frame
    estimation(i,2:end) = cam2world(camLinkOptical, ...
                                    camOpticalJoint, ...
                                    estimation(i,2:end));
    % change the DOPE trained model to Gazebo model representation
    estimation(i,5:8) = squatmultiply(estimation(i,5:8),eul2quat([0,0,pi/2]));
    estimation(i,2:4) = estimation(i,2:4) + ...
        tform2trvec(quat2tform(estimation(i,5:8)) * trvec2tform([0,0,-.05]));
end


%% EKF
if ekf
    % Define state vector and transition function
    p = sym('p', [3 1]);    % position of origin of the target w.r.t camera C
    o = sym('o', [4 1]);    % quaternion, orientation of the target w.r.t C
    % v = sym('v', [3 1]);    % translation velocities of the target w.r.t C
    cc = sym('cc', [3 1]);  % circle center position w.r.t C
    cq = sym('nq', [4 1]);  % circle coordinate system orientation w.r.t C
    rv = sym('rv', [1 1]);  % circular motion angular velocity
    r = sym('r', [3 1]);    % Euler angle (ZYX) representation for twist angular velocities


    % x = [p;o;v;r];
    x = [p;o;cc;cq;rv;r];

    syms deltat;
    dTheta = rv * deltat;

    % circle
    H1 = sym(eye(4));
    H1(1:3, 1:3) = squat2rotm(cq');
    H1(1:3, 4) = cc;

    % motion
    H2 = sym(eye(4));
    H2(1:3, 1:3) = squat2rotm([cos(dTheta/2), sin(dTheta/2).*[0 0 1]]);

    nextP = H1 * H2 * [p;1];
    nextP = nextP(1:3);
    g = [ 
        % p + v*deltat; 
        nextP;
        squatmultiply(o', seul2quat(r'*deltat))';
        cc;
        cq;
        rv;
        r;
    ];
    G = jacobian(g, x);

    h = [p;o;cc;cq;rv;r];
    H = jacobian(h, x);

    stat = [
        zeros(3,1);   % position
        [1;0;0;0];   % orientation in quaternion [d,i,j,k]
        zeros(3,1);
        [1;0;0;0];
        0;
        zeros(3,1);
    ];
    % stat = zeros(18,1);
    statSig = eye(size(x,1));
    R = eye(size(x,1))*0.5;
    Q = eye(size(x,1))*0.5;

    statHis = [];
    statPredHis = [];
    zHis = [];

    % DEBUG
    % estimation = [estimation(1:31,:);estimation(32:33,:)];
    % estimation = estimation(1:30,:);
    for i = 3:size(estimation,1)
        fprintf("EKF Progress: %d/%d\n",i-1, size(estimation,1)-1)

        timestamp = estimation(i,1);
        difft = timestamp - estimation(i-1,1);

        statPred = eval(subs(g, [x; deltat], [stat; difft]));
        Gt       = eval(subs(G, [x; deltat], [stat; difft]));
        Ht       = eval(subs(H, [x; deltat], [stat; difft]));

        statSigPred = Gt * statSig * Gt' + R;
        Kt = statSigPred * Ht' / (Ht * statSigPred * Ht' + Q);

        % generate z_t from measurement data
        % quatRot = squatmultiply(estimation(i,5:8), squatinv(estimation(i-1,5:8)));
        % quatRot = squatnormalize(quatRot);
        % eulRot = squat2eul(quatRot);
        % deltaEul = quat2eul(estimation(i,5:8)) - quat2eul(estimation(i-1,5:8));
        % angvel = deltaEul/difft;

        % axangRot = quat2axang(quatRot);
        % axangRot(1:3) = axangRot(1:3)./sqrt(sum(axangRot(1:3).^2));
        % axangRot(4) = axangRot(4)/difft;
        
        % measurement vector
        % zt = [
        %     estimation(i,2:4)';             % current position
        %     estimation(i,5:8)';             % current orientation (quaternion)
        %     (estimation(i,2:4)-estimation(i-1,2:4))'/difft; % linear velocities
        %     eulRot'/difft;                        % euler angular velocities
        %     % angvel';
        % ];
        % zt'
        zt = genMeasurementCircle(estimation(i-2:i, :));

        % stat = statPred + Kt * (zt - eval(subs(h, x, statPred)));
        stat = statPred + Kt * (zt - statPred);
        statSig = (eye(size(x,1)) - Kt * Ht) * statSigPred;

        statHis = [statHis; [timestamp, stat']];
        statPredHis = [statPredHis; [timestamp, statPred']];
        zHis = [zHis; [timestamp, zt']];
    end
end
fprintf('\n');

%% Visualization
if viz
    posFig = figure();
    figure(posFig);
    t = tiledlayout(2, 3, 'TileSpacing','compact');
    title(t, 'Position Filter Results', 'FontSize', 20);
    tiledtitles = ['X-Axis Position'; 'Y-Axis Position'; 'Z-Axis Position'; ...
        'X-Axis Velocity'; 'Y-Axis Velocity'; 'Z-Axis Velocity'
    ];
    colidx = [2:4, 9:11];
    % for i = 1:6   % position & translation velocities
    for i = 1:3 % only position
        nexttile, hold on

        plot(groundtruth(2:end,1),groundtruth(2:end,colidx(i)));
        plot(zHis(:,1), zHis(:,colidx(i)), '--^');
        plot(statHis(:,1), statHis(:,colidx(i)), 'm');
        plot(statPredHis(:,1) ,statPredHis(:,colidx(i)), '--x');

        set(gca, 'FontSize', 12);
        set(get(gca, 'XLabel'), 'String', 'time', 'Interpreter', 'latex');
        set(get(gca, 'YLabel'), 'String', 'value', 'Interpreter', 'latex');
        set(get(gca, 'Title'), 'String', tiledtitles(i,:), 'Interpreter', 'latex');
        legend('Groundtruth', '$$z$$', '$$x$$', '$$x_{pred}$$', ...
            'location', 'best', 'Interpreter', 'latex');
        hold off;
    end

    orienFig = figure();
    figure(orienFig);
    t = tiledlayout(2, 4, 'TileSpacing','compact');
    title(t, 'Orientation Filter Results', 'FontSize', 20);
    tiledtitles = [ ...
        "Orientation Quat $$q_d$$"; 
        "Orientation Quat $$q_i$$";
        "Orientation Quat $$q_j$$"; 
        "Orientation Quat $$q_k$$";
        "Angular Axis $$Z$$";
        "Angular Axis $$Y$$"; 
        "Angular Axis $$X$$"; 
    ];
    colidx = [5:8, 16:18];
    for i = 1:7    % orientation & angular velocities
    % for i = 7:10    % only orientation
        nexttile, hold on

        plot(groundtruth(2:end,1), groundtruth(2:end, colidx(i)));
        plot(zHis(:,1), zHis(:,colidx(i)), '--^');
        plot(statHis(:,1), statHis(:,colidx(i)), 'm');
        plot(statPredHis(:,1) ,statPredHis(:,colidx(i)), '--x');

        set(gca, 'FontSize', 12);
        set(get(gca, 'XLabel'), 'String', 'time', 'Interpreter', 'latex');
        set(get(gca, 'YLabel'), 'String', 'value', 'Interpreter', 'latex');
        set(get(gca, 'Title'), 'String', tiledtitles(i,:), 'Interpreter', 'latex');
        legend('Groundtruth', '$$z$$', '$$x$$', '$$x_{pred}$$', ...
            'location', 'best', 'Interpreter', 'latex');
        hold off;
    end
end