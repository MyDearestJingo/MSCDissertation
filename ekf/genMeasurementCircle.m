function z = genMeasurementCircle(estimations)
    difft = estimations(2,1)-estimations(3,1);
    [center,rad,v1,v2] = circlefit3d(estimations(1,2:4), ...
                            estimations(2,2:4), ...
                            estimations(3,2:4));

    u = estimations(2,2:4) - center;
    v = estimations(3,2:4) - center;

    axang = vrrotvec(u,v);
    rotAxis = axang(1:3);
    angVel = axang(4)/difft;

    quatRot = squatmultiply(estimations(3,5:8), squatinv(estimations(2,5:8)));
    quatRot = squatnormalize(quatRot);
    eulRot = squat2eul(quatRot)';
    z = [
        estimations(3,2:4)';                % position
        estimations(3,5:8)';                % orientation
        center';                            % circle center
        % v3/norm(v3);                      % normal vector of circle plane
        axang2quat(vrrotvec([0,0,1],rotAxis))'; % orientation
        angVel;                             % angular velocity
        eulRot/difft;
    ];

end