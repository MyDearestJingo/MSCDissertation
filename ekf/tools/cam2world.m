%% Convert object pose from camera frame to world frame
%% @Params:
%   - camPose: a 7-D row vector including 3-D position and an orientation 
%       quaternion of the camera in world frame
%   - eulOpticalJoint: 3-D row vector indicating camera optical frame rotation 
%       based on camera pose
%   - objPose: a 7-D row vector including 3-D position and an orientation
%       quaternion of the object
%% @Return: 
%   - pose: a 7-D row vector including 3-D position and an orientation 
%       quaternion of the object in world frame
function pose = cam2world(camPose, eulOpticalJoint, objPose)
    cwR = quat2tform(camPose(4:7)) * eul2tform(eulOpticalJoint);
    cwH = trvec2tform(camPose(1:3)) * cwR;
    pose = zeros(1,7);
    pose(1:4) = (cwH * [objPose(1:3), 1]')';
    pose(4:7) = tform2quat(cwR * quat2tform(objPose(4:7)));
end