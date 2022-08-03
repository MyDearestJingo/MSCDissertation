function data = loadData(path)
    data = readmatrix(path);
    % fuse secs and nsecs columns of each timestamp 
    data(:,2) = data(:,1) + data(:,2).*1e-9;
    % ! ROS and MatLab have different quaternion vector representation order
    if size(data, 2) < 10 % DOPE estimation data
        % insert column 9 (w of quat) before column 6 (quat x) and drop column 1 (secs)
        data = [data(:,2:5), data(:,9), data(:,6:8)];
    else % Gazebo groundtruth data
        % change Euler order from XYZ to ZYX
        data = [data(:,2:5), data(:,9), data(:,6:8), data(:,10:12), ...
                data(:,15), data(:, 14), data(:, 13)];
    end
end