%Read in joint position values
armFileID = fopen('arm','r');
sizeArm = [2 Inf];
arm = fscanf(armFileID, '%f %f', sizeArm);
arm = arm';
numLinks = arm(1,1);
lambda = arm(1,2);
arm = arm(2:end,:);

%Read in trajectory data
trajFileID = fopen('trajectory', 'r');
sizeTraj = [2 Inf];
traj = fscanf(trajFileID, '%f %f', sizeTraj);
traj = traj';
numPositions = traj(1,1);
traj = traj(2:end,:);


dhParams = constructDHTable(arm, numLinks);
endEffectorPos = forwardKinematicsPlanar(dhParams, numLinks);
disp(endEffectorPos);

%%Forward Kinematics
function dhTable = constructDHTable(lengthsAndAngles, linkCount)
    alpha = zeros(linkCount, 1);
    a = lengthsAndAngles(:,1);
    d = zeros(linkCount, 1);
    theta = lengthsAndAngles(:,2);
    dhTable = [alpha, a, d, theta];
end

% TODO: ensure link num j is within DH table bounds
function homoTransform = constructHomoTransform(dhParams, j)


    % Transform from i to j
    alpha_j = dhParams(j, 1);
    a_j = dhParams(j, 2);
    d_j = dhParams(j, 3);
    theta_j = dhParams(j, 4);

    Tij = [cos(theta_j),-sin(theta_j)*cos(alpha_j), sin(theta_j)*sin(alpha_j),  a_j*cos(theta_j);
           sin(theta_j), cos(theta_j)*cos(alpha_j), -cos(theta_j)*sin(alpha_j), a_j*sin(theta_j);
           0,            sin(alpha_j),              cos(alpha_j),               d_j;
           0,            0,                         0,                          1];

    homoTransform = Tij;
end

function pos = forwardKinematicsPlanar(dhParams, linkCount)
   
    for j = 1:linkCount
        if j == 1
            T0j = constructHomoTransform(dhParams, j);
        else
            Tij = constructHomoTransform(dhParams, j);
            T0j = T0j*Tij;
        end
    end    

    pos = T0j(1:2, 4);
end



function jointAngles = inverseKinematics(currPos, nextPos)


end