main()


function out = main()

    %Read in joint position values
    armFileID = fopen('arm','r');
    sizeArm = [2 Inf];
    arm = fscanf(armFileID, '%f %f', sizeArm);
    arm = arm';
    
    % Initialize robot arm values
    numLinks = arm(1,1);
    lambda = arm(1,2);
    arm = arm(2:end,:);
    currJointAngles = arm(:,2);
    linkLengths = arm(:,1);
    thetaMin = 1;
    
    %Read in trajectory data
    trajFileID = fopen('trajectory', 'r');
    sizeTraj = [2 Inf];
    traj = fscanf(trajFileID, '%f %f', sizeTraj);
    traj = traj';
    numPositions = traj(1,1);
    traj = traj(2:end,:);
    

    % Initialize output file
    outputAngles = zeros(numPositions, numLinks);
    
    
    for i = 1:numPositions 
        desPos = [traj(i, :), 0]'; % Add zero z val and turn pos into column vector
        
        % while true
        for loop = 1:2
            dhParams = constructDHTable(linkLengths, currJointAngles, numLinks);
            endEffectorPos = forwardKinematicsPlanar(dhParams, numLinks);
    
            delta_pos = desPos - endEffectorPos;
            J = computeJacobian(dhParams, numLinks);
            delta_theta = DLS(J, lambda, delta_pos);
            
            currJointAngles = currJointAngles + delta_theta; %TODO: verify compatible sizes
    
            % TODO: add check for arm reaching edge of workspace
            % TODO: Take magnitude here, since vectors
            %if delta_theta < thetaMin
            %    break;
            %end
        end
        
        outputAngles(i,:) = currJointAngles';
    end

    out = outputAngles;
end

%%Forward Kinematics
function dhTable = constructDHTable(linkLengths, jointAngles, linkCount)
    
    if size(linkLengths,1) ~= linkCount || size(jointAngles,1) ~= linkCount
        error('Number of arm parameters does not match number of links');
    end
    
    alpha = zeros(linkCount, 1); % Planar robot, no link twist
    a = linkLengths;
    d = zeros(linkCount, 1); % Planar robot, no link offset
    theta = jointAngles;
    dhTable = [alpha, a, d, theta];
end

function homoTransform = constructHomoTransform_ij(dhParams, j)

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

function homoTransform = constructHomoTransform_0n(dhParams, n)
    for j = 1:n
        if j == 1
            T0j = constructHomoTransform_ij(dhParams, j);
        else
            Tij = constructHomoTransform_ij(dhParams, j);
            T0j = T0j*Tij;
        end
    end 
    
    homoTransform = T0j;
end

function pos = forwardKinematicsPlanar(dhParams, linkCount)
   
    T0j = constructHomoTransform_0n(dhParams, linkCount) ;  

    pos = T0j(1:3, 4);
end

function jacobian = computeJacobian(dhParams, linkCount)
   jacobian = zeros(6, linkCount);
   T0n = constructHomoTransform_0n(dhParams,linkCount);
    
   on = T0n(1:3, 4);
   
   z0 = [0;0;1];
   o0 = [0;0;0];
   Jv0 = cross(z0, (on-o0));
   Jw0 = z0;
   J0 = [ Jv0; Jw0 ];
   jacobian(:,1) = J0;

   for j = 1:linkCount-1
       T0j = constructHomoTransform_0n(dhParams, j);
       zj = T0j(1:3, 3);
       oj = T0j(1:3, 4);
       Jv = cross(zj, (on-oj));
       Jw = zj;
       Jj = [ Jv; Jw ];
       jacobian(:,j+1) = Jj;
   end
end

%TODO: check J has valid transpose
function delta_theta = DLS(jacobian, lambda, delta_pos)
   J = jacobian;
   I = eye(size(J, 1));
   A = J*J';
   B = A + lambda^2*I;
   C = J'/B;
   D = C * delta_pos;
   %delta_theta = (J'/(J*J' + lambda^2.*I) )*delta_pos ;
end