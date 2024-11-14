main()


function main()

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
    thetaMin = 10^-3;
    
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
        desPos = traj(i, :)';

        while true

            endEffectorPos = forwardKinematicsPlanar(linkLengths, currJointAngles, numLinks);
    
            delta_pos = desPos - endEffectorPos;
            J = computeJacobian(linkLengths, currJointAngles, numLinks);
            delta_theta = DLS(J, lambda, delta_pos);

            currJointAngles = currJointAngles + delta_theta; %TODO: verify compatible sizes
            % TODO: add check for arm reaching edge of workspace
            if norm(delta_theta) < thetaMin
               break;
            end

        end
        
        outputAngles(i,:) = currJointAngles';
    end

    writematrix(outputAngles, 'angles.dat');
end


%%Forward Kinematics
function pos = forwardKinematicsPlanar(linkLengths, jointAngles, linkCount)
   theta = 0;
   x = 0;
   y = 0;

   for i = 1:linkCount
       link = linkLengths(i);
       theta = theta + jointAngles(i);
       
       x = x + link*cos(theta) ;
       y = y + link*sin(theta);
   end

   pos = [x;y];
end

function jacobian = computeJacobian(linkLengths, jointAngles, linkCount)
   jacobian = zeros(2, linkCount);
   theta = 0;
   Jx = 0;
   Jy = 0;

   for i = 1:linkCount
       link = linkLengths(i);
       theta = theta + jointAngles(i);
       
       Jx = Jx - link*sin(theta);
       Jy = Jy + link*cos(theta);

       jacobian(:, i) = [Jx;Jy];
   end
end

%TODO: check J has valid transpose
function delta_theta = DLS(jacobian, lambda, delta_pos)
    J = jacobian;
    I = eye(size(J, 1));
    A = (J'/(J*J' + lambda^2.*I) );
    delta_theta = A*delta_pos ;
end