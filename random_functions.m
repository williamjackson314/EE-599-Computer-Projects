function random_functions()
end

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