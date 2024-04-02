function points = GenerateSphericalPoints()
    % Define parameters
    r = 3:3:15; % Radius values
    num_r = numel(r);
    num_azimuth = 4; % Number of azimuth angles
    num_polar = 8; % Number of polar angles
    
    % Preallocate space for points
    num_points = num_azimuth * num_polar * num_r;
    points = zeros(3,num_points);
    
    % Generate points
    idx = 1;
    for i = 1:num_r
        radius = r(i);
        for theta = linspace(0, 2*pi-2*pi/num_polar, num_polar)
            for phi = linspace(0, pi-pi/num_azimuth, num_azimuth)
                x = radius * sin(phi) * cos(theta);
                y = radius * sin(phi) * sin(theta);
                z = radius * cos(phi);
                points(:,idx) = [x, y, z];
                idx = idx + 1;
            end
        end
    end
end