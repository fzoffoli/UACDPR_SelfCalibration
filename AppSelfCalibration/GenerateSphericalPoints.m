function points = GenerateSphericalPoints()

    % Define parameters
    r = flip(0.1:0.1:0.4); % Radius values
    num_r = numel(r);
    num_azimuth = 5; % Number of angles in azimuth direction
    num_polar = 8; % Number of angles in polar direction
    
    % Preallocate space for points
    num_points = num_azimuth * num_polar * num_r;
    points = zeros(3,num_points);
    
    % Generate points
    idx = 1;
    for i = 1:num_r
        radius = r(i);
        for theta = linspace(0, 2*pi-2*pi/num_polar, num_polar)
            for phi = linspace(0, pi, num_azimuth)
                x = radius * sin(phi) * cos(theta);
                y = radius * sin(phi) * sin(theta);
                z = radius * cos(phi);
                points(:,idx) = [x; y; z];
                idx = idx + 1;
            end
        end
    end
end