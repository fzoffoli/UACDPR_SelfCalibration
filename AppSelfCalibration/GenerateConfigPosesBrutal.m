function [Z,k] = GenerateConfigPosesBrutal(MyUACDPR,grid_axis,pose_bounds,force_bounds)
%GENERATECONFIGPOSESBRUTAL compute a pose set with equally distributed
%poses within the position bounds given as input.

x = linspace(pose_bounds(1,2),pose_bounds(1,1),grid_axis(1));
y = linspace(pose_bounds(2,2),pose_bounds(2,1),grid_axis(2));
z = linspace(pose_bounds(3,2),pose_bounds(3,1),grid_axis(3));

Z = zeros(6,prod(grid_axis));
cnt = 1;

for k=1:grid_axis(3)
    for j=1:grid_axis(2)
        for i=1:grid_axis(1)
            
            igsp_opts = optimoptions('fmincon','Display','none');
            tau_eps=fmincon(@(tau_eps)0,[mean(force_bounds)*ones(4,1);0;0;0], ...
                [],[],[],[],[force_bounds(1)*ones(4,1);-pi/2;-pi/2;-pi/2],[force_bounds(2)*ones(4,1);pi/2;pi/2;pi/2], ...
                @(tau_eps)IGSPconstraints(MyUACDPR,[x(i);y(j);z(k)],tau_eps),igsp_opts);
            zeta = [x(i);y(j);z(k);tau_eps(5:end)]; %initial equilibrium pose computation
            tau = tau_eps(1:4);

            % zeta_f = fsolve(@(pose_free)InverseGSproblem(MyUACDPR,[x(i);y(j);z(k);0],pose_free),zeros(2,1));
            % zeta = MyUACDPR.PermutMatrix'*[x(i);y(j);z(k);0;zeta_f];
            % tau = CalcInverseStaticsAndGradient(MyUACDPR,zeta);
            if (any(tau<force_bounds(1)) || any(tau>force_bounds(2)))
                warning('Tension bounds exceeded in pose: %d', cnt);
            end
            Z(:,cnt) = zeta;

            cnt = cnt+1;
        end
    end
end
k = length(Z);
% Z = reshape(Z,[k*6 1]);
if grid_axis==3*ones(3,1)
    Z(:,[4 6 10 11 12 13 14 15 16 17 18 22 24]) = Z(:,[6 4 18 17 16 13 14 15 12 11 10 24 22]);
elseif grid_axis==2*ones(3,1)
    Z(:, [4 3 5 6 7 8]) = Z(:,[3 4 7 8 6 5]);
end
end