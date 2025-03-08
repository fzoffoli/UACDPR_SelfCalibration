function tau = ComputeStaticTensions(UACDPR)
%COMPUTESTATICTENSIONS Computes the static tensions for a given EE pose
%   INPUT: UACDPR obj with the updated pose
%   OUTPUT: tension vector that satisfies static constraints
% NB the equilibrium condition and tension feasibility are not ensured.

tau = UACDPR.GeomJac.Cables_par'*UACDPR.Wrench;
end

