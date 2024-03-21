function [tau_comp]=compute_tau_dampedvelocities(Cost,vel,tau_0)

% tau_comp=(Cost*vel.^2).*sign(vel)+tau_0;
% tau_comp=(Cost*vel.^2).*sign(vel)+tau_0;
% tau_comp(1:2)=max(tau_comp(1:2));
% tau_comp(3:4)=max(tau_comp(3:4));
% tau_comp=(Cost/4*sqrt(sqrt(abs(vel)))).*sign(vel)+tau_0;
tau_comp=(Cost*vel)+tau_0;
end