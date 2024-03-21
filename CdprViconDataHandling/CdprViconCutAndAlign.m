function st_out = CdprViconCutAndAlign(eps_idx,epsilon_rv,t_r,cable_length_r,swivel_r,tensions_r,pose_v,t_v,show)

t_rv = linspace(t_r(eps_idx(1)),t_r(eps_idx(end)),length(epsilon_rv));
t_r_cut = t_r(eps_idx(1):eps_idx(end));
cable_length_r_cut = cable_length_r(:,eps_idx(1):eps_idx(end));
swivel_r_cut = swivel_r(:,eps_idx(1):eps_idx(end));
tensions_r_cut = tensions_r(:,eps_idx(1):eps_idx(end));
epsilon_rv = spline(t_rv,epsilon_rv,t_r_cut);
[~,idx_max_r] = max(epsilon_rv(1,:));
[~,idx_max_v] = max(pose_v(4,:));
delta_N = idx_max_r-idx_max_v;
delta_t = t_r_cut(idx_max_r)-t_v(idx_max_v);
t_aligned = t_r_cut-delta_t;


if show
    figure()
    plot(t_v,pose_v(4,:))
    hold on
    plot(t_aligned,epsilon_rv(1,:))
    title('Check data aligning')
end

% cutting tails vicon
cut_v_start=0;
cut_v_end=0;
for i=1:length(pose_v)
    if t_v(i)>=t_aligned(1) && cut_v_start==0
        cut_v_start = i;
    end
    if t_v(i)>=t_aligned(end) && cut_v_end==0
        cut_v_end = i;
    end
end
t_v = t_v(cut_v_start:cut_v_end);
pose_v = pose_v(:,cut_v_start:cut_v_end);

% cutting tails robot
cut_r_start=0;
cut_r_end=0;
for i=1:length(t_aligned)
    if t_aligned(i)>=t_v(1) && cut_r_start==0
        cut_r_start = i;
    end
    if t_aligned(i)>=t_v(end-1) && cut_r_end==0
        cut_r_end = i;
    end
end
st_out.t = t_aligned(cut_r_start:cut_r_end);
st_out.epsilon = epsilon_rv(:,cut_r_start:cut_r_end);
st_out.cable_length = cable_length_r_cut(:,cut_r_start:cut_r_end);
st_out.swivel = swivel_r_cut(:,cut_r_start:cut_r_end);
st_out.tensions = tensions_r_cut(:,cut_r_start:cut_r_end);

sampling_ratio = (t_v(2)-t_v(1))/(st_out.t(2)-st_out.t(1));
st_out.vicon_idx = 1:int64(sampling_ratio):length(st_out.t);
st_out.pose_v = pose_v(:,1:end-1);

if show
    figure()
    plot(st_out.t(st_out.vicon_idx),st_out.pose_v(4,:))
    hold on
    plot(st_out.t,st_out.epsilon(1,:))
    title('Check data cutting')
end
end