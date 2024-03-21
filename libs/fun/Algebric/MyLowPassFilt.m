function now_filt_value = MyLowPassFilt(now_raw_value,prev_filt_value,t,dt,freq)

alpha = freq * dt;
% alpha=dt/(1/(2*pi*freq)+ dt);
if (t > 0)
    now_filt_value = prev_filt_value + alpha * (now_raw_value - prev_filt_value);
else
    now_filt_value = now_raw_value;
end

end