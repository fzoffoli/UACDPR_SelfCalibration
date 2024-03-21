function der = FiniteDifferentiation(val,val_prev,diff_prev,t,dt)
freq=10;
der = (val-val_prev)/dt;
der = MyLowPassFilt(der,diff_prev,t,dt,freq);

end