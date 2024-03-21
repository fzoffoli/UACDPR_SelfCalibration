function Pulley = ChangeFixedFrame(Pulley,NewFrame)

Pulley.FixedFrame=NewFrame;
Pulley.SwivelAxis=Pulley.FixedFrame{3};

end

