function cable_length_measures = FixActuatorsTransmissionRatio(cable_length_measures_wrong)
%FIXACTUATORSTRANSMISSIONRATIO fixes the cable length variation computed
% from the experiments on the IRMA-4U with the old transmission ratios: 
% -2.747236952757055e-08; -2.732421689203843e-08;
% -2.713028601161863e-08; -2.731442160182809e-08.
% and applies the new transmission ratio: -2.676436866832799e-08.

% This is the relation between the motor counts and the cable length
% length = motor_counts*transmission_ratio;

winch_1_ratio = -2.747236952757055e-08;
winch_2_ratio = -2.732421689203843e-08;
winch_3_ratio = -2.713028601161863e-08;
winch_4_ratio = -2.731442160182809e-08;

winch_new_ratio = -2.676436866832799e-08;

cable_length_measures = [cable_length_measures_wrong(1,:)./winch_1_ratio;
                        cable_length_measures_wrong(2,:)./winch_2_ratio;
                        cable_length_measures_wrong(3,:)./winch_3_ratio;
                        cable_length_measures_wrong(4,:)./winch_4_ratio].*winch_new_ratio;
end