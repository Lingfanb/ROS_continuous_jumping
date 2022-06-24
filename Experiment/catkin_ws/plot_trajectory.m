clc;
clear;

load("pos.txt")
load("src/laikago_ros/optimization_data/jump2D/jumping_x70_z0_cs503050_060322_motor_constraints_interp.mat")
dt = 0.001;
time= 0:dt:dt*(length(pos(:,1))-1);                                                                                         
% % 
figure();
stairs(time,pos(:,10))
hold on;
stairs(time,pos(:,12))
xlabel("time/s")
ylabel("Force/N")
legend("Front leg","Rear leg",Location="best")
% % %
% % title("jumping MPC Force ")

%plot trajectory & ref
inih = 0.15;
figure()
plot(pos(:,4),pos(:,6));hold on
plot(Q(1,:),Q(2,:)+inih)
legend("actual","ref")


% figure()
% load("MPC_F.txt")
% stairs(time,MPC_F(:,3))
% xlabel("time/s")
% ylabel("Force/N")
% title("MPC out front z")
% 
% figure()
% stairs(time,MPC_F(:,6))
% xlabel("time/s")
% ylabel("Force/N")
% title("MPC out rear z")