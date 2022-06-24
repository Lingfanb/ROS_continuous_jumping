clc;clear
load('jumping_70cm_stepping_stone_noshaking_interp_cs10302540.mat');
F = [Ff;Fr];

filename = '70cm_F.csv';
writematrix(F,filename);