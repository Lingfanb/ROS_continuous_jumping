clc;clear
load('jumping_50cm_stepping_stone_noshaking_interp_cs10202535.mat');
F = [Ff;Fr];

filename = 'data_F.csv';
writematrix(F,filename);