clc;clear
load('jumping_55cm_stepping_stone_noshaking_interp_cs10202535.mat');
F = [Ff;Fr];

filename = '55cm_F.csv';
writematrix(F,filename);