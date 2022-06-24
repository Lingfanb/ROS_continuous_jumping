clc;clear
load('jumping_65cm_stepping_stone_noshaking_interp_cs10202540.mat');
F = [Ff;Fr];

filename = '65cm_F.csv';
writematrix(F,filename);