clc;clear
load('jumping_85cm_stepping_stone_noshaking_interp_cs10303040.mat');
F = [Ff;Fr];

filename = '85cm_F.csv';
writematrix(F,filename);
