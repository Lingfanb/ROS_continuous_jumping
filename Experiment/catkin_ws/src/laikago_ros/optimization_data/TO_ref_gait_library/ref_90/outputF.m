clc;clear
load('jumping_90cm_stepping_stone_noshaking_interp_cs10304040.mat');
F = [Ff;Fr];

filename = '90cm_F.csv';
writematrix(F,filename);
