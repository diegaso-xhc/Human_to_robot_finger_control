clear all,
close all;
clc

filename='sample';
T = readtable(strcat(filename,'.csv'));
T = T(2:end,2:end);
stackedplot(T);
