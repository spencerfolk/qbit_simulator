%%% Script for sweeping over different prop effiencies and forward flight
%%% speeds, saving the relevant data and plotting stuff to gain insight on
%%% prop efficiency and forward flight. 
%%% Spencer Folk 2020
clear
clc
close all

eta = 0:0.05:1;
V_s = 1:30;

% Tabular data: 
% [eta V_s T_top(end) T_bot(end) theta(end) mean(alpha_e) mean(Vw) mean(Va) mean(L) mean(D) mean(M_air)];
table = [];

% Now run through sweep simulation, saving data at each point
for i = 1:length(V_s)
    for j = 1:length(eta)
        fprintf("Iteration count: %d",(i-1)*length(eta) + j)
        fprintf(" of %d\n",length(V_s)*length(eta))
        data_out = qbit_simulate_master_sweep(eta(j),V_s(i));
        table = [table ; data_out];
    end
end

table(:,6:8) = table(:,6:8)*180/pi;  % Convert angles to deg
csvwrite("prop_wash_sweep.csv",table);