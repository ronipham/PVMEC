function para = paras
% default parameters
para.N = 20;
para.M = 10;
para.r =  200;
para.r_u = 100;

para.Tx_Power = 30; %dBm
para.Power_Noise = -114; %dBm
para.interference = 0;
para.B = 10; %MHz

para.gt = 2.5;
para.p_j(1)=0.1;
para.p_j(2:para.M+1) = 0.1*ones(para.M,1);

%communication unit price
para.c_comm = 0.15*10^-3;
end