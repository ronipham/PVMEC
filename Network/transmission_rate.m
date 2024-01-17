function rate = transmission_rate(Tx_Power, B, d, Power_Noise, interference)
Tx_Power = power(10, Tx_Power / 10) / 1000; % dBm to Watts
Power_Noise = power(10, Power_Noise/ 10)/1000; % dBm to Watts
channel_gain = power(d,-3.4);
SINR = Tx_Power * channel_gain / (Power_Noise + interference);
rate = B * log2(1 + SINR);
rate = rate * 1000 / 8; %Mbps -> KBps
end
% Pr = 46dBm, d=100m, noise=-100dBm, bandwidth = 10MHz, n=20  ---> 10.96 Mbps
