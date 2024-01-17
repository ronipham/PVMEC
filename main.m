clear all
clc
addpath(genpath('Network'))

para = paras;
f_MEC = 10;

U_Proposed = [];
U_FO = [];
U_MO = [];
U_VO = [];
U_Greedy = [];

for dd_i=50:50:300
    d_i = dd_i*ones(para.N,1);
    [x,y, x_m, y_m] = Network(para.N, para.M, para.r);
    filename = sprintf('Network/DataSizefile_%d.mat',dd_i);
    if exist(filename, 'file')
        load(filename);
    else
        %d_i = 50+100*rand(N,1); %U[50,150] KB
        c_i=(1+rand(para.N,1)); %U[1, 2]Megacycles/KB -> gigacycles
        f_local = 0.5+0.5*rand(para.N,1);
        f_veh = 1 + 0.5*rand(para.M,1);
        save(filename,'c_i','f_local','f_veh');
    end
    c_i=0.001*d_i.*c_i; %U[1, 2]Megacycles/KB -> gigacycles

    %Transmission time
    rate_all = zeros(para.N,para.M+1);
    T_c_all = zeros(para.N,para.M+1);
    Nj = cell(para.N,1); %using cell to save list of number of PV j within the range of MD i
    for i = 1:para.N
        di0 = sqrt(x(i)^2 + y(i)^2);
        rate_all(i,1) = transmission_rate(para.Tx_Power, para.B, di0, para.Power_Noise, para.interference);
        T_c_all(i,1) = d_i(i)/rate_all(i,1);
        for j = 2:para.M+1
            dij = sqrt((x(i) - x_m(j-1))^2 + (y(i) - y_m(j-1))^2); % index in x_m starts from 1, so it is j-1
            if dij <= para.r_u
                Nj{i} = [Nj{i} j];
                rate_all(i,j) = transmission_rate(para.Tx_Power, para.B, dij, para.Power_Noise, para.interference);
                T_c_all(i,j) = d_i(i)/rate_all(i,j);
            end
        end
    end

    %Computation time
    T_i_local = zeros(para.N,para.M+1);
    for i = 1:para.M+1
        T_i_local(:,i) = c_i./f_local;
    end

    U_Proposed(end+1:end+1) =  Proposed(para, Nj, c_i, d_i, f_MEC, f_local, f_veh, rate_all, T_c_all, T_i_local);
    U_FO(end+1:end+1) =  FO(para, Nj, c_i, d_i, f_MEC, f_veh, T_c_all, T_i_local);
    U_MO(end+1:end+1) =  MO(para, c_i, d_i, f_MEC, f_local, rate_all);
    U_VO(end+1:end+1) =  VO(para, Nj, c_i, d_i, f_veh, T_c_all, T_i_local);
    U_Greedy(end+1:end+1) =  Greedy(para, Nj, c_i, d_i, f_MEC, f_local, f_veh, rate_all, T_c_all, T_i_local);
end

index = 1:1:length(U_Proposed);
index = index*50;
figure;
plot(index, U_FO, '-mv', index, U_MO, '-k*', 'linewidth', 1, 'markers', 7);
hold on
plot(index, U_VO, '-pentagram', 'Color' , [0.9290 0.6940 0.1250], 'linewidth', 1, 'markers', 7);
hold on
plot(index, U_Greedy, '-rs', index, U_Proposed, '-bo', 'linewidth', 1, 'markers', 7);
set(gca,'FontSize',14,'XLim',50*[1 length(U_Proposed)], 'YLim',[0 14]);
xlabel('Input data size (KB)','FontSize',16);
ylabel('System utility','FontSize',16);
grid on;
legend('FO', 'MO', 'VO', 'Greedy', 'Proposed', 'FontSize',12);
