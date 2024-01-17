function U_Greedy = Greedy(para, Nj, c_i, d_i, f_MEC, f_local, f_veh, rate_all, T_c_all, T_i_local)
Utility = zeros(para.N,1);
strategy = zeros(para.N,1);
U = zeros(para.N,para.M+1);
a_star = zeros(para.N,1);
fi0 = zeros(para.N,1);
weightK = zeros(para.N,1);

for i=1:para.N
    B = para.gt*c_i(i)/f_local(i)-para.c_comm*d_i(i);
    C = para.p_j(1)*c_i(i)*f_local(i)*rate_all(i,1);
    D = c_i(i)*rate_all(i,1);
    E = D + d_i(i)*f_local(i);
    A = (D-sqrt(C*D/B))/E;

    if B<=0
        a_star(i) = 0;
    elseif A<=0
        a_star(i) = 0;
    else
        a_star(i) = A;
    end
    fi0(i) = (a_star(i)*c_i(i))/(((1-a_star(i))*c_i(i)/f_local(i))-(a_star(i)*d_i(i)/rate_all(i,1)));
    Utility(i)=B*a_star(i)-(C*a_star(i))/(D-E*a_star(i));
    U(i,1) = Utility(i);
    weightK(i) = Utility(i)/fi0(i);
end
[order, sortIndex] = sort(weightK, 'descend');
eLimit = 0;
for i = 1:length(sortIndex)
    eLimit = eLimit + fi0(sortIndex(i));
    if eLimit > f_MEC
        break
    else
        strategy(sortIndex(i)) = 1;
    end
end

N_veh = [];
for i=1:para.N
    if strategy(i)~=1
        N_veh = [N_veh i];
        for j=Nj{i}
            a(i,j) = T_i_local(i,j)/(T_i_local(i,j)+T_c_all(i,j)+c_i(i)/f_veh(j-1));
            t(i,j) = (1-a(i,j))*T_i_local(i,j);
            U(i,j)=max(para.gt*(T_i_local(i,j)-t(i,j))-para.p_j(j)*f_veh(j-1)-para.c_comm*a(i,j)*d_i(i),0);
        end
    end
end

U1 = U(:, 2:end);
U1(U1==0)=-1000000;
U1 = -1*transpose(U1);
M1 = matchpairs(U1,1000);

for i = 1:size(M1,1)
    strategy(M1(i,2)) = M1(i,1)+1;
    Utility(M1(i,2)) = U(M1(i,2), M1(i,1)+1);
end

for i=1:para.N
    if(strategy(i)==0)
        Utility(i)=0;
    end
end
U_Greedy = sum(Utility);
end




