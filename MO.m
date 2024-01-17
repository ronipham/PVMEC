function U_MO = MO(para, c_i, d_i, f_MEC, f_local, rate_all)
Utility = zeros(para.N,1);
strategy = zeros(para.N,1);
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

for ii = 1:para.N
    if strategy(ii)~=1
        Utility(ii) = 0;
    end
end

U_MO = sum(Utility);
end
