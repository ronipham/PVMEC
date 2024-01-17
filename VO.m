function U_VO = VO(para, Nj, c_i, d_i, f_veh, T_c_all, T_i_local)
Utility = zeros(para.N,1);
strategy = ones(para.N,1);
U = zeros(para.N,1);

for i=1:para.N
    t(i,1) = T_i_local(i,1);
    for j=Nj{i}
        a(i,j) = T_i_local(i,j)/(T_i_local(i,j)+T_c_all(i,j)+c_i(i)/f_veh(j-1));
        t(i,j) = (1-a(i,j))*T_i_local(i,j);
        U(i,j)=max(para.gt*(T_i_local(i,j)-t(i,j))-para.p_j(j)*f_veh(j-1)-para.c_comm*a(i,j)*d_i(i),0);
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
U_VO= sum(Utility);
end
