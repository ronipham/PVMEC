function U_FO = FO(para, Nj, c_i, d_i, f_MEC, f_veh, T_c_all, T_i_local)

Utility = zeros(para.N,1);
strategy = ones(para.N,1);
U = zeros(para.N,1);

for i=1:para.N
    for j=Nj{i}
        t(i,j) = T_c_all(i,j)+c_i(i)/f_veh(j-1);
        U(i,j)=max(para.gt*(T_i_local(i,j)-t(i,j))-para.p_j(j)*f_veh(j-1)-para.c_comm*d_i(i),0);
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

N0 = [];
for ii = 1:para.N
    if strategy(ii)==1
        N0 =[N0 ii];
    end
end


for i=1:para.N
    if strategy(i)==1
        b(i) = sqrt(c_i(i))/sum(sqrt(c_i(N0)));
        t(i,1) = T_c_all(i,1)+c_i(i)/(b(i)*f_MEC);
        Utility(i)=max(para.gt*(T_i_local(i,1)-t(i,1))-para.p_j(1)*b(i)*f_MEC-para.c_comm*d_i(i),0);
    end

end
U_FO= sum(Utility);
end
