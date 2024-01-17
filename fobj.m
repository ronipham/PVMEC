function fitness = fobj(X, U, fi0, f_MEC)
N = size(U,1);
M = size(U,2);
Utility = size(1,N);
%contraints
X1 = zeros(N,M);
for i = 1:N
    for j = 1:M
         X1(i,j) = X(M*(i-1)+j);
    end
end

muy = 1e14;    
penalty1 = 0;
penalty2 = 0;
for i = 1:N
    Utility(i) = sum(X1(i,:).*U(i,:));
    H = (sum(X1(i,:)) > 1);
    penalty1 = penalty1 + muy*H*((sum(X1(i,:))-1)^2);
end

for j=2:M
    H = (sum(X1(:,j)) > 1);
    penalty2 = penalty2 + muy*H*((sum(X1(:,j))-1)^2);
end

H = (sum(X1(:,1).*fi0) > f_MEC);
penalty3 = muy*H*((sum(X1(:,1).*fi0) - f_MEC)^2);
fitness = -sum(Utility)+penalty1+penalty2+penalty3;




 
