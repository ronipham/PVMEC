function U_Proposed = Proposed(para, Nj, c_i, d_i, f_MEC, f_local, f_veh, rate_all, T_c_all, T_i_local)

K1 = zeros(1,para.N);
K2 = zeros(1,para.N);
K3 = zeros(1,para.N);
K4 = zeros(1,para.N);
a_star = zeros(1,para.N);
fi0 = zeros(para.N,1);
U = zeros(para.N,1);
a = zeros(para.N,1);
t = zeros(para.N,1);

% Joint Offloading Ratio and Resource Allocation
for i=1:para.N
    K1(i) = para.gt*c_i(i)/f_local(i)-para.c_comm*d_i(i);
    K2(i) = para.p_j(1)*c_i(i)*f_local(i)*rate_all(i,1);
    K3(i) = c_i(i)*rate_all(i,1);
    K4(i) = K3(i) + d_i(i)*f_local(i);
    A = (K3(i)-sqrt(K2(i)*K3(i)/K1(i)))/K4(i);
    if K1(i)<=0
        a_star(i) = 0;
    elseif A<=0
        a_star(i) = 0;
    else
        a_star(i) = A;
    end
    fi0(i) = (a_star(i)*c_i(i))/(((1-a_star(i))*c_i(i)/f_local(i))-(a_star(i)*d_i(i)/rate_all(i,1)));
    % MEC Utility
    U(i,1)=K1(i)*a_star(i)-(K2(i)*a_star(i))/(K3(i)-K4(i)*a_star(i));
    weightK(i) = U(i,1)/fi0(i);
    % Vehicle Utility
    for j=Nj{i}
        a(i,j) = T_i_local(i,j)/(T_i_local(i,j)+T_c_all(i,j)+c_i(i)/f_veh(j-1));
        t(i,j) = (1-a(i,j))*T_i_local(i,j);
        U(i,j)=para.gt*(T_i_local(i,j)-t(i,j))-para.p_j(j)*f_veh(j-1)-para.c_comm*a(i,j)*d_i(i);
    end
end

dim = para.N;
D=(para.M+1)*para.N;
SearchAgents_no = 20;
% initialize position vector and score for the leader
Leader_pos=zeros(1,dim);
Leader_score=inf; %change this to -inf for maximization problems


%---------------------------Initilize the positions of search agents--------------------------------%
Positions = zeros(SearchAgents_no,D);
[order, sortIndex] = sort(weightK, 'descend');
eLimit = 0;
strategy = zeros(para.N,1);
for i = 1:length(sortIndex)
    eLimit = eLimit + fi0(sortIndex(i));
    if eLimit > f_MEC
        break
    else
        Positions(1,(sortIndex(i)-1)*(para.M+1)+1) = 1;
        strategy(sortIndex(i)) = 1;
    end
end

UU = zeros(para.N,1);

for i=1:para.N
    if strategy(i)~=1
        for j=Nj{i}
            a(i,j) = T_i_local(i,j)/(T_i_local(i,j)+T_c_all(i,j)+c_i(i)/f_veh(j-1));
            t(i,j) = (1-a(i,j))*T_i_local(i,j);
            UU(i,j)=para.gt*(T_i_local(i,j)-t(i,j))-para.p_j(j)*f_veh(j-1)-para.c_comm*a(i,j)*d_i(i);
        end
    end
end

U1 = UU(:, 2:end);
U1(U1==0)=-1000000;
U1 = -1*transpose(U1);
M1 = matchpairs(U1,1000);
for i = 1:size(M1,1)
    Positions(1, (M1(i,2)-1)*(para.M+1)+M1(i,1)+1) = 1;
end

%----------------------------------------------------------------------------------------%

for i = 2:size(Positions,1) 	% For each seach agent
    userID=1;
    list_node = 1:para.M+1;
    eLimit = 0;
    for j = 1:(para.M+1):size(Positions,2)% For each variable
        inter = intersect(Nj{userID},list_node);
        if inter
            rindex = floor(length(inter)*rand()+1);
            Positions(i,j+inter(rindex)-1)=1;
            list_node = setdiff(list_node, inter(rindex));
        else
            eLimit = eLimit + fi0(userID);
            if eLimit <=f_MEC
                Positions(i,j)=1;
            end
        end
        userID = userID +1;
    end
end

iter=0;
Max_iter = 100;
D_X = zeros(SearchAgents_no,D);


% Main loop
while iter<Max_iter
    for i=1:size(Positions,1)

        % Calculate objective function for each search agent
        fitness=fobj(Positions(i,:), U, fi0, f_MEC);

        % Update the leader
        if fitness<Leader_score % Change this to > for maximization problem
            Leader_score=fitness; % Update alpha
            Leader_pos=Positions(i,:);
        end
    end
    a = 2-iter*((2)/Max_iter); % a decreases linearly fron 2 to 0

    % Update the Position of search agents
    for i = 1:size(Positions,1)
        r1 = rand();    % r1 is a random number in [0,1]
        r2 = rand();    % r2 is a random number in [0,1]

        A = 2*a*r1-a;
        C = 2*r2;

        p = rand();
        userID=1;

        for j = 1:(para.M+1):size(Positions,2)
            % follow the shrinking encircling mechanism or prey search
            if p < 0.5
                % search for prey (exploration phase)
                if abs(A) >= 1
                    rand_leader_index = floor(SearchAgents_no*rand()+1);
                    X_rand = Positions(rand_leader_index, :);
                    D_X_rand = abs(C*X_rand(j:j+para.M) - Positions(i,j:j+para.M));
                    D_X(i,j:j+para.M) = D_X_rand;
                elseif abs(A) < 1
                    D_Leader = abs(C*Leader_pos(j:j+para.M) - Positions(i,j:j+para.M));
                    D_X(i,j:j+para.M) = D_Leader;
                end
                % follow the spiral-shaped path (exploitation phase)
            elseif p >= 0.5
                distance2Leader = abs(Leader_pos(j:j+para.M)-Positions(i,j:j+para.M));
                D_X(i,j:j+para.M) = distance2Leader;
            end
            s = 1/(1+exp(-10*(A*max(D_X(i,j:j+para.M))-0.5)));
            if rand < s
                Positions(i,j:j+para.M) = Positions(i,j:j+para.M)*0;
                inter = [1 Nj{userID}];
                rindex = floor(length(inter)*rand()+1);
                Positions(i,j+inter(rindex)-1)=1;
            end
            userID = userID +1;
        end
    end
    iter = iter+1;
end
U_Proposed = abs(Leader_score);
end




