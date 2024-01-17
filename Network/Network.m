function [x,y, x_m, y_m] = Network(N, M, r)

filename = sprintf('Network/networkfile_%d.mat',N);
if exist(filename, 'file')
    load(filename);
else
    x = 2*r*rand(1,N)-r;
    y = 2*r*rand(1,N)-r;
    location = x.^2+y.^2;
    index1 = find(location>r.^2);         %According to the Pythagorean theorem, find the (not! Appropriate) x, y value, that is, the location of the user, at a point outside the base station
    len1 = length(index1) ;             %Dimension of matrix index1
    x(index1) = [];                          %Blank the randomly generated points outside the base station
    y(index1) = [];
    while len1  %If len1 is not 0, there are points that do not meet the conditions, and new points need to be regenerated
        xt = 2*r*rand(1,len1)-r;          %len1 is the number of missing points, just regenerate the missing points
        yt = 2*r*rand(1,len1)-r;
        index2 = find(xt.^2+yt.^2>r.^2);
        len1 = length(index2);
        xt(index2) = [];
        yt(index2) = [];
        x = [x xt];                      %Put the new points that meet the conditions into the array
        y = [y yt];
    end
    
    x_m = 2*r*rand(1,M)-r;
    y_m = 2*r*rand(1,M)-r;
    
    location = x_m.^2+y_m.^2;
    index1 = find(location>r.^2);        
    len1 = length(index1) ;             
    x_m(index1) = [];                         
    y_m(index1) = [];
    while len1  
        xt = 2*r*rand(1,len1)-r;          
        yt = 2*r*rand(1,len1)-r;
        index2 = find(xt.^2+yt.^2>r.^2);
        len1 = length(index2);
        xt(index2) = [];
        yt(index2) = [];
        x_m = [x_m xt];                      
        y_m = [y_m yt];
    end
    save(filename,'x','y', 'x_m', 'y_m')
end

end