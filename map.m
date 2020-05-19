function map_num = map(A,B,C,D,x)
    map_num = ((x-A)./(B-A))*(D-C)+C;
end