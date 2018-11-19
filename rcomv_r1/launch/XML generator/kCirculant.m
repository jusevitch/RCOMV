function matrix = kCirculant(n,k,type) %#codegen
% Creates Laplacian matrix for a k-circulant graph with n agents and with
% parameter k.

matrix = zeros(n);

if type == 1 %strcmp(type,'dir')
    for i=1:1:n
        for j=1:1:n
            if i == j
                matrix(i,j) = k;
            elseif i < j
                if ((mod(j+k,n)-i) < k) && ((mod(j+k,n)-i) >= 0)
                    matrix(i,j) = -1;
                end
            else
                if ((i-j)<=k)
                    matrix(i,j) = -1;
                end
            end
        end
    end
    
elseif type == 2 %strcmp(type,'undir')
    % Test to see if k is too large to make a proper graph or not. The code
    % only runs if ok == 1. If k is too big, ok is set to 0 and an error
    % message is displayed.
    ok = 1;
    if (mod(n,2) == 1) 
        if k > floor(n/2)
            ok = 0;
        end
    elseif (mod(n,2) == 0)
        if k >= (n/2)
            ok = 0;
        end
    end

    if ok == 0
        disp('Warning: ERROR -- k too big')
    elseif ok == 1
        for i=1:1:n
            for j=1:1:n
                if i == j
                    matrix(i,j) = 2*k;
                else
                    if ((j+k > n) && mod(j+k,n) >= i) || ((i+k > n) && mod(i+k,n) >= j)
                        matrix(i,j) = -1;
                    elseif (abs(i-j)<=k)
                        matrix(i,j) = -1;
                    end
                end
            end
        end
    end
end
    