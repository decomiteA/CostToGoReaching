function Ae = extendA(A,n_red)
%extendA expands the matrix A to simulate movements towards a redundant
%target
%
% INPUTS
% ======
% - A is the matrix capturing the state space dynamics 
% - n_red is the number of targets composing the redundant target
%
% OUTPUTS
% =======
% - Ae is the expanded matrix
%
% @Antoine de Comite

Ae = zeros(size(A)*(n_red+1));
Ae(1:size(A),1:size(A)) = A;
for ii = 2 : n_red+1
    Ae(1+(ii-1)*8:ii*8,:) = [zeros(size(A,1),size(A,1)*(ii-1)), eye(size(A,1),size(A,1)),zeros(size(A,1),(n_red+1-ii)*size(A,1))]; 
end
end