function [L_matrix,S_matrix,s_vector] = computeLQ(Q, QN, R, A, B, nsteps, SigmaM)

L_matrix = zeros(size(B,2), size(B,1), nsteps);
S_matrix = zeros(size(QN,1), size(QN,1), nsteps+1);
S_matrix(:,:,end) = QN;
s_vector = zeros(nsteps+1);

for ii = 1 : nsteps
   L_matrix(:,:,nsteps+1-ii) = (R+B'*S_matrix(:,:,nsteps-ii+2)*B)\(B'*S_matrix(:,:,nsteps-ii+2)*A);
   S_matrix(:,:,nsteps+1-ii) = Q(:,:,nsteps+1-ii) + A'*S_matrix(:,:,nsteps-ii+2)*(A-B*L_matrix(:,:,nsteps-ii+1));
   s_vector(nsteps+1-ii) = s_vector(nsteps+2-ii) + trace(S_matrix(:,:,nsteps+1-ii)*SigmaM); 
end

end