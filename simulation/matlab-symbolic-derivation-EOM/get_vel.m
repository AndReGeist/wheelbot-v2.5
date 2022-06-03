function diff_p_q_res = get_vel(p,q,dq)
% Credits to mec560sbu
% See: https://github.com/mec560sbu/mec560sbu.github.io/blob/master/mec560_MATLAB_codes/Deriving_EOMs/fcn_support/get_vel.m
for i = 1:length(q)
    diff_p_q(:,i) = diff(p,q(i))*dq(i); 
end

diff_p_q_res = simplify(expand(sum(diff_p_q,2)));