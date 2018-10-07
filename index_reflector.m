
function [Reflect_vec_ID] = index_reflector(match_dist_vector_pool)
% NP_enable?
% function [matched_reflect_ID] = match_reflector(match_dist_vector_pool,Reflect_vec_ID,detect_Ref_dist_vector,detect_vect_ID,thres_dist_match,NP_enable)
% Define matching threshold value here
%-- match the distance matrix with reflector tables
%%% Label distance with reference point ID
N_reflector=((8*length(match_dist_vector_pool)+1)^0.5+1)/2;
z=1;
k=z;
for j=1:length(match_dist_vector_pool)
    k;
    N_reflector;
    if k<N_reflector
        k=k+1;
        Reflect_vec_ID(j,1)=z;
        Reflect_vec_ID(j,2)=k;
    elseif k>=N_reflector
        z=z+1;
        k=z+1;
        Reflect_vec_ID(j,1)=z;
        Reflect_vec_ID(j,2)=k;
    end
end
