
function [Reflect_dist_vector] = calc_distance(Reflect_Table,Reflect_ID)
%-- calculate distance matrix between two reflectors
(Reflect_Table);
Reflect_ID;
Reflect_vector=[Reflect_Table(1:length(Reflect_ID),1) Reflect_Table(1:length(Reflect_ID),2)]';
Reflect_dist_vector=pdist(Reflect_vector');  % calculate the reference distance vector 
