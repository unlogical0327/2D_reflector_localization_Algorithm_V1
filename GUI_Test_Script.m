%%%%%%%%%%%
% Test script
%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
% --Define algorithm parameters here
interrupt=0;
scan_freq=8;   % set scan frequency from 8 up to 30Hz, measurement mode will do the loop and send back the data
reflector_source_flag=2;    % set the list flag to 0--read from file, 1--manually set the reflector location 2--generate 120x110 reflector array 2--generate from random location
%data_source_flag=1;
req_update_match_pool=0;
num_ref_pool=5;
num_detect_pool=3;
Reflector_map=0;
scan_data=0;
amp_thres=80;
angle_delta=0.2;
dist_delta=10;
thres_dist_match=1;
thres_dist_large=300;
% Execute mode manager to run GUI test script
[mode,status,update_match_pool] = mode_manager(interrupt,scan_freq,reflector_source_flag,req_update_match_pool,num_ref_pool,num_detect_pool,Reflector_map,scan_data,amp_thres,angle_delta,dist_delta,thres_dist_match,thres_dist_large)

if status == 0
    disp('RLA is running properly!!')
elseif status == 1
    error('Error happen!!')
end
