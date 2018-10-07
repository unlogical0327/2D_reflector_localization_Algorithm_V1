%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calibration mode module
% -- this is the calibration mode module used to find the initial location
% of Lidar
% Calibration is needed to locate Lidar before Lidar starts to measure the location
%% Calibration mode:
%% 1. Load the reference reflector map from reading CVS file/manual list.
%% 2. Read distance data from Lidar. 
%% 3. Identify the reflectors from data point based on the reflection amplitude, angle and distance difference.
%% 4. Match with referenced reflector table and find the current position of Robot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cali_status,Lidar_init_xy] = calibration_mode(Reflector_map,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large)
% --calibration_data:  x, y coordibates with amplitude 
% --scan_data:         raw Lidar data-> angle/distance/amplitude
%% Define the variables and tables here
Lidar_x=0;    % x coordinate of Lidar in meter
Lidar_y=0;    % y coordinate of Lidar in meter
theta=0;      % initialize angle and distance to generate simulated test data
dist=0;
%% 1. Read the reference reflector map and 2. the distance data from Lidar. 
%% 3. Identify the reflectors from data point based on the reflection amplitude, the angle and distance difference.
% continuity. Identify the reflector from background and check if the reflector is identical.
%%% identify the reflectors from Lidar data, return detected reflector array
amp_thres=80;  % loss should be proportional to distance -- constant/distance ?
angle_delta=0.0;   % angle delta value to identify different reflectors  
distance_delta=10;   % distance to tell the different reflectors
[detected_ID,detected_reflector]=identify_reflector(amp_thres,angle_delta,distance_delta,calibration_data,scan_data);
detected_ID;
detected_reflector;
%% 4. Match with referenced reflector table and find the current position of Robot.
%thres_dist_match=2;
%thres_dist_large=300;   % distance to filter the reflectors which are far away from each others
%% 4.a Calculate distance between any two reflectors and sort the reflectors
[Reflect_dist_vector] = calc_distance(Reflector_map,Reflector_ID)
[detect_Ref_dist_vector] = calc_distance(detected_reflector,detected_ID)

Reflect_vec_ID = index_reflector(Reflect_dist_vector)
detected_vec_ID = index_reflector(detect_Ref_dist_vector)
%% 4.b match detected reflectors with match reflector pool and return matched point ID.

[matched_reflect_ID,matched_reflect_vec_ID,matched_detect_ID,matched_detect_vec_ID,match_result] = match_reflector(Reflect_dist_vector,Reflect_vec_ID,detect_Ref_dist_vector,detected_vec_ID,thres_dist_large,thres_dist_match);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4.c calculate R and T with matched reflector array and detected reflector array
% find Rotation and transition of matrix A and B and Lidar initial location
% as the start point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
matched_reflect_ID
matched_detect_ID
Reflector_map(matched_reflect_ID,2)
detected_reflector(matched_detect_ID,2)
match_result
[ret_R, ret_T, Lidar_init_xy]=locate_reflector_xy(Reflector_map,matched_reflect_ID,detected_reflector,matched_detect_ID,Lidar_x,Lidar_y);
Lidar_init_xy;
ret_R;
ret_T;
% Calculate reflector rmse 
[reflector_rmse]=reflector_rmse_error(ret_R,ret_T,Reflector_map,matched_reflect_ID,detected_reflector,matched_detect_ID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot x and y coordinate 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(100)
plot(Reflector_map(:,1),Reflector_map(:,2),'or')
font_color='k';
Reflector_ID
plot_reflector(Reflector_map,Reflector_ID,font_color)
hold on;plot(Lidar_init_xy(1,1),Lidar_init_xy(1,2),'ok');hold on
Lidar_trace=Lidar_init_xy;
reflector_rmse
if reflector_rmse<10
    cali_status=0;
elseif reflector_rmse>100
    cali_status=2;
else
    cali_status=1;
end

