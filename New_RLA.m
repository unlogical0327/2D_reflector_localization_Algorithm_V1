%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% New RLA flow design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% New Reflector Localization Algorithm
% This program is developed and copyright owned by Soleilware LLC
% The code is writen to build the blocks for the localization
% algorithm process and efficiency.
% --------------------------------
% Created by Qi Song on 9/18/2018
% The process if designed as followed 
%%%%%%%%%%%
%% Calibration mode:
%% 1. Load the reference reflector map from reading CVS file/manual list.
%% 2. Read distance data from Lidar. 
%% 3. Identify the reflectors from data point based on the reflection amplitude, angle and distance difference.
%% 4. Match with referenced reflector table and find the current position of Robot.
%% 5. Choose M x reflectors and load to matching reflector table. (M should be match smaller than the total number of reflector map)
%% Measurement mode:
%% 1. Read the scan data, identify reflectors and define how many scanned reflectors are used from the list(nearest distance or most distingushed).
%% 2. Match the N x scanned reflectors with match reflector table and find the location of lidar.
%% 3. Compare with location at the previous moment and find the location change direction. Find the possible reflectiors from scanned data along the moving direction.
%% 4. Add the reflectors back to match reflector table.
%% 5. Estimate the possible location errors from moving direction and location change from last moment.
%----------------debugging question list-----------------------------------
%% Q: Do we really need to predict the expectation locations of reflector?
% This may impact the accurary of localization and depends on the update
% rate of Lidar distance data. Need to check with experiment
%% A: See step 5.
%% Q: To predict the distance change and angle, what algorithm we need to apply? SLAM or any other data asistance like IMU, Odometer?
%% A: To be update
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define the variables and tables here
clear all
Table_size=30; %the maximum number of reflectors 
%Reflector_map=zeros(3,Table_size);   % Define the table as distance/angle/intensity
Expectation_Table=zeros(3,Table_size);   % Expectation table to predict the current location
map_size_x = 1000;   % map x dimension in meter
map_size_y = 1000;   % map y dimension in meter
colunm_x = 6;
row_y = Table_size/colunm_x;
Lidar_x=0;    % x coordinate of Lidar in meter
Lidar_y=0;    % y coordinate of Lidar in meter
theta=0;      % initialize angle and distance to generate simulated test data
dist=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define simulation configuration here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
list_source_flag=1;  % set the list flag to 0--read from file, 1--manually set the reflector location 2--generate 120x100 reflector array 3--generate from random location
Prediction_flag=0;   % enable feature to calculate the location error prediction in measurement mode
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation starts from here !!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Load the reference reflector map from reading CVS file/manual list.
if list_source_flag == 0 % read from file
    fname = ['Reflector_map_example'];  % file only contains reference reflector location.
    raw_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(raw_data)
        Reflector_ID(ii) = ii;
    Reflector_map(1,Reflector_ID(ii))=cos(raw_data(1,ii)/180*pi)*raw_data(2,ii);   % generate reflector array x 
    Reflector_map(2,Reflector_ID(ii))=sin(raw_data(1,ii)/180*pi)*raw_data(2,ii);   % generate reflector array y
    end
elseif list_source_flag == 1 % manually set reflector location as reference reflector distribution
    % input at least 3 points to calculate the robot location 
    % 45.0000 565.7  100
    % 45.0000 707.1  100
    % 45.0000 848.5  100
    Reflector_ID(1)=1;   
    Reflector_map(1,Reflector_ID(1))=40;    % x 
    Reflector_map(2,Reflector_ID(1))=40;    % y
    Reflector_ID(2)=2;
    Reflector_map(1,Reflector_ID(2))=50;    % x
    Reflector_map(2,Reflector_ID(2))=0;     % y
    Reflector_ID(3)=3;
    Reflector_map(1,Reflector_ID(3))=0;
    Reflector_map(2,Reflector_ID(3))=60;
    Reflector_ID(4)=4;
    Reflector_map(1,Reflector_ID(4))=-50;
    Reflector_map(2,Reflector_ID(4))=50;
    Reflector_ID(5)=5;
    Reflector_map(1,Reflector_ID(5))=-60;
    Reflector_map(2,Reflector_ID(5))=-60;
    
elseif list_source_flag == 2 % generate the 12000 reflector matrix
    x_increm=map_size_x/(colunm_x-1);
    y_increm=map_size_y/(row_y-1);
    ii=1;
    x_offset=200;
    y_offset=50;
    amp=1;  % amplitude of random x and y errors
    for i_x=1:colunm_x
        for i_y=1:row_y
           x_pos = x_offset+x_increm*(i_x-1)+amp*(-1+2*randi(100)/100);   % x location
           y_pos = y_offset+y_increm*(i_y-1)+amp*(-1+2*randi(100)/100);   % y location
          Reflector_ID(ii)=(i_x-1)*row_y+i_y;
          Reflector_map(1,Reflector_ID(ii))= x_pos;
          Reflector_map(2,Reflector_ID(ii))= y_pos;
          ii=ii+1;
        end
    end
elseif list_source_flag == 3 % generate the reflector table randomly
    Reflector_map = rand(120,100)*1000;
end
%%%--------- Plot the reflector map
figure(101)
plot(Reflector_map(1,:),Reflector_map(2,:),'or');
title('Map with reflector array');
xlabel('X dimension (M)');
ylabel('Y dimension (M)');
%xlim([0 100])
%ylim([0 100])
a = Reflector_ID'; b = num2str(a); c = cellstr(b);
dx = 1.5; dy = 1.5;
hold on;
text(Reflector_map(1,Reflector_ID)+dx,Reflector_map(2,Reflector_ID)+dy, c);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Read distance data from Lidar. 
if list_source_flag==1; 
    fname = ['Lidar_data_example'];
    Lidar_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(Lidar_data)
    Lidar_Table(1,ii)=cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    Lidar_Table(2,ii)=sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    end
elseif list_source_flag==2; 
    Noise_table=rand(2,10)*1000;
    Noise_data_amp=rand(1,10)*10;
    Noise_data=[Noise_table;Noise_data_amp];
    test_Table= [Noise_table,Reflector_map,Noise_table];
    test_data= [Noise_data,Reflector_data,Noise_data]; 
    [Lidar_Table,Lidar_data]=simulate_lidar_movement(theta,dist,list_source_flag,test_data);
       
end

%%%------------ plot Lidar data
figure(102)
plot(Lidar_Table(1,:),Lidar_Table(2,:),'+b');
title('Lidar reflection');
xlabel('X dimension (M)');
ylabel('Y dimension (M)');
%xlim([-1000 1000])
%ylim([-1000 1000])
%% 3. Identify the reflectors from data point based on the reflection amplitude, angle and distance difference.
% continuity. Identify the reflector from background and check if the reflector is identical.
%%% identify the reflectors from Lidar data, return detected reflector array
amp_thres=80;  % loss should be proportional to distance -- constant/distance ?
angle_delta=0.4;   % angle delta value to identify different reflectors  
distance_delta=10;   % distance to tell the different reflectors
[detected_ID,detected_reflector]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_data,Lidar_Table);
%%%%%%%%%%%%
% hold on;plot(detected_reflector(1,:),detected_reflector(2,:),'or')    
% a = detected_ID'; b = num2str(a); c = cellstr(b);
% dx = 5; dy = 5;
% hold on;
% text(detected_reflector(1,:)+dx,detected_reflector(2,:)+dy, c);
color='b';
plot_reflector(detected_reflector,detected_ID,color)
%% 4. Match with referenced reflector table and find the current position of Robot.
thres_dist_match=2;
thres_dist_large=300;   % distance to filter the reflectors which are far away from each others
%% 4.a Calculate distance between any two reflectors and sort the reflectors
[Reflect_dist_vector,Reflect_vec_ID] = calc_distance(Reflector_map,Reflector_ID);
[detect_Ref_dist_vector,detected_vec_ID] = calc_distance(detected_reflector,detected_ID);
[Reflect_vec_ID] = index_reflector(Reflect_dist_vector)
[detected_vec_ID] = index_reflector(detect_Ref_dist_vector)
%% 4.b match detected reflectors with match reflector pool and return matched point ID.

[matched_reflect_ID,matched_reflect_vec_ID,matched_detect_ID,matched_detect_vec_ID,match_result] = match_reflector(Reflect_dist_vector,Reflect_vec_ID,detect_Ref_dist_vector,detected_vec_ID,thres_dist_large,thres_dist_match);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4.c calculate R and T with matched reflector array and detected reflector array
% find Rotation and transition of matrix A and B and Lidar initial location
% as the start point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ret_R, ret_T, reflector_rmse, Lidar_init_xy]=locate_reflector_xy(Reflector_map,matched_reflect_ID,detected_reflector,matched_detect_ID,Lidar_x,Lidar_y);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot x and y coordinate 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on;plot(Lidar_init_xy(1,1),Lidar_init_xy(1,2),'ok')
Lidar_trace=Lidar_init_xy;

%% 5. Choose M x reflectors and load to matching reflector table. (M should be match smaller than the total number of reflector map)
% --Select at least 3 reflectors to locate the robot
% --Manually choose ix reflectors(ix nearest points)
num_ref_pool=3   % --define M value (how many reflectors in reference reflector pool)
num_detect_pool=3;   % --define N value (how many reflectors to be used)
nearest_ID_en=0;   % --enable nearest ID selection?0-not enabled; 1-enabled

if num_detect_pool>length(detected_ID)
    error('defined reflector number is larger than detected relfectors!!!')
elseif num_detect_pool>num_ref_pool
    disp('defined reflector number is larger than matching reflector pool!!!')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load M x nearest points to matching reflector pool
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_init_xy);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% above could be the calibration process at the initialization of
% software localization
%
% Below could be taken as the iterative process to update new Lidar scan
% and locate Lidar itself, convert to the world coordinate and plot the Lidar
% moving trace.

%% Measurement mode
%% ---- Loop starts from here ----
%  set the loop number of simulation and generate random movement at each moment
%% 1. Read the scan data, identify reflectors and define how many scanned reflectors are used from the list(nearest distance or most distingushed).
Loop_num=20;
for ll=1:Loop_num     % simulation loop start from here!!!
%theta=theta+randi(360)/180*pi;  % define random rotation angle
theta=theta+5/180*pi;
%dist=randi(500)/20;   % define random transition
dist=50;
[Lidar_Table1,Lidar_data1]=simulate_lidar_movement(theta,dist,list_source_flag,Lidar_data);    %simulate random displacement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% identify the reflectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[detected_ID1,detected_reflector1]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_data1,Lidar_Table1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 2. Match the N x scanned reflectors with match reflector table and find the location of lidar.
% -- create match detect pool 
[match_detected_pool,match_detected_pool_ID] = create_match_detect_pool(num_detect_pool,detected_reflector1,detected_ID1,Lidar_init_xy);

%% 2.a Calculate distance between any two reflectors
%[match_reflect_vector_pool1,match_reflect_vec_ID1] = calc_distance(match_reflect_pool,match_reflect_pool_ID);
%[match_detected_vector_pool1,match_detected_vec_ID1] = calc_distance(match_detected_pool,match_detected_pool_ID);
%% 2.b match detected reflectors with match reflector pool and return matched point ID.
% -- match the distance vector and return point array and point ID
% -- update match pool if 
num_ref_pool=3;
a=1;
[match_reflect_vector_pool1,match_reflect_vec_ID1] = calc_distance(match_reflect_pool,match_reflect_pool_ID);
[match_detected_vector_pool1,match_detected_vec_ID1] = calc_distance(match_detected_pool,match_detected_pool_ID);
[Reflect_vec_ID1] = index_reflector(match_reflect_vector_pool1)
[detected_vec_ID1] = index_reflector(match_detected_vector_pool1)

while a==1
    [matched_reflect_ID1,matched_reflect_vec_ID1,matched_detect_ID1,matched_detect_vec_ID1,match_result] = match_reflector(match_reflect_vector_pool1,Reflect_vec_ID1,match_detected_vector_pool1,detected_vec_ID1,thres_dist_large,thres_dist_match);
    num_ref_pool;
    if match_result==1
    disp('No matched distance found, update reference map with new reflectors')
    num_ref_pool=num_ref_pool+1
    [match_reflect_pool1,match_reflect_pool_ID1] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_init_xy);
    [match_reflect_vector_pool1,match_reflect_vec_ID1] = calc_distance(match_reflect_pool1,match_reflect_pool_ID1);
    [Reflect_vec_ID1] = index_reflector(match_reflect_pool1)
    elseif match_result==0 
    disp('Matched reflector found!!')
    break
end
end
%% plot map new Lidar scan
figure(102)
hold on;plot(Lidar_Table1(1,:),Lidar_Table1(2,:),'+g');
color='g';
%% plot map with random displacement
%plot_reflector(detected_reflector1,detected_ID1,color)
%% 2.c calculate rotation and transition
[ret_R1, ret_T1, reflector_rmse, Lidar_update_xy]=locate_reflector_xy(match_reflect_pool,matched_reflect_ID1,detected_reflector1,matched_detect_ID1,Lidar_x,Lidar_y);
%% 2.d calculate updated map in the world map 
[Lidar_update_Table,Lidar_update_xy]=update_Lidar_scan_xy(ret_R1,ret_T1,Lidar_Table1,Lidar_Table,Lidar_x,Lidar_y);
%% Plot reference map in the world coordinate
figure(103)
plot(Lidar_Table(1,:),Lidar_Table(2,:),'+b');
hold on;
color='b';
%% Plot the reflectors in the world map
plot_reflector(detected_reflector,detected_ID,color) % plot reference reflector 
hold on;
%% Plot update map in the world map
plot(Lidar_update_Table(1,:),Lidar_update_Table(2,:),'+g');
% identify the reflector in the updated map
[detected_ID2,detected_reflector2]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_data,Lidar_update_Table);
hold on;
color='g';
plot_reflector(detected_reflector2,detected_ID2,color)
hold on;
%% Save new Lidar location to the trace and plot
Lidar_trace=[Lidar_trace; Lidar_update_xy];
plot(Lidar_trace(:,1),Lidar_trace(:,2),'o-k')
num_ref_pool
disp(sprintf("RMSE: %f for %i th step", reflector_rmse,ll));
disp("RMSE errors for each reflector matching calculation ");
pause(1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Last Step!!!
%% Update match reflector pool to get the latest nearest points from reflector map
[match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_update_xy);

end  % Simulation loop end up here!!!