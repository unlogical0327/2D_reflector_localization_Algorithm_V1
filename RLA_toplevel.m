%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RLA flow design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is the top level code to implement matlab-to-C++
% verification platform
% RLA has options to generate test vectors to verify the algorithm.
% This program is developed and copyright owned by Soleilware LLC
% The code is writen to build the blocks for the localization
% algorithm process and efficiency.
% --------------------------------
% Created by Qi Song on 9/18/2018
function [status]=RLA_toplevel(list_source_flag)% RLA top level function to convert Matlab code to C++ package and run C++ test code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define simulation configuration here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
list_source_flag=1;  % set the list flag to 0--read from file, 1--manually set the reflector location 2--generate 120x100 reflector array 3--generate from random location
Prediction_flag=0;   % enable feature to calculate the location error prediction in measurement mode

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -- Load the test data or scan data from Lidar
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if list_source_flag==1; 
    fname = ['Lidar_data_example'];
    Lidar_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(Lidar_data)
    calibration_data(1,ii) = cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    calibration_data(2,ii) = sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    calibration_data(3,ii) = Lidar_data(3,ii);
    end
elseif list_source_flag==2; 
    Noise_table = rand(2,10)*1000;
    Noise_data_amp = rand(1,10)*10;
    Noise_data = [Noise_table;Noise_data_amp];
    test_Table = [Noise_table,Reflector_map,Noise_table];
    test_data = [Noise_data,Reflector_data,Noise_data]; 
    [calibration_data] = simulate_lidar_movement(theta,dist,list_source_flag,test_data);      
end

%% Calibration mode
[Lidar_init_xy] = calibration_mode(Reflector_map,calibration_data)

%% Measurement mode
%-- need to read the scan data and process the data at each scan
%measurement
[status] = measurement_mode(status,scan_data,amp_thres,angle_delta,dist_delta)
