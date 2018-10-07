% Load Lidar data and add random displacement to x and y
% Data format in Lidar data example
% Angle\Distance\Amplitude
function [Lidar_Table_disp]=simulate_lidar_movement(theta,dist,list_source_flag,Lidar_Table,Lidar_data)

if list_source_flag==1    
    fname = ['Lidar_data_example1'];
    Lidar_data_file = dlmread( fname, ' ', 3, 0)';
    Lidar_data=Lidar_data_file;
    for ii=1:length(Lidar_data)
    Lidar_Table(ii,1)=cos(Lidar_data_file(1,ii)/180*pi)*Lidar_data_file(2,ii)/10;
    Lidar_Table(ii,2)=sin(Lidar_data_file(1,ii)/180*pi)*Lidar_data_file(2,ii)/10;
    end
elseif list_source_flag==2
    for ii=1:length(Lidar_Table)
    Lidar_data(1,ii) = atan(Lidar_Table(ii,2)/Lidar_Table(ii,1))/pi*180; 
    Lidar_data(2,ii) = (Lidar_Table(ii,2)^2+Lidar_Table(ii,1)^2)^0.5; 
    Lidar_data(3,ii) = Lidar_data(3,ii); %Lidar_Table(ii,3);
    end
end

    %R = orth(rand(2,2)) % random rotation matrix
    R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
    if det(R) < 0
      V(:,2) = -1*V(:,2);
      R = V*U';
    end
    %t = rand(2,1) % random translation
    t=dist;
    n = length(Lidar_data); % number of points
    Lidar_Table_xy(:,1)=Lidar_Table(:,1);
    Lidar_Table_xy(:,2)=Lidar_Table(:,2);
    Lidar_Table_disp = R*Lidar_Table_xy' + repmat(t, 1, n);
    Lidar_Table_disp = Lidar_Table_disp';
  