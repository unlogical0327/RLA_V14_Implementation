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
function [mode,status] = mode_manager(interrupt,scan_freq,num_ref_pool,num_detect_pool,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,thres_dist_match,thres_dist_large,thres_angle_match)
%% -interrupt:              interrupt from GUI console to control the Lidar computing engine
%% -reflector_source_flag:  flag to define the reflector source from GUI
%% -data_source_flag:       flag to define the data source from GUI
%% -req_update_match_pool:  request to ask match pool to update to include more reflectors
%% -Reflector_map:          load Reflector map from GUI console
%% -scan_data:              load 3D Lidar data to module
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

moving_estimate_matching= 1;   % flag to update detected reflector with estimated offset
moving_estimate_calc= 1;   % flag to update detected reflector with estimated offset
ref_gauss_data_fit_cali= 1;  % flag to enable noise cancelling algorithm
ref_gauss_data_fit_mea= 1;  % flag to enable noise cancelling algorithm
cali_flag = 0;
Gauss_flag = 1;
mea_status_trace=0;
Lidar_trace_filt=[0 0];
vel_trace_filt=[0 0];
%% Load Reflector map

    %fname_moving = ['Data/2/Lidar_data.txt']; % Load moving data to test moving compensation algorithm
    %fname_moving = ['Data/20hz/20hz/Lidar_data.txt']; % Load moving data to test moving compensation algorithm    
    %fname_moving = ['Data/50hz/50hz/Lidar_data.txt']; % Load moving data to test moving compensation algorithm    
    fname_moving = ['D:/lidar_mmwave/RLA_c/RLA_c/Lidar_data_long2.txt'];
    mode=1;
    [Lidar_data,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    scan_data=Lidar_data;
    %% -- Read map from data
    %[Reflector_map,Reflector_map_polar,Reflector_ID,load_ref_map_status]=reflector_map_cali_scan(ref_gauss_data_fit_cali,amp_thres,dist_thres,reflector_diameter,dist_delta,scan_data);
    % Read reflector map from txt file
    %fname_map = ['Data/20hz/20hz/Reflector_Map_12022018_final.txt'];
    %fname_map = ['Data/50hz/50hz/Reflector_Map_12022018_final.txt'];
    fname_map = ['D:/lidar_mmwave/RLA_c/RLA_c/Reflector_Map_12022018_exp.txt'];
    [Reflector_map,Reflector_map_polar,Reflector_ID,load_ref_map_status]=reflector_map_read(fname_map);        
    a=1;
    tic;
    Lidar_x=0;
    Lidar_y=0;
if cali_flag==1
   while(a==1)
        tstart_cali=tic;
        %% convert polar data to rectangle data
        [calibration_data,scan_data]=PolarToRect(Lidar_data);
        %%-- Run calibration mode: calculate initial x, y and pose 
        [cali_status,Lidar_trace,rotation_trace,reflector_rmse] = calibration_mode(ref_gauss_data_fit_cali,amp_thres,dist_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_map_polar,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large,thres_angle_match,Lidar_x,Lidar_y)
        if cali_status==0
            disp('Calibration successful! Proceed to measurement mode....')
            break
        elseif cali_status==3
            disp('Data is bad, wait for new Lidar data for new Cali!!')
        else
            disp('Calibration failed, please check Lidar data!!')
            break
        end
     tlapsed_cali=toc(tstart_cali);  % monitor calibration running time   
        end
else
    disp('Skip calibration mode....!')
    Lidar_trace=[0 0];
    rotation_trace=0;
    reflector_rmse=0;
end

%% Measurement mode
%-- need to read the scan data and process the data at each scan
% initialize variables
    
    [rot_vel_trace,rot_acc_trace,radius_trace,dist_err,angle_err,ref_ID_hist,dist_err_trace,angle_err_trace,Lidar_expect_trace,matched_ref_ID_hist,matched_detect_ID_hist,rmse_trace,vel_trace,acc_trace, ...
    Lidar_trace_p,Lidar_expect_trace_p,rotation_trace_p,Lidar_update_Table_p,detected_ID_p,detected_reflector_p,match_reflect_pool_p,match_reflect_ID_p]=init_variable(num_detect_pool);

%% Moving mode
%--Use moving mode to calculate the AGV in moving. 
% This mode will use moving estimation to reduce the errors generated
% during AGV is moving at the fast speed.
%-- read the rest of scan data
    mode=3; 
    [Lidar_data_total,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    minTime=Inf;
    tic;
 for ii=2:200 %1095   on trail
    tstart=tic;
    %--Call moving mode to calculate expected pose and location
    Lidar_data = Lidar_data_total(:,(ii-1)*data_length:ii*data_length);
    [measurement_data4,scan_data]=PolarToRect(Lidar_data);
   %%-- Plot raw data
    %plot_Lidar_data(measurement_data4);
   %[moving_status,Lidar_trace,rotation_trace] = moving_mode(moving_thres,rot_angle_thres,amp_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_ID,measurement_data4,scan_data,match_reflect_pool,match_reflect_ID,reflector_index,pose_his,thres_dist_match,thres_dist_large)
    [mea_status,Lidar_trace,Lidar_expect_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,match_reflect_ID,reflector_index,detected_reflector,detected_ID,dist_err,angle_err,wm_detected_reflector,wm_detected_ID,unmatched_detected_reflector,unmatched_detect_ID,matched_ref_ID_hist,matched_detect_ID_hist,map_rmse,reflector_rmse,xy_vel_acc] ...
    = measurement_mode(num_ref_pool,num_detect_pool,scan_freq,Reflector_map,Reflector_ID,measurement_data4,scan_data,moving_estimate_matching,moving_estimate_calc,ref_gauss_data_fit_mea,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,Lidar_expect_trace,rotation_trace,dist_err,angle_err,thres_dist_match,thres_dist_large,thres_angle_match,reflector_rmse,matched_ref_ID_hist,matched_detect_ID_hist);
    ii
    tlapsed_m(ii)=toc(tstart);
    minTime=min(tlapsed_m(ii),minTime);
      if mea_status==3
       disp('Cant find any matched reflector, go to the another round data......')
       [Lidar_trace,Lidar_expect_trace,rotation_trace,Lidar_update_Table,detected_ID,detected_reflector,match_reflect_pool,match_reflect_ID,wm_detected_ID,wm_detected_reflector]=load_previous_data ...
       (Lidar_trace_p,Lidar_expect_trace_p,rotation_trace_p,Lidar_update_Table_p,detected_ID_p,detected_reflector_p,match_reflect_pool_p,match_reflect_ID_p);         
      else            
       [Lidar_trace_p,Lidar_expect_trace_p,rotation_trace_p,Lidar_update_Table_p,detected_ID_p,detected_reflector_p,match_reflect_pool_p,match_reflect_ID_p] ...
        =save_current_data(Lidar_trace,Lidar_expect_trace,rotation_trace,Lidar_update_Table,detected_ID,detected_reflector,match_reflect_pool,match_reflect_ID);
      end
      Iter_num=ii;
      if length(Lidar_trace)>2
        [vel_update,vel_trace,acc_trace,rmse_trace,dist_err_trace,angle_err_trace,rot_vel_trace,rot_acc_trace,radius_trace,ref_ID_hist] = ...
        update_data_trace(Iter_num,scan_freq,xy_vel_acc,vel_trace,acc_trace,reflector_rmse,rmse_trace,matched_ref_ID_hist,ref_ID_hist,dist_err,dist_err_trace,angle_err,angle_err_trace,Lidar_trace,rotation_trace,rot_vel_trace,rot_acc_trace,radius_trace);
      end
      mea_status_trace=[mea_status_trace;mea_status];
        Lidar_xy_filt=[0 0];
        gaussian_step=3;
        filter_length=10;
        sigma=1; 
      if Gauss_flag==1 && length(Lidar_trace)>filter_length
        [Lidar_trace]=gaussian_location_smooth(Lidar_trace,gaussian_step,sigma,filter_length);
      end
    if mod(ii,20)==0
%             plot_lidar_xy(Lidar_trace,Lidar_expect_trace)
%             plot_dist_angle_err(dist_err_trace,angle_err_trace)
%             plot_running_time(tlapsed_m,rmse_trace)
%             plot_vel_acc_time(vel_trace,acc_trace)
%             plot_rot_acc_time(rotation_trace,rot_vel_trace,rot_acc_trace)
            Plot_world_map(Lidar_update_Table,match_reflect_pool,match_reflect_ID,wm_detected_reflector,wm_detected_ID,Lidar_trace);
            %Plot_worldmap_Lidar_center_view(Lidar_update_Table,match_reflect_pool,match_reflect_ID,wm_detected_reflector,wm_detected_ID,Lidar_trace)
            Plot_worldmap_Lidar_center_color(Lidar_update_Table,scan_data,match_reflect_pool,match_reflect_ID,wm_detected_reflector,wm_detected_ID,Lidar_trace)
            pause(0.1)
            %%%%%%%%%  
     end
 end
 
% figure(188)
% plot(radius_trace)
% title('Radius trace(mm)')
%%%%%%%%%%%%%%%%%  Plot time lapse

 plot_lidar_xy(Lidar_trace,Lidar_expect_trace)
 plot_dist_angle_err(dist_err_trace,angle_err_trace) 
 minTime;
 tlapsed_m;
 rmse_trace
 %Lidar_trace;
 %plot_running_time(tlapsed_m,rmse_trace)
 plot_vel_acc_time(vel_trace,acc_trace)
 ref_ID_hist
 mea_status_trace
 %rotation_trace;
 %angle_err_trace;
 %rot_acc_trace;
 %Lidar_trace;
 %Lidar_expect_trace;
 status='done';
 %%%%%%%%%%%%%%%%%%% . Plot Velocity and Acceleration
