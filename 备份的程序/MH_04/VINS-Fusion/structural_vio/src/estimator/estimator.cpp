/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"
#include <boost/iterator/iterator_concepts.hpp>
using namespace cv;

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionStructuralLine::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionStructuralLine_Oneframe::sqrt_info = FOCAL_LENGTH / 2.5 * Matrix2d::Identity();
    lineProjectionFactor::sqrt_info = FOCAL_LENGTH / 2.5 * Matrix2d::Identity();
    
    distance_StructuralLine_Point::sqrt_info = (1/0.15) * Matrix2d::Identity();
    
    cout<<" sqrt_info is \n"<< ProjectionTwoFrameOneCamFactor::sqrt_info<<endl;
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}



void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)
    {
        clearState();
        setParameter();
    }
}

cv::Point2d Estimator::To_picture( cv::Point2d  &point_xy)
{
//     float fx = 4.616e+02;
//     float fy = 4.603e+02;
//     float cx = 3.630e+02;
//     float cy = 2.481e+02;
    float fx = 4.616e+02;
    float fy = 4.603e+02;
    float cx = 3.760e+02;
    float cy = 2.401e+02;
     
    Point2d point_uv;
    point_uv.x = fx*point_xy.x+cx;
    point_uv.y = fy*point_xy.y+cy;
    
    return point_uv;
    
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
 
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }
        if(MULTIPLE_THREAD)  
 
    {     
        if(inputImageCnt % 1 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
	
  if(0)
  {
    
//     for (auto &it_per_id : f_manager.feature)
//     {
//         int used_num = it_per_id.feature_per_frame.size();
//       
//         int imu_i = it_per_id.start_frame;
// 	
// 	cout<<"point id is "<<it_per_id.feature_id<<" used_num "<<used_num<<" start_frame "<<imu_i<<" solve_flag "<< it_per_id.solve_flag<<endl;
//     }
    
    ROS_INFO("%f line info \n \n ",t);
    
     for (auto &it_per_id : f_manager.linefeature)
    {
        int used_num = it_per_id.linefeature_per_frame.size();
        int imu_i = it_per_id.linefeature_per_frame[0].In_frame_count;
	cout<<" line id is "<<it_per_id.feature_id<<" used_num "<<used_num<<" start_frame "<<imu_i<<endl; 
	
	for(auto &it_per_frame : it_per_id.linefeature_per_frame)
	{
	  cout<<it_per_frame.lineobs.transpose()<<endl;
	  if(it_per_frame.is_stereo)
	    cout<<"R obs "<<it_per_frame.lineobs_R.transpose()<<endl;
	}

    }//输出所有线特征的观测
    
    
    
//         cout<<"Finanl All Window is "<<endl;
// 	    
// 	    for(unsigned int i = 0; i<= WINDOW_SIZE; i++)
// 	    {
// 	       cout << "frameCnt: " << i << " pnp P " << Ps[i].transpose() << endl;
// 	    }
   
 
   
    
  }
  
  //输出窗口内的运动状态量 
  if(0)
   {
      ROS_INFO("State of window :");
        for(unsigned int i = 0; i<= WINDOW_SIZE; i++)
	    {  
// 	      Eigen::Quaterniond Q(Rs[i]);
	          Vector3d euler_angles;
                  euler_angles = Rs[i].eulerAngles ( 2,1,0 ); 
	      cout << "frameCnt: " << i << " P " << Ps[i].transpose() <<  " R_euler_angles " << euler_angles.transpose()<< endl;
	    }
     
   }

	 
	cv:imshow("image",_img);
	cvWaitKey(1);
	
	
        printf("process time: %f\n", processTime.toc());
    }
    
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

//     fastPredictIMU(t, linearAcceleration, angularVelocity);
//     if (solver_flag == NON_LINEAR)
//         pubLatestOdometry(latest_P, latest_Q, latest_V, t);
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
	pair<double, map<int, vector<pair<int, Vector4d>>>> line_feature;
	vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> line_feature_all;
	pair<double, vector<cv::Point3d>> vps;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
	
	
//  　用于判断点线特征的对齐	
// 	if(!featureBuf.empty() && !line_1_Buf.empty() && !line_2_Buf.empty() && !line_3_Buf.empty())
// 	{
// 	  cout<<"point time are "<<endl;
// 	  while(!featureBuf.empty())
// 	  {
// 	     feature = featureBuf.front();
// 	     double time_point = feature.first;
// 	     featureBuf.pop();
// 	    ROS_INFO("%f ", time_point);
// 	     
// 	  }
// 	 
// 	  cout<<"line_1 time are "<<endl;
// 	  while(!line_1_Buf.empty())
// 	  {
// 	     line_feature = line_1_Buf.front();
// 	     double time_point = line_feature.first;
// 	     line_1_Buf.pop();
// 	     ROS_INFO("%f ", time_point);
// 	     
// 	  }
// 	  cout<<"line_3 time are "<<endl;
// 	   while(!line_3_Buf.empty())
// 	  {
// 	     line_feature = line_3_Buf.front();
// 	     double time_point = line_feature.first;
// 	     line_3_Buf.pop();
// 	     ROS_INFO("%f ", time_point);
// 	     
// 	  }
// 	  
// 	}
// 	
 if(!featureBuf.empty() && !line_1_Buf.empty() && !line_2_Buf.empty() && !line_3_Buf.empty() && !vp_Buf.empty())
  {
	   feature = featureBuf.front();
            curTime = feature.first + td;
	    line_feature=line_1_Buf.front();
            ROS_INFO("process image %f",curTime);
	    
	    
	     if(prevTime == -1)
	    {
	      prevTime =  curTime;
	      
	      mBuf.lock();
	      featureBuf.pop();
	      line_1_Buf.pop();
	      line_2_Buf.pop();
	      line_3_Buf.pop();
	      vp_Buf.pop();
	      mBuf.unlock();

	      ROS_INFO("Skip the first image");
	      continue;
	  
	    }
	    
            while(1)
            {
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))   //若最新的imu时间还没有到当前视觉的时间，返回1，  否则0
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            bool imu_flag;
            mBuf.lock();
            if(USE_IMU)
             imu_flag =getIMUInterval(prevTime, curTime, accVector, gyrVector);//获取帧间imu测量

            featureBuf.pop();
	    
	    line_feature_all.push_back(line_1_Buf.front());
	    line_1_Buf.pop();
	    
	    line_feature_all.push_back(line_2_Buf.front());
	    line_2_Buf.pop();
	    
	    line_feature_all.push_back(line_3_Buf.front());
	    line_3_Buf.pop();
	    
	    vps = vp_Buf.front();
	    vp_Buf.pop();
	    
	    mBuf.unlock();
	    
	     if(imu_flag)
	    ROS_INFO("The number of imu data is %d  curTime %f imu is %f ~%f",accVector.size(),curTime, accVector[0].first,accVector[accVector.size()-1].first);
	   
	    else
	    {
	      ROS_INFO("imu error");
	      continue;
	    }
	    

            if(USE_IMU)
            {
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);  
		//旋转矩阵给了一个初值,对齐了重力方向，但航向角保持不变
		
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            mProcess.lock();
	    
	    ROS_ASSERT(line_feature_all[0].first == (line_feature_all[1].first + line_feature_all[2].first)/2);
            cout<<"Processing image frame_count is"<<frame_count<<endl;
	    assert(vps.second.size() == 3 && line_feature_all.size() == 3 );
	    processImage(feature, line_feature_all,vps);
            prevTime = curTime;
            cout<<"finish processImage"<<endl;
            printStatistics(*this, 0);   //显示一些状态

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
// 	    pubLinesCloud(*this, header);
	    pubStructuralonly(*this, header);
	    publine_by_point(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            mProcess.unlock();
	   
	    
        }

      
        if (! MULTIPLE_THREAD)
            break;
	
	

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    //第一个相机相对于重力的方向
    
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);   
    double yaw = Utility::R2ypr(R0).x();
    //经过验证，此处的yaw角已经很小，因为上面得到的R0已经将yaw消去
    
    //得到w系往imu系转换是的欧拉角的航向角
    cout<<"Finanl yaw is " <<yaw<<endl;
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}


void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

void Estimator::processImage(const pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image, 
			     const vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> &line_feature_all,
			     const pair<double, vector<cv::Point3d>> &vps)
{
 
    
   ROS_ASSERT(image.first == line_feature_all[0].first && image.first == vps.first);
   double header = image.first;
  
   
   if(1)
   {
      ready_use_line = true; 
      if_use_line = true;
   }
   else
   {//加入开头，一定时间后，再使用线特征
         if( header > 1403638149  && !if_use_line)
    {
       frame_count_ready_line++;  
       ready_use_line = true;   //表示在系统中开始添加线特征
       waitKey(0);
       
    }
    
    if(frame_count_ready_line == WINDOW_SIZE)
      if_use_line = true;   //添加线特征超过10帧后，将已经三角化的线特征引入优化环节
   }
   

      
      
      
      
      
      
//     else
//     {  if_use_line = false;
//        ROS_WARN("NO use line ");
//        int num_line = 0;
//        for(auto & it_per_id : f_manager.linefeature)
//        {
// 	 num_line++;
//        }
//        
//        cout<<"Num of line :"<<num_line<<endl;
//     }
    
    f_manager.ready_use_line = ready_use_line;
    //表示在linefeature里开始添加线特征
    
    
   cout<<"\n\n"<<endl;
    ROS_DEBUG("new image coming  %f  ------------------------------------------", header);
    ROS_DEBUG("Adding feature points %lu", image.second.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image.second, line_feature_all, vps.second, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }  
    
    if(!if_use_line && ready_use_line)
        marginalization_flag = MARGIN_OLD;
    //系统中已经开始有线特征，但还没有充满整个窗口，则marge掉最老帧，使其尽快充满整个窗口
    
//     if(solver_flag == NON_LINEAR && Rws_state[0] == false)
//     
//     //发现0帧的Rws有问题，则尽快marge掉
    
    
    if(ready_use_line)
    get_start_frame(vps.second);
    //得到s系在相机系下的表示 
//     会得到Rcs[frame_count] and Rws_state[frame_count]
    
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image.second, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    ROS_INFO("Check line data ...");
    
    find_line_on_point(image, line_feature_all);
    //可视化显示接收到的线特征的观测,并输出线在每帧的观测状态
    if(1)
  {
     int line_num = 0;
     
     cv::Mat image_show, image_show_right;
     cv::Mat image_show_point, image_show_right_point;
      featureTracker.cur_img_undistor.copyTo(image_show);
      featureTracker.rightImg_undistor.copyTo(image_show_right);
      featureTracker.cur_img_undistor.copyTo(image_show_point);
      featureTracker.rightImg_undistor.copyTo(image_show_right_point);
      
//       cv::remap(curimage, image_show, featureTracker.m_undist_map1[0], featureTracker.m_undist_map2[0], CV_INTER_LINEAR);
//       cv::remap(curimage_right, image_show_right, featureTracker.m_undist_map1[1], featureTracker.m_undist_map2[1], CV_INTER_LINEAR);
//      
//       cv::remap(curimage, image_show_point, featureTracker.m_undist_map1[0], featureTracker.m_undist_map2[0], CV_INTER_LINEAR);
//       cv::remap(curimage_right, image_show_right_point, featureTracker.m_undist_map1[1], featureTracker.m_undist_map2[1], CV_INTER_LINEAR);
     
      
      cv::cvtColor(featureTracker.cur_img_undistor, image_show, CV_GRAY2BGR);
      cv::cvtColor(featureTracker.rightImg_undistor, image_show_right, CV_GRAY2BGR);
      cv::cvtColor(featureTracker.cur_img_undistor, image_show_point, CV_GRAY2BGR);
      cv::cvtColor(featureTracker.rightImg_undistor, image_show_right_point, CV_GRAY2BGR);
      
      
      for (auto &it_per_id : f_manager.linefeature)
      { /*cout<<"id "<<it_per_id.feature_id;*/
	char write_id[10];
	sprintf(write_id, "%5d", it_per_id.feature_id);
//         lineFeaturePerFrame last_observe = it_per_id.linefeature_per_frame.back();	
	for(auto &last_observe : it_per_id.linefeature_per_frame)
	{  /*cout<<" "<<last_observe.In_frame_count;*/
	    if(last_observe.In_frame_count == frame_count)
	  {  
	    line_num++;
	    cv::Point2d pt1, pt2;
	    char write_pt1[20], write_pt2[20];
             	    
	    pt1.x = last_observe.lineobs[0];
	    pt1.y = last_observe.lineobs[1];
	    pt2.x = last_observe.lineobs[2];
	    pt2.y = last_observe.lineobs[3];
	    
	    snprintf(write_pt1, sizeof(write_pt1), "%.3f %.3f", last_observe.lineobs[0],last_observe.lineobs[1]);
	    snprintf(write_pt2, sizeof(write_pt2), "%.3f %.3f", last_observe.lineobs[2],last_observe.lineobs[3]);
	    

	    
	    pt1 = To_picture(pt1);
	    pt2 = To_picture(pt2);
	    
	    
// 	    cv::putText(image_show, write_pt1, pt1, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
// 	    cv::putText(image_show, write_pt2, pt2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	    
	    cv::putText(image_show, write_id, (pt1+pt2)/2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	    cv::line(image_show, pt1, pt2,cv::Scalar(0,0,255), 2);
	    
	    if(last_observe.is_stereo)
	    {
	      char write_pt1_right[20], write_pt2_right[20];
	      
	    pt1.x = last_observe.lineobs_R[0];
	    pt1.y = last_observe.lineobs_R[1];
	    pt2.x = last_observe.lineobs_R[2];
	    pt2.y = last_observe.lineobs_R[3];
	    
	    snprintf(write_pt1_right, sizeof(write_pt1_right), "%.3f %.3f", pt1.x,pt1.y);
	    snprintf(write_pt2_right, sizeof(write_pt2_right), "%.3f %.3f", pt2.x,pt2.y);
	    pt1 = To_picture(pt1);
	    pt2 = To_picture(pt2);
	    
//             cv::putText(image_show_right, write_pt1_right, pt1, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
// 	    cv::putText(image_show_right, write_pt2_right, pt2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	    cv::putText(image_show_right, write_id, (pt1+pt2)/2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	    cv::line(image_show_right, pt1, pt2,cv::Scalar(0,0,255), 2);
	    }
	  }
	}
// 	cout<<endl;

      
      }
      
       for (auto &it_per_id : f_manager.feature)
      { /*cout<<"id "<<it_per_id.feature_id;*/
	char write_id[10];
	sprintf(write_id, "%5d", it_per_id.feature_id);
//         lineFeaturePerFrame last_observe = it_per_id.linefeature_per_frame.back();	
	  /*cout<<" "<<last_observe.In_frame_count;*/
	   if((it_per_id.start_frame + it_per_id.feature_per_frame.size() -1 ) == frame_count)
	  {  //表示该点的最后一次观测在当前帧上
	    Vector3d point = it_per_id.feature_per_frame.back().point;
	    cv::Point2d pt1;
	    pt1.x = point.x();
	    pt1.y = point.y();

	   
	    pt1 = To_picture(pt1);
	
	    
// 	    cv::putText(image_show, write_pt1, pt1, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
// 	    cv::putText(image_show, write_pt2, pt2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	    
// 	    cv::putText(image_show_point, write_id, pt1, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
            cv::circle(image_show,pt1,3,CV_RGB(0,255,0),-1);
	    
	    if(it_per_id.feature_per_frame.back().is_stereo)
	    {
	    
	      	 
	      point = it_per_id.feature_per_frame.back().pointRight;
	      Point2d pt1;
	      pt1.x = point.x();
	      pt1.y = point.y();

	   
	    pt1 = To_picture(pt1);
	


//             cv::putText(image_show_right, write_pt1_right, pt1, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
// 	    cv::putText(image_show_right, write_pt2_right, pt2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
// 	    cv::putText(image_show_right_point, write_id, pt1, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	    cv::circle(image_show_right,pt1,3,CV_RGB(0,255,0),-1);
	    }
	  }
	}  
	
      map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points = image.second;
//        for(int i = 0; i<4; i++)
//        {
// 	 for(auto &it_per_id : point_set[i])
// 	 {
// 	   vector<pair<int, Eigen::Matrix<double, 7, 1>>> point_info = points[it_per_id];
// 	   Eigen::Matrix<double, 7, 1> point_observe = point_info[0].second;
// 	    
// 	    cv::Point2d pt1;
// 	    pt1.x = point_observe(0);
// 	    pt1.y = point_observe(1);
//             pt1 = To_picture(pt1);
// 	    if(i == 0)
// 	    cv::circle(image_show,pt1,3,CV_RGB(0,255,0),-1);
// 	     else if(i == 1)
// 	    cv::circle(image_show,pt1,3,CV_RGB(0,0,255),-1);
// 	    
// 	    else if(i == 2)
// 	    cv::circle(image_show,pt1,3,CV_RGB(0,255,255),-1);
// 	        
// 	    else 
// 	    cv::circle(image_show,pt1,3,CV_RGB(255,255,0),-1);
// 	 }
//        }
//      

      
      
      for(auto &it_per_line : line_on_point)
      {
	char write_id[10];
	sprintf(write_id, "%5d", it_per_line.first);
	
	 cv::Point2d pt1, pt2;
	 vector<pair<int, Vector4d>> line_info =  all_lines[it_per_line.first];
	 Vector4d line_left_ob = line_info[0].second;
	 pt1.x = line_left_ob(0);
	 pt1.y = line_left_ob(1);
	 
	 pt2.x = line_left_ob(2);
	 pt2.y = line_left_ob(3);
	 

	    pt1 = To_picture(pt1);
	    pt2 = To_picture(pt2);
	    
        cv::line(image_show_point, pt1, pt2,cv::Scalar(0,0,255), 2);
	cv::putText(image_show_point, write_id, (pt1+pt2)/2, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
	vector<pair<int, Vector2d>> id_points = it_per_line.second;
	
	for(auto &it_per_point : id_points)
	{
	  sprintf(write_id, "%5d", it_per_point.first);
	   vector<pair<int, Eigen::Matrix<double, 7, 1>>> point_info = points[it_per_point.first];
	   Eigen::Matrix<double, 7, 1> point_observe = point_info[0].second;
	   
	    cv::Point2d pt3;
	    pt3.x = point_observe(0);
	    pt3.y = point_observe(1);
            pt3 = To_picture(pt3);
	    
	    cv::putText(image_show_point, write_id, pt3, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255 ), 1);
	     cv::circle(image_show_point,pt3,3,CV_RGB(0,255,0),-1);
             cv::line(image_show_point, pt1, pt3,cv::Scalar(0,255,0), 1);
	    
	}
	 
      }
      
      
      cv::imshow("Check left",image_show);
      cvWaitKey(1);
      cv::imshow("Check right",image_show_right);
      cvWaitKey(1);
      
      cv::imshow("left point ",image_show_point);
      cvWaitKey(1);
      cv::imshow("right point ",image_show_right_point);
      cvWaitKey(1);
      ROS_INFO("line_num is %d",line_num);
//       waitKey(0);
  }
    
    //用于检查线的观测，观测到的线的坐标，对应的消失点
    if(0)
    {
      for (auto &it_per_id : f_manager.linefeature)
      { 
	  lineFeaturePerFrame last_observe = it_per_id.linefeature_per_frame.back();
          if(last_observe.In_frame_count == frame_count)
	  {
	    
	    cout<<"id: "<<it_per_id.feature_id<<" observation "<<last_observe.lineobs.transpose()<<" VP "<<last_observe.corresp_vp.transpose()<<endl;
	
	    
	    if(last_observe.is_stereo)
	    {
	      cout<<"right observation "<<last_observe.lineobs_R.transpose()<<endl;
	    
	    }
	  }
      }
      

  }
    
    
    
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    //从这里开始去估算位姿等操作
    if (solver_flag == INITIAL)
    {
        

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {   cout<<" Start triangulate frame_count "<<frame_count<<endl;
	  
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
	    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            
	    if(ready_use_line)
	    {
	      //得到Rws的初始值
	     //判断是否有新的Rws
	      //调整每帧的Rcs朝向（表示对结构化场景的观测），与Rws保持一致
	      get_start_frame_in_world(frame_count);
// 	      f_manager.process_direction(Ps, tic, ric, Rws, Which_Rws, Rws_state);
// 	      f_manager.triangulateLine_from_point(Ps, tic, ric, Rws, Which_Rws, Rws_state, line_on_point);
// 	      std_msgs::Header header_;
//               header_.frame_id = "world";
//               header_.stamp = ros::Time(header); 
// 	      publine_by_point(*this, header_);
// 	       waitKey(0);
	      f_manager.triangulateLine(Ps, tic, ric, Rws, Which_Rws, Rws_state, line_on_point);
	      
	      //线的三角化
	    }

	
	     
	    if (frame_count == WINDOW_SIZE)
            {  
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }//写入窗口中各帧的位姿
                
                solveGyroscopeBias(all_image_frame, Bgs); 
		//求解陀螺仪的bias
		
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }//重新计算预积分
              
		
// 		onlyLineOpt();
	 if(if_use_line)
	 {
	   onlyStructuralLineOpt();
// 		visual_Rws();
// 	       if(1)
// 		{
// 		   std_msgs::Header header_;
//                     header_.frame_id = "world";
//                     header_.stamp = ros::Time(header);
// 		    
// 		cv::waitKey(0);
// 		pub_Lines_change(*this, header_);
// 		cout<<"waitting to go on next step"<<endl;
// 		cv::waitKey(0);
// 		}
		
	   optWithStructuralLine();
	
// 		 if(1)
// 		{
// 		   std_msgs::Header header_;
//                     header_.frame_id = "world";
//                     header_.stamp = ros::Time(header);
// 		    
// 		cv::waitKey(0);
// 		pub_Lines_change(*this, header_);
// 		cout<<"waitting to go on next step"<<endl;
// 		cv::waitKey(0);
// 		}
	  }
	  else
	   optimization();

// 	     visual_Rws();
		
          
		
// 		optimization();
		solver_flag = NON_LINEAR;
                slideWindow();
		//后边是否应该有外点的拒绝
		ROS_INFO("Initialization finish!");
            }
        }

     
       ROS_DEBUG("check_Sframe_between_camera before initial ");
        check_Sframe_between_camera();
	
        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else  //NON_LINEAR 
    { 
      cout<<"NON_LINEAR  "<<frame_count<<endl;
        TicToc t_solve;
        if(!USE_IMU)
        f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
	
	
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

	
 
// 	ROS_DEBUG("check start_frame before optimization");
// 	check_Sframe_between_camera();
	if(if_use_line)
	{
	  onlyStructuralLineOpt();
	  optWithStructuralLine();

	}
      else
       optimization();
     
       if(ready_use_line)
       {
	 	
	 get_start_frame_in_world(frame_count);
// 	 f_manager.process_direction(Ps, tic, ric, Rws, Which_Rws, Rws_state);
// 	 f_manager.triangulateLine_from_point(Ps, tic, ric, Rws, Which_Rws, Rws_state, line_on_point);
// 	       std_msgs::Header header_;
//               header_.frame_id = "world";
//               header_.stamp = ros::Time(header); 
// 	      pubStructuralonly(*this, header_);
// 	       waitKey(0);
	       
         f_manager.triangulateLine(Ps, tic, ric, Rws, Which_Rws, Rws_state, line_on_point);
       }
       
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
	//根据点平均重投影误差，移除外点
	
        if (! MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

//        ROS_DEBUG("check start_frame after optimization and before slideWindow");
// 	check_Sframe_between_camera();
        slideWindow();
        f_manager.removeFailures();
	//移除的是失败的点特征
	
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
	
    }  
    
    if(if_use_line)
    {
     f_manager.update_lineEndpoint(Ps, tic, ric, Rws);
     TicToc merge_line_time;
     f_manager.merge_line(Ps, tic, ric, Rcs);
     ROS_DEBUG("merge_line_time %fms", merge_line_time); 
     
     ROS_DEBUG("line_on_point_triangulate :");
     
     
//      for(auto &it_line : f_manager.line_on_point_all_frame)
//      {
//        cout <<" l_id:"<<it_line.first;
//        
//     	int feature_id = it_line.first;
// 	Vector3d line_end_p1, line_end_p2;
// 	 
// 	
// 	 auto it = find_if(f_manager.linefeature.begin(), f_manager.linefeature.end(), [feature_id](const lineFeaturePerId &it)
//         {
//             return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
//         });
//         if (it == f_manager.linefeature.end())
//            continue;
// 	
//         else if (it->feature_id == feature_id)
//         {
// 	  //表示找到了那个在线上的点
// 	  if(!it->is_triangulation )
//            continue;
// 	  
// 	  line_end_p1 = it->ptw1;
// 	  line_end_p2 = it->ptw2;
//   
// 	}
// 	
//        
//        
//        
//        for(auto &it_point : it_line.second)
//        {
// 	 Vector3d w_pts_i;
// 	 feature_id = it_point.first;
// 	 
// 	 auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(), [feature_id](const FeaturePerId &it)
//         {
//             return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
//         });
// 	 
//         if (it == f_manager.feature.end())
//            continue;
// 	else if (it->feature_id == feature_id)
//         {
// 	  //表示找到了那个在线上的点
// 	 if(it->estimated_depth <= 0 )
//            continue;
// 	 
// 	   int imu_i = it->start_frame;
//            Vector3d pts_i = it->feature_per_frame[0].point * it->estimated_depth;
//             w_pts_i = Rs[imu_i] * (ric[0] * pts_i + tic[0]) + Ps[imu_i];
//   
// 	}
// 	 
// 	 double distance = point_line_distance(line_end_p1, line_end_p2, w_pts_i);
// 	 
// // 	Vector2d point_observe_info = it_point.second;
// // 	
// // 	if(point_observe_info(1) > 3)
// // 	{
// // 	   ofstream foutC(Output_Rws, ios::app);
// //            foutC.precision(4);
// //            foutC << distance<<" " ; 
// // 	   foutC.close();
// // 	}
// 	 
// 	 
// 	 cout<<"-"<<it_point.first<<"("<<it_point.second.transpose()<<" "<<distance<<")";
// 	 cout<<" "<<w_pts_i.transpose();
//        }
//        
//        cout<<endl;
//      }
     
     cout<<"line_on_point_all_framem size : "<<f_manager.line_on_point_all_frame.size()<<endl;
    }

//     ROS_DEBUG("Marge line triangulate info");
//     get_triangulated_line();

    cout<<"Number of triangulate line will be marged : "<<ids_triangulated_line.size()<<endl;
//     for(auto &it_per_id : ids_triangulated_line)
//     {
//       cout<<" "<<it_per_id;
//     }
//     cout<<endl;
    
//     check_structural_line_reproj();
    //检查冲投影误差及Rws的朝向，
    
    
    
}


    //根据消失点探测曼哈顿世界方向
void Estimator::get_start_frame(const vector<cv::Point3d> &vps)
{
     Vector3d vp1, vp2, vp3;
     double angle_1, angle_2, angle_3;
     vp1 = Vector3d(vps[0].x, vps[0].y, vps[0].z);
     vp2 = Vector3d(vps[1].x, vps[1].y, vps[1].z);
     vp3 = Vector3d(vps[2].x, vps[2].y, vps[2].z);

      angle_1 = acos(abs(vp1.dot(vp2)/(vp1.norm()*vp2.norm())));
      angle_1 = (angle_1*180)/acos(-1.0);
      angle_2 = acos(abs(vp1.dot(vp3)/(vp1.norm()*vp3.norm())));
      angle_2 = (angle_2*180)/acos(-1.0);
      angle_3 = acos(abs(vp2.dot(vp3)/(vp2.norm()*vp3.norm())));
      angle_3 = (angle_3*180)/acos(-1.0);
      
      
 if(angle_1>80 && angle_2>80 && angle_3>80)
  {//初始的消失点提取完全符合曼哈顿世界
      vp1.normalize();
      vp2.normalize();
      vp3.normalize();
  }
  //只有两个垂直，第三个通过正交化得到
  else if(angle_1 > 80)
  {
     vp1.normalize();
     vp2.normalize();
     vp3 = vp1.cross(vp2);
     vp3.normalize();
   }
  else if(angle_2 > 80)
  {
     vp1.normalize();
     vp3.normalize();
     vp2 = vp1.cross(vp3);
     vp2.normalize();
   }
   
  else if(angle_3 > 80)
  {
     vp3.normalize();
     vp2.normalize();
     vp1 = vp2.cross(vp3);
     vp1.normalize();
   }
   
     
  else 
  {  
     ROS_WARN("angle : %f %f %f", angle_1, angle_2, angle_3);
     Rws_state[frame_count] = false; 
     cout<<"This frame is not a structural woerld"<<endl;
//      waitKey(0);
     return;
  }
   
    Eigen::Matrix3d R_vp;
      R_vp.col(0) = vp1;
      R_vp.col(1) = vp2;
      R_vp.col(2) = vp3;
      
      orthonormalize(R_vp);  
      //正交化，会把第一个向量保持不变，验证过正交化后的向量与原始向量的夹角，很小
      
      double max_z = abs(R_vp(2,0));
      double max_y = abs(R_vp(1,0));
      int max_z_col=0;
      int max_y_col=0;
      for(int i=1;i<3;i++)
      {
	if(max_z < abs(R_vp(2,i)))
	{
	  max_z =abs(R_vp(2,i));
	  max_z_col = i;
	}
	
	if(max_y < abs(R_vp(1,i)))
	{
	  max_y = abs(R_vp(1,i));
	  max_y_col = i;
	}
	
      }
    
      assert(max_z_col != max_y_col);
      
     Eigen::Matrix3d  R_cs;
     
    //S系的X轴与相机系的Z轴朝向一致
   if(R_vp(2,max_z_col)<0)  
     R_cs.col(0) = -1*R_vp.col(max_z_col);
   else  
     R_cs.col(0) = R_vp.col(max_z_col);
     
    //s系的z轴方向与相机系的y轴相反
   if(R_vp(1,max_y_col)>0) 
     R_cs.col(2) = -1*R_vp.col(max_y_col);
   else  
     R_cs.col(2) = R_vp.col(max_y_col);
   
  //S系的Y轴与相机的X轴相反
   if(R_vp(0,3-max_y_col-max_z_col)>0)  
     R_cs.col(1) = -1*R_vp.col(3-max_y_col-max_z_col);
   else  
     R_cs.col(1) = R_vp.col(3-max_y_col-max_z_col);
   
  cout<<"R_cs is \n"<<R_cs<<endl;

  Rcs[frame_count] = R_cs;
  //让Rcs的z为1，就是观测得到的归一化平面的消失点
  Rws_state[frame_count] = true;
      
  return; 
     
}

 
void orthonormalize(Eigen::Matrix3d &ColVecs)
{
	ColVecs.col(0).normalize();
	double temp;
	for(std::size_t k = 0; k != ColVecs.cols() - 1; ++k)
	{
		for(std::size_t j = 0; j != k + 1; ++j)
		{
			temp = ColVecs.col(j).transpose() * ColVecs.col(k + 1);
			ColVecs.col(k + 1) -= ColVecs.col(j) * temp;
		}
		ColVecs.col(k + 1).normalize();
	}
}


bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
  ROS_WARN("before optimizaiton , state are :");
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
      cout<<" Ps "<< i <<" : "<<Ps[i].transpose();
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
	
	para_Rcs[i][0] = 0;
	para_Rcs[i][1] = 0;
	para_Rcs[i][2] = 0;
	
    Vector3d Ri = Utility::R2ypr(Rs[i]);
    cout<<" Rs "<<i<<" : "<<Ri.transpose()<<endl;
	
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
	
       q = Eigen::Quaterniond (Rcs[i]);
       para_Rcs[i][3] = q.x();
       para_Rcs[i][4] = q.y();
       para_Rcs[i][5] = q.z();
       para_Rcs[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }
    


       

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
    
    
  if(if_use_line)
  {
    Vector3d Ri = Utility::R2ypr(Rws.back());
    cout<<" Rws "<<Ri.transpose()<<endl;
   
    //将Rws放入待优化的参数中
     {
        para_Rws[0][0] = 0;
	para_Rws[0][1] = 0;
	para_Rws[0][2] = 0;
	
       Quaterniond q{Rws.back()};
       para_Rws[0][3] = q.x();
       para_Rws[0][4] = q.y();
       para_Rws[0][5] = q.z();
       para_Rws[0][6] = q.w();
     }
     
         //获得结构线的二参数
  MatrixXd structural_line = f_manager.get_structural_line_Vector();
  for (int i = 0; i < f_manager.getLineFeatureCount(); ++i) {
        para_structural_line[i][0] = structural_line.row(i)[0];
        para_structural_line[i][1] = structural_line.row(i)[1];
	
	para_Rsl[i][0] = 0;
	para_Rsl[i][1] = 0;
	para_Rsl[i][2] = 0;
	para_Rsl[i][3] = structural_line.row(i)[2];
	para_Rsl[i][4] = structural_line.row(i)[3];
	para_Rsl[i][5] = structural_line.row(i)[4];
	para_Rsl[i][6] = structural_line.row(i)[5];
	
    }
     
     
  }
    
  
}

void Estimator::double2vector()
{  //初始的欧拉角
  ROS_WARN("after optimizaiton state is ");
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];
    Matrix3d rot_diff;
    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(USE_IMU)
    { //优化以后的欧拉角
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
	ROS_WARN("y_diff: %f", y_diff);
	
        //TODO
         rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }
        

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
				    
            Rcs[i] = Quaterniond(para_Rcs[i][6], para_Rcs[i][3], para_Rcs[i][4], para_Rcs[i][5]);

                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
		
		
		
               cout<<" Ps "<< i <<" : "<<Ps[i].transpose();
	       Vector3d Ri = Utility::R2ypr(Rs[i]);
               cout<<" Rs "<<i<<" : "<<Ri.transpose()<<endl;
            
        }
        
        
        
        

        
        
//         Matrix3d Rws_set = Utility::ypr2R(Vector3d(50, -0.0715, 0.271));
// 	Matrix3d Rws_set = Utility::ypr2R(Vector3d(51.7, -0.141, 0.143));
//          Matrix3d Rws_set = Utility::ypr2R(Vector3d(46.1, 1.07, -0.936));
//              Matrix3d Rws_set = Utility::ypr2R(Vector3d(47, 0.801, -0.698));
// 	     Matrix3d Rws_set = Utility::ypr2R(Vector3d( 48.6, 0.223, -0.499));
//             Matrix3d Rws_set = Utility::ypr2R(Vector3d(46.8, 0.757, -0.937));
// 	  
// 
// 
// 	 Rws[Rws.size() - 1] = Rws_set;
	
 
/*	  
	 if(!If_Rws_constant)
	 {
	    if(yaw_buffer.empty())
	    {
	      yaw_buffer.push_back(Ri(0));
	      pich_buffer.push_back(Ri(1));
	    }
	    
	   else
	   {
	     if(yaw_buffer.size() == 1)
	     {
	       if(fabs(Ri(0) - yaw_buffer[0]) < 0.2 )
	       {
		  yaw_buffer.push_back(Ri(0));
		  pich_buffer.push_back(Ri(1));
	       }
	      
	       else
	       { 
	        yaw_buffer.clear();
		pich_buffer.clear();
	        yaw_buffer.push_back(Ri(0));
		pich_buffer.push_back(Ri(1));
	       }
	     }
	     else if(yaw_buffer.size() == 2)
	     {
	       if(fabs(Ri(0) - yaw_buffer[0]) < 0.3 && fabs(Ri(0) - yaw_buffer[1]) < 0.2)
	        {
		  yaw_buffer.push_back(Ri(0));
		  pich_buffer.push_back(Ri(1));
	       }
	       else
	       { 
	        yaw_buffer.clear();
		pich_buffer.clear();
	        yaw_buffer.push_back(Ri(0));
		pich_buffer.push_back(Ri(1));
	       }
	     }
	     else if(yaw_buffer.size() == 3)
	     {
	       if(fabs(Ri(0) - yaw_buffer[0]) < 0.3 && fabs(Ri(0) - yaw_buffer[1]) < 0.3
		  && fabs(Ri(0) - yaw_buffer[2]) < 0.2 
		  && fabs(Ri(1) - pich_buffer[0]) < 0.05
		  && fabs(Ri(1) - pich_buffer[1]) < 0.05
	          && fabs(Ri(1) - pich_buffer[2]) < 0.03)
	        If_Rws_constant = true;
	       else
	       { 
	        yaw_buffer.clear();
		pich_buffer.clear();
	        yaw_buffer.push_back(Ri(0));
		pich_buffer.push_back(Ri(1));
	       }
	     } 
	     
	   }
	 }*/
	  
	   
	
  }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).toRotationMatrix();
        }
    }

    
   
    
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];
    
 if(if_use_line)
 {
  if(solver_flag  != INITIAL && If_Rws_constant)
     rot_diff.setIdentity();
     { 
      Rws[Rws.size() - 1] = rot_diff *Quaterniond(para_Rws[0][6], para_Rws[0][3], para_Rws[0][4], para_Rws[0][5]).normalized().toRotationMatrix();
     }
   
    Vector3d Ri = Utility::R2ypr(Rws.back());
    cout<<" Rws "<<Ri.transpose()<<endl;
	   
   int num_getline = f_manager.getLineFeatureCount();
   MatrixXd structural_line_vec(num_getline, 2);
    for (int i = 0; i < num_getline; ++i) {
        Vector2d structural_line(para_structural_line[i][0], para_structural_line[i][1]);
        structural_line_vec.row(i) = structural_line;
    }
   f_manager.set_structural_line(structural_line_vec,Ps,Rs,tic,ric);
   
 }
    
    

   
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}



void  Estimator::onlyLineOpt()
{
    //固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
        // 固定 pose
        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    for (int i = 0; i < 1; i++)         // 现只用一个相机
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }
    vector2double();// 将那些保存在 vector向量里的参数 移到 double指针数组里去

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && 
	      it_per_id.linefeature_per_frame[0].In_frame_count < WINDOW_SIZE - 2 &&
	      it_per_id.is_triangulation))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
        /*
        std::cout << para_LineFeature[feature_index][0] <<" "
                << para_LineFeature[feature_index][1] <<" "
                << para_LineFeature[feature_index][2] <<" "
                << para_LineFeature[feature_index][3] <<"\n";
        */
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock( para_LineFeature[feature_index], SIZE_LINE, local_parameterization_line);  // p,q

        int imu_i = it_per_id.triangulate_frame_count;
	int imu_j;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j = it_per_frame.In_frame_count;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector4d obs = it_per_frame.lineobs;                          // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs);     // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[feature_index]);
	    //将世界系下的线又转回到相机系下，
            f_m_cnt++;
        }
    }

    if(feature_index < 3)
    {
        return;
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    std::cout <<"!!!!!!!!!!!!!onlyLineOpt!!!!!!!!!!!!!\n";
    double2vector();
    std::cout << summary.FullReport()<<std::endl;

//     f_manager.removeLineOutlier(Ps,tic,ric);

}



void Estimator::onlyStructuralLineOpt()
{
 //固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)    // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // p,q
	
	// 固定 pose
        problem.SetParameterBlockConstant(para_Pose[i]);
   }

 
 //添加世界系下的Rws
   ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Rws[0], SIZE_POSE, local_parameterization);  
    if(solver_flag  != INITIAL && If_Rws_constant)
       problem.SetParameterBlockConstant(para_Rws[0]);
  
    vector2double();
    // 将那些保存在 vector向量里的参数 移到 double指针数组里去

    // 所有特征
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation /*&& it_per_id.triangulate_frame_count < WINDOW_SIZE-1*/))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
        /*
        std::cout << para_LineFeature[feature_index][0] <<" "
                << para_LineFeature[feature_index][1] <<" "
                << para_LineFeature[feature_index][2] <<" "
                << para_LineFeature[feature_index][3] <<"\n";
        */
        it_per_id.solve_flag = 1;
	
	//如果该线已经存在已关联的点，则首先判断在本次优化中，这个点是否还存在
	 if(it_per_id.related_point_id != -1 && f_manager.point_para_position.count(it_per_id.related_point_id) == 0)
	   it_per_id.related_point_id = -1;
	
	 //如果该线还没有被绑定，且该线存在共线点，则寻找到合适的点(观测到共线)，与之绑定
	if(it_per_id.related_point_id == -1 && f_manager.line_on_point_all_frame.count(it_per_id.feature_id))
	{
	 map<int, Vector2d> point_info = f_manager.line_on_point_all_frame[it_per_id.feature_id];
	  int max = -1, id_point;
	  for(auto &it_point : point_info)
	  {
	    if(f_manager.point_para_position.count(it_point.first))
	    {
	      //还是首先判断这个点是否在本次的优化中
	      Vector2d point_relaion = it_point.second;
	      if(max < point_relaion(1))
	      {
		max = point_relaion(1);
		id_point = it_point.first;
	      }
	    }
	  }
	  
	  if(max > 4)
	    it_per_id.related_point_id = id_point;
	  //实现点线的绑定
	  
	}
	
	//上面已经确保了与之绑定的点在本次优化中是存在的
	//经过上面的操作，到这里，就可以根据是否有与之绑定的点，选择相应的点构建线参数及重投影误差
	
	if(it_per_id.related_point_id != -1)  
       {
	 Vector4d point_para = f_manager.point_para_position[it_per_id.related_point_id];
	 int point_index = point_para(0);
	 int point_imu_i = point_para(1);
	 Vector3d pts_i = Vector3d(point_para(2), point_para(3), 1);
	 double inv_depth = para_Feature[point_index][0];
	 
	 int imu_j;
	 cout<<"l_ID:"<<it_per_id.feature_id<<" p_id:"<<it_per_id.related_point_id<<" direction "<<it_per_id.L_frame_flag<<endl;
	 
	 
	    
	    Vector2d residual;
// 	      residual = check_line_point_reproj_err(pts_i, obs,  it_per_id.Rsl, point_imu_i,  imu_j, inv_depth);
	    
	    residual = check_line_point_distance_L(pts_i, point_imu_i,  inv_depth,  it_per_id.theta, it_per_id.d_inv, it_per_id.Rsl);
	      
	   if(residual(0) < 1.5 && residual(1) < 1.5 && 0)
	   {
	       distance_StructuralLine_Point *f = new distance_StructuralLine_Point(pts_i, it_per_id.Rsl, ric[0], tic[0]);     
              
	       problem.AddResidualBlock(f, loss_function,
                                        para_Pose[point_imu_i],
                                        para_Rws[0],
				        para_Feature[point_index], 
				        para_structural_line[feature_index]);
	   } 
	    
	    
             cout<<" ("<<residual.transpose()<<")"<<endl;
	     if(residual(0) < 1.5 && residual(1) < 1.5 )
	     {
	         ofstream foutC(Output_Rws, ios::app);
	      foutC.precision(4);
             foutC << residual(0)<<" "<<residual(1)<<" \n" ; 
	     
	     foutC.close();
	    }
	 
	  
	 cout<<endl;
	}
	
	
// 	else
	{
	  //没有绑定的点，优化线的参数
	  int imu_i = it_per_id.triangulate_frame_count;
	  int imu_j;
          for (auto &it_per_frame : it_per_id.linefeature_per_frame)
          {
            imu_j = it_per_frame.In_frame_count;
	     Vector4d obs = it_per_frame.lineobs;    
           
           
	    // 在第j帧图像上的观测
            ProjectionStructuralLine *f = new ProjectionStructuralLine(obs, it_per_id.Rsl, ric[0], tic[0]); 
	   //重投影参差的构造函数，给一些固定的参数赋值
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Rws[0],
				     para_structural_line[feature_index]);
	 
            f_m_cnt++;
          }
	}
 
    }

    if(feature_index < 3)
    {
        return;
    }
    
//     cout<<"before optimization Rcs "<<endl;
//     for(int i=0; i<=WINDOW_SIZE; i++)
//     {
//       for(int j=0; j<7;j++)
//       {
// 	cout<<" "<<para_Rcs[i][j];
//       }
// 	cout<<endl;
//     }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);
    
    double2vector();
//     std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps,tic,ric,Rws);
}


void Estimator::optWithStructuralLine()
{
  TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
      	
	if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    
    {
       //添加世界系下的Rws
     ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
     problem.AddParameterBlock(para_Rws[0], SIZE_POSE, local_parameterization); 
    }
    
       if(solver_flag  != INITIAL && If_Rws_constant)
       problem.SetParameterBlockConstant(para_Rws[0]);

    
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
	  //这里的CamFactor表示的是左右相机
	  //Frame就是一个相机位姿，可能包含左右相机
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }
//点特征三角化所在帧的观测
            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
		  
		  //同时优化前后两个相机位姿，左右两个相机的外参，共四个位姿参数
		  
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
		  //是用来优化左右两个相机外参的
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
               
            }
            f_m_cnt++;
        }
    }

  bool if_opt_line = 1;
  if(if_opt_line)
  {
      
       //line optimization
  int line_m_cnt = 0;
    int linefeature_index = -1;
     for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
         if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation /*&& it_per_id.triangulate_frame_count < WINDOW_SIZE -1*/))  // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
  
  
        it_per_id.solve_flag = 1;
	//表示该特征进入了优化
      
	
		//如果该线已经存在已关联的点，则首先判断在本次优化中，这个点是否还存在
	 if(it_per_id.related_point_id != -1 && f_manager.point_para_position.count(it_per_id.related_point_id) == 0)
	   it_per_id.related_point_id = -1;
	
	 //如果该线还没有被绑定，且该线存在共线点，则寻找到合适的点(观测到共线)，与之绑定
	if(it_per_id.related_point_id == -1 && f_manager.line_on_point_all_frame.count(it_per_id.feature_id))
	{
	 map<int, Vector2d> point_info = f_manager.line_on_point_all_frame[it_per_id.feature_id];
	  int max = -1, id_point;
	  for(auto &it_point : point_info)
	  {
	    if(f_manager.point_para_position.count(it_point.first))
	    {
	      //还是首先判断这个点是否在本次的优化中
	      Vector2d point_relaion = it_point.second;
	      if(max < point_relaion(1))
	      {
		max = point_relaion(1);
		id_point = it_point.first;
	      }
	    }
	  }
	  
	  if(max > 4)
	    it_per_id.related_point_id = id_point;
	  //实现点线的绑定
	  
	}
	
	//上面已经确保了与之绑定的点在本次优化中是存在的
	//经过上面的操作，到这里，就可以根据是否有与之绑定的点，选择相应的点构建线参数及重投影误差
	
	if(it_per_id.related_point_id != -1)  
       {
	 Vector4d point_para = f_manager.point_para_position[it_per_id.related_point_id];
	 int point_index = point_para(0);
	 int point_imu_i = point_para(1);
	 Vector3d pts_i = Vector3d(point_para(2), point_para(3), 1);
	 double inv_depth = para_Feature[point_index][0];
	 
	 int imu_j;

	 
	 
	    
	    Vector2d residual;
// 	      residual = check_line_point_reproj_err(pts_i, obs,  it_per_id.Rsl, point_imu_i,  imu_j, inv_depth);
	    
	    residual = check_line_point_distance_L(pts_i, point_imu_i,  inv_depth,  it_per_id.theta, it_per_id.d_inv, it_per_id.Rsl);
	      
	    
           if(residual(0) < 1.5 && residual(1) < 1.5 )
	   {
	     
	       distance_StructuralLine_Point *f = new distance_StructuralLine_Point(pts_i, it_per_id.Rsl, ric[0], tic[0]);     
              
	       problem.AddResidualBlock(f, loss_function,
                                        para_Pose[point_imu_i],
                                        para_Rws[0],
				        para_Feature[point_index], 
				        para_structural_line[linefeature_index]);
	       
	      cout<<"l_ID:"<<it_per_id.feature_id<<" p_id:"<<it_per_id.related_point_id<<" direction "<<it_per_id.L_frame_flag<<endl;
	   } 

	  
	 cout<<endl;
	}
	
	
	
	
	
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
          int imu_j = it_per_frame.In_frame_count;
	    Vector4d obs = it_per_frame.lineobs;

	    // 在第j帧图像上的观测
            ProjectionStructuralLine *f = new ProjectionStructuralLine(obs, it_per_id.Rsl, ric[0], tic[0]);     
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Rws[0],
				     para_structural_line[linefeature_index]);
	 
            line_m_cnt++;
        }
    }
    
    ROS_INFO("lineFactor: %d, pointFactor:%d", line_m_cnt, f_m_cnt);
    
    
//         if(line_m_cnt > 20)
//     {
//         double scale = std::min(f_m_cnt /(2. * line_m_cnt), 10.);
//         ProjectionStructuralLine::sqrt_info =  scale * FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
//         std::cout << "========== Line factor weight ========== " << scale  << std::endl; 
//     }


      
    }

    
    
    
    
    
//     ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 4;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    
    //暂时不设置优化的时间限制
//     if (marginalization_flag == MARGIN_OLD)
//         options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//     else
//         options.max_solver_time_in_seconds = SOLVER_TIME;
    
    
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);
	
       Vector3d Ri = Utility::R2ypr(Rws.back());
       cout<<" Rws "<<Ri.transpose()<<endl;
//     
//     ofstream foutC(Output_Rws, ios::app);
//         foutC.precision(3);
//         foutC << Ri(0)<<" "  
// 	      << Ri(1)<<" "
// 	      << Ri(2)<<" \n";
// 	      
// 	  foutC.close();
    
    	 if(!If_Rws_constant)
	 {
	    if(yaw_buffer.size() < 10)
	    {
	      yaw_buffer.push_back(Ri(0));
	   
	    }
	    
	   else
	   {
	     for(int i = 1; i<10; i++)
	      yaw_buffer[i-1] = yaw_buffer[i];
	     
	     yaw_buffer[9] = Ri(0);
	    
	     bool interval  = false;
	    for(int i = 1; i<10; i++)
	    {
	      if(fabs(yaw_buffer[i] - yaw_buffer[i-1]) > 0.2)
	      {
		interval = true;
		break;
	      }
		
	    }
	    
	    if(!interval && fabs(yaw_buffer[9] - yaw_buffer[0]) < 0.2)
	    {
	        If_Rws_constant = true;
		for(int i = 0; i<yaw_buffer.size(); i++)
		cout<<yaw_buffer[i]<<" ";
		cout<<endl;
		waitKey(0);
	    }
	     
	     
	    
	   }
	 }
    
    
    if(frame_count < WINDOW_SIZE)
        return;
    //得改成结构线的
//     f_manager.removeLineOutlier(Ps,tic,ric,Rws); 
  
    
    //开始边缘化操作
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
		//1.当第一次marg——old,,第二次还是marge—old，此处不会有Rcs
		
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {//margin point feature 
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        
     //margin line
     if(if_opt_line)
         {
            // Line feature
            int linefeature_index = -1;
            for (auto &it_per_id : f_manager.linefeature)
            {
                it_per_id.used_num = it_per_id.linefeature_per_frame.size();                // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
                 if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation /*&& it_per_id.triangulate_frame_count < WINDOW_SIZE -1*/)) // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
                    continue;
                ++linefeature_index;            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

                int imu_i = it_per_id.linefeature_per_frame[0].In_frame_count;
		//                 int imu_i = it_per_id.triangulate_frame_count;
		 if (imu_i != 0)             // ` 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;
		 
	
                for (auto &it_per_frame : it_per_id.linefeature_per_frame)
                {
                  int imu_j = it_per_frame.In_frame_count;
                    Vector4d obs = it_per_frame.lineobs;  
                    std::vector<int> drop_set;
               
                    if(imu_i == imu_j)
		    {
		      drop_set = vector<int>{0,2}; 
// 		      continue;
		    }
		          
		    else
		       drop_set = vector<int>{2};    
		    
                     ProjectionStructuralLine *f = new ProjectionStructuralLine(obs, it_per_id.Rsl, ric[0], tic[0]);     
          
	             ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                             vector<double *>{para_Pose[imu_j], para_Rws[0], para_structural_line[linefeature_index]},
                                                                             drop_set);
		         marginalization_info->addResidualBlockInfo(residual_block_info);
            
                
                }
            }

        }

     
        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
	    // 在getParameterBlocks函数里，是把先验部分要保留下来的那些参数变量的double指针存下来，
	    //其他帧的Rcs根本不包括在内，所以，实际上添加了这部分，也没用
	    
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

           addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
	   addr_shift[reinterpret_cast<long>(para_Rws[0])] = para_Rws[0];
	   
   
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
         //这个函数也是没有变化的
	//在这里，最终得到的是先验部分要保留的那些参数变量(其他各帧的位姿，外参，)的相关，
	//Rcs表示着那些被marge掉的特征线的方向，所以跟特征一样，marge掉以后，就相当于彻底删了，
	
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        //下一次优化的时候，上一次先验部分保留的参数变量在下一次的double指针，位置
    }
    else
    {//判断上一次的先验部分，是否有与次新帧相关的
      //在没有出现marge-old以前，其实是一直没有先验项的，，但是一旦出现一次marge-old以后，就有了先验项
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {//上一个先验项保留的参数变量里是否有次新帧，找的是double指针，或者说位置
	  //上一次为marge-old时，先验项里有次新帧相关

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                
		    //因为理论上，保留下来的只有先验相关的是位姿
		    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
		//表示WINDOW_SIZE - 1帧被marge掉了
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
	    addr_shift[reinterpret_cast<long>(para_Rws[0])] = para_Rws[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());

  
}


void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
               
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
   
    //暂时不设置优化的时间限制
    
//     if (marginalization_flag == MARGIN_OLD)
//         options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//     else
//         options.max_solver_time_in_seconds = SOLVER_TIME;
    
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    
    if(frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
//   check_Sframe_front();
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
	if(if_use_line)
	back_Rcs = Rcs[0];
	
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];    //依次往后顺延,后一个帧的往前移
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
		if(if_use_line || (WINDOW_SIZE - frame_count_ready_line <= i+1) )
		{
		  Rcs[i].swap(Rcs[i+1]);
		  Rws_state[i] = Rws_state[i+1];
		}
	
		
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1]; 
	    if(if_use_line || (WINDOW_SIZE - frame_count_ready_line <= WINDOW_SIZE - 1))
	    {
	      Rcs[WINDOW_SIZE] = Rcs[WINDOW_SIZE - 1]; 
	      Rws_state[WINDOW_SIZE] = Rws_state[WINDOW_SIZE - 1];
	    //最后一个用前一个作为初值
	    }


            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
		//删除最老的图像所对应的预积分，删除滑动窗口中最老的帧
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {  //记录将要被marge帧的这些状态量
	  
	   back_R0 = Rs[frame_count - 1];
           back_P0 = Ps[frame_count - 1];
	   if(if_use_line)
	   {
	    back_Rcs = Rcs[frame_count - 1];
	    Rcs[frame_count - 1] = Rcs[frame_count];
	    Rws_state[frame_count -1] = Rws_state[frame_count];
	   }
	
	   
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
	
         //最新的状态量后延一个，覆盖掉倒数第二新的，
            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }//被覆盖的倒数第二新帧的预积分传递到最新帧

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
    
    last_marginalization_flag = marginalization_flag;
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count, back_R0, back_P0, back_Rcs, Ps, tic, Rcs, Rws_state);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    //只要帧数满了，就为NON_LINEAR状态
    if (shift_depth)
    {
        Matrix3d R0, R1, Rcs_1;
        Vector3d P0, P1;
	int new_frame_count = 0;
	
	  R0 = back_R0 * ric[0];
	  P0 = back_P0 + back_R0 * tic[0];
	  R1 = Rs[0] * ric[0];   //此处的Rs【0】，是之前的Rs【1】
          P1 = Ps[0] + Rs[0] * tic[0];
	
	  f_manager.removeBackShiftDepth(R0, P0, R1, P1); 
	 
// 	  while(Rws_state[new_frame_count] == false)
// 	    new_frame_count++;
// 	  
//       if(new_frame_count != 0)
// 	{
// 	  R1 = Rs[new_frame_count] * ric[0];   
//           P1 = Ps[new_frame_count] + Rs[new_frame_count] * tic[0];
// 	  Rcs_1 = Rcs[new_frame_count];
//         }
	  
   if( !if_use_line && frame_count_ready_line >=0 )
   {
     //仅仅转移一些状态量
   for (auto it = f_manager.linefeature.begin(), it_next =f_manager.linefeature.begin();
         it != f_manager.linefeature.end(); it = it_next)
    {
        it_next++;
      if (it->linefeature_per_frame[0].In_frame_count != 0)   
        {
           for(auto &it_per_frame : it->linefeature_per_frame)
	   {
	     it_per_frame.In_frame_count--;
	   }
	    it->start_frame = it->linefeature_per_frame[0].In_frame_count;
	   
	   if(it->is_triangulation)
	     it->triangulate_frame_count--;
        }
       else
	 assert(0);
       
        
    }
   }
   else if(if_use_line)
   {
     Rcs_1 = Rcs[0];  //1帧的Rcs  
     f_manager.removeBackStartframe(R0, P0, R1, P1, back_Rcs, Rcs_1, new_frame_count);
   }

    //现在，三角化的线是基于世界系下的，所以只需要调整线观测相关的状态量即可
    
	//问题是1帧上不一定有该线的观测
	//处理线特征，线的观测往后移动，
	//主要工作是，把在0帧上三角化的线转移到1帧，主要涉及Rsl，线的二参数
	//0帧的线转到世界系，像三角化时一样，继续又转到相应的S系，L系
	
    }//因为点特征的3d坐标是基于起始观测帧的深度，所以，在滑窗的时候，要更新其深度
    //把基于第0帧相机系下的深度转换到第1帧
    else
    {  //正常情况不会进到这个地方
       f_manager.removeBack();
       assert(0);
    }
       
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mBuf.unlock();
}


void Estimator::check_structural_line_reproj()
{

  //检查结构线的重投影误差
    for (auto &it_per_id : f_manager.linefeature)
    {
       if(it_per_id.is_triangulation && it_per_id.linefeature_per_frame.size() >1 )
       {  
	  cout<<it_per_id.feature_id<<" rp_e ";
	  int base_frame = it_per_id.triangulate_frame_count;
	  int based_structural = it_per_id.based_structural;
	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, 0;
	 
	  Vector3d w_pts_1 = Rws[based_structural]*it_per_id.Rsl*ptl_1;
	  //得到世界系下的交点
	  
	  Vector3d vp_w = Rws[based_structural]*it_per_id.Rsl.col(2);
	  //直线的方向，即为L系的ｚ轴方向，转到世界系下，
	
	 for(auto &it_per_frame : it_per_id.linefeature_per_frame)
	  {
	    int imu_j = it_per_frame.In_frame_count;
// 	    if(base_frame == imu_j)  continue;
	    
	    Vector3d pt_imu_j = Rs[imu_j].transpose()*(w_pts_1-Ps[imu_j]);
	    Vector3d pt_cam_j = ric[0].transpose()*(pt_imu_j - tic[0]);
	    //由世界系下的交点，转到j相机系下
	    
	    if(abs(pt_cam_j(2)) <= 1e-5)  continue;
	    
	    pt_cam_j = pt_cam_j/pt_cam_j(2);
	    
	    Vector3d vp_cam_j = (Rs[imu_j]*ric[0]).transpose()*vp_w;  
	    //世界系下消失点的方向转到j相机下
	
	     
	    if(abs(vp_cam_j(2))<= 1e-5)  continue;
	    
	    vp_cam_j = vp_cam_j/vp_cam_j(2);
	    
	    Vector3d reproj_line = pt_cam_j.cross(vp_cam_j);
	    Vector3d line_obs_s, line_obs_e;
	    line_obs_s<<it_per_frame.lineobs(0), it_per_frame.lineobs(1), 1.0;
	    line_obs_e<<it_per_frame.lineobs(2), it_per_frame.lineobs(3), 1.0;
	    
	   
	     Vector2d reproj_err;
             reproj_err(0) = abs(reproj_line.dot(line_obs_s)/sqrt(reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1)));
	     reproj_err(1) = abs(reproj_line.dot(line_obs_e)/sqrt(reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1)));
	      
	     cout<<" o_f"<<imu_j<<" "<<setw(4)<< setprecision(3) << fixed << right << reproj_err.transpose();
	  }
	
      cout<<endl;
	  
	
      }
    }
    
    //检查世界系下唯一的Rws与每个帧观测的消失点的观测误差
    {
      
      for(int i = 0; i < frame_count; i++)
      {
	Matrix3d Rcs_reproj =  (Rs[i]*ric[0]).transpose()*Rws.back();
        cout <<"Rcs_reproj \n "<<Rcs_reproj<<endl;
	Rcs_reproj.col(0) = Rcs_reproj.col(0)/Rcs_reproj(2,0);
	Rcs_reproj.col(1) = Rcs_reproj.col(1)/Rcs_reproj(2,1);
	Rcs_reproj.col(2) = Rcs_reproj.col(2)/Rcs_reproj(2,2);
	
	Matrix3d Rcs_ob =  Rcs[i];
	  cout <<"Rcs_ob \n "<<Rcs_ob<<endl;
	Rcs_ob.col(0) = Rcs_ob.col(0)/Rcs_ob(2,0);
	Rcs_ob.col(1) = Rcs_ob.col(1)/Rcs_ob(2,1);
	Rcs_ob.col(2) = Rcs_ob.col(2)/Rcs_ob(2,2);
	
	
       Vector2d vp_err;
       cout<<"vp_reproj_err "<<i;
       for(int j= 0; j<3; j++)
       {
	  vp_err<< Rcs_reproj(0,j) - Rcs_ob(0,j), Rcs_reproj(1,j) - Rcs_ob(1,j);
	  cout<<" "<<setw(4)<< setprecision(3) << fixed << right << vp_err.norm();
       }
       
       cout<<endl;
	
	
      }
    }
    
   
    {/* cout<<"check Rws"<<endl;

      for(int i=0; i<=WINDOW_SIZE; i++)
      {  
	Vector3d euler_angles;
	
	 euler_angles = Rws[i].eulerAngles ( 2,1,0 ); 
	 euler_angles = (euler_angles*180)/acos(-1.0);
	 cout<<"frame_count "<<i<<" eulerAngles "<<euler_angles.transpose()<<endl;
      }*/
      
      
      
      //检查各帧间S系的朝向
//       for(int i=1; i<=frame_count-1; i++)
//       {
// 	Matrix3d Rs0si = Rws[0].transpose()*Rws[i];
// 	cout<<"Rs0s"<<i<<"\n"<<Rs0si<<endl;
// 
// 	if(abs(Rs0si(0,0)-1) < 0.05)
// 	{
// 	  if(abs(Rs0si(1,1)-1) < 0.05 && abs(Rs0si(2,2)-1)< 0.05)
// 	    Rs0si = Rs0si.setIdentity();
// 	  else 
// 	    assert(0);
// 	}
// 	
// 	else
// 	{
// 	  if(abs(Rs0si(1,0)+1)<0.05 && abs(Rs0si(0,1)-1)<0.05 && abs(Rs0si(2,2)-1)<0.05)
// 	    Rs0si<< 0, 1, 0,
// 	           -1, 0, 0,
// 		    0, 0, 1; 
// 	else 
// 	  assert(0);
//            
// 	}
// 	
// 	cout<<"Finanl Rs0s"<<i<<"\n"<<Rs0si<<endl;
//       }
    }
   
}

//用于观察Rws对重投影误差的影响
void Estimator::visual_Rws()
{
  //检查结构线的重投影误差
  
  for(int j = 0; j<3; j++)
  {
 for(int i =0; i <30; i++)
 {
   Matrix3d rot_diff;
   if(j==0)  rot_diff = Utility::ypr2R(Vector3d(i*0.3, 0, 0));
   else if(j==1)  rot_diff = Utility::ypr2R(Vector3d(0, i*0.3, 0));
   else  rot_diff = Utility::ypr2R(Vector3d(0, 0, i*0.3));
  
  for(int frame = 0; frame<=10; frame++ )
  {
  cv::Mat image_show(480, 752, CV_8UC3, cv::Scalar(0, 0, 0));
  for (auto &it_per_id : f_manager.linefeature)
    {
      if(it_per_id.solve_flag !=1)
	continue;
      
     for(auto &it_per_frame : it_per_id.linefeature_per_frame)
     {
       if(it_per_frame.In_frame_count != frame)
        continue;
	 
	  int base_frame = it_per_id.triangulate_frame_count;
	  int based_structural = it_per_id.based_structural;
	  Matrix3d Rws_ob = Rws[based_structural];
	  Rws_ob = Rws_ob*rot_diff;
	  
	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, 0;
	 
	   
	  Vector3d w_pts_1 = Rws_ob*it_per_id.Rsl*ptl_1;
	  //得到世界系下的交点
	  
	  Vector3d vp_w = Rws_ob*it_per_id.Rsl.col(2);
	  //直线的方向，即为L系的ｚ轴方向，转到世界系下，
	
	 
	  
	   
	    int imu_j = frame;
      
	    
	    Vector3d pt_imu_j = Rs[imu_j].transpose()*(w_pts_1-Ps[imu_j]);
	    Vector3d pt_cam_j = ric[0].transpose()*(pt_imu_j - tic[0]);
	    //由世界系下的交点，转到j相机系下
	    
	    if(abs(pt_cam_j(2)) <= 1e-5)  continue;
	    
	    pt_cam_j = pt_cam_j/pt_cam_j(2);
	    
	    Vector3d vp_cam_j = (Rs[imu_j]*ric[0]).transpose()*vp_w;  
	    //世界系下消失点的方向转到j相机下
	
	     
	    if(abs(vp_cam_j(2))<= 1e-5)  continue;
	    
	    vp_cam_j = vp_cam_j/vp_cam_j(2);
	    
	    cv::Point2d point_xy;
	    point_xy.x = pt_cam_j(0);
	    point_xy.y = pt_cam_j(1);
	    point_xy = To_picture(point_xy);
	    
	    cv::Point2d point_xy_r;
	    point_xy_r.x = vp_cam_j(0);
	    point_xy_r.y = vp_cam_j(1);
	    point_xy_r = To_picture(point_xy_r);
	    
	    
	    cv::Point2d point_ob_1;
	    point_ob_1.x = it_per_frame.lineobs(0);
	    point_ob_1.y = it_per_frame.lineobs(1);
	    point_ob_1 = To_picture(point_ob_1);

	    cv::Point2d point_ob_2;
	    point_ob_2.x = it_per_frame.lineobs(2);
	    point_ob_2.y = it_per_frame.lineobs(3);
	    point_ob_2 = To_picture(point_ob_2);
 
	    
	   cv::line(image_show, point_xy, point_xy_r,cv::Scalar(0,255,0), 1);
           cv::line(image_show, point_ob_1, point_ob_2,cv::Scalar(255,0,0), 2);
	     
	
     }
    }
    
    
    char frame_id[5];
   sprintf(frame_id, "%d", frame);
   cv::imshow(frame_id, image_show);
   
   if(frame == 10)
     waitKey(0);
   else
      waitKey(1);
	 
    }
	 
     }
      
       
   
  }
 
 
}


void Estimator::check_Sframe_between_camera()
{
  ROS_DEBUG("Rws_state:");
  for(int i=0; i<= WINDOW_SIZE; i++)
    cout<<" "<<Rws_state[i];
  cout<<endl;
  //这里的Rws表示的是上一个窗口内的状态
  
  
  //而这里的角度表示的是，当前窗口的状态，是即将要去被检测的
   Matrix3d Rws0 = Rs[0]*ric[0]*Rcs[0];
   for(int i=0; i<=frame_count; i++)
   {
     Matrix3d Rwsi = Rs[i]*ric[0]*Rcs[i];
     Matrix3d Rs0si = Rws0.transpose()*Rwsi;
     
     Vector3d euler_angles;
     euler_angles = Rs0si.eulerAngles ( 2,1,0 ); 
     euler_angles = (euler_angles*180)/acos(-1.0);
      cout<<"0 and "<<i<<" eulerAngles "<<euler_angles.transpose()<<endl;
   } 
   
  if(frame_count >=9)
  {
     Matrix3d Rws8 = Rs[8]*ric[0]*Rcs[8];
   for(int i=0; i<=frame_count; i++)
   {
     Matrix3d Rwsi = Rs[i]*ric[0]*Rcs[i];
     Matrix3d Rs8si = Rws8.transpose()*Rwsi;
     
     Vector3d euler_angles;
     euler_angles = Rs8si.eulerAngles ( 2,1,0 ); 
     euler_angles = (euler_angles*180)/acos(-1.0);
      cout<<"8 and "<<i<<" eulerAngles "<<euler_angles.transpose()<<endl;
   } 
 }

   
}


void Estimator::check_Sframe_front()
{
  ROS_WARN("start check_Sframe_front ");
  
//  if(last_err_Rcs_frame_count == 1 && last_marginalization_flag == MARGIN_OLD)
//  {
//     last_err_Rcs_frame_count == 0;
//     ROS_DEBUG("Now frame 0 is a err Rcs, skip check_Sframe_front ");
//     return ;
//  }
//  else if(last_err_Rcs_frame_count == 0 && last_marginalization_flag == MARGIN_SECOND_NEW)
//  {
//     ROS_DEBUG("Now frame 0 is a err Rcs, skip check_Sframe_front ");
//     return ;
//  }
//   
 
  //从后往前，找到一个正确的帧
    int a_right_frame = WINDOW_SIZE - 2;
     while(Rws_state[a_right_frame] == false)  
     {a_right_frame--;}
    Matrix3d Rwsr = Rs[a_right_frame]*ric[0]*Rcs[a_right_frame];
    
    ROS_DEBUG("a_right_frame : %d", a_right_frame);
    
//   if(Rws_state[0] == false)
  
  //0帧的Rws有问题,只是去判断一下9帧
    //8帧在之前未检测出有问题，所以选择相信8帧
    //从8帧往前去找一个正确的帧
    
   for(int i=0; i < WINDOW_SIZE-2; i++)
     {
 
       Matrix3d Rwsi = Rs[i]*ric[0]*Rcs[i];
       if(check_stuctural_direction(Rwsr, Rwsi,0) == false )
       {
	     for(auto &it_per_id : f_manager.linefeature)
             {
	        if(it_per_id.is_triangulation && it_per_id.triangulate_frame_count == i)
	        it_per_id.is_triangulation = false;
	              //清空在i帧上三角化的线
             }
             Rws_state[i] = false;
       }
       
       else
	  Rws_state[i] = true;
	 
     }

    Matrix3d Rws9 = Rs[WINDOW_SIZE-1]*ric[0]*Rcs[WINDOW_SIZE-1];
    if(check_stuctural_direction(Rwsr, Rws9, 1) == false)
   {//9帧的曼哈顿世界出现差异
     Matrix3d Rws10 = Rs[WINDOW_SIZE]*ric[0]*Rcs[WINDOW_SIZE];
       if(check_stuctural_direction(Rwsr,Rws10, 1) == false)
       {  ROS_WARN("frame 9 may be a new Structural wolrd");
	 waitKey(0);
//           assert(0);
       }
       
      else
      {
	ROS_WARN("only 9 frame ");
	for(auto &it_per_id : f_manager.linefeature)
           {
	        if(it_per_id.is_triangulation && it_per_id.triangulate_frame_count == WINDOW_SIZE -1 )
	        it_per_id.is_triangulation = false;
	              //清空在i帧上三角化的线
           }
             Rws_state[WINDOW_SIZE-1] = false;
        
      } 
      
   }
   else
     Rws_state[WINDOW_SIZE-1] = true;
      
  ROS_DEBUG("After check Rws_state:");
  for(int i=0; i<= WINDOW_SIZE; i++)
    cout<<" "<<Rws_state[i];
  cout<<endl;
 
  //只能说明0帧在上一个状态下没问题，但是当前这个状态下仍然可能出问题
  //越靠近窗口刚进的地方，越不容易出错,所以选择从后边去找正确的帧
  //
  
//  else
//  {
//    
//    Matrix3d Rws0 = Rs[0]*ric[0]*Rcs[0];
//    Matrix3d Rws9 = Rs[WINDOW_SIZE-1]*ric[0]*Rcs[WINDOW_SIZE-1];
//    if(check_stuctural_direction(Rws0, Rws9))
//   {
//     //0帧与9帧是没问题的
//    for(int i=1; i<WINDOW_SIZE-1; i++)
//    {
//      if(Rws_state[i] == false)
//        continue;
//      
//      Matrix3d Rwsi = Rs[i]*ric[0]*Rcs[i];
//      if(check_stuctural_direction(Rws9, Rwsi))
//        continue;
//      //实际发现，用9帧，更不容易出现错误检测
//   else
//     {//在0跟9都没问题的情况下，0与i的S系旋转有问题，即可判断i的S是有问题的
//       for(auto &it_per_id : f_manager.linefeature)
//       {
// 	if(it_per_id.is_triangulation && it_per_id.triangulate_frame_count == i)
// 	  it_per_id.is_triangulation = false;
// 	//清空在i帧上三角化的线
//       }
//      Rws_state[i] = false;
//      ROS_DEBUG("Rws in frame %d is error ",i);
//      cv::waitKey(0);
//     } 
//     
//   }
//  
//  }
//   else
//   {//0与9的曼哈顿世界不对齐
//      ROS_WARN("frame 9 may be a new Structural wolrd");
//      assert(0);
//   }
//    
//  } 
}


bool Estimator::check_stuctural_direction(Matrix3d& Rws1, Matrix3d& Rws2, bool is_check_windowsize)
{

    Matrix3d Rs1s2 = Rws1.transpose()*Rws2;
    Vector3d euler_angles;
    euler_angles = Rs1s2.eulerAngles ( 2,1,0 ); 
     euler_angles = (euler_angles*180)/acos(-1.0);
     euler_angles << abs(euler_angles(0)), abs(euler_angles(1)),abs(euler_angles(2));
    
     double angle_threshold;
     if(is_check_windowsize)
      angle_threshold = 5;
     else
       angle_threshold = 3.5;
       
    if((euler_angles(0)< angle_threshold || abs(euler_angles(0) - 90)<angle_threshold || abs(euler_angles(0)-180)<angle_threshold) &&
       (euler_angles(1)< angle_threshold || abs(euler_angles(1) - 90)<angle_threshold || abs(euler_angles(1)-180)<angle_threshold) &&
       (euler_angles(2)< angle_threshold || abs(euler_angles(2) - 90)<angle_threshold || abs(euler_angles(2)-180)<angle_threshold))  
    {
     return true;
    }
    
    else 
      return false;
}

void Estimator::get_start_frame_in_world(int frame_count)
{
  //得到Rws的初始值
  if( Rws.empty()  &&  Rws_state[frame_count])  
  {
     Matrix3d Rws_init = Rs[frame_count]*ric[0]*Rcs[frame_count];
     Rws.push_back(Rws_init);
     cout<<"get init Rws\n"<<Rws.back()<<endl;
     Which_Rws[0] = 0;
  }
  else 
  {
     //判断当前的帧的Rcs与Rws的朝向，是否是新的曼哈顿世界，
     //调整Rcs的朝向，与Rws保持一致
  
    if(!Rws_state[frame_count])
      {
	cout<<"This frame is not a structural world "<<endl;
	return;
      }
    
   Matrix3d Rws_ob = Rs[frame_count]*ric[0]*Rcs[frame_count];
   Matrix3d Rws_cur_ob = Rws.back().transpose() * Rws_ob;
   
   Vector3d euler_angles;
	
	 euler_angles = Rws_cur_ob.eulerAngles ( 2,1,0 ); 
	 euler_angles = (euler_angles*180)/acos(-1.0);
	
   euler_angles << fabs(euler_angles(0)), fabs(euler_angles(1)), fabs(euler_angles(2)); 
   
   cout<<"Rws_cur_ob"<<euler_angles.transpose()<<endl;
   
   double angle_threshold = 30;
   if(euler_angles(0) < angle_threshold || fabs(euler_angles(0) - 180) < angle_threshold)
   {
      Which_Rws[frame_count] = Rws.size() - 1;
      if(!Rws_buffer.empty())
	Rws_buffer.clear();
   }
      
     
   else if(fabs(euler_angles(0) - 90 ) < angle_threshold)
   {
     if(Rws_cur_ob(1,0) < 0)
     {
       Matrix3d Rss;
       Rss << 0, -1, 0,
              1,  0, 0,
	      0,  0, 1;
	    
      Rcs[frame_count] = Rcs[frame_count]*Rss;
      Which_Rws[frame_count] = Rws.size() - 1;
     }
     
     else 
     {
       Matrix3d Rss;
       Rss << 0,  1, 0,
             -1,  0, 0,
	      0,  0, 1;
	    
       Rcs[frame_count] = Rcs[frame_count]*Rss;
       Which_Rws[frame_count] = Rws.size() - 1;
     }
       
       
       if(!Rws_buffer.empty())
	Rws_buffer.clear();
   }
   
   else
   {
     if(Rws_buffer.size() < 3)
     { 
       cout<<"euler_angles "<<euler_angles.transpose()<<endl;
       Rws_buffer.push_back(Rws_ob);
       Rws_state[frame_count] = false;
     } 
    
    else if( check_stuctural_direction(Rws_buffer[0], Rws_buffer[1], 1) && 
              check_stuctural_direction(Rws_buffer[0], Rws_buffer[2], 1) )
     {
        ROS_WARN("May be a new structural world");
	assert(0);
     }
     
     else
     {
       cout<<"euler_angles "<<euler_angles.transpose()<<endl;
       Rws_buffer.clear();
       Rws_state[frame_count] = false;
       
     }
    
   
//      waitKey(0);
//      assert(0);
   }
     
   
//      Rws_ob = Rs[frame_count]*ric[0]*Rcs[frame_count];
//     Rws_cur_ob = Rws.back().transpose() * Rws_ob;
// 
// 	
// 	 euler_angles = Rws_cur_ob.eulerAngles ( 2,1,0 ); 
// 	 euler_angles = (euler_angles*180)/acos(-1.0);
// 	
//  
//    cout<<"relative S:"<<euler_angles.transpose()<<endl;
   
    
  }

  
 
  
  
}

void Estimator::get_triangulated_line()
{
  if(solver_flag == NON_LINEAR)
  {
     for(auto &it_per_id : f_manager.linefeature)
   {
       if (it_per_id.linefeature_per_frame.front().In_frame_count == 0 && it_per_id.linefeature_per_frame.size() <= 2
            && it_per_id.is_triangulation == true )
	ids_triangulated_line.insert(it_per_id.feature_id);
    }
  }
  
     
}




void Estimator::find_line_on_point( const pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image,
				   const vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> &line_feature_all)
{
   line_on_point.clear();
    point_set[0].clear();
    point_set[1].clear();
     point_set[2].clear();
      point_set[3].clear();
      all_lines.clear();
      
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> point_features = image.second;   
  
  for(auto &it_per_id : point_features)
  {
    int feature_id = it_per_id.first;
    Eigen::Matrix<double, 7, 1> point_observe = it_per_id.second[0].second;
    if(point_observe(0) >= 0 )
    {
      if(point_observe(1) >= 0)
	point_set[0].push_back(feature_id);
      else
	point_set[1].push_back(feature_id);
    }
    else
    {
        if(point_observe(1) >= 0)
	point_set[2].push_back(feature_id);
      else
	point_set[3].push_back(feature_id);
    }
  }
  

    for(unsigned int i=0; i<line_feature_all.size(); i++)
    {
       map<int, vector<pair<int, Vector4d>>> lines_observe;
       lines_observe = line_feature_all[i].second;
       
      for(auto &id_line : lines_observe)
      { 
	if(!(id_line.second.size() == 2 && id_line.second[1].first == 1) ) 
	  continue;
	  
        all_lines[id_line.first].emplace_back(id_line.second[0]);
	all_lines[id_line.first].emplace_back(id_line.second[1]);
	
	set<int> search_range;
	
	Vector4d line_observe =  id_line.second[0].second;
	Vector4d line_observe_right =  id_line.second[1].second;
	
	Vector2d mid_point ;
	mid_point << (line_observe(0)+line_observe(2))/2.0 , (line_observe(1)+line_observe(3))/2.0;
	
	if(mid_point(0) >= 0 && mid_point(1) >= 0 )
	  search_range.insert(0);
	else if(mid_point(0) >= 0 && mid_point(1) < 0 )
	  search_range.insert(1);
	else if(mid_point(0) < 0 && mid_point(1) >= 0 )
	  search_range.insert(2);
        else if(mid_point(0) < 0 && mid_point(1) < 0 )
	  search_range.insert(3);
    
       for(int j = 0; j<4; j+=2)	
      {
       	if(line_observe(0+j) >= 0 && line_observe(1+j) >= 0 )
	  search_range.insert(0);
	else if(line_observe(0+j) >= 0 && line_observe(1+j) < 0 )
	  search_range.insert(1);
	else if(line_observe(0+j) < 0 && line_observe(1+j) >= 0 )
	  search_range.insert(2);
        else if(line_observe(0+j) < 0 && line_observe(1+j) < 0 )
	  search_range.insert(3);
      }
    
     Vector3d end_p1, end_p2, end_p1_right, end_p2_right;
     end_p1 << line_observe(0), line_observe(1), 1;
     end_p2 << line_observe(2), line_observe(3), 1;
     end_p1_right << line_observe_right(0), line_observe_right(1), 1;
     end_p2_right << line_observe_right(2), line_observe_right(3), 1;
     
     Vector3d line_form = end_p1.cross(end_p2);
     Vector3d line_form_right = end_p1_right.cross(end_p2_right);
     
     for(auto &it_range : search_range)
     {
        for(auto &it_point : point_set[it_range])
	{
// 	    vector<pair<int, Eigen::Matrix<double, 7, 1>>> point_info = points[it_per_id];
	  vector<pair<int, Eigen::Matrix<double, 7, 1>>> point_info =point_features[it_point] ;
	  
	   Eigen::Matrix<double, 7, 1> point_left = point_info[0].second;
	  
	   Vector3d point_observe;
	   point_observe << point_left(0), point_left(1), 1;
	   double distance = fabs(line_form.dot(point_observe)/sqrt(line_form(0)*line_form(0)+line_form(1)*line_form(1)));
	if(distance <= 4.5 / 460.0)
	{ 
	  if(point_info.size() == 2 && point_info[1].first == 1)
	  {
	   Eigen::Matrix<double, 7, 1> point_right = point_info[1].second;
	   Vector3d point_observe_right;
	   point_observe_right << point_right(0), point_right(1), 1;
	   
	   distance = fabs(line_form_right.dot(point_observe_right)/sqrt(line_form_right(0)*line_form_right(0)+line_form_right(1)*line_form_right(1)));
	   
	   if(distance <= 4.4 / 460.0)
	   {
	     Vector2d  p_lp1, p_lp2;
	     p_lp1 = (end_p1 - point_observe).head(2);
	     p_lp2 = (end_p2 - point_observe).head(2);
	   
	     double angle_cos = p_lp1.dot(p_lp2)/(p_lp1.norm()*p_lp2.norm());
	   
	     if(angle_cos < 0)
	     {
	       line_on_point[id_line.first].emplace_back(make_pair(it_point, Vector2d(-1, 0)));
	       
	       if(f_manager.line_on_point_all_frame.count(id_line.first) == 0)
	       {
		 map<int, Vector2d> point_on_line;
		 point_on_line.insert(make_pair(it_point, Vector2d(-1, 1)));
		 f_manager.line_on_point_all_frame.insert(make_pair(id_line.first, point_on_line));
	       }
	       else
	       {
	         if(f_manager.line_on_point_all_frame[id_line.first].count(it_point) == 0) 
		   f_manager.line_on_point_all_frame[id_line.first].insert(make_pair(it_point, Vector2d(-1, 1)));
		 else
		 {
		   map<int, Vector2d> point_on_line =  f_manager.line_on_point_all_frame[id_line.first];
		   Vector2d point_num = point_on_line[it_point];
		   point_num = point_num + Vector2d(0, 1);
		   point_on_line[it_point] = point_num;
		   f_manager.line_on_point_all_frame[id_line.first] = point_on_line;
		 }
	       }
	       
	     }
	    else
	     {//意味着点在直线以外
	       double length = min(p_lp1.norm(), p_lp2.norm());
	       if(length < 120.0/460.0)
	       {
		 line_on_point[id_line.first].emplace_back(make_pair(it_point, Vector2d(1, length)));
		 
		if(f_manager.line_on_point_all_frame.count(id_line.first) == 0)
	       {
		 map<int, Vector2d> point_on_line;
		 point_on_line.insert(make_pair(it_point, Vector2d(1, 1)));
		 f_manager.line_on_point_all_frame.insert(make_pair(id_line.first, point_on_line));
	       }
	       else
	       {
	         if(f_manager.line_on_point_all_frame[id_line.first].count(it_point) == 0) 
		   f_manager.line_on_point_all_frame[id_line.first].insert(make_pair(it_point, Vector2d(1, 1)));
		 else
		 {
		   map<int, Vector2d> point_on_line =  f_manager.line_on_point_all_frame[id_line.first];
		   Vector2d point_num = point_on_line[it_point];
		   point_num = point_num + Vector2d(0, 1);
		   point_on_line[it_point] = point_num;
		   f_manager.line_on_point_all_frame[id_line.first] = point_on_line;
		 }
	       }
		 
	       }
		 
// 	       else
// 		 cout<<"length "<<length*460<<endl;
	     }
	   }
	     
	  }
	  
	
	}
	  
	}
     }
     
      }
      
      
      
    }
  
  
}

Vector2d Estimator::check_line_point_reproj_err(Vector3d pts_i, Vector4d obs, Matrix3d Rsl, int imu_i, int imu_j, double inv_depth)
{
  Vector2d residual;
    Vector3d w_p = Rs[imu_i]*ric[0]*(pts_i/inv_depth) + Rs[imu_i]*tic[0] + Ps[imu_i];
    cout<<" w_p "<<w_p.transpose();
    
    Matrix3d R;
    R<<1,0,0,
       0,1,0,
       0,0,0;
     Vector3d ptl_1 = R*Rsl.transpose()*Rws.back().transpose()*w_p;
    
       Vector3d w_pts_1 = Rws.back()*Rsl*ptl_1;
	  //得到世界系下的交点
	  
	  Vector3d vp_w = Rws.back()*Rsl.col(2);
	  //直线的方向，即为L系的ｚ轴方向，转到世界系下，
	
	   //将结构线的交点转到J系 
	    Vector3d pt_imu_j = Rs[imu_j].transpose()*(w_pts_1-Ps[imu_j]);
	    Vector3d pt_cam_j = ric[0].transpose()*(pt_imu_j - tic[0]);
	    
// 	    if(abs(pt_cam_j(2)) <= 1e-5)  continue;
	    
	    Vector3d  pt_cam_j_n = pt_cam_j/pt_cam_j(2);
	    	
	    Vector3d vp_cam_j = (Rs[imu_j]*ric[0]).transpose()*vp_w;  
	    //把直线所在的Ｓ系朝向转到当前的相机系下
	     Vector3d vp_cam_j_n = vp_cam_j/vp_cam_j(2);
	    
	    Vector3d reproj_line = pt_cam_j_n.cross(vp_cam_j_n);
	    Vector3d line_obs_s, line_obs_e;
	    line_obs_s<<obs(0), obs(1), 1.0;
	    line_obs_e<<obs(2), obs(3), 1.0;
	   
	  double l_norm = reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1);
	  double l_sqrtnorm = sqrt( l_norm );
	  
	  double e1 = reproj_line.dot(line_obs_s);
          double e2 = reproj_line.dot(line_obs_e);
            residual(0) = e1/l_sqrtnorm;
            residual(1) = e2/l_sqrtnorm;
	    
	    return residual;
  
}


Vector2d Estimator::check_line_point_distance_L(Vector3d pts_i, int imu_i, double inv_depth, double theta, double d_inv, Matrix3d Rsl)
{
  
    Vector2d residual;
    Vector3d w_p = Rs[imu_i]*ric[0]*(pts_i/inv_depth) + Rs[imu_i]*tic[0] + Ps[imu_i];
   
    Matrix3d R;
    R<<1,0,0,
       0,1,0,
       0,0,0;
     Vector3d ptl_1 = R*Rsl.transpose()*Rws.back().transpose()*w_p;
     
     
    double a_l = cos(theta)/d_inv;
    double b_l = sin(theta)/d_inv;
    
    Vector3d lp;
    lp<<a_l, b_l, 0;
	 
    residual = (lp - ptl_1).head(2);
    residual << fabs(residual(0)), fabs(residual(1));
    
     return residual;
}

