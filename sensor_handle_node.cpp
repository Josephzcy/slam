
#include<iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#define foreach BOOST_FOREACH
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include<sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring; // 0~15 (16线激光雷达)
    float tm; // 第一个point的time是0 (相对时间)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, tm, tm)
)

typedef pcl::PointXYZINormal PointType;



double timeInterval=40.f;
double startLidarTime=0.0f;
double lastLidarStamp=0.0;

rosbag::Bag bagSensorWriter;
const std::string lidar_topic_out="/points_raw";
// double sensorStartTime=1455208290.12045;
// double timeShift=1455208536.312518120-2404.687749000;
// double timeShift=1455208381.797745705-3231.115227000;
double sensorStartTime=1640077824.821;
double timeShift=1640077916.498316765-716.668352000;
double cntLidarIndex=0;
double lastLidarTime=0;
double  timeJump=0;
void callback(const sensor_msgs::PointCloud2ConstPtr&  laserCloudMsg){
    if(laserCloudMsg!=nullptr){
        cntLidarIndex++;
        // if(cntLidarIndex>=40)
        //     ros::shutdown();
        std::cout<<" === current Index ===:"<<cntLidarIndex<<std::endl;
        sensor_msgs::PointCloud2 currentCloudMsg=*laserCloudMsg;
        double cntLidarStamp= currentCloudMsg.header.stamp.toSec();
        if((cntLidarStamp-lastLidarTime)<0){   
            timeJump=lastLidarTime;
            std::cout<<" === time junp ===:"<<timeJump<<std::endl;

        }
        lastLidarTime=cntLidarStamp;

        currentCloudMsg.header.stamp=ros::Time().fromSec(timeJump+cntLidarStamp);
        
        std::cout<<" === lidar current time ===:"<<cntLidarStamp+timeJump<<std::endl;
        bagSensorWriter.write(lidar_topic_out,currentCloudMsg.header.stamp,currentCloudMsg);

    }
}


int main(int argc, char** argv){

    // double timeShift=1455208444.581460476-408.882445000;
    // bagSensorWriter.open("/home/joseph/Mybag/MySensor_12_23.bag", rosbag::bagmode::Write);

    //TODO:read imu
   
    // rosbag::Bag bagImuWriter;
    // bagImuWriter.open("/home/joseph/Mybag/11_23/MyImu.bag", rosbag::bagmode::Write);

    // std::string strMyImu="/home/joseph/Mybag/11_23/MyBackPack.bag";
    // rosbag::Bag bagImu;
    // bagImu.open(strMyImu, rosbag::bagmode::Read);
    // rosbag::View view(bagImu);
    // std::uint32_t inuIndex=0;


    // foreach(rosbag::MessageInstance const m, view){
    //     // std::cout << "Found message of type "  << m.getDataType() << std::endl;
    //     // std::string curTopic = m.getTopic();
    //     std::string imu_topic = "/imu_raw";
    //     sensor_msgs::ImuConstPtr ptrIMU = m.instantiate<sensor_msgs::Imu>();
    //     if(ptrIMU!=nullptr){
    //         sensor_msgs::Imu imu_data= (*ptrIMU); 
    //         double cntImuTime=imu_data.orientation_covariance[1]+imu_data.orientation_covariance[2]-timeShift;

    //         inuIndex++;
    //         // if(inuIndex>=1200)
    //         //     break;  
            
            
    //         imu_data.header.stamp=ros::Time().fromSec(cntImuTime);
    //         imu_data.header.seq=inuIndex-1;
    //         imu_data.header.frame_id="imu_link";

            
    //         // imu_data_new.linear_acceleration.x = -imu_data.linear_acceleration.y; 
    //         // imu_data_new.linear_acceleration.y = -imu_data.linear_acceleration.x;
    //         // imu_data_new.linear_acceleration.z = -imu_data.linear_acceleration.z;
            
    //         // imu_data_new.angular_velocity.x = -imu_data.angular_velocity.y ;
    //         // imu_data_new.angular_velocity.y = -imu_data.angular_velocity.x ;
    //         // imu_data_new.angular_velocity.z = -imu_data.angular_velocity.z;

    //         // Eigen::Quaterniond quaternion(imu_data.orientation.w,imu_data.orientation.x,
    //         // imu_data.orientation.y,imu_data.orientation.z);

    //         // Eigen::Matrix3d imuRPY;
    //         // imuRPY<<-1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,1.0;
    //         // Eigen::Quaterniond quaternionNew1;
    //         // quaternionNew1=imuRPY*quaternion;
    //         // imu_data_new.orientation.x = quaternionNew1.x() ;
    //         // imu_data_new.orientation.y = quaternionNew1.y() ;
    //         // imu_data_new.orientation.z = quaternionNew1.z();
    //         // imu_data_new.orientation.w = quaternionNew1.w();

    //         bagImuWriter.write(imu_topic,imu_data.header.stamp,imu_data);
    //     }
    // }
    
    
    // //TODO:read camera
    // std::string videoPath ="/home/joseph/libbackpack/SensorData/11_23/MyCameraVideo.mp4";
	// cv::VideoCapture cap(videoPath);
	// if(!cap.isOpened()){
	// 	std::cout<<"open video failed!"<<std::endl;
	// 	return -1;
	// }
	// else
    //     std::cout<<"open video success!"<<std::endl;

    // cv::Mat image; //this is an image
    // std::uint32_t imageIndex=0;
    // std::string image_topic = "/camera/image_raw/compressed";
    // double imageStartTime =sensorStartTime-timeShift;
   
        
    
    // bool finishgFlag = true;
	// while(cap.read(image)){
    //     if(!finishgFlag){                      
    //         std::cout<<"video ends"<<std::endl;
    //         break;
	//     }

    //     std::uint32_t frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    //     double fps = cap.get(cv::CAP_PROP_FPS);
    //     cv::Size size = cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    //     // std::cout <<"camera frames:"<< frames << std::endl;
    //     // std::cout << "camera fps:"<<fps << std::endl;
    //     // std::cout << "size:"<<size <<std::endl;
    //     // cv::Mat row=cv::Mat::ones(1,3);
    //     imageIndex++;
    //     std::cout<<"imageIndex:"<<imageIndex<<std::endl;

    //     double timeUse=imageIndex*1/fps;

    //     // if(imageIndex>=120)
    //     //     break;
        
    //     cv::Mat targetImage;
    //     cv::Mat rotationImage; //+90
    //     resize(image,targetImage, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
        
    //     cv::Point2f center(targetImage.rows/2,targetImage.rows/2);
    //     cv::Size rotationImageSize(targetImage.rows,targetImage.cols);
    //     double angle =90;
    //     cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, 1.0);
    //     cv::warpAffine(targetImage, rotationImage, rotMat, rotationImageSize, cv::INTER_LINEAR);
        
    //     std_msgs::Header imageHeader;
    //     imageHeader.stamp=ros:: Time().fromSec(imageStartTime+timeUse);
    //     imageHeader.seq=imageIndex-1;
    //     imageHeader.frame_id="camera";

    //     sensor_msgs::CompressedImagePtr compressed_image_msg = cv_bridge::CvImage(imageHeader, "bgr8", rotationImage).toCompressedImageMsg();
    //     sensor_msgs::CompressedImage compressedImage=(*compressed_image_msg);
    //     std::cout<<"current camera timeStamp:"<<compressedImage.header.stamp<<std::endl;

    //     bagSensorWriter.write(image_topic,compressedImage.header.stamp,compressedImage);
    //     // cv::imshow("image",targetImage);

    //     // std::string strImageIndex;
    //     // std::stringstream ss;
    //     // ss<<imageIndex;
    //     // ss>>strImageIndex;
    //     // std::string imagePath="/home/joseph/Mybag/camera/"+strImageIndex+".jpg";
    //     // cv::imwrite(imagePath,rotationImage);
    //     // cv::imwrite("/home/joseph/Mybag/camera/image4.jpg",rotationImage);    
        
    // }
    //TODO::Lidar handle
    // ros::init(argc, argv, "point_cloud_publisher");
    // ros::NodeHandle nh;
    // std::cout<<"====deal with  lidar data==="<<std::endl;
    // // std::string lidar_topic="/ns1/velodyne_points";
    // std::string lidar_topic="/velodyne_points";

    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, callback);
    // ros::spin();


    //TODO:make bag including lidar、imu data,read original data bag separately

    rosbag::Bag bagSensorWriter;
    // bagSensorWriter.open("/home/joseph/Mybag/11_23/r2live_lio.bag", rosbag::bagmode::Write);

    std::string strMyImu="/media/joseph/软件/MyDataSet/backpack/12_21/have_done/MyImu.bag";
    rosbag::Bag bagImu;
    bagImu.open(strMyImu, rosbag::bagmode::Read);
    rosbag::View view(bagImu);

    std::uint16_t imuIndex=-1;
    foreach(rosbag::MessageInstance const m, view){
        std::string imu_topic = "/livox/imu/";
        sensor_msgs::ImuConstPtr ptrIMU = m.instantiate<sensor_msgs::Imu>();
        if(ptrIMU!=nullptr){
            sensor_msgs::Imu imu_data= (*ptrIMU); 
            double cntImuTime=imu_data.orientation_covariance[1]+imu_data.orientation_covariance[2]-timeShift;

            imuIndex++;
            // if(imuIndex>=1200)
            //     break;  
            // std::cout<<"imuIndex:"<<imuIndex<<std::endl;
            std::cout<<"cntImuTime:"<<cntImuTime<<std::endl;
            imu_data.header.stamp=ros::Time().fromSec(cntImuTime);
            imu_data.header.seq=imuIndex;
            // bagSensorWriter.write(imu_topic,imu_data.header.stamp,imu_data);
        }

    }
    //TODO:Lidar_data_handle
    std::string strMyLidar="/media/joseph/软件/MyDataSet/backpack/12_21/have_done/MyLidar.bag";
    rosbag::Bag bagLidar;
    bagLidar.open(strMyLidar, rosbag::bagmode::Read);
    rosbag::View viewLidar(bagLidar);

    std::uint16_t lidarIndex=-1;
    foreach(rosbag::MessageInstance const m, viewLidar){
        std::string lidar_topic = "/velodyne_points";

        sensor_msgs::PointCloud2ConstPtr ptrCloud = m.instantiate<sensor_msgs::PointCloud2>();

        if(ptrCloud!=nullptr){

            
            sensor_msgs::PointCloud2 currentCloudMsg=*ptrCloud;
            pcl::PointCloud<PointXYZIRT>::Ptr pointCloud (new pcl::PointCloud<PointXYZIRT>());
            
            pcl::fromROSMsg(*ptrCloud, *pointCloud);

            pcl::PointCloud<PointType>::Ptr pointCloudOut (new pcl::PointCloud<PointType>());
            pointCloudOut->points.resize(pointCloud->size());

            //TODO：把点云变成PointXYZINormal,lidar 坐标变换
        
            for(auto pointIndex=0;pointIndex<pointCloud->size();pointIndex++ ){
                float temx=pointCloud->points[pointIndex].x;
                float temy=pointCloud->points[pointIndex].y;
                pointCloud->points[pointIndex].x=temy;
                pointCloud->points[pointIndex].y=-temx;

                pointCloudOut->points[pointIndex].x=pointCloud->points[pointIndex].x;
                pointCloudOut->points[pointIndex].y=pointCloud->points[pointIndex].y;
                pointCloudOut->points[pointIndex].z=pointCloud->points[pointIndex].z;
                pointCloudOut->points[pointIndex].intensity=pointCloud->points[pointIndex].intensity;
                pointCloudOut->points[pointIndex].curvature=0;

            }
            
            sensor_msgs::PointCloud2 tempCloud;
            pcl::toROSMsg(*pointCloudOut, tempCloud);


            double currentTime = currentCloudMsg.header.stamp.toSec()-0.1;  //start timeStamp
            
            tempCloud.header.stamp=ros:: Time().fromSec(currentTime);
            tempCloud.header.frame_id = currentCloudMsg.header.frame_id;
            
           
            lidarIndex++;
            std::cout<<"lidarIndex:"<<lidarIndex<<std::endl;
            // if(lidarIndex>=30)
            //     break;  
            
            // bagSensorWriter.write(lidar_topic,tempCloud.header.stamp,tempCloud);

        }
    }
    //TODO:image_handle

   
    // std::string videoPath="/media/joseph/软件/MyDataSet/backpack/12_21/MyVideo.mp4";
	// cv::VideoCapture cap(videoPath);
	// if(!cap.isOpened()){
	// 	std::cout<<"open video failed!"<<std::endl;
	// 	return -1;
	// }
	// else
    //     std::cout<<"open video success!"<<std::endl;

    // cv::Mat image; //this is an image
    // std::uint32_t imageIndex=0;
    // std::string image_topic = "/camera/image_color";
    // double imageStartTime =sensorStartTime-timeShift;
   
    // bool finishgFlag = true;
	// while(cap.read(image)){
    //     if(!finishgFlag){                      
    //         std::cout<<"video ends"<<std::endl;
    //         break;
	//     }

    //     std::uint32_t frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    //     double fps = cap.get(cv::CAP_PROP_FPS);
    //     cv::Size size = cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    //     imageIndex++;
    //     std::cout<<"imageIndex:"<<imageIndex<<std::endl;

    //     double timeUse=imageIndex*1/fps;

    //     // if(imageIndex>=120)
    //     //     break;
        
    //     cv::Mat targetImage;
    //     cv::Mat rotationImage; //+90
    //     resize(image,targetImage, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
        
    //     cv::Point2f center(targetImage.rows/2,targetImage.rows/2);
    //     cv::Size rotationImageSize(targetImage.rows,targetImage.cols);
    //     double angle =90;
    //     cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, 1.0);
    //     cv::warpAffine(targetImage, rotationImage, rotMat, rotationImageSize, cv::INTER_LINEAR);
        
    //     std_msgs::Header imageHeader;
    //     imageHeader.stamp=ros:: Time().fromSec(imageStartTime+timeUse);
    //     imageHeader.seq=imageIndex-1;
    //     imageHeader.frame_id="camera";

    //     sensor_msgs::CompressedImagePtr compressed_image_msg = cv_bridge::CvImage(imageHeader, "bgr8", rotationImage).toCompressedImageMsg();
    //     sensor_msgs::CompressedImage compressedImage=(*compressed_image_msg);
    //     std::cout<<"current camera timeStamp:"<<compressedImage.header.stamp<<std::endl;

    //     bagSensorWriter.write(image_topic,compressedImage.header.stamp,compressedImage);
    // }
    bagSensorWriter.close();

    return 0; 
}