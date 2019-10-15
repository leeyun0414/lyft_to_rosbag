#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/conversions.h>

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <tf/transform_broadcaster.h>
#include <string>
#include <json/json.h>
#include <json/value.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
using namespace tf;

typedef struct sensorData
{
	int index;
    string filename;
    string ego_token;
    //string sample_token;
    double ego_trans[3]; //x,y,z
    double ego_rota[4];  //x,y,z,w

}sensorData;

void findPath(int nbr_sample,Json::Value value,string sample_token[],string sensor,sensorData (*data)){
    //int count = 0;
    for(int i=0;i<nbr_sample;i++){
        for(int obj_index=0;obj_index<value.size();obj_index++){
            if(value[obj_index]["sample_token"].asString()==sample_token[i]){
                //cout << "sample_token:" << sample_token[i] << endl;
                
                //cout << value[obj_index]["filename"].asString().npos << endl;
                if(value[obj_index]["filename"].asString().find(sensor) != value[obj_index]["filename"].asString().npos ){
                    //cout << "sensor:" <<value[obj_index]["filename"].asString() << endl;
                    //if(value[obj_index]["filename"].asString().find("samples")!=value[obj_index]["filename"].asString().npos){
                        data[i].filename=value[obj_index]["filename"].asString();
                        data[i].index = obj_index;
                        data[i].ego_token=value[obj_index]["calibrated_sensor_token"].asString();
                        //data[i].sample_token = value[obj_index]["sample_token"].asString();
                        //cout << "sensor:" <<value[obj_index]["filename"].asString() << endl;
                        //count = -1;
                        break;
                    //}
                }
                
            }
        }
    }
}
void findEgo(int nbr_sample,Json::Value value,sensorData (*data)){
    for(int i=0;i<nbr_sample;i++){
        for(int j=0;j<value.size();j++){
            if(value[j]["token"].asString()==data[i].ego_token){
                data[i].ego_trans[0]=value[j]["translation"][0].asFloat();
                data[i].ego_trans[1]=value[j]["translation"][1].asFloat();
                data[i].ego_trans[2]=value[j]["translation"][2].asFloat();
                data[i].ego_rota[0]=value[j]["rotation"][1].asFloat();
                data[i].ego_rota[1]=value[j]["rotation"][2].asFloat();
                data[i].ego_rota[2]=value[j]["rotation"][3].asFloat();
                data[i].ego_rota[3]=value[j]["rotation"][0].asFloat();
                //cout << data[i].ego_rota[0] <<endl;
                break;
            }
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"deng");
	ros::NodeHandle nh;
	ros::Publisher pub1,pub2,pub0;
    cv_bridge::CvImage cv_image1,cv_image2,cv_image3, cv_image4, cv_image5, cv_image6, cv_image7;
    ros::Publisher pub_cam1 = nh.advertise<sensor_msgs::Image>("/image_front", 1);
    ros::Publisher pub_cam2 = nh.advertise<sensor_msgs::Image>("/image_front_right", 1);
    ros::Publisher pub_cam3 = nh.advertise<sensor_msgs::Image>("/image_front_left", 1);
    ros::Publisher pub_cam4 = nh.advertise<sensor_msgs::Image>("/image_back_right", 1);
    ros::Publisher pub_cam5 = nh.advertise<sensor_msgs::Image>("/image_back_left", 1);
    ros::Publisher pub_cam6 = nh.advertise<sensor_msgs::Image>("/image_back", 1);
    ros::Publisher pub_cam7 = nh.advertise<sensor_msgs::Image>("/image_top", 1);
   
    TransformBroadcaster br;
    Transform transform1,transform2,transform0;
    ///////////////////////////////read json//////////////////////////////
    Json::Reader reader;
    Json::Value attribute,calibrated_sensor,category,ego_pose,
                instance,log,map,sample,sample_annotation,
                sample_data,scene,visibility;
    string dir_path = "/home/ee904-pc4/dataset/catkin_ws/src/lyft/v1.01-train/v1.01-train/";
    string data_path = "/home/ee904-pc4/dataset/catkin_ws/src/lyft/v1.01-train/";
    string lidar_path = "/home/ee904-pc4/dataset/catkin_ws/src/lyft/v1.01-train/pcd/";
    //ifstream in_attribute(dir_path+"attribute.json",ios::binary);
    ifstream in_calibrated_sensor(dir_path+"calibrated_sensor.json",ios::binary);
    //ifstream in_category(dir_path+"category.json",ios::binary);
    ifstream in_ego_pose(dir_path+"ego_pose.json",ios::binary);
    //ifstream in_instance(dir_path+"instance.json",ios::binary);
    ifstream in_log(dir_path+"log.json",ios::binary);
    //ifstream in_map(dir_path+"map.json");
    ifstream in_sample(dir_path+"sample.json",ios::binary);
    ifstream in_sample_annotation(dir_path+"sample_annotation.json",ios::binary);
    ifstream in_sample_data(dir_path+"sample_data.json",ios::binary);
    ifstream in_scene(dir_path+"scene.json",ios::binary);
    //ifstream in_sensor(dir_path+"sensor.json",ios::binary);
    //ifstream in_visibility(dir_path+"visibility.json",ios::binary);
    //cout << in_log.size() <<endl;
    //if(!in_log){
	//	cerr << "Could not read file: " <<dir_path<<"log.json" <<endl;
		//exit(EXIT_FAILURE);
	//}
    //in_log >> log;
    //in_scene >> scene;
    reader.parse(in_sample_data,sample_data);
    reader.parse(in_sample,sample);
    reader.parse(in_sample_annotation,sample_annotation);
    reader.parse(in_scene,scene);
    reader.parse(in_log,log);
    reader.parse(in_ego_pose,ego_pose);
    //reader.parse(in_category,category);
    //reader.parse(in_instance,instance);
    reader.parse(in_calibrated_sensor,calibrated_sensor);
    //cout << log <<  endl;
    string log_token;  
    for(int i=0;i<log.size();i++){
        if(log[i]["vehicle"].asString()=="a009"){
            log_token = log[i]["token"].asString();
            break;
        }
    }

    string first_sample_token;
    int nbr_sample;
    //cout << log_token << endl;
    for(int j=0;j<scene.size();j++){
        if(scene[j]["log_token"].asString()==log_token){
            //cout << "inin"<<endl;
            first_sample_token = scene[j]["first_sample_token"].asString();
            nbr_sample = scene[j]["nbr_samples"].asInt();   //get the sample number of the scene
            cout<<"nbr sample:"<<nbr_sample<<endl;
            break;
        }
    }
    //cout << first_sample_token << endl;
    //for(int j=0;j<scene.size();j++){
     //   if(scene[j]["token"].asString()=="1e9283d1ce98428aaa3b851487eb4f58"){
      //      first_sample_token = scene[j]["first_sample_token"].asString();
       //     nbr_sample = scene[j]["nbr_samples"].asInt();   //get the sample number of the scene
        //    cout<<"nbr sample:"<<nbr_sample<<endl;
         //   break;
       // }
   // }
    //cout << first_sample_token << "ss";
    string sample_token[nbr_sample];
    sample_token[0]=first_sample_token;
    int sample_index;
    //get the all sample_token of the scene
    cout << sample.size() ;
    for(sample_index=1;sample_index<sample.size();sample_index++){
        if(sample[sample_index]["token"].asString()==sample_token[0]){
            sample_token[1]=sample[sample_index]["next"].asString();
            break;
        }
    }
    for(int i=1;i<nbr_sample-1;i++){
        for (int j=1;j<sample.size();j++)
        {
            if (sample[j]["token"].asString()==sample_token[i])
                sample_token[i+1]=sample[j]["next"].asString();
        }
    }
    
     
    //get the sensor file path
    sensorData cam_front[nbr_sample];
    sensorData cam_front_left[nbr_sample];
    sensorData cam_front_right[nbr_sample];
    sensorData cam_back[nbr_sample];
    sensorData cam_back_left[nbr_sample];
    sensorData cam_back_right[nbr_sample];
    sensorData cam_top[nbr_sample];
    findPath(nbr_sample, sample_data, sample_token,"cam0",cam_front);
    findPath(nbr_sample, sample_data, sample_token,"cam1", cam_front_left);
    findPath(nbr_sample, sample_data, sample_token,"cam2", cam_front_right);
    findPath(nbr_sample, sample_data, sample_token,"cam3",cam_back);
    findPath(nbr_sample, sample_data, sample_token,"cam4",cam_back_left);
    findPath(nbr_sample, sample_data, sample_token,"cam5", cam_back_right);
    findPath(nbr_sample, sample_data, sample_token,"cam6", cam_top);
     //   cout << "cam:" << cam_front[0].filename << endl;
    sensorData lidar_front_right[nbr_sample];
    sensorData lidar_front_left[nbr_sample];
    sensorData lidar_top[nbr_sample];
    sensorData temp0[nbr_sample];
    sensorData temp1[nbr_sample];
    sensorData temp2[nbr_sample];

    findPath(nbr_sample, sample_data, sample_token,"lidar1",temp1);
    //cout << "lidar:" << temp1[0].filename << endl;
    for(int i=0;i<nbr_sample;i++){
        int pos = temp1[i].filename.find("lidar1");
        int end = temp1[i].filename.find(".bin");
        //cout << pos << "//" << end << "//" << i << endl;
        if (pos < 0 || end < 0)
            continue;
        lidar_top[i].filename = temp1[i].filename.substr(pos,end-pos);
        lidar_top[i].index = temp1[i].index;
        lidar_top[i].ego_token=temp1[i].ego_token;
    }
    cout << lidar_top[0].filename << endl;
    findPath(nbr_sample, sample_data, sample_token,"lidar0",temp0);
    for(int i=0;i<nbr_sample;i++){
        int pos = temp0[i].filename.find("lidar0");
        int end = temp0[i].filename.find(".bin");
        if (pos < 0 || end < 0)
            continue;
        lidar_front_left[i].filename = temp0[i].filename.substr(pos,end-pos);
        lidar_front_left[i].index = temp0[i].index;
        lidar_front_left[i].ego_token=temp0[i].ego_token;
    }
    findPath(nbr_sample, sample_data, sample_token,"lidar2",temp2);
    for(int i=0;i<nbr_sample;i++){
        int pos = temp2[i].filename.find("lidar2");
        int end = temp2[i].filename.find(".bin");
        if (pos < 0 || end < 0)
            continue;
        lidar_front_right[i].filename = temp2[i].filename.substr(pos,end-pos);
        lidar_front_right[i].index = temp2[i].index;
        lidar_front_right[i].ego_token=temp2[i].ego_token;
    }
    //findEgo(nbr_sample,calibrated_sensor,radar_front);
    findEgo(nbr_sample,calibrated_sensor,lidar_front_left);
    findEgo(nbr_sample,calibrated_sensor,lidar_front_right);
    //findEgo(nbr_sample,calibrated_sensor,radar_back_left);
    //findEgo(nbr_sample,calibrated_sensor,radar_back_right);
    findEgo(nbr_sample,calibrated_sensor,lidar_top);
    ///////////////////////////////read json end//////////////////////////

    //pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    //pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    //pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT>);
    //pcl::PointCloud<PointT>::Ptr cloud4(new pcl::PointCloud<PointT>);
    //pcl::PointCloud<PointT>::Ptr cloud5(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr points1(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2::Ptr cloud1(new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr points2(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2::Ptr cloud2(new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr points0(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2::Ptr cloud0(new sensor_msgs::PointCloud2);

    //pub1 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_radar1",1000);
    //pub2 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_radar2",1000);
    //pub3 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_radar3",1000);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_lidar_front_right",1000);
    pub0 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_lidar_front_left",1000);
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_lidar_top",1000);
    sensor_msgs::Image ros_image1,ros_image2,ros_image3, ros_image4, ros_image5, ros_image6, ros_image7;

    ros::Rate tate(10.0);
    for(int i=0;i<nbr_sample;i++){
        
        //string pcd_path1=data_path+radar_front[i].filename;
        //string pcd_path2=data_path+radar_front_left[i].filename;
        //string pcd_path3=data_path+radar_front_right[i].filename;
        //string pcd_path4=data_path+radar_back_left[i].filename;
        //string pcd_path5=data_path+radar_back_right[i].filename;
        string pcd_path0=lidar_path+"host-a009_"+lidar_front_left[i].filename+".pcd";
        string pcd_path1=lidar_path+"host-a009_"+lidar_top[i].filename+".pcd";
        string pcd_path2=lidar_path+"host-a009_"+lidar_front_right[i].filename+".pcd";
        //<< pcd_path1 << endl << pcd_path2 << endl;
        //pcl::io::loadPCDFile<PointT>(pcd_path1,*cloud1);
        //pcl::io::loadPCDFile<PointT>(pcd_path2,*cloud2);
        //pcl::io::loadPCDFile<PointT>(pcd_path3,*cloud3);
        //pcl::io::loadPCDFile<PointT>(pcd_path4,*cloud4);
        //pcl::io::loadPCDFile<PointT>(pcd_path5,*cloud5);
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path0,*points0);
        pcl::toROSMsg(*points0, *cloud0);
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path1,*points1);
        //if (points1)
            //cout << pcd_path1 << endl ;
        pcl::toROSMsg(*points1, *cloud1);
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path2,*points2);
        pcl::toROSMsg(*points2, *cloud2);
        
        
        cv_image1.image = cv::imread(data_path+cam_front[i].filename,CV_LOAD_IMAGE_COLOR);
        //cout << data_path+cam_front[i].filename <<endl;
        //if (cv_image1)
          //  cout <<"gggggggggggggggggggggggg"<<endl;
        cv_image1.encoding = "bgr8";
        cv_image1.toImageMsg(ros_image1);
        //ros_image1.header.frame_id = "/map";
        pub_cam1.publish(ros_image1);
         cv_image2.image = cv::imread(data_path+cam_front_right[i].filename,CV_LOAD_IMAGE_COLOR);
        cv_image2.encoding = "bgr8";
        cv_image2.toImageMsg(ros_image2);
        pub_cam2.publish(ros_image2);
        cv_image3.image = cv::imread(data_path+cam_front_left[i].filename,CV_LOAD_IMAGE_COLOR);
        cv_image3.encoding = "bgr8";
        cv_image3.toImageMsg(ros_image3);
        pub_cam3.publish(ros_image3);
        cv_image4.image = cv::imread(data_path+cam_back[i].filename,CV_LOAD_IMAGE_COLOR);
        cv_image4.encoding = "bgr8";
        cv_image4.toImageMsg(ros_image4);
        pub_cam4.publish(ros_image4);
        cv_image5.image = cv::imread(data_path+cam_back_right[i].filename,CV_LOAD_IMAGE_COLOR);
        cv_image5.encoding = "bgr8";
        cv_image5.toImageMsg(ros_image5);
        pub_cam5.publish(ros_image5);
        cv_image6.image = cv::imread(data_path+cam_back_left[i].filename,CV_LOAD_IMAGE_COLOR);
        cv_image6.encoding = "bgr8";
        cv_image6.toImageMsg(ros_image6);
        pub_cam6.publish(ros_image6);
        cv_image7.image = cv::imread(data_path+cam_top[i].filename,CV_LOAD_IMAGE_COLOR);
        cv_image7.encoding = "bgr8";
        cv_image7.toImageMsg(ros_image7);
        pub_cam7.publish(ros_image7);

        cloud0->header.frame_id = "/nusceneslidar_left";
        cloud1->header.frame_id = "/nusceneslidar_top";
        cloud2->header.frame_id = "/nusceneslidar_right";
//cout << Quaternion(lidar_top[i].ego_rota[0],lidar_top[i].ego_rota[1],lidar_top[i].ego_rota[2],lidar_top[i].ego_rota[3]).normalize ()<<endl;
         transform1.setOrigin(Vector3(lidar_top[i].ego_trans[0],lidar_top[i].ego_trans[1],lidar_top[i].ego_trans[2]));
        transform1.setRotation(Quaternion(lidar_top[i].ego_rota[0],lidar_top[i].ego_rota[1],lidar_top[i].ego_rota[2],lidar_top[i].ego_rota[3]));
        br.sendTransform(StampedTransform(transform1,ros::Time::now(),"map","nusceneslidar_top"));
        //cout<<"1111"<<endl;
        pub1.publish(*cloud1);
        transform0.setOrigin(Vector3(lidar_front_left[i].ego_trans[0],lidar_front_left[i].ego_trans[1],lidar_front_left[i].ego_trans[2]));
        transform0.setRotation(Quaternion(lidar_front_left[i].ego_rota[0],lidar_front_left[i].ego_rota[1],lidar_front_left[i].ego_rota[2],lidar_front_left[i].ego_rota[3]));
        br.sendTransform(StampedTransform(transform0,ros::Time::now(),"map","nusceneslidar_left"));
        //cout<<"0000"<<endl;
        pub0.publish(*cloud0);
        transform2.setOrigin(Vector3(lidar_front_right[i].ego_trans[0],lidar_front_right[i].ego_trans[1],lidar_front_right[i].ego_trans[2]));
        transform2.setRotation(Quaternion(lidar_front_right[i].ego_rota[0],lidar_front_right[i].ego_rota[1],lidar_front_right[i].ego_rota[2],lidar_front_right[i].ego_rota[3]));
        br.sendTransform(StampedTransform(transform2,ros::Time::now(),"map","nusceneslidar_right"));
        //cout<<"2222"<<endl;
        pub2.publish(*cloud2); 
        //usleep(1000000); //0.8s=800000
    }

    //while(nh.ok()){
        
     //cout <<"aaaaaa";
     //sleep(10000);
     //   br.sendTransform(StampedTransform(transform1,ros::Time::now(),"map","nusceneslidar"));
     //   pub1.publish(*cloud1);
        /* br.sendTransform(StampedTransform(transform0,ros::Time::now(),"map","nusceneslidar_left"));
        pub0.publish(*cloud0);
        br.sendTransform(StampedTransform(transform2,ros::Time::now(),"map","nusceneslidar_right"));
        pub2.publish(*cloud2);*/

        /*  ros_image1->header.frame_id = "nusceceslidar";
        ros_image2->header.frame_id = "nusceceslidar";
        ros_image3->header.frame_id = "nusceceslidar";
        ros_image4->header.frame_id = "nusceceslidar";
        ros_image5->header.frame_id = "nusceceslidar";
        ros_image6->header.frame_id = "nusceceslidar";
        ros_image7->header.frame_id = "nusceceslidar";*/
      //  pub_cam1.publish(ros_image1);
        /* pub_cam2.publish(ros_image2);
        pub_cam3.publish(ros_image3);
        pub_cam4.publish(ros_image4);
        pub_cam5.publish(ros_image5);
        pub_cam6.publish(ros_image6);    
        pub_cam7.publish(ros_image7);*/

    //}


}
