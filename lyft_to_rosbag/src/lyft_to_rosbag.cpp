#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include <iostream>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <tf/transform_broadcaster.h>
#include <string>
#include <json/json.h>
#include <json/value.h>
#include <vector>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace pcl;
using namespace std;
using namespace tf;

typedef struct sensorData
{
	int index;
    string filename;
    string ego_token;
    string time;
    double ego_trans[3]; //x,y,z
    double ego_rota[4];  //x,y,z,w
    Transform ttr;
}sensorData;
typedef struct labelData
{
    string sample_token; 
    string instance_token;
    string category_token;
    string type;
    float centor[3]; //x,y,z
    float rotate[4];  //x,y,z,w
    float size[3];
    //Transform ttr;
}labelData;
struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(double x, double y, double z, double w)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}
void findPath(int nbr_sample,Json::Value value,string sample_token[],string sensor,sensorData (*data)){
    //int count = 0;
    for(int i = 0; i < nbr_sample; i++){
        for(int obj_index = 0; obj_index < value.size(); obj_index++){
            if(value[obj_index]["sample_token"].asString() == sample_token[i]){
                //cout << "sample_token:" << sample_token[i] << endl;
                
                //cout << value[obj_index]["filename"].asString().npos << endl;
                if(value[obj_index]["filename"].asString().find(sensor) != value[obj_index]["filename"].asString().npos ){
                    //cout << "sensor:" <<value[obj_index]["filename"].asString() << endl;
                    //if(value[obj_index]["filename"].asString().find("samples")!=value[obj_index]["filename"].asString().npos){
                        data[i].filename = value[obj_index]["filename"].asString();
                        data[i].index = obj_index;
                        data[i].ego_token = value[obj_index]["calibrated_sensor_token"].asString();
                        if (sensor == "lidar1")
                            data[i].time = value[obj_index]["ego_pose_token"].asString();
                        //cout << "sensor:" <<value[obj_index]["filename"].asString() << endl;
                        //count = -1;
                        break;
                    //}
                }
                
            }
        }
    }
}
void getcolor(string name, int &a, int &b, int &c)
{
    if (name =="bicycle" || name == "motorcycle"){
        a = 0;
        b = 0;
        c = 255;
    }
    else if (name == "truck" || name == "bus" || name == "car" || name == "emergency_vehicle"){
        a = 255;
        b = 0;
        c = 0;
    }
    else if (name == "pedestrian"){
        a = 0;
        b = 255;
        c = 0;
    }
    else if (name == "animal") {
        a = 0;
        b = 0;
        c = 0;
    }
    else {
        a = 255;
        b = 255;
        c = 255;
    }
}
void findEgo(int nbr_sample,Json::Value value,sensorData (*data)){
    for(int i = 0; i < nbr_sample; i++){
        for(int j = 0; j < value.size(); j++){
            if(value[j]["token"].asString()==data[i].ego_token){
                data[i].ego_trans[0] = value[j]["translation"][0].asFloat();
                data[i].ego_trans[1] = value[j]["translation"][1].asFloat();
                data[i].ego_trans[2] = value[j]["translation"][2].asFloat();
                data[i].ego_rota[0] = value[j]["rotation"][1].asFloat();
                data[i].ego_rota[1] = value[j]["rotation"][2].asFloat();
                data[i].ego_rota[2] = value[j]["rotation"][3].asFloat();
                data[i].ego_rota[3] = value[j]["rotation"][0].asFloat();
                //cout << data[i].ego_rota[0] <<endl;
                break;
            }
        }
    }
}
cv::Mat compute_3d_box_cam2(float x, float y, float z, float l, float w, float h, float yaw)
{
    float Rdata[] = { cos(yaw) , -sin(yaw), sin(yaw), cos(yaw)};
    float eightpointdata[] = {l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2};
    float zpointdata[] = {-h / 2, -h / 2, -h / 2, -h / 2, h / 2, h / 2, h / 2, h / 2};
    float Tdata[] = {x, x, x, x, x, x, x, x, y, y, y, y, y, y, y, y, z, z, z, z, z, z, z, z};
    cv::Mat R = cv::Mat(2, 2, CV_32F, Rdata).clone();
    cv::Mat eightpoint = cv::Mat(2, 8, CV_32F, eightpointdata).clone();
    cv::Mat zpoint = cv::Mat(1, 8, CV_32F, zpointdata).clone();
    cv::Mat T = cv::Mat(3, 8, CV_32F, Tdata).clone();

    cv::Mat A;
    A = R * eightpoint;
    A.push_back(zpoint);
    A = A + T;

    return A;
}
int main(int argc, char **argv){
	
///////////////////////////////read json//////////////////////////////
    cout << "reading the json files...\n";
    Json::Reader reader;
    Json::Value attribute,calibrated_sensor,category,ego_pose,
                instance,log,map,sample,sample_annotation,
                sample_data,scene,visibility;
    string dir_path = "/home/ee904-pc4/dataset/catkin_ws/src/lyft/v1.01-train/v1.01-train/";
    string data_path = "/home/ee904-pc4/dataset/catkin_ws/src/lyft/v1.01-train/";
    string lidar_path = "/home/ee904-pc4/dataset/catkin_ws/src/lyft/v1.01-train/pcd/";
    //ifstream in_attribute(dir_path+"attribute.json",ios::binary);
    ifstream in_calibrated_sensor(dir_path + "calibrated_sensor.json",ios::binary);
    ifstream in_category(dir_path + "category.json",ios::binary);
    ifstream in_ego_pose(dir_path + "ego_pose.json",ios::binary);
    ifstream in_instance(dir_path + "instance.json",ios::binary);
    ifstream in_log(dir_path + "log.json",ios::binary);
    //ifstream in_map(dir_path+"map.json");
    ifstream in_sample(dir_path + "sample.json",ios::binary);
    ifstream in_sample_annotation(dir_path + "sample_annotation.json",ios::binary);
    ifstream in_sample_data(dir_path + "sample_data.json",ios::binary);
    ifstream in_scene(dir_path + "scene.json",ios::binary);
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
    reader.parse(in_category,category);
    reader.parse(in_instance,instance);
    reader.parse(in_calibrated_sensor,calibrated_sensor);
    //cout << log <<  endl;
    string log_token;  
    int count, count_reach;
    string car_name = "a011";//choose car name
    //cout << log.size() <<endl;
    int count_max = 51;//choose scene from count_reach to count_max 

    for (int count_reach = 10; count_reach <= count_max; count_reach++){

        ros::init(argc,argv,"deng");
        ros::NodeHandle nh;
        count = 0;
        cout << "getting the information of scene...\n";
        cout << "car_name:" << car_name << endl;
        for(int i = 0; i < log.size(); i++){//find the specific scene.
            //cout << log[i]["vehicle"].asString() <<endl;
            if(log[i]["vehicle"].asString() == car_name){
                count ++;
                if (count == count_reach){
                    log_token = log[i]["token"].asString();
                    cout << "scene:" << count << endl;
                    break;
                }
            }
        }
    cout << "getting the information of frame...\n";
        string first_sample_token;
        int nbr_sample;
        //cout << log_token << endl;
        for(int j = 0; j < scene.size(); j++){//get the sample number of the scene, get the first sample token
            if(scene[j]["log_token"].asString() == log_token){
                //cout << "inin"<<endl;
                first_sample_token = scene[j]["first_sample_token"].asString();
                nbr_sample = scene[j]["nbr_samples"].asInt();
                cout << "nbr_sample:" << nbr_sample << endl;
                break;
            }
        }
                
        string sample_time[nbr_sample];
        string sample_token[nbr_sample];
        sample_token[0] = first_sample_token;
        int sample_index;
        //get the all sample_token of the scene
        //cout << sample.size() ;
        cout << "getting the information of timestamp...\n";
        for(sample_index = 1; sample_index < sample.size(); sample_index++){//get the every sample token, get the timestamp of every sample.
            if(sample[sample_index]["token"].asString() == sample_token[0]){
                sample_token[1] = sample[sample_index]["next"].asString();
                sample_time[0] = sample[sample_index]["timestamp"].asString();
                break;
            }
        }
        for(int i = 1; i < nbr_sample - 1; i++){
            for (int j = 0; j < sample.size(); j++)
            {
                if (sample[j]["token"].asString() == sample_token[i])
                {
                    sample_token[i + 1] = sample[j]["next"].asString();
                    sample_time[i] = sample[j]["timestamp"].asString();
                }
            }
        }
        for (int i = 0; i < sample.size(); i++)
            if (sample[i]["token"].asString() == sample_token[nbr_sample - 1])
                sample_time[nbr_sample - 1] = sample[i]["timestamp"].asString();
        //for (int i; i < nbr_sample; i++)
        //    cout << sample_token[i] << "   " << sample_time[i] << endl;
        vector<labelData> label_data[nbr_sample];
        labelData temp_lab;
        sensorData time[nbr_sample];

        

        float px, py, pz;
        int LINES[] = {0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,4,0,5,1,6,2,7,3,4,1,5,0};
        cout << "getting the path of sensor_data...\n";
        //get the sensor file path
        sensorData cam_front[nbr_sample];
        sensorData cam_front_left[nbr_sample];
        sensorData cam_front_right[nbr_sample];
        sensorData cam_back[nbr_sample];
        sensorData cam_back_left[nbr_sample];
        sensorData cam_back_right[nbr_sample];
        sensorData cam_top[nbr_sample];
        findPath(nbr_sample, sample_data, sample_token,"cam0",cam_front);//0
        findPath(nbr_sample, sample_data, sample_token,"cam2", cam_front_left);//4
        findPath(nbr_sample, sample_data, sample_token,"cam1", cam_front_right);//1
        findPath(nbr_sample, sample_data, sample_token,"cam5",cam_back);//5
        findPath(nbr_sample, sample_data, sample_token,"cam4",cam_back_left);//3
        findPath(nbr_sample, sample_data, sample_token,"cam3", cam_back_right);//2
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
            lidar_top[i].ego_token = temp1[i].ego_token;
            lidar_top[i].time = temp1[i].time;
        }
        //cout << lidar_top[0].filename << endl;
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
        cout << "getting the ego_pose of every timestamp...\n";
        for (int i = 0; i < nbr_sample; i++){//get the ego_pose every timestamp
            for (int j = 0; j < ego_pose.size(); j++)
            {
                if (sample_time[i] == ego_pose[j]["timestamp"].asString() && lidar_top[i].time == ego_pose[j]["token"].asString()){
                    time[i].ego_trans[0] = ego_pose[j]["translation"][0].asFloat() ;
                    time[i].ego_trans[1] = ego_pose[j]["translation"][1].asFloat() ;
                    time[i].ego_trans[2] = ego_pose[j]["translation"][2].asFloat() ;
                    time[i].ego_rota[0] = ego_pose[j]["rotation"][1].asFloat();
                    time[i].ego_rota[0] = ego_pose[j]["rotation"][2].asFloat();
                    time[i].ego_rota[2] = ego_pose[j]["rotation"][3].asFloat();
                    time[i].ego_rota[3] = ego_pose[j]["rotation"][0].asFloat(); 
                    time[i].ttr.setOrigin(Vector3(ego_pose[j]["translation"][0].asFloat(), ego_pose[j]["translation"][1].asFloat(), ego_pose[j]["translation"][2].asFloat()));
                    time[i].ttr.setRotation(Quaternion(ego_pose[j]["rotation"][1].asFloat(),  ego_pose[j]["rotation"][2].asFloat(),  ego_pose[j]["rotation"][3].asFloat(),  ego_pose[j]["rotation"][0].asFloat()));
                    //cout << temp_lab.centor[0] << endl << temp_lab.rotate[0] << endl << temp_lab.size[0] << endl << endl;
                    break;
                }
            }
        }
        cout << "getting the annotations_data of frame...\n";
        for (int i = 0; i < nbr_sample; i++){//get the annotation_data every sample
            for (int j = 0; j < sample_annotation.size(); j++)
            {
                if (sample_token[i] == sample_annotation[j]["sample_token"].asString()){
                    //temp_lab.sample_token = sample_token[i];
                    temp_lab.instance_token = sample_annotation[j]["instance_token"].asString();
                    temp_lab.centor[0] = sample_annotation[j]["translation"][0].asFloat();
                    temp_lab.centor[1] = sample_annotation[j]["translation"][1].asFloat();
                    temp_lab.centor[2] = sample_annotation[j]["translation"][2].asFloat();
                    temp_lab.rotate[0] = sample_annotation[j]["rotation"][1].asFloat() ;
                    temp_lab.rotate[1] = sample_annotation[j]["rotation"][2].asFloat() ;
                    temp_lab.rotate[2] = sample_annotation[j]["rotation"][3].asFloat() ;
                    temp_lab.rotate[3] = sample_annotation[j]["rotation"][0].asFloat() ;
                
                    temp_lab.size[0] = sample_annotation[j]["size"][0].asFloat();
                    temp_lab.size[1] = sample_annotation[j]["size"][1].asFloat();
                    temp_lab.size[2] = sample_annotation[j]["size"][2].asFloat();

                    label_data[i].push_back(temp_lab);
                        //cout << temp_lab.centor[0] << endl << temp_lab.centor[1] << endl << temp_lab.centor[2] << endl << endl;
                }
            }
        }
        cout << "getting the category of annotations_data...\n";
        for(int i = 0; i < nbr_sample; i++){  //getting the category of annotations_data 
            for (vector<labelData>::iterator j = label_data[i].begin(); j != label_data[i].end(); ++j){
                for (int k = 0; k < instance.size(); k++){
                    if (j->instance_token == instance[k]["token"].asString()){
                        j->category_token = instance[k]["category_token"].asString();
                        for (int l = 0; l < category.size(); l++){
                            if (j->category_token == category[l]["token"].asString())
                                j->type = category[l]["name"].asString();
                        }
                    }
                }
            }
        }
        //declare publish node
        pcl::PointCloud<pcl::PointXYZI>::Ptr points1(new pcl::PointCloud<pcl::PointXYZI>);
        sensor_msgs::PointCloud2::Ptr cloud1(new sensor_msgs::PointCloud2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points2(new pcl::PointCloud<pcl::PointXYZI>);
        sensor_msgs::PointCloud2::Ptr cloud2(new sensor_msgs::PointCloud2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points0(new pcl::PointCloud<pcl::PointXYZI>);
        sensor_msgs::PointCloud2::Ptr cloud0(new sensor_msgs::PointCloud2);

        //pub1 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_radar1",1000);
        //pub2 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_radar2",1000);
        //pub3 = nh.advertise<sensor_msgs::PointCloud2>("/nuscenes_radar3",1000);
        ros::Publisher pub1,pub2,pub0;
        pub2 = nh.advertise<sensor_msgs::PointCloud2>("/lidar_front_right",1000);
        pub0 = nh.advertise<sensor_msgs::PointCloud2>("/lidar_front_left",1000);
        pub1 = nh.advertise<sensor_msgs::PointCloud2>("/lidar_top",1000);
        ros::Publisher pub_cam1 = nh.advertise<sensor_msgs::Image>("/image_front", 1);
        ros::Publisher pub_cam2 = nh.advertise<sensor_msgs::Image>("/image_front_right", 1);
        ros::Publisher pub_cam3 = nh.advertise<sensor_msgs::Image>("/image_front_left", 1);
        ros::Publisher pub_cam4 = nh.advertise<sensor_msgs::Image>("/image_back_right", 1);
        ros::Publisher pub_cam5 = nh.advertise<sensor_msgs::Image>("/image_back_left", 1);
        ros::Publisher pub_cam6 = nh.advertise<sensor_msgs::Image>("/image_back", 1);
        ros::Publisher pub_cam7 = nh.advertise<sensor_msgs::Image>("/image_top", 1);
        ros::Publisher ann_pub = nh.advertise<visualization_msgs::MarkerArray>("/markerarray", 1);
        sensor_msgs::Image ros_image1,ros_image2,ros_image3, ros_image4, ros_image5, ros_image6, ros_image7;
        TransformBroadcaster br;
        Transform transform1,transform2,transform0;
        EulerAngles lab_angle, time_angle;
        int red, green, blue;
        cv_bridge::CvImage cv_image1,cv_image2,cv_image3, cv_image4, cv_image5, cv_image6, cv_image7;
        ros::Rate tate(10.0);
        //for (int i = 0; i < nbr_sample; i++)
        //  cout << sample_token[i] <<endl;
        cout << "drawing the marker_array of annotations_data...\n";
        for(int i = 0; i < nbr_sample; i++){    
            visualization_msgs::MarkerArray marker_array;   
            int banana = 0; 
            //time_angle = ToEulerAngles(time[i].ego_rota[0], time[i].ego_rota[1], time[i].ego_rota[2], time[i].ego_rota[3]);
            for (vector<labelData>::iterator j = label_data[i].begin(); j != label_data[i].end(); ++j)
            {
                //lab_angle = ToEulerAngles(j->rotate[0], j->rotate[1], j->rotate[2], j->rotate[3]);
                //cv::Mat C = compute_3d_box_cam2(j->centor[0], j->centor[1], j->centor[2], j->size[1], j->size[0], j->size[2], lab_angle.yaw);
                //cout << C << endl;
                //cout << "banana:" << banana << endl;
                //string type_list[] = {"dsfnvj","car","big vehicles","pedestrian","animal","other_vehicle","bus", "motorcycle", "truck", "emergency_vehicle", "bicycle"};
                banana++;
                visualization_msgs::Marker marker;
                marker.header.frame_id = "ann";
                marker.header.stamp = ros::Time();
                marker.ns = j->type;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration(0.36);//0.63//0.36
                marker.pose.position.x = j->centor[0];
                marker.pose.position.y = j->centor[1];
                marker.pose.position.z = j->centor[2];
                marker.pose.orientation.x = j->rotate[0];
                marker.pose.orientation.y = j->rotate[1];
                marker.pose.orientation.z = j->rotate[2];
                marker.pose.orientation.w = j->rotate[3];
                marker.scale.x = j -> size[1];                                              
                marker.scale.y = j -> size[0];
                //marker.lifetime = ros::Duration();
                marker.scale.z = j -> size[2];
                marker.id = banana;
                marker.color.a = 0.5; // Don't forget to set the alpha!
                //cout << j->type << endl;
                getcolor(j->type, red, green, blue);
                marker.color.r = red / 255;
                marker.color.g = green / 255;
                marker.color.b = blue / 255;
                marker_array.markers.push_back(marker);
                //cout << endl << endl;
            }
        //reading file
            string pcd_path0 = lidar_path + "host-" + car_name + "_" + lidar_front_left[i].filename + ".pcd";
            string pcd_path1 = lidar_path + "host-" + car_name + "_" + lidar_top[i].filename + ".pcd";
            string pcd_path2 = lidar_path + "host-" + car_name + "_" + lidar_front_right[i].filename + ".pcd";
            pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path0,*points0);
            pcl::toROSMsg(*points0, *cloud0);
            pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path1,*points1);

            pcl::toROSMsg(*points1, *cloud1);
            pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path2,*points2);
            pcl::toROSMsg(*points2, *cloud2);
        
            
            cv_image1.image = cv::imread(data_path+cam_front[i].filename,CV_LOAD_IMAGE_COLOR);
            //if (!cv_image1)
            //cout <<"/home/ee904-pc4/dataset/ann_a101/"+ sample_token[i]+"FRONT.jpeg"<<endl;
            //cout << data_path+cam_front[i].filename <<endl;
            //if (cv_image1)
            //  cout <<"gggggggggggggggggggggggg"<<endl;
            //start publishing
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
            //sending tf of every sensor
    //cout << Quaternion(lidar_top[i].ego_rota[0],lidar_top[i].ego_rota[1],lidar_top[i].ego_rota[2],lidar_top[i].ego_rota[3]).normalize ()<<endl;
            transform1.setOrigin(Vector3(lidar_top[i].ego_trans[0], lidar_top[i].ego_trans[1], lidar_top[i].ego_trans[2]));
            transform1.setRotation(Quaternion(lidar_top[i].ego_rota[0],lidar_top[i].ego_rota[1],lidar_top[i].ego_rota[2],lidar_top[i].ego_rota[3]));
            br.sendTransform(StampedTransform(transform1, ros::Time::now(),"map", "nusceneslidar_top"));
            pub1.publish(*cloud1);
            //cout<<"1111"<<endl;
        
            transform0.setOrigin(Vector3(lidar_front_left[i].ego_trans[0],lidar_front_left[i].ego_trans[1],lidar_front_left[i].ego_trans[2]));
            transform0.setRotation(Quaternion(lidar_front_left[i].ego_rota[0],lidar_front_left[i].ego_rota[1],lidar_front_left[i].ego_rota[2],lidar_front_left[i].ego_rota[3]));
            br.sendTransform(StampedTransform(transform0, ros::Time::now(), "map", "nusceneslidar_left"));
            //cout<<"0000"<<endl;
            pub0.publish(*cloud0);
            transform2.setOrigin(Vector3(lidar_front_right[i].ego_trans[0],lidar_front_right[i].ego_trans[1],lidar_front_right[i].ego_trans[2]));
            transform2.setRotation(Quaternion(lidar_front_right[i].ego_rota[0],lidar_front_right[i].ego_rota[1],lidar_front_right[i].ego_rota[2],lidar_front_right[i].ego_rota[3]));
            br.sendTransform(StampedTransform(transform2, ros::Time::now(), "map", "nusceneslidar_right"));
            //cout<<"2222"<<endl;
            pub2.publish(*cloud2);
        
            br.sendTransform(StampedTransform(time[i].ttr.inverse(), ros::Time::now(), "map", "ann"));
            ann_pub.publish( marker_array );   
        }
    }
}
