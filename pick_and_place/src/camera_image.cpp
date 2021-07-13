#include <camera_node.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

camera_node::camera_node()
{
    pnh_ = new ros::NodeHandle("~");
    parameter_set();
    operate();
}

void camera_node::parameter_set()
{
    pnh_->getParam("camera_topic_name", camera_topic_name_);
    pnh_->getParam("image_topic_name", image_topic_name_);
    pnh_->getParam("output_topic_name", output_topic_name_);
}

void camera_node::operate()
{
    operate_pub_ =nh_.advertise<sensor_msgs::Image>(output_topic_name_,10);
    image_point_pub_ = nh.advertise<geometry_msgs::Point>("/image_point",10);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_type>(Sync_type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&camera_node::callback, this, _1, _2));
}

void camera_node::callback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
{
    ROS_INFO_STREAM(cam_msgs->header.frame_id);
    ROS_INFO_STREAM("width: " << cam_msgs->width << "  height: " << cam_msgs->height);
    for (std::size_t i = 0; i < cam_msgs->K.size() - 3; i+=3)
    {
        ROS_INFO_STREAM(cam_msgs->K[i] << " " << cam_msgs->K[i+1] << " " << cam_msgs->K[i+2]);
    }

    cv_bridge::CvImagePtr input_bridge, cv_3;
    input_bridge = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::BGR8);
    cv_3 = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::MONO8);

    cv::Mat img, out, gaussian, red, mask;
    input_bridge->image;
    // cv::cvtColor(input_bridge->image, img, cv::COLOR_BGR2GRAY);
    camera_node::detect_red(input_bridge->image, mask);
    // cv::threshold(red, cv_3->image, 130, 255, cv::THRESH_BINARY);
    camera_node::centroid_image(mask);
    camera_node::pixel_to_world(mc);
    // sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(image_msgs->header, "bgr8", out).toImageMsg();
    // input_bridge->image = out;
    // input_bridge->encoding = "bgr8";
    operate_pub_.publish(cv_3->toImageMsg());
    image_point_pub_.publish(image_point);
    cv::imshow("windo", mask);
    cv::waitKey(1);
}

void camera_node::detect_red(cv::Mat img, cv::Mat &mask)
{
    cv::Mat mask_1;
    cv::Mat mask_2;
    cv::Mat hsv;

    //change img from bgr to hsv
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    //define hsv limit for mask_1
    std::vector<int> hsv_min;
    hsv_min = {0, 127, 0};
    std::vector<int> hsv_max;
    hsv_max = {30, 255, 255};
    cv::inRange(hsv, hsv_min, hsv_max, mask_1);

    //define hsv lint for mask_2
    hsv_min = {150, 127, 0};
    hsv_max = {179, 255, 255};
    cv::inRange(hsv, hsv_min, hsv_max, mask_2);

    mask = mask_1 + mask_2;
}

void camera_node::centroid_image(cv::Mat mask)
{
    // std::vector<std::vector<cv::Point> > contours;
    // cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // double max_size=0;
    // int max_id = -1;

    // for(int i=0; i < contours.size();i++){
    //     if(contours[i].size() > max_size){
    //         max_size = contours[i].size();
    //         max_id=1;
    //     }
    // }

    // mu = cv::moments(contours[max_id]);
    // mc = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
    // cv::circle(mask, mc, 4, cv::Scalar(100), 2, 4);
    // ROS_INFO_STREAM("pixcel.x:" << mc.x << "  pixcel.y" << mc.y);

    mu = cv::moments(mask, true);
    mc = cv::Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
    cv::circle(mask, mc, 4, cv::Scalar(100), 2, 4);
    ROS_INFO_STREAM("pixcel.x:" << mc.x << "  pixcel.y" << mc.y);
}

void camera_node::pixel_to_world(cv::Point2f pixcel)
{
    double w = 640;
    double h = 360;
    double horizontal_fov = 1.3962634;
    double z = 1.95;
    double zz = 1.53;
    std::vector<std::vector<double> > a;
    a = {{pixcel.x - w/2},
         {pixcel.y - h/2}};
    std::vector<std::vector<double> > matrix_1;
    matrix_1 = {{2*zz*tan(horizontal_fov/2)/w, 0},
                {0, 2*zz*tan(horizontal_fov/2)/w}};
    std::vector<std::vector<double> > screen_to_camera;
    screen_to_camera = {{0.0},
                        {0.0}};
    multiple_matrix(matrix_1, a, screen_to_camera);
    std::vector<std::vector<double> > R_x;
    R_x = {{1.0, 0.0, 0.0},
           {0.0, cos(M_PI), -sin(M_PI)},
           {0.0, sin(M_PI), cos(M_PI)}};
    std::vector<std::vector<double> > R_y;
    R_y = {{cos(M_PI), 0.0, sin(M_PI)},
           {0.0, 1.0, 0.0},
           {-sin(M_PI), 0.0, cos(M_PI)}};
    std::vector<std::vector<double> > R_z;
    R_z = {{cos(M_PI), 0.0, sin(M_PI)},
           {0.0, 1.0, 0.0},
           {-sin(M_PI), 0.0, cos(M_PI)}};
    std::vector<std::vector<double> > R;
    R = {{cos(M_PI), 0.0, sin(M_PI)},
           {0.0, 1.0, 0.0},
           {-sin(M_PI), 0.0, cos(M_PI)}};
    std::vector<std::vector<double> > camera_pixcel;
    camera_pixcel = {{screen_to_camera[0][0]},
                     {screen_to_camera[1][0]},
                     {zz}};
    std::vector<std::vector<double> > camera_move;
    camera_move = {{0.3},
                   {0.0},
                   {z}};
    std::vector<std::vector<double> > b;
    b = {{0.0},
         {0.0},
         {0.0}};
    std::vector<std::vector<double> > camera_to_world;
    camera_to_world = {{0.0},
         {0.0},
         {0.0}};
    multiple_matrix(R_x, camera_pixcel, b);
    sum_matrix(b, camera_move, camera_to_world);
    // sum_matrix(camera_pixcel, camera_move, camera_to_world);
    ROS_INFO_STREAM("world.x:" << camera_to_world[0][0] << "world.y:" << camera_to_world[1][0] << "world.z:" << camera_to_world[2][0]);
    float i_x = camera_to_world[0][0];
    float i_y = camera_to_world[1][0];
    float i_z = camera_to_world[2][0] + 0.3;

    image_point.x = i_x;
    image_point.y = i_y;
    image_point.z = i_z;
}

void camera_node::multiple_matrix(std::vector<std::vector<double> > Matrix_1, std::vector<std::vector<double> > Matrix_2, std::vector<std::vector<double> > &ans)
{
    for (int i = 0; i < Matrix_1.size(); i++){
        for (int j = 0; j < Matrix_2[i].size(); j++){
            for(int k = 0; k < Matrix_2.size(); k++){
                ans[i][j] += Matrix_1[i][k] * Matrix_2[k][j];
            }
        }
    }
}

void camera_node::sum_matrix(std::vector<std::vector<double> > Matrix_1, std::vector<std::vector<double> > Matrix_2, std::vector<std::vector<double> > &ans)
{
    for(int i = 0; i < Matrix_1.size(); i++){
        for (int j = 0; j < Matrix_1[i].size();j++){
            ans[i][j] = Matrix_1[i][j] + Matrix_2[i][j];
        }
    }
}