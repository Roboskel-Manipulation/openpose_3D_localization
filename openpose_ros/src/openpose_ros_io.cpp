#include <openpose_ros_io.hpp>

using namespace openpose_ros;

OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): nh_("openpose_ros_node"), it_(nh_)
{
    #ifdef PROFILING
    ros::Time begin_config = ros::Time::now();
    #endif

    // Subscribe to input video feed and publish human lists as output
    std::string image_topic, output_topic, depth_topic;

    nh_.param("image_topic", image_topic, std::string("/zed/left/image_raw_color"));
    nh_.param("output_topic", output_topic, std::string("/openpose_ros/human_list"));
    nh_.param("depth_topic", depth_topic, std::string("/zed/depth/depth_registered"));
    nh_.param("display_output", display_output_flag_, true);
    nh_.param("print_keypoints", print_keypoints_flag_, false);
    nh_.param("save_original_video", save_original_video_flag_, false);
    nh_.param("save_openpose_video", save_openpose_video_flag_, false);
    nh_.param("original_video_file_name", original_video_file_name_, std::string(""));
    nh_.param("openpose_video_file_name", openpose_video_file_name_, std::string(""));
    nh_.param("video_fps", video_fps_, 10);
    nh_.param("depth_queue_size", depth_queue_size_, 1);
    nh_.param("human_list_queue_size", human_list_queue_size_, 1);

    #ifdef PROFILING
    ros::Time end_config = ros::Time::now();
    ROS_INFO("config: %f secs", (end_config-begin_config).toSec());

    ros::Time begin_init = ros::Time::now();
    #endif

    depth_sub_ = nh_.subscribe(depth_topic, depth_queue_size_, &OpenPoseROSIO::storeDepth, this);
    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this);
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, human_list_queue_size_);
    cv_img_ptr_ = nullptr;
    depths_ptr_ = nullptr;
    img_width_ = 0;
    openpose_ = &openPose;

    #ifdef PROFILING
    ros::Time end_init = ros::Time::now();
    ROS_INFO("initialization: %f secs", (end_init-begin_init).toSec());

    ros::Time begin_save = ros::Time::now();
    #endif

    if(save_original_video_flag_)
    {
        if(original_video_file_name_.empty())
        {
            std::cout << "No original video filename was provided. Not saving original video." << std::endl; 
            save_original_video_flag_ = false;
        }
        else
        {
            original_video_writer_initialized_ = false;
        }
    }
    if(save_openpose_video_flag_)
    {
        if(openpose_video_file_name_.empty())
        {
            std::cout << "No openpose video filename was provided. Not saving openpose video." << std::endl; 
            save_openpose_video_flag_ = false;
        }
        else
        {
            openpose_video_writer_initialized_ = false;
        }
    }

    #ifdef PROFILING
    ros::Time end_save = ros::Time::now();
    ROS_INFO("saving: %f secs", (end_save-begin_save).toSec());
    #endif
}

void OpenPoseROSIO::storeDepth(const sensor_msgs::Image::ConstPtr& msg)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    // Use a pointer to the depth values, also cast the data pointer to floating point
    size_t size = msg->width * msg->height;
    if (!depths_ptr_)
    {
        #ifdef PROFILING
        ros::Time begin_malloc = ros::Time::now();
        #endif

        depths_ptr_ = (float*) malloc(size * sizeof(float));

        #ifdef PROFILING
        ros::Time end_malloc = ros::Time::now();
        ROS_INFO("--> storeDepth --> malloc: %f secs", (end_malloc-begin_malloc).toSec());
        #endif
    }

    #ifdef PROFILING    
    ros::Time begin_store = ros::Time::now();
    #endif

    for (size_t i = 0; i < size; i++)
        depths_ptr_[i] = *((float*)(&msg->data[0])+i);

    #ifdef PROFILING
    ros::Time end_store = ros::Time::now();
    ROS_INFO("--> storeDepth --> store: %f secs", (end_store-begin_store).toSec());
    #endif

    img_width_ = msg->width;

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> storeDepth: %f secs", (end-begin).toSec());
    #endif
}

void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    convertImage(msg);
    std::shared_ptr<std::vector<op::Datum>> datumToProcess = createDatum();

    #ifdef PROFILING
    ros::Time begin_emplaceDatum = ros::Time::now();
    #endif

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);

    #ifdef PROFILING
    ros::Time end_emplaceDatum = ros::Time::now();
    ROS_INFO("--> processImage --> waitAndEmplace datum: %f secs", (end_emplaceDatum-begin_emplaceDatum).toSec());

    ros::Time begin_popFrame = ros::Time::now();
    #endif

    // Pop frame
    std::shared_ptr<std::vector<op::Datum>> datumProcessed;
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {
        if(display_output_flag_)
        {
            display(datumProcessed);
        }
        if(print_keypoints_flag_)
        {
            printKeypoints(datumProcessed);
        }
        if(save_original_video_flag_)
        {
            saveOriginalVideo(datumToProcess);
        }
        if(save_openpose_video_flag_)
        {
            saveOpenPoseVideo(datumProcessed);
        }
        publish(datumProcessed);
    }
    else
    {
        op::log("Processed datum could not be emplaced.", op::Priority::High,
                __LINE__, __FUNCTION__, __FILE__);
    }

    #ifdef PROFILING
    ros::Time end_popFrame = ros::Time::now();
    ROS_INFO("--> processImage --> waitAndPop datum: %f secs", (end_popFrame-begin_popFrame).toSec());

    ros::Time end = ros::Time::now();
    ROS_INFO("--> processImage: %f secs", (end-begin).toSec());
    #endif
}

void OpenPoseROSIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> convertImage: %f secs", (end-begin).toSec());
    #endif
}

std::shared_ptr<std::vector<op::Datum>> OpenPoseROSIO::createDatum()
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    // Close program when empty frame
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else // if (cv_img_ptr_ == nullptr)
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
        datumsPtr->emplace_back();
        auto& datum = datumsPtr->at(0);

        // Fill datum
        datum.cvInputData = cv_img_ptr_->image;

        return datumsPtr;
    }

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> createDatum: %f secs", (end-begin).toSec());
    #endif
}

bool OpenPoseROSIO::display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        #ifdef PROFILING
        ros::Time begin_imshow = ros::Time::now();
        #endif

        cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        key = (char)cv::waitKey(1);
        
        #ifdef PROFILING
        ros::Time end_imshow = ros::Time::now();
        ROS_INFO("--> display --> imshow: %f secs", (end_imshow-begin_imshow).toSec());
        #endif
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> display: %f secs", (end-begin).toSec());
    #endif

    return (key == 27);
}

bool OpenPoseROSIO::saveOriginalVideo(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::Mat current_image = datumsPtr->at(0).cvInputData;
        if(!current_image.empty())
        {
            if(!original_video_writer_initialized_)
            {
                original_video_writer_ = cv::VideoWriter(original_video_file_name_, CV_FOURCC('M','J','P','G'), video_fps_, current_image.size());
                original_video_writer_initialized_ = true;
            }   
            original_video_writer_.write(current_image);
        }
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> saveOriginalVideo: %f secs", (end-begin).toSec());
    #endif

    return (key == 27);
}

bool OpenPoseROSIO::saveOpenPoseVideo(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::Mat current_image = datumsPtr->at(0).cvOutputData;
        if(!current_image.empty())
        {
            if(!openpose_video_writer_initialized_)
            {
                openpose_video_writer_ = cv::VideoWriter(openpose_video_file_name_, CV_FOURCC('M','J','P','G'), video_fps_, current_image.size());
                openpose_video_writer_initialized_ = true;
            }   
            openpose_video_writer_.write(current_image);
        }
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    
    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> saveOpenPoseVideo: %f secs", (end-begin).toSec());
    #endif

    return (key == 27);
}

cv_bridge::CvImagePtr& OpenPoseROSIO::getCvImagePtr()
{
    return cv_img_ptr_;
}

void OpenPoseROSIO::printKeypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    // Example: How to use the pose keypoints
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        op::log("\nKeypoints:");
        // Accesing each element of the keypoints
        const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
        op::log("Person pose keypoints:");
        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            op::log("Person " + std::to_string(person) + " (x, y, score):");
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                std::string valueToPrint;
                for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                    valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                op::log(valueToPrint);
            }
        }
        op::log(" ");
        // Alternative: just getting std::string equivalent
        op::log("Face keypoints: " + datumsPtr->at(0).faceKeypoints.toString());
        op::log("Left hand keypoints: " + datumsPtr->at(0).handKeypoints[0].toString());
        op::log("Right hand keypoints: " + datumsPtr->at(0).handKeypoints[1].toString());
        // Heatmaps
        const auto& poseHeatMaps = datumsPtr->at(0).poseHeatMaps;
        if (!poseHeatMaps.empty())
        {
            op::log("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                    + std::to_string(poseHeatMaps.getSize(1)) + ", "
                    + std::to_string(poseHeatMaps.getSize(2)) + "]");
            const auto& faceHeatMaps = datumsPtr->at(0).faceHeatMaps;
            op::log("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                    + std::to_string(faceHeatMaps.getSize(1)) + ", "
                    + std::to_string(faceHeatMaps.getSize(2)) + ", "
                    + std::to_string(faceHeatMaps.getSize(3)) + "]");
            const auto& handHeatMaps = datumsPtr->at(0).handHeatMaps;
            op::log("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(3)) + "]");
            op::log("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(3)) + "]");
        }
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> printKeypoints: %f secs", (end-begin).toSec());
    #endif
}

void OpenPoseROSIO::publish(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    while (!depths_ptr_)
    {
        ROS_WARN("NO DEPTH");
        ros::spinOnce();
    }

    if (datumsPtr != nullptr && !datumsPtr->empty() && !FLAGS_body_disable)
    {
        const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
        const auto& leftHandKeypoints = datumsPtr->at(0).handKeypoints[0];
        const auto& rightHandKeypoints = datumsPtr->at(0).handKeypoints[1];
        std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0).faceRectangles;

        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        human_list_msg.header.stamp = ros::Time::now();
        human_list_msg.image_header = image_header_;
        human_list_msg.num_humans = poseKeypoints.getSize(0);
        
        std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(poseKeypoints.getSize(0));

        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            openpose_ros_msgs::OpenPoseHuman human;
            int keypointIdx = 0;

            int num_body_key_points_with_non_zero_prob = 0;
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                openpose_ros_msgs::PointWithProb body_point_with_prob;
                body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}];
                body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];
                if(body_point_with_prob.prob > 0)
                {
                    num_body_key_points_with_non_zero_prob++;

                    /* Calculate linear index of the keypoint's pixel */
                    keypointIdx = (int) body_point_with_prob.x + img_width_ * ((int) body_point_with_prob.y - 1);

                    body_point_with_prob.z = depths_ptr_[keypointIdx];

                    // ROS_INFO("Person %d body keypoint no.%d: (x = %f, y= %f, c = %f), depth = %g", person, bodyPart, body_point_with_prob.x, body_point_with_prob.y, body_point_with_prob.prob, depths_ptr_[keypointIdx]);
                }
                human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
            }
            human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;

            if(FLAGS_face)
            {
                int num_face_key_points_with_non_zero_prob = 0;

                for (auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
                {
                    openpose_ros_msgs::PointWithProb face_point_with_prob;
                    face_point_with_prob.x = faceKeypoints[{person, facePart, 0}];
                    face_point_with_prob.y = faceKeypoints[{person, facePart, 1}];
                    face_point_with_prob.prob = faceKeypoints[{person, facePart, 2}];
                    if(face_point_with_prob.prob > 0)
                    {
                        num_face_key_points_with_non_zero_prob++;

                        /* Calculate linear index of the keypoint's pixel */
                        keypointIdx = (int) face_point_with_prob.x + img_width_ * ((int) face_point_with_prob.y - 1);

                        face_point_with_prob.z = depths_ptr_[keypointIdx];

                        // ROS_INFO("Person %d face keypoint no.%d: (x = %f, y= %f, c = %f), depth = %g", person, facePart, face_point_with_prob.x, face_point_with_prob.y, face_point_with_prob.prob, depths_ptr_[keypointIdx]);
                    }
                    human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
                }  
                human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

                openpose_ros_msgs::BoundingBox face_bounding_box;
                face_bounding_box.x = face_rectangles.at(person).x;
                face_bounding_box.y = face_rectangles.at(person).y;
                face_bounding_box.width = face_rectangles.at(person).width;
                face_bounding_box.height = face_rectangles.at(person).height;
                human.face_bounding_box = face_bounding_box;
            }
            
            if(FLAGS_hand)
            {

                int num_right_hand_key_points_with_non_zero_prob = 0;
                int num_left_hand_key_points_with_non_zero_prob = 0;

                for (auto handPart = 0 ; handPart < rightHandKeypoints.getSize(1) ; handPart++)
                {
                    openpose_ros_msgs::PointWithProb right_hand_point_with_prob;
                    openpose_ros_msgs::PointWithProb left_hand_point_with_prob;
                    right_hand_point_with_prob.x = rightHandKeypoints[{person, handPart, 0}];
                    right_hand_point_with_prob.y = rightHandKeypoints[{person, handPart, 1}];
                    right_hand_point_with_prob.prob = rightHandKeypoints[{person, handPart, 2}];
                    if(right_hand_point_with_prob.prob > 0)
                    {
                        num_right_hand_key_points_with_non_zero_prob++;
                        
                        /* Calculate linear index of the keypoint's pixel */
                        keypointIdx = (int) right_hand_point_with_prob.x + img_width_ * ((int) right_hand_point_with_prob.y - 1);

                        right_hand_point_with_prob.z = depths_ptr_[keypointIdx];

                        // ROS_INFO("Person %d hand keypoint no.%d (right hand): (x = %f, y= %f, c = %f), depth = %g", person, handPart, right_hand_point_with_prob.x, right_hand_point_with_prob.y, right_hand_point_with_prob.prob, depths_ptr_[keypointIdx]);
                    }
                    left_hand_point_with_prob.x = leftHandKeypoints[{person, handPart, 0}];
                    left_hand_point_with_prob.y = leftHandKeypoints[{person, handPart, 1}];
                    left_hand_point_with_prob.prob = leftHandKeypoints[{person, handPart, 2}];
                    if(left_hand_point_with_prob.prob > 0)
                    {
                        num_left_hand_key_points_with_non_zero_prob++;
                        
                        /* Calculate linear index of the keypoint's pixel */
                        keypointIdx = (int) left_hand_point_with_prob.x + img_width_ * ((int) left_hand_point_with_prob.y - 1);

                        left_hand_point_with_prob.z = depths_ptr_[keypointIdx];

                        // ROS_INFO("Person %d hand keypoint no.%d (left hand): (x = %f, y= %f, c = %f), depth = %g", person, handPart, left_hand_point_with_prob.x, left_hand_point_with_prob.y, left_hand_point_with_prob.prob, depths_ptr_[keypointIdx]);
                    }
                    human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;
                    human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
                }
                human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;
                human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;
            }

            human_list.at(person) = human;
        }

        human_list_msg.human_list = human_list;

        openpose_human_list_pub_.publish(human_list_msg);

    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> publish: %f secs", (end-begin).toSec());
    #endif
}

void OpenPoseROSIO::stop()
{
    #ifdef PROFILING
    ros::Time begin = ros::Time::now();
    #endif

    free(depths_ptr_);
    depths_ptr_ = nullptr;

    if(save_original_video_flag_)
    {
        original_video_writer_.release();
    }
    if(save_openpose_video_flag_)
    {
        openpose_video_writer_.release();
    }

    #ifdef PROFILING
    ros::Time end = ros::Time::now();
    ROS_INFO("--> stop: %f secs", (end-begin).toSec());
    #endif
}