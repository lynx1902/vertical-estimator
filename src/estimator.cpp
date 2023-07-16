/* Includes // */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_lib/transformer.h>
#include <std_srvs/SetBool.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/UavState.h>
#include <sensor_msgs/Imu.h>
#include <mrs_lib/lkf.h>


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <sensor_msgs/Range.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <mrs_msgs/Float64Stamped.h>
#include <tf/transform_listener.h>

#include <vertical_estimator/rangeTopicSwitcherConfig.h>



/* Filter preliminary //{ */
#define Po 0 // position
#define Ve 1 // velocity
#define Ac 2 // acceleration


namespace mrs_lib
{
    
    const int n_states = 7;
    const int n_inputs = 1;
    const int n_measurements = 3;

    using nblkf_t = LKF<n_states, n_inputs, n_measurements>;
} // namespace mrs_lib


using A_t = mrs_lib::nblkf_t::A_t;
using B_t = mrs_lib::nblkf_t::B_t;
using H_t = mrs_lib::nblkf_t::H_t;
using Q_t = mrs_lib::nblkf_t::Q_t;
using u_t = mrs_lib::nblkf_t::u_t;

using x_t = mrs_lib::nblkf_t::x_t;
using P_t = mrs_lib::nblkf_t::P_t;
using R_t = mrs_lib::nblkf_t::R_t;

using nbstatecov_t = mrs_lib::nblkf_t::statecov_t;
//}

namespace vertical_estimator
{
    class VerticalEstimator : public nodelet::Nodelet
    {
    public:
        /* onInit() //{ */

        /**
         * @brief Initializer - loads parameters and initializes necessary structures
         */
        void onInit()
        {
            ros::NodeHandle nh_("~");
            // ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

            /* Parameter loader //{ */
            mrs_lib::ParamLoader param_loader(nh_, "VerticalEstimator");

            param_loader.loadParam("uav_name", uav_name);
            param_loader.loadParam("UAVs", uavs_ids);
            param_loader.loadParam("odom_main_topic", odom_main_topic);
            param_loader.loadParam("odom_state_topic", odom_state_topic);   
            param_loader.loadParam("odom_imu_topic", odom_imu_topic);
            param_loader.loadParam("measured_poses_topics", measured_poses_topics, measured_poses_topics);           

            param_loader.loadParam("range_topic", range_topic);
            param_loader.loadParam("range2_publish_topic", range2_publish_topic);
            
            
            param_loader.loadParam("mass_estimate_topic", mass_estimate_topic);
            param_loader.loadParam("thrust_force_topic",thrust_force_topic);

            param_loader.loadParam("height_topic",height_topic);

            estimation_frame = uav_name + "/gps_origin";
            body_frame = uav_name + "/fcu_untilted";
            garmin_frame = uav_name + "/garmin";

            //}

            /* Transformer and Timers //{ */
            transformer_ = std::make_shared<mrs_lib::Transformer>("VerticalEstimator");
            transformer_->setDefaultPrefix(uav_name);
            

            timer_publisher_ = nh_.createTimer(ros::Duration(main_rate), &VerticalEstimator::TimerMain, this, false);
            timer_debug_ = nh_.createTimer(ros::Duration(0.1), &VerticalEstimator::TimerDebug, this, false);


            // Publishers
            pub_debug = nh_.advertise<visualization_msgs::MarkerArray>("debug",1);
            pub_debug_position = nh_.advertise<visualization_msgs::Marker>("debug_position",1);
            pub_uvdar_debug = nh_.advertise<visualization_msgs::Marker>("uvdar_debug",1);

            pub_velocity_imu = nh_.advertise<geometry_msgs::Point>("imu_velocity",1);
            pub_acc_imu = nh_.advertise<geometry_msgs::Point>("imu_acc",1);
            
            pub_estimator_output = nh_.advertise<sensor_msgs::Range>("range_output", 1); /*vert estimator*/
            pub_vertical_estimator_output = nh_.advertise<sensor_msgs::Range>(range2_publish_topic, 1);

            pub_velocity_uvdar_fcu = nh_.advertise<geometry_msgs::Point>("uvdar_velocity", 1);
            pub_vertical_estimator_output_fcu = nh_.advertise<geometry_msgs::Point>("velocity_output",1);

            pub_uvdar_pos_debug = nh_.advertise<geometry_msgs::Point>("uvdar_pos_debug",1);
            pub_thrust_debug = nh_.advertise<geometry_msgs::Point>("thrust_debug",1);

            pub_pitch = nh_.advertise<geometry_msgs::Point>("pitch_angle",1);

            pub_quat = nh_.advertise<geometry_msgs::Quaternion>("quat_orientation",1);

            pub_garmin_transform = nh_.advertise<sensor_msgs::Range>("garmin_transform",1);

            pub_transformed_estimator = nh_.advertise<sensor_msgs::Range>("transformed_estimator",1);

            // Subscribers
            sub_garmin_range = nh_.subscribe(range_topic, 1, &VerticalEstimator::GarminRange, this);
            
            sub_mass_estimate = nh_.subscribe(mass_estimate_topic, 1, &VerticalEstimator::MassEstimate, this);
            sub_thrust_force = nh_.subscribe(thrust_force_topic,1, &VerticalEstimator::ThrustCorrection, this);

            sub_main_odom = nh_.subscribe(odom_main_topic, 1, &VerticalEstimator::MainOdom, this );
            sub_imu_odom = nh_.subscribe(odom_imu_topic, 1, &VerticalEstimator::ImuOdom, this);
            sub_state_odom = nh_.subscribe(odom_state_topic, 1, &VerticalEstimator::StateOdom, this);

            use_garmin = true;
            use_estimator = false;

            dyn_reconf_server.setCallback(boost::bind(&VerticalEstimator::dynamicReconfigureCallback, this, _1, _2));

            //}

            /* UAVs initialization //{ */
            for (int i = 0; i + 1 < (int)uavs_ids.size(); i += 2)
            {
                Neighbor new_nb;
                new_nb.virt_id = (int)agents.size();
                new_nb.name_id = uavs_ids[i];
                new_nb.uv_id = uavs_ids[i + 1];
                new_nb.uav_name = "uav" + std::to_string(new_nb.name_id);

                uv_ids.push_back(new_nb.uv_id);

                new_nb.filter_init = false;

                std::string new_odom_topic = "/" + new_nb.uav_name + odom_main_topic;
                // std::cout << new_odom_topic << std::endl;
                odom_main_topics.push_back(new_odom_topic);
                
                std::string new_height_topic = "/" + new_nb.uav_name + height_topic;
                // std::cout << new_height_topic << std::endl;
                height_topics.push_back(new_height_topic);

                if (lut_id.empty())
                {
                    for (int j = 0; j <= new_nb.uv_id; j++)
                    {
                        lut_id.push_back(-1);
                    }
                }
                else
                {
                    if (new_nb.uv_id + 1 >= (int)lut_id.size())
                    {
                        for (int j = 0; j <= new_nb.uv_id + 1 - (int)lut_id.size(); j++)
                        {
                            lut_id.push_back(-1);
                        }
                    }
                }

                lut_id[new_nb.uv_id] = new_nb.virt_id;

                if (!uav_name.compare(new_nb.uav_name))
                {
                    virt_id = new_nb.virt_id;
                    new_nb.focal = true;
                }
                agents.push_back(new_nb);
            }

            std::cout << std::endl;
            std::cout << "LUT IDs: ";

            for (auto &lid : lut_id)
            {
                std::cout << lid;
                std::cout << " | ";
            }

            std::cout << std::endl;

            bool is_member = false;
            for (int i = 0; i < (int)agents.size(); i++)
            {
                if (agents[i].focal)
                {
                    is_member = true;
                }
                ROS_INFO("[DataCollector]: V_ID: %d, N_ID: %d, U_ID:, %d, name: %s, focal: %d", agents[i].virt_id,
                         agents[i].name_id, agents[i].uv_id, agents[i].uav_name.c_str(), agents[i].focal);

                if (i + 1 >= (int)agents.size())
                {
                    continue;
                } 
                for (int j = i + 1; j < (int)agents.size(); j++)
                {
                    if (agents[i].name_id == agents[j].name_id || agents[i].uv_id == agents[j].uv_id)
                    {
                        ROS_ERROR("Two UAVs with the same ID detected, exiting...");
                        exit(0);
                    }
                }
            }

            if (!is_member)
            {
                ROS_ERROR("Focal UAV is not part of the group, exiting...");
                exit(0);
            }
            //}

            /* Cooperative subscribers //{ */
            for (int i = 0; i < (int)agents.size(); ++i)
            {
                nb_state_callback callback_state = [i, this](const nav_msgs::OdometryConstPtr &stateMessage)
                {
                    NeighborsStateReduced(stateMessage, i);
                };
                callbacks_nb_state.push_back(callback_state);
                sub_nb_state.push_back(nh_.subscribe(odom_main_topics[i], 1, callbacks_nb_state[i]));
            }

            if (measured_poses_topics.empty())
            {
                ROS_WARN("[UVDARKalman]: No topics of measured_poses_topics were supplied. Returning.");
                return;
            }

            for (auto &topic : measured_poses_topics)
            {
                ROS_INFO_STREAM("[UVDARKalman]: Subscribing to " << topic);

                sub_uvdar_measurements.push_back(nh_.subscribe(topic, 3, &VerticalEstimator::callbackUvdarMeasurement, this,
                                                               ros::TransportHints().tcpNoDelay()));
            }
            //}

           // For communicating heights
            for(int i = 0; i < (int)agents.size(); ++i){
                nb_height_callback callback_height = [i, this](const mrs_msgs::Float64StampedConstPtr &heightMessage){
                    NeighborsHeight(heightMessage, i);
                };
                callbacks_nb_height.push_back(callback_height);
                sub_nb_height.push_back(nh_.subscribe(height_topics[i], 1, callbacks_nb_height[i]));
            }


            A.resize(7,7);
            B.resize(7,1);
            H.resize(3,7);
            Qq.resize(7,7);

            A << 1, def_dt, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0,
                0, 0, 1, def_dt, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 1, def_dt, def_dt*def_dt/2.0,
                0, 0, 0, 0, 0, 1, def_dt,
                0, 0, 0, 0, 0, 0, 0, 1;

            B << 0,
                0,
                0,
                0,
                0,
                0,
                0;

            H << 1, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0;

            Qq << 1.0, 0, 0, 0, 0, 0, 0,
                0, 1.0, 0, 0, 0, 0, 0,
                0, 0, 1.0, 0, 0, 0, 0,
                0, 0, 0, 1.0, 0, 0, 0,
                0, 0, 0, 0, 1.0, 0, 0,
                0, 0, 0, 0, 0, 1.0, 0,
                0, 0, 0, 0, 0, 0, 1.0;
              

            
            filter = std::make_unique<mrs_lib::nblkf_t>(A, B, H);

            last_imu_meas = ros::Time::now();



            //}

            ROS_INFO("[VerticalEstimator]: Waiting for time...");
            ros::Time::waitForValid();

            initialized_ = true;
            ROS_INFO("[VerticalEstimator]: Initialized.");
        }
        //}

        /* destructor //{ */
        /**
         * @brief destructor
         */
        ~VerticalEstimator()
        {
        }
        //}

    private:
        /* TIMERS //{ */
        /* The Main Timer //{ */
        void TimerMain([[maybe_unused]] const ros::TimerEvent &te)
        {
            if (initialized_)
            {
                // ROS_INFO("[VerticalEstimator]: Spinning");

                /* getIMUvelocity(); */

                ROS_WARN_THROTTLE(3.0, "Estimation frame %s", estimation_frame.c_str());
                ROS_WARN_THROTTLE(3.0, "Body frame %s", body_frame.c_str());


                for (auto &nb : agents)
                {
                    if (!nb.filter_init || nb.focal)
                    {
                        continue;
                    }

                    if ((ros::Time::now() - nb.last_meas_u).toSec() > 8.0)
                    {
                        nb.filter_init = false;
                        ROS_WARN_THROTTLE(0.5, "Agent data not updated for a long time. ID: %d NAME: %s", nb.virt_id,
                                          nb.uav_name.c_str());
                        continue;
                    }
                    if ((ros::Time::now() - nb.last_meas_s).toSec() > 3.0)
                    {
                        nb.filter_init = false;
                        ROS_WARN_THROTTLE(0.5, "Agent data not updated for a long time. ID: %d NAME: %s", nb.virt_id,
                                          nb.uav_name.c_str());
                        continue;
                    }
                    
                }

                const u_t u = u_t::Random();

                double new_dt = std::fmax(std::fmin((ros::Time::now() - last_updt_focal).toSec(), (ros::Time::now() - last_meas_focal).toSec()), 0.0);
                
                filter->A = A_dt(new_dt);

                filter_state_focal = filter->predict(filter_state_focal, u, Qq, new_dt);

                if (isnan(filter_state_focal.x(4)))
                {
                    ROS_ERROR("Filter error on line 340");
                    
                }
                last_updt_focal = ros::Time::now();

                if ((ros::Time::now() - last_meas_focal_main).toSec() < 10.0)
                {
                    filter_valid = true;
                }
                else
                {
                    filter_valid = false;
                }
                
                sensor_msgs::Range vert_est_output;
                vert_est_output.header.stamp = ros::Time::now();
                vert_est_output.header.frame_id = estimation_frame;
                vert_est_output.radiation_type = vert_est_output.INFRARED;
                vert_est_output.min_range = 0.0;
                vert_est_output.max_range = 20.0;
                vert_est_output.field_of_view = M_PI;
                vert_est_output.range = filter_state_focal.x(4);

                pub_estimator_output.publish(vert_est_output);

                sensor_msgs::Range smoothOutput = smoothRange(vert_est_output, 0.2);

                sensor_msgs::Range ret_transform = transformRangeMessage(smoothOutput);
                
                pub_transformed_estimator.publish(ret_transform);

                if(use_garmin){
                    pub_vertical_estimator_output.publish(rmsg);
                    
                } else if (use_estimator) {
                    pub_vertical_estimator_output.publish(smoothOutput);
                }
                // pub_vertical_estimator_output.publish(ret_transform);  
                // pub_vertical_estimator_output.publish(rmsg);
            

                geometry_msgs::Point velo;
                velo.x = filter_state_focal.x(1) * cos(-focal_heading) - filter_state_focal.x(3) * sin(-focal_heading);
                velo.y = filter_state_focal.x(1) * sin(-focal_heading) + filter_state_focal.x(3) * cos(-focal_heading);
                velo.z = filter_state_focal.x(5);
                pub_vertical_estimator_output_fcu.publish(velo);

                std::string output_frame = estimation_frame;

                // Visualisation

                visualization_msgs::Marker sphere;
                sphere.header.frame_id = output_frame;
                sphere.header.stamp = ros::Time::now();
                sphere.id = 1; 
                sphere.type = visualization_msgs::Marker::CUBE;
                sphere.action = 0;

                sphere.pose.position.x = filter_state_focal.x(0);
                sphere.pose.position.y = filter_state_focal.x(2);
                sphere.pose.position.z = filter_state_focal.x(4);

                sphere.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
                sphere.scale.x = 0.3;
                sphere.scale.y = 0.3;
                sphere.scale.z = 0.3;
                sphere.color.a = 1.0; 
                sphere.color.r = 0.0;
                sphere.color.g = 0.0;
                sphere.color.b = 1.0;
                
                pub_debug_position.publish(sphere);
                
                ROS_INFO_THROTTLE(0.25, "H: %f, H_dt: %f, H_ddt: %f", filter_state_focal.x(4), filter_state_focal.x(5), filter_state_focal.x(6));
            }
        }
        //}
        void TimerDebug([[maybe_unused]] const ros::TimerEvent& te) {
            if (initialized_){
        
            visualization_msgs::MarkerArray relative_poses;
            
            int a = 0;

            for(auto nb : agents){
                if(!nb.filter_init) continue;

                a++; 
                std::string output_frame = estimation_frame;
                
                visualization_msgs::Marker arrow;
                arrow.header.frame_id = output_frame;
                arrow.header.stamp = ros::Time::now();
                arrow.id = a; 
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.action = 0;

                arrow.pose.position.x = nb.filter_state.x(0);
                arrow.pose.position.y = nb.filter_state.x(2);
                arrow.pose.position.z = nb.filter_state.x(4);

                arrow.pose.orientation = mrs_lib::AttitudeConverter(0, 0, focal_heading);
                arrow.scale.x = 1.0;
                arrow.scale.y = 0.1;
                arrow.scale.z = 0.1;
                arrow.color.a = 1.0; 
                arrow.color.r = 0.0;
                arrow.color.g = 0.0;
                arrow.color.b = 0.0;
                
                relative_poses.markers.push_back(arrow);
                
            }
            pub_debug.publish(relative_poses);

            for(auto& ps : past_spheres){
                if((ros::Time::now() - ps.last_debug).toSec() > 0.2){
                        
                std::string output_frame = estimation_frame;
                
                visualization_msgs::Marker sphere;
                sphere.header.frame_id = output_frame;
                sphere.header.stamp = ros::Time::now();
                sphere.id = ps.id; 
                sphere.type = visualization_msgs::Marker::SPHERE;
                sphere.action = 2;

                pub_debug_position.publish(sphere);
                ps.valid = false;
                }
            }
            }
        }
        /* Filter matrices update //{ */

        A_t A_dt(double dt)
        {
            A << 1, dt, 0, 0, 0, 0, 0, 
                0, 1, 0, 0, 0, 0, 0,
                0, 0, 1, dt, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 1, dt, dt*dt/2.0,
                0, 0, 0, 0, 0, 1, dt,
                0, 0, 0, 0, 0, 0, 1;
                

                return A;
        }

        //}

        /* Covariance to eigen method //{ */

        Eigen::MatrixXd rosCovariancetoEigen(const boost::array<double, 36> input) /*for 6x6 matrices*/
        {
            Eigen::MatrixXd output(6, 6);

            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    output(j, i) = input[6 * j + i];
                }
            }
            return output;
        }
        //}

         void NeighborsHeight(const mrs_msgs::Float64StampedConstPtr &height_msg, size_t nb_index)
        {
            if(virt_id == int(nb_index)){
                return;
            }

            if(!agents[nb_index].filter_init){
                return;
            }

            u_t u = u_t::Zero();

            double new_dt = std::fmax(std::fmin(std::fmin((ros::Time::now()-agents[nb_index].last_updt).toSec(), (ros::Time::now()-agents[nb_index].last_meas_u).toSec()), (ros::Time::now()-agents[nb_index].last_meas_s).toSec()),0.0);

            filter->A = A_dt(new_dt);

            agents[nb_index].filter_state = filter->predict(agents[nb_index].filter_state, u, Qq, new_dt);

            agents[nb_index].last_updt = ros::Time::now();    

            mrs_msgs::Float64Stamped height_msg_local;
            height_msg_local.header.frame_id = height_msg->header.frame_id;
            height_msg_local.value = height_msg->value;

                // std::string output_frame = agents[nb_index].uav_name + "/gps_origin";

                // mrs_msgs::Float64Stamped transformed_height_msg = transformHeightMessage(height_msg_local, output_frame);


            try{
                filter->H << 0, 0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0;

                R_t R;
                R = Eigen::MatrixXd::Identity(3,3) * 1;
                Eigen::VectorXd z(3);

                z(0) = height_msg_local.value;

                agents[nb_index].filter_state = filter->correct(agents[nb_index].filter_state, z, R);

                if(isnan(filter_state_focal.x(4))){
                    ROS_ERROR("Filter error on line 579");
                }

                agents[nb_index].last_meas_s = ros::Time::now();
            }
            catch([[maybe_unused]] std::exception e){
                ROS_ERROR("LKF failed: %s", e.what());
            }

        }
        

        /* Cooperative subscribers //{ */
        /* Subscriber of neighbors derivative states //{ */
        // todo do it without communication
        void NeighborsStateReduced(const nav_msgs::OdometryConstPtr &odom_msg, size_t nb_index)
        {
            if (virt_id == (int)nb_index)
            {
                return;
            }

            if (!agents[nb_index].filter_init)
            {
                return;
            }

            // u_t u = u_t::Zero();

            // double new_dt = std::fmax(std::fmin(std::fmin((ros::Time::now()-agents[nb_index].last_updt).toSec(), (ros::Time::now()-agents[nb_index].last_meas_u).toSec()), (ros::Time::now()-agents[nb_index].last_meas_s).toSec()),0.0);

            // // New updated filter
            // filter->A = A_dt(new_dt);

            // agents[nb_index].filter_state = filter->predict(agents[nb_index].filter_state, u, Qq, new_dt);

            // agents[nb_index].last_updt = ros::Time::now();   

            try
            {
                filter->H << 0, 1, 0, 0, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 0, 0, 1, 0;
                                    
                R_t R;
                R = Eigen::MatrixXd::Identity(3,3) * 1;
                Eigen::VectorXd z(3);

                z(0) = odom_msg->twist.twist.linear.x;
                z(1) = odom_msg->twist.twist.linear.y;
                z(2) = odom_msg->twist.twist.linear.z;

                agents[nb_index].filter_state = filter->correct(agents[nb_index].filter_state, z, R);

                if(isnan(filter_state_focal.x(4))){
                    ROS_ERROR("Filter error on line 608");
                }

                agents[nb_index].last_meas_s = ros::Time::now();
            }
            catch ([[maybe_unused]] std::exception e)
            {
                ROS_ERROR("LKF failed: %s", e.what());
            }

        }

       

        // //}

        void callbackUvdarMeasurement(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
        {
            if ((int)(msg.poses.size()) < 1)
                return;

            mrs_msgs::PoseWithCovarianceArrayStamped msg_local = msg;

            std::string output_frame = estimation_frame;

            tf2lf_ = transformer_->getTransform(msg_local.header.frame_id, output_frame, ros::Time(0));
            if (!tf2lf_)
            {
                ROS_ERROR("[UVDARKalman]: Could not obtain transform from %s to %s", msg_local.header.frame_id.c_str(),
                          output_frame.c_str());
                return;
            }

            output_frame = uav_name + "/fcu_untilted";
            // output_frame = body_frame;
            tf2bf_ = transformer_->getTransform(msg_local.header.frame_id, output_frame, ros::Time(0));
            if (!tf2bf_)
            {
                ROS_ERROR("[UVDARKalman]: Could not obtain transform from %s to %s", msg_local.header.frame_id.c_str(),
                          output_frame.c_str());
                return;
            }

            Intersection sum_of_ints;
            sum_of_ints.ints_count = 0;

            for (auto &meas : msg_local.poses)
            {
                geometry_msgs::PoseWithCovarianceStamped meas_s;
                meas_s.pose.pose = meas.pose;
                meas_s.pose.covariance = meas.covariance;
                meas_s.header = msg_local.header;
                

                if ((int)meas.id < 0 || (int)meas.id >= (int)lut_id.size() ||
                    !std::count(uv_ids.begin(), uv_ids.end(), (int)meas.id))
                {
                    ROS_ERROR("Wrong UVDAR ID %d", (int)meas.id);
                    continue;
                }
                int aid = lut_id[(int)meas.id];
                auto res_l_ = transformer_->transform(meas_s, tf2lf_.value());
                auto res_b_ = transformer_->transform(meas_s, tf2bf_.value());


                if (res_b_)
                {
                    Eigen::MatrixXd poseCovB(6, 6);
                    poseCovB = rosCovariancetoEigen(res_b_.value().pose.covariance);
                    Eigen::EigenSolver<Eigen::MatrixXd> es(poseCovB.topLeftCorner(3, 3));
                    auto eigvals_b = es.eigenvalues();

                    /* if(agents[aid].filter_init && eigvals_b(0).real() < 500 && eigvals_b(1).real() < 500 && eigvals_b(2).real() <
                     * 500){ */
                    if (agents[aid].filter_init)
                    {
                        agents[aid].eigens.x = eigvals_b(0).real();
                        agents[aid].eigens.y = eigvals_b(1).real();
                        agents[aid].eigens.z = eigvals_b(2).real();

                        agents[aid].pfcu.x = res_b_.value().pose.pose.position.x;
                        agents[aid].pfcu.y = res_b_.value().pose.pose.position.y;
                        agents[aid].pfcu.z = res_b_.value().pose.pose.position.z;
                        agents[aid].last_pfcu = ros::Time::now();
                        agents[aid].hdg_at_last_pfcu = focal_heading;
                        agents[aid].pfcu_init = true;
                        agents[aid].angle_z = atan2(agents[aid].pfcu.y,agents[aid].pfcu.x);
                        // agents[aid].quat = res_b_.value().pose.pose.orientation;
                        tf2::Quaternion q_temp;
                        tf2::fromMsg(res_b_.value().pose.pose.orientation, q_temp);
                        q_temp = q_temp.normalized();
                        geometry_msgs::Quaternion normalizedQuaternion;
                        normalizedQuaternion = tf2::toMsg(q_temp);
                        agents[aid].quat = normalizedQuaternion;
                        // agents[aid].quat = res_l_.value().pose.pose.orientation;
                       
                        

                        for (auto &nb : agents)
                        {
                            if (nb.focal || !nb.pfcu_init || nb.virt_id == agents[aid].virt_id)
                            {
                                continue;
                            }
                            double nb_dt = (ros::Time::now() - nb.last_pfcu).toSec();
                            if (nb_dt < 0.5)
                            {
                                double hdg_diff = nb.hdg_at_last_pfcu - agents[aid].hdg_at_last_pfcu;
                                double nb_hdg = nb.angle_z + hdg_diff;
                                double gap;
                                

                                if (nb_hdg > agents[aid].angle_z)
                                {
                                    gap = abs(nb_hdg - agents[aid].angle_z);
                                }
                                else
                                {
                                    gap = abs(agents[aid].angle_z - nb_hdg);
                                }

                                if (gap > M_PI)
                                {
                                    gap = 2 * M_PI - gap;
                                }

                                double lgl = M_PI / 6;
                                double ugl = 5 * M_PI / 6;

                                 geometry_msgs::Point new_int;

                                if (gap > lgl && gap < ugl)

                                {   
                                    // /* Method 1 Intersection Calculation //{ */
                                    geometry_msgs::Point A;
                                    A.x = nb.filter_state.x(0);
                                    A.y = nb.filter_state.x(2);
                                    A.z = nb.filter_state.x(4);

                                    geometry_msgs::Point focal;
                                    focal.x = filter_state_focal.x(0);
                                    focal.y = filter_state_focal.x(2);
                                    focal.z = filter_state_focal.x(4);

                                    double nb_dist = distanceForElevation(A,focal_position);

                                    geometry_msgs::Point B;
                                    B.x = A.x + cos(nb_hdg + focal_heading);
                                    B.y = A.y + sin(nb_hdg + focal_heading);
                                    // B.z = A.z + 0.1*sin(Quat2Eul(nb.quat));
                                    B.z = A.z + 0.1*sin(atan((focal_position.z- A.z)/nb_dist));
                                    
                                    geometry_msgs::Point C;
                                    C.x = agents[aid].filter_state.x(0);
                                    C.y = agents[aid].filter_state.x(2);
                                    C.z = agents[aid].filter_state.x(4); 

                                    double agent_dist = calculateDistance(C,focal_position);
                                    
                                    geometry_msgs::Point D;
                                    D.x = C.x + cos(agents[aid].angle_z + focal_heading);
                                    D.y = C.y + sin(agents[aid].angle_z + focal_heading);
                                    // D.z = C.z + 0.1*sin(Quat2Eul(agents[aid].quat));
                                    D.z = C.z + 0.1*sin(atan((focal_position.z - C.z)/agent_dist));

                                    Eigen::Vector3d dirAB, dirCD;
                                    dirAB(0) = B.x - A.x;
                                    dirAB(1) = B.y - A.y;
                                    dirAB(2) = B.z - A.z;
                                    dirCD(0) = D.x - C.x;
                                    dirCD(1) = D.y - C.y;
                                    dirCD(2) = D.z - C.z;

                                    // Vector connecting any arbitrary point on AB to any arbitrary point on CD
                                    Eigen::Vector3d L;
                                    L(0) = C.x - A.x;
                                    L(1) = C.y - A.y;
                                    L(2) = C.z - A.z;

                                    Eigen::Vector3d crossProduct;
                                    crossProduct = dirAB.cross(dirCD);
                                  
                                    if(crossProduct.norm() > 1e-6){
                                        // Non zero cross product magnitude indicates skew lines or intersecting lines
                                        // Handle those cases here
                                        
                                        // based on the source: https://www.quora.com/How-do-you-know-if-lines-are-parallel-skew-or-intersecting
                                        double checkSkeworIntersect = L.dot(crossProduct);


                                        
                                        //https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
                                        double tAB = ((dirCD.cross(crossProduct)).dot(L))/(crossProduct.dot(crossProduct));
                                        double tCD = ((dirAB.cross(crossProduct)).dot(L))/(crossProduct.dot(crossProduct));

                                        if(checkSkeworIntersect > 1e-6){
                                            // Skew lines
                                            // https://www.quora.com/How-do-I-find-the-shortest-distance-between-two-skew-lines
                                            geometry_msgs::Point intersectionAB;  
                                            intersectionAB.x = A.x + tAB * dirAB(0);
                                            intersectionAB.y = A.y + tAB * dirAB(1);
                                            intersectionAB.z = A.z + tAB * dirAB(2);

                                            geometry_msgs::Point intersectionCD;
                                            intersectionCD.x = C.x + tCD * dirCD(0);
                                            intersectionCD.y = C.y + tCD * dirCD(1);
                                            intersectionCD.z = C.z + tCD * dirCD(2);

                                            Eigen::Vector3d unitVector = crossProduct.normalized();

                                            double minDistance = abs(L.dot(crossProduct)/crossProduct.norm());

                                            Eigen::Vector3d distanceComponents = minDistance * unitVector;

                                            double dist_x = abs(distanceComponents(0));
                                            double dist_y = abs(distanceComponents(1));
                                            double dist_z = abs(distanceComponents(2));
                                            
                                            if(minDistance == dist_z){
                                                ROS_WARN("Values are same!Some issue here");
                                            }

                                            if(dist_z > 0.0){
                                            ROS_WARN("Skew Lines");
                                            if(dirAB(2) < 0.0 && dirCD(2) < 0.0){
                                              new_int.x = (intersectionAB.x + intersectionCD.x)/2.0;
                                              new_int.y = (intersectionAB.y + intersectionCD.y)/2.0;
                                              if(intersectionAB.z > intersectionCD.z){
                                                new_int.z = (intersectionAB.z) - (dist_z/2.0);
                                              } else {
                                                new_int.z = (intersectionCD.z) - (dist_z/2.0);
                                              }
                                            //   new_int.z = (intersectionAB.z + intersectionCD.z)/2.0;
                                                if(new_int.z < 0){
                                                    ROS_ERROR("Line 714 calc");
                                                    new_int.z = (A.z + C.z)/2.0;
                                                    if(new_int.z < 0.0){
                                                        ROS_ERROR("Still negative at 714");
                                                    }
                                                }
                                            }
                                            else if(dirAB(2) > 0.0 && dirCD(2) > 0.0){
                                                new_int.x = (intersectionAB.x + intersectionCD.x)/2.0;
                                                new_int.y = (intersectionAB.y + intersectionCD.y)/2.0;
                                                if(intersectionAB.z < intersectionCD.z){
                                                    new_int.z = intersectionAB.z + (dist_z/2.0);
                                                } else {
                                                    new_int.z = intersectionCD.z + (dist_z/2.0);
                                                }
                                                // new_int.z = (intersectionAB.z + intersectionCD.z)/2.0;
                                                if(new_int.z < 00){
                                                    ROS_ERROR("Line 720 calc");
                                                    new_int.z = (A.z + C.z)/2.0;
                                                    if(new_int.z < 0.0){
                                                        ROS_ERROR("Terrible error!!!");
                                                        
                                                    }
                                                }
                                            }
                                            else {
                                                ROS_WARN("Opposite Slopes");
                                                if(intersectionAB.z < intersectionCD.z){
                                                    new_int.x = (intersectionAB.x + intersectionCD.x)/2.0;
                                                    new_int.y = (intersectionAB.y + intersectionCD.y)/2.0;
                                                    double calc_z = intersectionAB.z + (dist_z/2.0);
                                                    // new_int.z = calc_z;
                                                    if(calc_z < 0.0){
                                                        new_int.z = (A.z + C.z)/2.0;
                                                    } else {
                                                        new_int.z = calc_z;
                                                    }
                                                    if(new_int.z < 00){
                                                    ROS_ERROR("This implementation sucks!!!");
                                                    }
                                                }
                                                else if (intersectionCD.z < intersectionAB.z){
                                                    new_int.x = (intersectionAB.x + intersectionCD.x)/2.0;
                                                    new_int.y = (intersectionAB.y + intersectionCD.y)/2.0;
                                                    double calc_z = intersectionCD.z + (dist_z/2.0);
                                                    // new_int.z = calc_z;
                                                    if(calc_z < 0){
                                                        new_int.z = (A.z + C.z)/2.0;
                                                    } else {
                                                        new_int.z = calc_z;
                                                    }
                                                    if(new_int.z < 00.0){
                                                    ROS_ERROR("Line 734 calc");
                                                }
                                                }
                                            }                                            
                                            }
                                            else {
                                                ROS_ERROR("Something is going terribly wrong!");
                                            }

                                        }
                                        else{
                                            // Intersecting lines
                                            // https://math.stackexchange.com/questions/270767/find-intersection-of-two-3d-lines
                                            double k = crossProduct.norm();

                                            Eigen::Vector3d crossCDandL;
                                            crossCDandL = dirAB.cross(L);

                                            double h = crossCDandL.norm();


                                            if(crossCDandL.dot(crossProduct) > 0){
                                                new_int.x = A.x + (h/k) * dirAB(0);
                                                new_int.y = A.y + (h/k) * dirAB(1);
                                                new_int.z = A.z + (h/k) * dirAB(2);
                                                if(new_int.z < 0.0){
                                                    ROS_ERROR("Line 755 calc");
                                                    new_int.z = (A.z + C.z)/2.0;
                                                }
                                            } else {
                                                new_int.x = A.x - (h/k) * dirAB(0);
                                                new_int.y = A.y - (h/k) * dirAB(1);
                                                new_int.z = A.z - (h/k) * dirAB(2);
                                                if(new_int.z < 0.0){
                                                    ROS_ERROR("Line 760 calc");
                                                    new_int.z = (A.z + C.z)/2.0;
                                                }
                                            }
                                            ROS_WARN("Intersecting Lines");
                                            
                                        }
                                    } 
                                    else {
                                        // Parallel or coincident lines
                                        // Handle that case here

                                        ROS_ERROR("Parallel Lines!!");
                                        
                                        double minDistance = abs(L.dot(crossProduct))/crossProduct.norm();
                                        Eigen::Vector3d distanceComponents = minDistance * crossProduct.normalized();

                                        double dist_z = abs(distanceComponents(2));
                                        double dist_y = abs(distanceComponents(1));
                                        double dist_x = abs(distanceComponents(0));
                                        
                                        
                                        
                                        new_int.x = (A.x) + (dist_x/2.0);
                                        new_int.y = (A.y) + (dist_y/2.0);
                                        new_int.z = (A.z) + (dist_z/2.0);
                                    
                                    } 
                                
                                    // } /Uncomment before this line */

                                    /*Method 2 */
                                    // This method actually requires 3 neighbors to simulataenously implement the technique, but the loop would need to be modified to accomodate that
                                    // geometry_msgs::Point nb1_pos;
                                    // nb1_pos.x = nb.filter_state.x(0);
                                    // nb1_pos.y = nb.filter_state.x(2);
                                    // nb1_pos.z = nb.filter_state.x(4);

                                    // geometry_msgs::Point nb2_pos;
                                    // nb2_pos.x = agents[aid].filter_state.x(0);
                                    // nb2_pos.y = agents[aid].filter_state.x(2);
                                    // nb2_pos.z = agents[aid].filter_state.x(4);

                                    // geometry_msgs::Point nb1_rel = calculateRelativePosition(nb1_pos,nb2_pos);

                                    // geometry_msgs::Point nb2_rel = calculateRelativePosition(nb2_pos,nb1_pos);
                                    
                                    // geometry_msgs::Point transformedPos1 = applyCoordinateTransformations(nb1_rel, nb.quat);
                                    // geometry_msgs::Point transformedPos2 = applyCoordinateTransformations(nb2_rel, agents[aid].quat);

                                    // geometry_msgs::Point averageRelativePos;
                                    // averageRelativePos.x = (transformedPos1.x + transformedPos2.x) / 2.0;
                                    // averageRelativePos.y = (transformedPos1.y + transformedPos2.y) / 2.0;
                                    // averageRelativePos.z = (transformedPos1.z + transformedPos2.z) / 2.0;
                                
                                    // new_int.x = nb1_pos.x + averageRelativePos.x;
                                    // new_int.y = nb1_pos.y + averageRelativePos.y;
                                    // new_int.z = nb1_pos.z + averageRelativePos.z;
                                    // if(new_int.z < 0){
                                    //     new_int.z = (nb1_pos.z + nb2_pos.z)/2.0;
                                    // }
                                    /*End of method 2*/

                                    /*Method 3*/
                                    // geometry_msgs::Point A;
                                    // A.x = nb.filter_state.x(0);
                                    // A.y = nb.filter_state.x(2);
                                    // A.z = nb.filter_state.x(4);

                                    // double nb_dist = calculateDistance(A, focal_position);

                                    // double nb_ang = atan2(-focal_position.y + A.y, focal_position.x - A.x);

                                    // double nb_elevation = atan2(focal_position.z - A.z, distanceForElevation(A,focal_position));
                                    
                                    // geometry_msgs::Point nb_rel;
                                    // nb_rel.x = nb_dist * cos(nb_elevation) * cos(nb_hdg + focal_heading);
                                    // nb_rel.y = nb_dist * cos(nb_elevation) * sin(nb_hdg + focal_heading);
                                    // nb_rel.z = nb_dist * sin(nb_elevation);

                                    // geometry_msgs::Point C;
                                    // C.x = agents[aid].filter_state.x(0);
                                    // C.y = agents[aid].filter_state.x(2);
                                    // C.z = agents[aid].filter_state.x(4); 

                                    // double agent_dist = calculateDistance(C, focal_position);

                                    // double agent_ang = atan2(-focal_position.y + C.y, focal_position.x - C.x);

                                    // double agent_elevation = atan2(focal_position.z - C.z, distanceForElevation(C,focal_position));
                                    
                                    // geometry_msgs::Point agent_rel;
                                    // agent_rel.x = agent_dist * cos(agent_elevation) * cos(agents[aid].angle_z + focal_heading);
                                    // agent_rel.y = agent_dist * cos(agent_elevation) * sin(agents[aid].angle_z + focal_heading);
                                    // agent_rel.z = agent_dist * sin(agent_elevation);
                                    
                                    // new_int.x = (nb_rel.x + agent_rel.x) / 2.0;
                                    // new_int.y = (nb_rel.y + agent_rel.y) /2.0;
                                    // new_int.z = (nb_rel.z + agent_rel.z) /2.0;



                                    geometry_msgs::Point uvdar_pos_debug;
                                    uvdar_pos_debug.x = new_int.x;
                                    uvdar_pos_debug.y = new_int.y;
                                    uvdar_pos_debug.z = new_int.z;
                                    pub_uvdar_pos_debug.publish(uvdar_pos_debug);

                                    double gt_dist = sqrt(pow(focal_position.z - new_int.z,2));

                                    if(gt_dist <= 5.0)
                                    {
                                        ROS_INFO(
                                            "D: %f, h_diff: %f, gap: %f, nb_dt: %f, eb1: %f, eb2: %f, eb3: %f, ea1: %f, ea2: %f, "
                                            "ea3: %f",
                                            gt_dist, hdg_diff, gap, nb_dt, nb.eigens.x, nb.eigens.y, nb.eigens.z, agents[aid].eigens.x,
                                            agents[aid].eigens.y, agents[aid].eigens.z);

                                        std::string output_frame = estimation_frame;

                                        visualization_msgs::Marker sphere;
                                        sphere.header.frame_id = output_frame;
                                        sphere.header.stamp = ros::Time::now();
                                        sphere.id = 10*nb.virt_id + agents[aid].virt_id; 
                                        sphere.type = visualization_msgs::Marker::SPHERE;
                                        sphere.action = 0;
                       
                                        bool added = false;
                                        for(auto& ps : past_spheres){
                                          if(ps.id == sphere.id){
                                            ps.last_debug = ros::Time::now();
                                            ps.valid = true;
                                            added = true;
                                            break;
                                          }
                                        }
                                        if(!added){
                                          Debug new_sphere;
                                          new_sphere.id = sphere.id;
                                          new_sphere.valid = true;
                                          new_sphere.last_debug = ros::Time::now();
                                          past_spheres.push_back(new_sphere);
                                        }
                       
                                        sphere.pose.position.x = new_int.x;
                                        sphere.pose.position.y = new_int.y;
                                        sphere.pose.position.z = new_int.z;
                       
                                        sphere.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
                                        sphere.scale.x = 0.3;
                                        sphere.scale.y = 0.3;
                                        sphere.scale.z = 0.3;
                                        sphere.color.a = 1.0; 
                                        sphere.color.r = 1.0;
                                        sphere.color.g = 0.0;
                                        sphere.color.b = 0.0;
                           
                                        pub_debug_position.publish(sphere);

                                        
                                        sum_of_ints.ints.x += new_int.x;
                                        sum_of_ints.ints.y += new_int.y;
                                        sum_of_ints.ints.z += new_int.z; /*calculate this correctly, not sure about current implementation*/
                                        sum_of_ints.ints_count += 1;
                                        


                                        if (!filter_init_focal)
                                        {
                                            Eigen::VectorXd poseVec(7);
                                            poseVec(0) = new_int.x; /*important part to figure out. Intersection of all poses to find actual
                                                                       coordinates*/
                                            poseVec(1) = 0;
                                            poseVec(2) = new_int.y;
                                            poseVec(3) = 0;
                                            poseVec(4) = new_int.z;
                                            poseVec(5) = 0;
                                            poseVec(6) = 0; 
                                            
                                            Eigen::MatrixXd poseCov(7,7);
                                            poseCov << Eigen::MatrixXd::Identity(7, 7);

                                            ROS_INFO("LKF initialized for the focal UAV at %f of GPS frame.", poseVec(4));

                                            last_meas_focal = ros::Time::now();
                                            last_meas_focal_main = ros::Time::now();
                                            last_updt_focal = ros::Time::now();
                                            filter_state_focal = {.x = poseVec, .P = poseCov};
                                            filter_init_focal = true;
                                        }
                                        else
                                        {

                                            try
                                            {
                                                filter->H << 1, 0, 0, 0, 0, 0, 0,
                                                                    0, 0, 1, 0, 0, 0, 0,
                                                                    0, 0, 0, 0, 1, 0, 0;
                                                                    

                                                R_t R;
                                                R = Eigen::MatrixXd::Identity(3, 3) * 0.1;
                                                Eigen::VectorXd z(3);
                                                z(0) = new_int.x;
                                                z(1) = new_int.y;
                                                z(2) = new_int.z; /* need intersection point of all poses*/

                                                filter_state_focal = filter->correct(filter_state_focal, z, R);
                                                if (isnan(filter_state_focal.x(4)))
                                                {
                                                    ROS_ERROR("Filter error on line 852");
                                                }
                                                last_meas_focal_main = ros::Time::now();
                                            }
                                            catch ([[maybe_unused]] std::exception e)
                                            {
                                                ROS_ERROR("LKF failed: %s", e.what());
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    ROS_WARN_THROTTLE(1.0, "UAV%d, and UAV%d could have singular solution with bearing gap %f", nb.name_id,
                                                      agents[aid].name_id, gap);
                                    continue;
                                }
                            }
                        }
                    }
                    else
                    {
                        ROS_WARN("[VerticalEstimator]: Linear model of uav%d not initialized.", agents[aid].name_id);
                    }
                }
                else
                {
                    ROS_INFO_STREAM("[VerticalEstimator]: Failed to get transformation for tf2bf measurement, returning.");
                    /* return; */
                }

                if (res_l_)
                {
                    Eigen::MatrixXd poseCov(6, 6);
                    poseCov = rosCovariancetoEigen(res_l_.value().pose.covariance);
                    Eigen::EigenSolver<Eigen::Matrix3d> es(poseCov.topLeftCorner(3, 3));
                    auto eigvals = es.eigenvalues();

                    if (!agents[aid].filter_init)
                    {
                        Eigen::VectorXd poseVec(7);
                        poseVec(0) = res_l_.value().pose.pose.position.x;
                        poseVec(1) = 0;
                        poseVec(2) = res_l_.value().pose.pose.position.y;
                        poseVec(3) = 0;
                        poseVec(4) = res_l_.value().pose.pose.position.z;
                        poseVec(5) = 0;
                        poseVec(6) = 0;
                        
                        Eigen::MatrixXd poseCov(7, 7);
                        poseCov << Eigen::MatrixXd::Identity(7, 7);

                        ROS_INFO("LKF initialized for UAV%d with UVDAR reloc at %f, %f, %f of gps frame.", agents[aid].name_id,
                                 poseVec(4), poseVec(5), poseVec(6));

                        agents[aid].last_meas_u = ros::Time::now();
                        agents[aid].last_meas_s = ros::Time::now();
                        agents[aid].last_updt = ros::Time::now();
                        agents[aid].filter_state = {.x = poseVec, .P = poseCov};
                        agents[aid].filter_init = true;
                    }

                    else
                    {
                        if (eigvals(0).real() < 500 && eigvals(1).real() < 500 && eigvals(2).real() < 500 &&
                            (ros::Time::now() - agents[aid].last_meas_u).toSec() >= 4.0)
                        // if(eigvals(2).real() < 500 && (ros::Time::now() - agents[aid].last_meas_u).toSec() >= 4.0)
                        {
                            
                            try
                            {
                                filter->H << 1, 0, 0, 0, 0, 0, 0,
                                                    0, 0, 1, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 1, 0, 0;

                                R_t R;
                                R = Eigen::MatrixXd::Identity(3,3) * 0.00001;
                                Eigen::VectorXd z(3);
                                z(0) = res_l_.value().pose.pose.position.x;
                                z(1) = res_l_.value().pose.pose.position.y;
                                z(2) = res_l_.value().pose.pose.position.z;
                                agents[aid].filter_state = filter->correct(agents[aid].filter_state, z, R);
                                
                                agents[aid].last_meas_u = ros::Time::now();

                                ROS_WARN("Agent state updated uav%d", agents[aid].name_id);

                            }
                            catch ([[maybe_unused]] std::exception e){
                                ROS_ERROR("LKF failed: %s", e.what());
                            }
                        }
                    }
                }
                else
                {
                    ROS_INFO_STREAM("[VerticalEstimator]: Failed to get transformation for tf2lf measurement, returning.");
                }
            }
            if (sum_of_ints.ints_count > 0.0)

            {
                sum_of_ints.ints.x = sum_of_ints.ints.x / sum_of_ints.ints_count;
                sum_of_ints.ints.y = sum_of_ints.ints.y / sum_of_ints.ints_count;
                sum_of_ints.ints.z = sum_of_ints.ints.z / sum_of_ints.ints_count;

                std::string output_fr = estimation_frame;
                visualization_msgs::Marker sphere;
                sphere.header.frame_id = output_fr;
                sphere.header.stamp = ros::Time::now();
                sphere.id = 666; 
                sphere.type = visualization_msgs::Marker::SPHERE;
                sphere.action = 0;
                sphere.pose.position.x = sum_of_ints.ints.x;
                sphere.pose.position.y = sum_of_ints.ints.y;
                sphere.pose.position.z = sum_of_ints.ints.z;

                sphere.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
                sphere.scale.x = 0.3;
                sphere.scale.y = 0.3;
                sphere.scale.z = 0.3;
                sphere.color.a = 1.0; 
                sphere.color.r = 0.0;
                sphere.color.g = 1.0;
                sphere.color.b = 0.2;
      
                pub_debug_position.publish(sphere);

                U_POS new_pose;
                new_pose.t = ros::Time::now();
                new_pose.pose = sum_of_ints.ints;

                past_u_pos.push_back(new_pose);
                if ((int)past_u_pos.size() > 10)
                {
                    geometry_msgs::Point u_vel;
                    for (int k = 0; k < 10; k++)
                    {
                        double u_dt = (past_u_pos.back().t - past_u_pos[(int)past_u_pos.size() - 2 - k].t).toSec();
                        if (u_dt > 0.0)
                        {
                            double dx = (past_u_pos.back().pose.x - past_u_pos[(int)past_u_pos.size() - 2 - k].pose.x) / u_dt;
                            if (abs(dx) > 8.0)
                            {
                                u_vel.x = 8.0;
                            }
                            else
                            {
                                u_vel.x += dx;
                            }

                            double dy = (past_u_pos.back().pose.y - past_u_pos[(int)past_u_pos.size() - 2 - k].pose.y) / u_dt;
                            if (abs(dy) > 8.0)
                            {
                                u_vel.y = 8.0;
                            }
                            else
                            {
                                u_vel.y += dy;
                            }

                            double dz = (past_u_pos.back().pose.z - past_u_pos[(int)past_u_pos.size() - 2 - k].pose.z) / u_dt;
                            if (abs(dz) > 8.0)
                            {
                                u_vel.z = 8.0;
                            }
                            else
                            {
                                u_vel.z += dz;
                            }
                        }
                    }
                    u_vel.x = u_vel.x / 10.0;
                    u_vel.y = u_vel.y / 10.0;
                    u_vel.z = u_vel.z / 10.0;
                    ROS_WARN("Velocity x, y, z: %f, %f, %f", u_vel.x, u_vel.y, u_vel.z);

                   

                    if (filter_init_focal)
                    {
                        try
                        {
                            filter->H << 0, 1, 0, 0, 0, 0, 0,
                                                0, 0, 0, 1, 0, 0, 0,
                                                0, 0, 0, 0, 0, 1, 0;

                            R_t R;
                            R = Eigen::MatrixXd::Identity(3, 3) * 3.0;
                            Eigen::VectorXd z(3);
                            z(0) = u_vel.x;
                            z(1) = u_vel.y;
                            z(2) = u_vel.z;
                            filter_state_focal = filter->correct(filter_state_focal, z, R);
                            if (isnan(filter_state_focal.x(4)))
                            {
                                ROS_ERROR("Filter error on line 990");
                            }
                            last_meas_focal = ros::Time::now();
                        }
                        catch ([[maybe_unused]] std::exception e)
                        {
                            ROS_ERROR("LKF failed: %s", e.what());
                        }
                    }
                    geometry_msgs::Point vel;
                    vel.x = u_vel.x * cos(-focal_heading) - u_vel.y * sin(-focal_heading);
                    vel.y = u_vel.x * sin(-focal_heading) + u_vel.y * cos(-focal_heading);
                    vel.z = u_vel.z;
                    pub_velocity_uvdar_fcu.publish(vel);
                }
            }
        }

        double calculateDistance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
            double dx = point2.x - point1.x;
            double dy = point2.y - point1.y;
            double dz = point2.z - point1.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        double distanceForElevation(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2){
            double dx = point2.x - point1.x;
            double dy = point2.y - point1.y;
            return std::sqrt(dx*dx + dy*dy);
        }

        geometry_msgs::Point applyCoordinateTransformations(const geometry_msgs::Point& relativePos, const geometry_msgs::Quaternion& orientation) {
            // Convert the quaternion to a rotation matrix using Eigen
            Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
            Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
        
            // Apply the rotation to the relative position using the rotation matrix
            Eigen::Vector3d relativePosition(relativePos.x, relativePos.y, relativePos.z);
            Eigen::Vector3d transformedPosition = rotationMatrix * relativePosition;
        
            // Convert the transformed position back to geometry_msgs::Point type
            geometry_msgs::Point transformedPos;
            transformedPos.x = transformedPosition.x();
            transformedPos.y = transformedPosition.y();
            transformedPos.z = transformedPosition.z();
        
            return transformedPos;
        }

        geometry_msgs::Point calculateRelativePosition(const geometry_msgs::Point& uav1Pos, const geometry_msgs::Point& uav2Pos) {
            geometry_msgs::Point relativePos;
            relativePos.x = uav2Pos.x - uav1Pos.x;
            relativePos.y = uav2Pos.y - uav1Pos.y;
            relativePos.z = uav2Pos.z - uav1Pos.z;
            return relativePos;
        }

        void MainOdom(const nav_msgs::Odometry &msg)
        {
            focal_heading = mrs_lib::AttitudeConverter(msg.pose.pose.orientation).getHeading();
            focal_height = msg.pose.pose.position.z;

            focal_position = msg.pose.pose.position;
        }

        void ImuOdom(const sensor_msgs::Imu& msg){
            double az_raw = msg.linear_acceleration.z;
            double ax = msg.linear_acceleration.x * cos(focal_heading) - msg.linear_acceleration.y * sin(focal_heading);
            double ay = msg.linear_acceleration.x * sin(focal_heading) + msg.linear_acceleration.y * cos(focal_heading);
            

            double az_linear = az_raw - 9.8066;

            for(int i=0; i < 1; i++){
                vel_outlier.push_back(0.0);
                acc_outlier.push_back(0.0);
            }

            double dt = (ros::Time::now() - last_imu_update).toSec();

            // if(az_raw > 9.70 && az_raw < 9.90){
            //     az_linear = 0.0;
            // }

            if(acc_outlier.back() - az_linear > 3.0 || acc_outlier.back() - az_linear < -3.0){
                ROS_WARN("REPLACING acc %f with acc %f", az_linear, acc_outlier.back());
                az_linear = acc_outlier.back();
            }
            acc_outlier.push_back(az_linear);

            if(az_linear < 0.01 || az_linear > -0.01){
                az_linear = 0.0;
            }

            velocity_imu_focal.z =  velocity_imu_focal.z + az_linear * dt;
            velocity_imu_focal.x = velocity_imu_focal.x + ax * dt;
            velocity_imu_focal.y = velocity_imu_focal.y + ay * dt;
            last_imu_update = ros::Time::now();
            
            if(vel_outlier.back() - velocity_imu_focal.z > 2.0 || vel_outlier.back() - velocity_imu_focal.z < -2.0){
                ROS_WARN("!!!!Replacing vel %f with %f!!!!",velocity_imu_focal.z, vel_outlier.back());
                velocity_imu_focal.z = vel_outlier.back();
            }
            vel_outlier.push_back(velocity_imu_focal.z);
           
            if(velocity_imu_focal.z < 0.01 || velocity_imu_focal.z > - 0.01){
                velocity_imu_focal.z = 0.0;
            }

            geometry_msgs::Point imu_vel;
            imu_vel.x = velocity_imu_focal.x;
            imu_vel.y = velocity_imu_focal.y;
            imu_vel.z = velocity_imu_focal.z;
            pub_velocity_imu.publish(imu_vel);

            geometry_msgs::Point imu_acc;
            imu_acc.x = ax;
            imu_acc.y = ay;
            imu_acc.z = az_linear;
            pub_acc_imu.publish(imu_acc);

            if((ros::Time::now() - last_imu_correction).toSec() > 0.1 && (ros::Time::now() - last_imu_meas).toSec() > 0.25){
                if(filter_init_focal){

                     try {
                        filter->H << 0, 1, 0, 0, 0, 0, 0,
                                            0, 0, 0, 1, 0, 0, 0,
                                            0, 0, 0, 0, 0, 1, 0;

                        R_t R;

                        R = Eigen::MatrixXd::Identity(3,3) * 1;
                        Eigen::VectorXd z(3);
                        z(0) = velocity_imu_focal.x;
                        z(1) = velocity_imu_focal.y;
                        z(2) = velocity_imu_focal.z;
                        filter_state_focal = filter->correct(filter_state_focal, z, R);
                        if(isnan(filter_state_focal.x(4))){
                            ROS_ERROR("Filter error on line 1045");
                        }
                        last_meas_focal = ros::Time::now();
                        last_imu_meas = ros::Time::now();
                    }
                    catch ([[maybe_unused]] std::exception e){
                        ROS_ERROR("LKF failed: %s", e.what());
                    }

                }
            }
            if(filter_init_focal){
                if(!velocity_imuac_initialized){
                    last_imuac_meas = ros::Time::now();
                    velocity_imuac_initialized = true;
                }
                else {
                    if((ros::Time::now() - last_imuac_meas).toSec() > 0.05){
                        try {
                            filter->H <<0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 1;

                            R_t R;
                            R = Eigen::MatrixXd::Identity(3,3) * 1;
                            Eigen::VectorXd z(3);
                            z(0) = ax;
                            z(1) = ay;
                            z(2) = az_linear;
                            filter_state_focal = filter->correct(filter_state_focal, z, R);
                            if(isnan(filter_state_focal.x(4))){
                                ROS_ERROR("Filter error on line 1071");
                            }
                            last_meas_focal = ros::Time::now();
                            last_imuac_meas = ros::Time::now();
                        }
                        catch ([[maybe_unused]] std::exception e){
                            ROS_ERROR("LKF failed: %s", e.what());
                        }

                    }
                }
            }
        }
        
        void StateOdom(const mrs_msgs::UavState& msg){
            if(!velocity_imu_initialized){
                velocity_imu_focal.x = msg.velocity.linear.x;
                velocity_imu_focal.y = msg.velocity.linear.y;
                velocity_imu_focal.z = msg.velocity.linear.z;
                last_imu_correction = ros::Time::now();
                last_imu_update = ros::Time::now();
                velocity_imu_initialized = true;

            } else {
                if((ros::Time::now() - last_imu_correction).toSec() > 1.0){
                    velocity_imu_focal.x += (velocity_imu_focal.x + msg.velocity.linear.x)/2;
                    velocity_imu_focal.y += (velocity_imu_focal.y + msg.velocity.linear.y)/2;
                    velocity_imu_focal.z += (velocity_imu_focal.z + msg.velocity.linear.z)/2;
                    last_imu_correction = ros::Time::now();
                    last_imu_update = ros::Time::now();
                }
            }
        }

        void GarminRange(const sensor_msgs::Range &rangemsg)
        {   
            rmsg = rangemsg;
            rangemsg_vector.push_back(rangemsg);
            if(rmsg.range == -INFINITY || rmsg.range == INFINITY){
                ROS_ERROR("!!!!One outlier inf detected!!!!!");
                std::vector<sensor_msgs::Range>::iterator itr = rangemsg_vector.end() - 2;
                if((*itr).range == -INFINITY || (*itr).range == INFINITY){
                    ROS_ERROR("Two consecutive inf values");
                }
                else {
                    rmsg.range = (*itr).range;
                }
            }

            sensor_msgs::Range tf_rangemsg = invTransformRangeMessage(rmsg);
            pub_garmin_transform.publish(tf_rangemsg);

            

            //Garmin frame is uav29/garmin. Have to transform to uav29/gps_origin before making filter correction.
            // try 
            // {
            //     filter->H << 0, 0, 0, 0, 1, 0, 0,
            //                         0, 0, 0, 0, 0, 0, 0,
            //                       0, 0, 0, 0, 0, 0, 0;

            //     R_t R;
            //     R = Eigen::MatrixXd::Identity(3,3) * 0.1;
            //     Eigen::VectorXd z(3);
            //     z(0) = tf_rangemsg.range;
            //     filter_state_focal = filter->correct(filter_state_focal, z, R);
            //     if(isnan(filter_state_focal.x(4))){
            //         ROS_ERROR("Filter error on line 1740");
            //     }
            //     last_meas_focal = ros::Time::now();
            // }
            // catch([[maybe_unused]] std::exception e){
            //     ROS_ERROR("LKF failed: %s", e.what());
            // } 
        }
        //}
        //}

        double Quat2Eul(const geometry_msgs::Quaternion quat){
            
            auto q = mrs_lib::AttitudeConverter(quat);
            
            geometry_msgs::Point pitch_test;
            pitch_test.x = q.getPitch()*(180/M_PI);
            
            // if(pitch_flag){
                pub_pitch.publish(pitch_test);
            // }            

            return q.getPitch();

        }

        void MassEstimate(const std_msgs::Float64 mass){
            mass_estimate.data = mass.data;
        }

        void ThrustCorrection(const mrs_msgs::Float64Stamped thrust) {
            
            double thrust_force = thrust.value;

            double thrust_acc = thrust.value/mass_estimate.data;
            
            double thrust_acc_corrected = thrust_acc - 9.8066;
            if(thrust_acc_corrected < -1.0){
               return;
            }

            if(thrust_acc_corrected < 0.01 || thrust_acc_corrected > -0.01){
                thrust_acc_corrected = 0.0;
            }

            geometry_msgs::Point th_debug;
            th_debug.z = thrust_acc_corrected;
            th_debug.y = thrust_acc;

            pub_thrust_debug.publish(th_debug);

            try {
                        filter->H << 0, 0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0;
                        R_t R;
                        R = Eigen::MatrixXd::Identity(3,3) * 1;
                        Eigen::VectorXd z(3);
                        z(0) = thrust_acc_corrected;

                        filter_state_focal = filter->correct(filter_state_focal, z, R);
                        if(isnan(filter_state_focal.x(4))){
                            ROS_ERROR("Filter error on line 1606");
                        }
                        
                        last_meas_focal = ros::Time::now();
            } 
            catch([[maybe_unused]] std::exception e){
                ROS_ERROR("LKF failed: %s", e.what());
            }
        }

        sensor_msgs::Range transformRangeMessage(const sensor_msgs::Range &range_msg){
            sensor_msgs::Range range_msg_local = range_msg;
            tf::StampedTransform tf2gf_;
            listener_.waitForTransform(garmin_frame, range_msg_local.header.frame_id, ros::Time(0), ros::Duration(3.0));

            try {
                listener_.lookupTransform(garmin_frame, range_msg_local.header.frame_id, ros::Time(0), tf2gf_);

            }
            catch(tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ROS_WARN("NO RANGE TRANSFORM!");
                return range_msg_local;
            }
            tf::Vector3 original_pos(range_msg_local.range, 0, 0);
            tf::Vector3 transformed_pos = tf2gf_ * original_pos;

            sensor_msgs::Range transformed_range_msg;
            

            transformed_range_msg.header.frame_id = garmin_frame;
            // transformed_range_msg.header.stamp = range_msg_local.header.stamp;
            transformed_range_msg.radiation_type = range_msg_local.radiation_type;
            transformed_range_msg.min_range = range_msg_local.min_range;
            transformed_range_msg.max_range = range_msg_local.max_range;
            transformed_range_msg.field_of_view = range_msg_local.field_of_view;

            

            transformed_range_msg.range = transformed_pos.getX();

            return transformed_range_msg;
        }

        sensor_msgs::Range invTransformRangeMessage(const sensor_msgs::Range &garmin_range){
            sensor_msgs::Range garmin_range_local = garmin_range;
            tf::StampedTransform tf2invgf_;
            listener_.waitForTransform(estimation_frame, garmin_range_local.header.frame_id, ros::Time(0), ros::Duration(3.0));

            try{
                listener_.lookupTransform(estimation_frame, garmin_range_local.header.frame_id, ros::Time(0), tf2invgf_);
            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
                ROS_WARN("No garmin_correction transform!");
                return garmin_range_local;
            }

            sensor_msgs::Range tf_garmin_range;

            tf_garmin_range.header.frame_id = estimation_frame;
            // tf_garmin_range.header.stamp = garmin_range_local.header.stamp;
            tf_garmin_range.radiation_type = garmin_range_local.radiation_type;
            tf_garmin_range.max_range = garmin_range_local.max_range;
            tf_garmin_range.min_range = garmin_range_local.min_range;
            tf_garmin_range.field_of_view = garmin_range_local.field_of_view;

            tf::Vector3 original_reading( 0 , 0, garmin_range_local.range);
            tf::Vector3 transformed_reading = tf2invgf_ * original_reading;

            tf_garmin_range.range = transformed_reading.z();

            return tf_garmin_range;
        }

        void dynamicReconfigureCallback(vertical_estimator::rangeTopicSwitcherConfig& config, uint32_t level){
            use_garmin = config.use_garmin_source;
            use_estimator = config.use_estimator_source;
        }

        sensor_msgs::Range smoothRange(const sensor_msgs::Range& msg, double alpha) {
            static double filteredValue = 0.0;
        
            filteredValue = alpha * msg.range + (1 - alpha) * filteredValue;
        
            sensor_msgs::Range smoothedMsg = msg;
            smoothedMsg.range = filteredValue;
        
            return smoothedMsg;
        }

        // mrs_msgs::Float64Stamped transformHeightMessage(const mrs_msgs::Float64Stamped &height_msg, const std::string target_frame){
        //     listener_.waitForTransform(target_frame, height_msg.header.frame_id, ros::Time(0), ros::Duration(2.0));

        //     try {
        //         listener_.lookupTransform(target_frame, height_msg.header.frame_id, ros::Time(0), tf2hf_);
        //     } 
        //     catch(tf::TransformException &ex) {
        //         ROS_ERROR("%s", ex.what());
        //         ROS_WARN("No Height Transform");
        //         return height_msg;
        //     }

        //     mrs_msgs::Float64Stamped transformed_height_msg;

        //     transformed_height_msg.header.frame_id = target_frame;

        //     tf::Vector3 original_position(height_msg.value, 0, 0);
        //     tf::Vector3 transformed_position = tf2hf_ * original_position;

        //     transformed_height_msg.value = transformed_position.x();

        //     return transformed_height_msg; 
        // }




    private:
        /* Global variables //{ */
        /* ROS variables, topics and global bools //{ */
        bool initialized_ = false;

        double main_rate = 0.01;
        ros::Timer timer_publisher_;
        ros::Timer timer_debug_;

        ros::Publisher pub_debug;
        ros::Publisher pub_debug_position;

        ros::Publisher pub_velocity;

        ros::Publisher pub_acc_imu;
        ros::Publisher pub_velocity_imu;
        ros::Publisher pub_velocity_uvdar;
        ros::Publisher pub_estimator_output;

        ros::Publisher pub_velocity_imu_fcu;
        ros::Publisher pub_velocity_uvdar_fcu;
        ros::Publisher pub_vertical_estimator_output_fcu;

        ros::Publisher pub_vertical_estimator_output;
        
        ros::Publisher pub_garmin_transform;

        ros::Publisher pub_ground_truth;
        ros::Publisher pub_uvdar_pos_debug;

        ros::Subscriber sub_main_odom;
        ros::Subscriber sub_state_odom;
        ros::Subscriber sub_imu_odom;
        std::vector<ros::Subscriber> sub_uvdar_measurements;

        ros::Subscriber sub_garmin_range;
        ros::Subscriber sub_attitude_cmd;
        ros::Subscriber sub_ground_truth;

        std::string odom_main_topic;
        std::string odom_state_topic;
        std::string odom_imu_topic;

        std::vector<std::string> odom_main_topics;

        std::vector<std::string> height_topics;

        std::vector<std::string> measured_poses_topics;

        std::string filtered_poses_topics;

        using nb_state_callback = boost::function<void(const nav_msgs::OdometryConstPtr &)>;
        std::vector<nb_state_callback> callbacks_nb_state;
        std::vector<ros::Subscriber> sub_nb_state;

        using nb_height_callback = boost::function<void(const mrs_msgs::Float64StampedConstPtr &)>;
        std::vector<nb_height_callback> callbacks_nb_height;
        std::vector<ros::Subscriber> sub_nb_height;

        tf::TransformListener listener_;

        std::shared_ptr<mrs_lib::Transformer> transformer_;
        std::optional<geometry_msgs::TransformStamped> tf2bf_;
        std::optional<geometry_msgs::TransformStamped> tf2lf_;

        // tf::StampedTransform tf2gf_;
        // tf::StampedTransform tf2hf_;
        // tf::StampedTransform tf2invgf_;
        //}

        double def_dt = 0.1;

        std::unique_ptr<mrs_lib::nblkf_t> filter;

        A_t A;
        B_t B;
        H_t H;
        Q_t Qq;
        //}

        /* Global variables of focal UAV and neighbors //{ */
        std::string uav_name;
        std::string estimation_frame;
        std::vector<int> uavs_ids;
        int virt_id;
        std::vector<int> lut_id;
        std::vector<int> uv_ids;

        bool vio_frame = false;

        double focal_heading = 0;
        double focal_height = 0;
        geometry_msgs::Point focal_position; // for mrse debug

        nbstatecov_t filter_state_focal;
        
        bool filter_valid = false;
        bool filter_init_focal = false;
        ros::Time last_updt_focal;
        ros::Time last_meas_focal;
        ros::Time last_meas_focal_main;

        geometry_msgs::Point velocity_imu_focal;
        ros::Time last_imu_correction;
        ros::Time last_imu_update;
        ros::Time last_imu_meas;
        bool velocity_imu_initialized = false;

        ros::Time last_imuac_meas;
        bool velocity_imuac_initialized = false;

        struct Debug
        {
            int id;
            bool valid;
            ros::Time last_debug;
        };

        std::vector<Debug> past_spheres;

        struct U_POS
        {
            geometry_msgs::Point pose;
            ros::Time t;
        };

        std::vector<U_POS> past_u_pos;

        struct Neighbor
        {
            bool focal = false;
            int virt_id;
            int uv_id;
            int name_id;
            std::string uav_name;

            geometry_msgs::Point pfcu;
            geometry_msgs::Point eigens;
            ros::Time last_pfcu;
            double hdg_at_last_pfcu;
            bool pfcu_init;

            ros::Time last_updt;
            ros::Time last_meas_u;
            ros::Time last_meas_s;

            bool filter_init;

            nbstatecov_t filter_state;

            double angle_z;

            geometry_msgs::Quaternion quat;

            geometry_msgs::Point focal_dist;
        };

        std::vector<Neighbor> agents;

        std::string range_topic;

        std::string range2_publish_topic;

        

        struct Intersection
        {
            geometry_msgs::Point ints;
            int ints_count;
        };

        std::vector<sensor_msgs::Range> rangemsg_vector;
        
        struct EulAng
        {
            double roll;
            double pitch;
            double yaw;
        };
        EulAng q2e;

        std::vector<double> uvdar_pos;

        std::vector<double> vel_outlier;
        std::vector<double> acc_outlier;

        std::string cmd_odom_topic;

        ros::Publisher pub_thrust_debug;
        
        struct thrustandMass {
            double thrust;
            double mass;
        };

        ros::Publisher pub_uvdar_debug;

        ros::Publisher pub_pitch;

        geometry_msgs::Quaternion odom_orientation;

        std::string body_frame;

        std_msgs::Float64 mass_estimate;

        std::string mass_estimate_topic;
        std::string thrust_force_topic;

        ros::Subscriber sub_mass_estimate;
        ros::Subscriber sub_thrust_force;

        ros::Publisher pub_quat;
        bool pitch_flag;
        
        //}
        //}
        sensor_msgs::Range rmsg;

        std::string garmin_frame;
    
        std::string height_topic;


        dynamic_reconfigure::Server<vertical_estimator::rangeTopicSwitcherConfig> dyn_reconf_server;

        bool use_garmin;
        bool use_estimator;

        ros::Publisher pub_transformed_estimator;
    };

} // namespace vertical_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vertical_estimator::VerticalEstimator, nodelet::Nodelet)
