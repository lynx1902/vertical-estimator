/* Includes //{ */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/UavState.h>
#include <sensor_msgs/Imu.h>
#include <mrs_lib/lkf.h>

#include <sensor_msgs/Range.h>

#define Q_c 6.35

/* Filter preliminary //{ */
#define Po 0 // position
#define Ve 1 // velocity
#define Ac 2 // acceleration

namespace mrs_lib
{
    const int n_states = 6;
    const int n_inputs = 1;
    const int n_measurements = 2;
    using lkf_t = LKF<n_states, n_inputs, n_measurements>;
}

using A_t = mrs_lib::lkf_t::A_t;
using B_t = mrs_lib::lkf_t::B_t;
using H_t = mrs_lib::lkf_t::H_t;
using Q_t = mrs_lib::lkf_t::Q_t;
using u_t = mrs_lib::lkf_t::u_t;

using x_t = mrs_lib::lkf_t::x_t;
using P_t = mrs_lib::lkf_t::P_t;
using R_t = mrs_lib::lkf_t::R_t;

using statecov_t = mrs_lib::lkf_t::statecov_t;
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
            param_loader.loadParam("velocity_reduced_topic", velocity_reduced_topic);
            param_loader.loadParam("odom_main_topic", odom_main_topic);
            // param_loader.loadParam("odom_imu_topic", odom_imu_topic);
            param_loader.loadParam("odom_state_topic", odom_state_topic);
            // param_loader.loadParam("vio_frame", vio_frame); /* using only gps for initial implementation */
            param_loader.loadParam("measured_poses_topics", measured_poses_topics, measured_poses_topics);

            estimation_frame = uav_name + "/gps_origin";

            //}

            /* Transformer and Timers //{ */
            transformer_ = std::make_shared<mrs_lib::Transformer>("VerticalEstimator");
            transformer_->setDefaultPrefix(uav_name);

            timer_publisher_ = nh_.createTimer(ros::Duration(main_rate), &VerticalEstimator::TimerMain, this, false);
            // timer_debug_ = nh_.createTimer(ros::Duration(0.1), &VerticalEstimator::TimerDebug, this, false);
            //}

            /* Services, subscribers and publishers //{ */
            /* what is the purpose? Visualisation in rviz?? */
            pub_debug = nh_.advertise<visualization_msgs::MarkerArray>("debug", 1);
            // pub_debug_position = nh_.advertise<visualization_msgs::Marker>("debug_position", 1);
            pub_velocity = nh_.advertise<geometry_msgs::Point>("velocity", 1);

            pub_velocity_uvdar = nh_.advertise<geometry_msgs::Point>("velocity_uvdar", 1);
            // pub_velocity_imu = nh_.advertise<geometry_msgs::Point>("velocity_imu", 1);
            pub_estimator_output = nh_.advertise<nav_msgs::Odometry>("estimator_output", 1); /*estimator output with position and velocity*/
            pub_vert_estimator_output = nh_.advertise<sensor_msgs::Range>("range2", 1);

            pub_velocity_uvdar_fcu = nh_.advertise<geometry_msgs::Point>("velocity_uvdar_fcu", 1);
            // pub_velocity_imu_fcu = nh_.advertise<geometry_msgs::Point>("velocity_imu_fcu", 1);
            pub_estimator_output_fcu = nh_.advertise<geometry_msgs::Point>("estimator_output_fcu", 1); /*estimator position output publisher*/

            // sub_main_odom = nh_.subscribe(odom_main_topic, 1, &VerticalEstimator::MainOdom, this);
            // sub_imu_odom = nh_.subscribe(odom_imu_topic, 1, &VerticalEstimator::ImuOdom, this);
            // sub_state_odom = nh_.subscribe(odom_state_topic, 1, &VerticalEstimator::StateOdom, this);
            sub_garmin_range = nh_.subscribe("range", 1, &VerticalEstimator::GarminRange, this);

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

                std::string new_topic_state = "/" + new_nb.uav_name + velocity_reduced_topic;
                velocity_reduced_topics.push_back(new_topic_state);

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
                    is_member = true;

                ROS_INFO("[DataCollector]: V_ID: %d, N_ID: %d, U_ID:, %d, name: %s, focal: %d", agents[i].virt_id, agents[i].name_id, agents[i].uv_id, agents[i].uav_name.c_str(), agents[i].focal);

                if (i + 1 >= (int)agents.size())
                    continue;

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
                { NeighborsStateReduced(stateMessage, i); };
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
                sub_uvdar_measurements.push_back(nh_.subscribe(topic, 3, &VerticalEstimator::callbackUvdarMeasurement, this, ros::TransportHints().tcpNoDelay()));
            }
            //}

            /* Filter initialization //{ */
            A.resize(3, 3);
            B.resize(3, 1);
            // H.resize(2, 3);
            Qq.resize(3, 3);

            // A << 1, 0, def_dt, 0, def_dt * def_dt / 2, 0,
            //     0, 1, 0, def_dt, 0, def_dt * def_dt / 2,
            //     0, 0, 1, 0, def_dt, 0,
            //     0, 0, 0, 1, 0, def_dt,
            //     0, 0, 0, 0, 1, 0,
            //     0, 0, 0, 0, 0, 1;

            A << 1, def_dt, def_dt * def_dt / 2.0,
                0, 1, def_dt,
                0, 0, 1;

            // B << 0,
            //     0,
            //     0,
            //     0,
            //     0,
            //     0;

            B << 0,
                0,
                0;

            // H << 1, 0, 0, 0, 0, 0,
            //     0, 1, 0, 0, 0, 0;

            H << 1, 0, 0,
                1, 0, 0;

            /* Qq << */
            /*   10.0, 0, 0, 0, 0, 0, */
            /*   0, 10.0, 0, 0, 0, 0, */
            /*   0, 0, 1.0, 0, 0, 0, */
            /*   0, 0, 0, 1.0, 0, 0, */
            /*   0, 0, 0, 0, 0.1, 0, */
            /*   0, 0, 0, 0, 0, 0.1; */

            // Qq << 1.0, 0, 0, 0, 0, 0,
            //     0, 1.0, 0, 0, 0, 0,
            //     0, 0, 1.0, 0, 0, 0,
            //     0, 0, 0, 1.0, 0, 0,
            //     0, 0, 0, 0, 1.0, 0,
            //     0, 0, 0, 0, 0, 1.0;

            Qq << 1.0, 0, 0,
                0, 1.0, 0,
                0, 0, 1.0;

            /* what is std::make_unique*/
            filter = std::make_unique<mrs_lib::lkf_t>(A, B, H);

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
                /* ROS_INFO("[VerticalEstimator]: Spinning"); */

                /* getIMUvelocity(); */

                ROS_WARN_THROTTLE(3.0, "Estimation frame %s", estimation_frame.c_str());

                for (auto &nb : agents)
                {
                    if (!nb.filter_init || nb.focal)
                    {
                        continue;
                    }

                    if ((ros::Time::now() - nb.last_meas_u).toSec() > 8.0)
                    {
                        nb.filter_init = false;
                        ROS_WARN_THROTTLE(0.5, "Agent data not updated for a long time. ID: %d NAME: %s", nb.virt_id, nb.uav_name.c_str());
                        continue;
                    }
                    if ((ros::Time::now() - nb.last_meas_s).toSec() > 3.0)
                    {
                        nb.filter_init = false;
                        ROS_WARN_THROTTLE(0.5, "Agent data not updated for a long time. ID: %d NAME: %s", nb.virt_id, nb.uav_name.c_str());
                        continue;
                    }
                    /* ROS_INFO("ID: %d,time: %f, bearing: %f", nb.name_id, (ros::Time::now() - nb.last_pfcu).toSec(), nb.pfcu.z); */
                }
                /* ROS_INFO("-------------------"); */

                const u_t u = u_t::Random();

                double new_dt = std::fmax(std::fmin((ros::Time::now() - last_updt_focal).toSec(), (ros::Time::now() - last_meas_focal).toSec()), 0.0);
                filter->A = A_dt(new_dt);
                filter_state_focal = filter->predict(filter_state_focal, u, Qq, new_dt);
                last_updt_focal = ros::Time::now();

                if ((ros::Time::now() - last_meas_focal_main).toSec() < 10.0)
                {
                    filter_valid = true;
                }
                else
                {
                    filter_valid = false;
                }

                /*publishing estimator output*/
                // nav_msgs::Odometry est_output;
                // est_output.header.stamp = ros::Time::now();
                // /* est_output.header.frame_id = uav_name + "/vio_origin"; */
                // est_output.header.frame_id = estimation_frame;
                // est_output.pose.pose.position.x = filter_state_focal.x(0);
                // est_output.pose.pose.position.y = filter_state_focal.x(1);
                // est_output.twist.twist.linear.x = filter_state_focal.x(2);
                // est_output.twist.twist.linear.y = filter_state_focal.x(3);
                // est_output.pose.pose.orientation.w = (int)filter_valid;
                // pub_estimator_output.publish(est_output);

                geometry_msgs::Point vel;
                vel.x = filter_state_focal.x(2) * cos(-focal_heading) - filter_state_focal.x(3) * sin(-focal_heading);
                vel.y = filter_state_focal.x(2) * sin(-focal_heading) + filter_state_focal.x(3) * cos(-focal_heading);
                pub_estimator_output_fcu.publish(vel);

                sensor_msgs::Range vert_est_output;
                vert_est_output.header.stamp = ros::Time::now();
                vert_est_output.header.frame_id = estimation_frame;

                vert_est_output.range = filter_state_focal.x(0);
                pub_estimator_output.publish(vert_est_output);

                // xxx
                ROS_INFO_THROTTLE(0.25, "H: %f, H_dt: %f, H_ddt: %f", filter_state_focal.x(0), filter_state_focal.x(1), filter_state_focal.x(2));
            }
        }
        //}

        /* Filter matrices update //{ */

        A_t A_dt(double dt)
        {
            // A << 1, 0, dt, 0, dt * dt / 2, 0,
            //     0, 1, 0, dt, 0, dt * dt / 2,
            //     0, 0, 1, 0, dt, 0,
            //     0, 0, 0, 1, 0, dt,
            //     0, 0, 0, 0, 1, 0,
            //     0, 0, 0, 0, 0, 1;
            A << 1, def_dt, def_dt * def_dt / 2,
                0, 1, def_dt,
                0, 0, 1;
            return A;
        }

        H_t H_n(int type)
        {
            if (type == 0)
            {
                // H << 1, 0, 0, 0, 0, 0,
                //     0, 1, 0, 0, 0, 0;
                H << 1, 0, 0;
            }
            else if (type == 1)
            {
                // H << 0, 0, 1, 0, 0, 0,
                //     0, 0, 0, 1, 0, 0;
                H << 0, 1, 0;
            }
            // else if (type == 2)
            // {
            //     H << 0, 0, 0, 0, 1, 0,
            //         0, 0, 0, 0, 0, 1;
            // }
            else
            {
                ROS_WARN_THROTTLE(1.0, "Wrong measurement matrix type");
            }
            return H;
        }

        //}

        /* Covariance to eigen method //{ */

        Eigen::MatrixXd rosCovarianceToEigen(const boost::array<double, 9> input)
        {
            Eigen::MatrixXd output(3, 3);

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    output(j, i) = input[3 * j + i];
                }
            }

            return output;
        }
        //}

        /* Cooperative subscribers //{ */
        /* Subscriber of neighbors derivative states //{ */
        // todo do it without communication
        void NeighborsStateReduced(const nav_msgs::OdometryConstPtr &odom_msg, size_t nb_index)
        {
            if (virt_id == (int)nb_index)
                return;

            if (!agents[nb_index].filter_init)
                return;

            u_t u = u_t::Zero();

            double new_dt = std::fmax(std::fmin(std::fmin((ros::Time::now() - agents[nb_index].last_updt).toSec(), (ros::Time::now() - agents[nb_index].last_meas_u).toSec()), (ros::Time::now() - agents[nb_index].last_meas_s).toSec()), 0.0);
            filter->A = A_dt(new_dt);
            agents[nb_index].filter_state = filter->predict(agents[nb_index].filter_state, u, Qq, new_dt);
            agents[nb_index].last_updt = ros::Time::now();

            try
            {
                filter->H = H_n(Ve);
                R_t R;
                R << Eigen::MatrixXd::Identity(2, 2) * 1;
                Eigen::VectorXd z(1);
                z(0) = odom_msg->twist.twist.linear.z;

                agents[nb_index].filter_state = filter->correct(agents[nb_index].filter_state, z, R);
                agents[nb_index].last_meas_s = ros::Time::now();
            }
            catch ([[maybe_unused]] std::exception e)
            {
                ROS_ERROR("LKF failed: %s", e.what());
            }
        }
        //}

        void callbackUvdarMeasurement(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
        {

            if ((int)(msg.poses.size()) < 1)
                return;

            mrs_msgs::PoseWithCovarianceArrayStamped msg_local = msg;

            std::string output_frame = estimation_frame;

            tf2lf_ = transformer_->getTransform(msg_local.header.frame_id, output_frame, ros::Time(0));
            if (!tf2lf_)
            {
                ROS_ERROR("[UVDARKalman]: Could not obtain transform from %s to %s", msg_local.header.frame_id.c_str(), output_frame.c_str());
                return;
            }

            output_frame = uav_name + "/fcu_untilted";
            tf2bf_ = transformer_->getTransform(msg_local.header.frame_id, output_frame, ros::Time(0));
            if (!tf2bf_)
            {
                ROS_ERROR("[UVDARKalman]: Could not obtain transform from %s to %s", msg_local.header.frame_id.c_str(), output_frame.c_str());
                return;
            }

            geometry_msgs::Point sum_of_ints;

            for (auto &meas : msg_local.poses)
            {
                geometry_msgs::PoseWithCovarianceStamped meas_s;
                meas_s.pose.pose = meas.pose;
                meas_s.pose.covariance = meas.covariance;
                meas_s.header = msg_local.header;

                if ((int)meas.id < 0 || (int)meas.id >= (int)lut_id.size() || !std::count(uv_ids.begin(), uv_ids.end(), (int)meas.id))
                {
                    ROS_ERROR("Wrong UVDAR ID %d", (int)meas.id);
                    continue;
                }
                int aid = lut_id[(int)meas.id];
                auto res_l_ = transformer_->transform(meas_s, tf2lf_.value());
                auto res_b_ = transformer_->transform(meas_s, tf2bf_.value());

                if (res_b_)
                {

                    /* if(agents[aid].filter_init && eigvals_b(0).real() < 500 && eigvals_b(1).real() < 500 && eigvals_b(2).real() < 500){ */
                    if (agents[aid].filter_init)
                    {

                        // agents[aid].pfcu.x = res_b_.value().pose.pose.position.x;
                        // agents[aid].pfcu.y = res_b_.value().pose.pose.position.y;
                        agents[aid].pfcu.z = res_b_.value().pose.pose.position.z;
                        agents[aid].last_pfcu = ros::Time::now();
                        // agents[aid].hdg_at_last_pfcu = focal_heading;
                        agents[aid].pfcu_init = true;

                        for (auto &nb : agents)
                        {
                            if (nb.focal || !nb.pfcu_init || nb.virt_id == agents[aid].virt_id)
                                continue;

                            if (!filter_init_focal)
                            {
                                Eigen::VectorXd poseVec(3);
                                poseVec(0) = 4; /*important part to figure out. Intersection of all poses to find actual coordinates*/
                                poseVec(1) = 0;
                                poseVec(2) = 0;
                                Eigen::MatrixXd poseCov(3, 3);
                                poseCov << Eigen::MatrixXd::Identity(3, 3);

                                ROS_INFO("LKF initialized for the focal UAV at %f, %f of GPS frame.", poseVec(0), poseVec(1));

                                last_meas_focal = ros::Time::now();
                                last_meas_focal_main = ros::Time::now();
                                last_updt_focal = ros::Time::now();

                                filter_init_focal = true;
                            }
                            else
                            {
                                try
                                {
                                    filter->H = H_n(Po);

                                    R_t R;
                                    R << Eigen::MatrixXd::Identity(2, 2) * 0.1;
                                    Eigen::VectorXd z(1);
                                    z(0) = 4; /* need intersection point of all poses*/

                                    filter_state_focal = filter->correct(filter_state_focal, z, R);
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
                        ROS_WARN("[StateEstimator]: Filter of uav%d not initialized.", agents[aid].name_id);
                    }
                }
                else
                {
                    ROS_INFO_STREAM("[StateEstimator]: Failed to get transformation for tf2bf measurement, returning.");
                    /* return; */
                }

                if (res_l_)
                {
                    if (!agents[aid].filter_init)
                    {
                        Eigen::VectorXd poseVec(3);
                        poseVec(0) = res_l_.value().pose.pose.position.z;
                        poseVec(1) = 0;
                        poseVec(2) = 0;
                        Eigen::MatrixXd poseCov(3, 3);
                        poseCov << Eigen::MatrixXd::Identity(3, 3);

                        ROS_INFO("LKF initialized for UAV%d with UVDAR reloc at %f, %f of gps frame.", agents[aid].name_id, poseVec(0), poseVec(1));

                        agents[aid].last_meas_u = ros::Time::now();
                        agents[aid].last_meas_s = ros::Time::now();
                        agents[aid].last_updt = ros::Time::now();
                        agents[aid].filter_state = {.x = poseVec, .P = poseCov};
                        agents[aid].filter_init = true;
                    }
                    else
                    {
                        if ((ros::Time::now() - agents[aid].last_meas_u).toSec() >= 4.0)
                        {
                            try
                            {
                                filter->H = H_n(Po);

                                R_t R;
                                R << Eigen::MatrixXd::Identity(2, 2) * 0.00001;
                                Eigen::VectorXd z(1);
                                z(0) = res_l_.value().pose.pose.position.z;
                                agents[aid].filter_state = filter->correct(agents[aid].filter_state, z, R);
                                agents[aid].last_meas_u = ros::Time::now();

                                ROS_WARN("Agent state updated uav%d", agents[aid].name_id);
                            }
                            catch ([[maybe_unused]] std::exception e)
                            {
                                ROS_ERROR("LKF failed: %s", e.what());
                            }
                        }
                        else {
                            ROS_INFO_STREAM("[StateEstimator]: Failed to get transformation for tf2lf measurement, returning.");

                        }
                    }
                }
                /*implementation of velocity via UVDAR is remaining*/
            }
        }
        void GarminRange(const sensor_msgs::Range &rangemsg)
        {
            sensor_msgs::Range ran = rangemsg;
        }
        //}
        //}

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

        ros::Publisher pub_velocity_imu;
        ros::Publisher pub_velocity_uvdar;
        ros::Publisher pub_estimator_output;

        ros::Publisher pub_velocity_imu_fcu;
        ros::Publisher pub_velocity_uvdar_fcu;
        ros::Publisher pub_estimator_output_fcu;

        ros::Publisher pub_vert_estimator_output;

        ros::Subscriber sub_main_odom;
        ros::Subscriber sub_state_odom;
        ros::Subscriber sub_imu_odom;
        std::vector<ros::Subscriber> sub_uvdar_measurements;

        ros::Subscriber sub_garmin_range;

        std::string odom_main_topic;
        std::string odom_state_topic;
        std::string odom_imu_topic;
        std::string velocity_reduced_topic;
        std::vector<std::string> velocity_reduced_topics;

        std::vector<std::string> odom_main_topics;

        std::vector<std::string> measured_poses_topics;

        using nb_state_callback = boost::function<void(const geometry_msgs::PointConstPtr &)>;
        std::vector<nb_state_callback> callbacks_nb_state;
        std::vector<ros::Subscriber> sub_nb_state;

        std::shared_ptr<mrs_lib::Transformer> transformer_;
        std::optional<geometry_msgs::TransformStamped> tf2bf_;
        std::optional<geometry_msgs::TransformStamped> tf2lf_;
        //}

        /* LKF global variables //{ */
        std::shared_ptr<mrs_lib::lkf_t> filter;

        A_t A;
        B_t B;
        H_t H;
        Q_t Qq;

        double def_dt = 0.1;
        //}

        /* Global variables of focal UAV and neighbors //{ */
        std::string uav_name;
        std::string estimation_frame;
        std::vector<int> uavs_ids;
        int virt_id;
        std::vector<int> lut_id;
        std::vector<int> uv_ids;

        bool vio_frame = true;

        double focal_heading = 0;
        double focal_height = 0;
        geometry_msgs::Point focal_position; // for mrse debug

        statecov_t filter_state_focal;
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
            statecov_t filter_state;
        };

        std::vector<Neighbor> agents;

        //}
        //}
    };

} // namespace vertical_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vertical_estimator::VerticalEstimator, nodelet::Nodelet);
