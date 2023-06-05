/* Includes // */
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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>



/* Filter preliminary //{ */
#define Po 0 // position
#define Ve 1 // velocity
#define Ac 2 // acceleration

namespace mrs_lib
{
    const int n_states = 3;
    const int n_inputs = 1;
    const int n_measurements = 1;
    using lkf_t = LKF<n_states, n_inputs, n_measurements>;
} // namespace mrs_lib

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
            param_loader.loadParam("odom_main_topic", odom_main_topic);
            param_loader.loadParam("odom_state_topic", odom_state_topic);
            param_loader.loadParam("odom_imu_topic", odom_imu_topic);
            param_loader.loadParam("measured_poses_topics", measured_poses_topics, measured_poses_topics);

            param_loader.loadParam("range_topic", range_topic);
            param_loader.loadParam("range2_publish_topic", range2_publish_topic);

            estimation_frame = uav_name + "/gps_origin";

            //}

            /* Transformer and Timers //{ */
            transformer_ = std::make_shared<mrs_lib::Transformer>("VerticalEstimator");
            transformer_->setDefaultPrefix(uav_name);

            timer_publisher_ = nh_.createTimer(ros::Duration(main_rate), &VerticalEstimator::TimerMain, this, false);

            // Publishers
            pub_vert_estimator_output = nh_.advertise<sensor_msgs::Range>("vert_est", 1); /*vert estimator*/

            pub_velocity_uvdar_fcu = nh_.advertise<geometry_msgs::Point>("velocity_uvdar_fcu", 1);
            pub_vert_estimator_output_fcu = nh_.advertise<geometry_msgs::Point>("vert_estimator_output_fcu",1);

            pub_uvdar_pos_check = nh_.advertise<geometry_msgs::Point>("uvdar_pos_check",1);

            // Subscribers
            sub_garmin_range = nh_.subscribe(range_topic, 1, &VerticalEstimator::GarminRange, this);
            
            sub_main_odom = nh_.subscribe(odom_main_topic, 1, &VerticalEstimator::MainOdom, this );
            sub_imu_odom = nh_.subscribe(odom_imu_topic, 1, &VerticalEstimator::ImuOdom, this);
            sub_state_odom = nh_.subscribe(odom_state_topic, 1, &VerticalEstimator::StateOdom, this);
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
                std::cout << new_odom_topic << std::endl;
                odom_main_topics.push_back(new_odom_topic);
                

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

            /* Filter initialization //{ */
            A.resize(3, 3);
            B.resize(3, 1);
            H.resize(1, 3);
            Qq.resize(3, 3);

            A << 1, def_dt, def_dt * def_dt / 2.0,
                0, 1, def_dt,
                0, 0, 1;

            B << 0,
                0,
                0;

            H << 1, 0, 0;

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
                // ROS_INFO("[VerticalEstimator]: Spinning");

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

                double new_dt = std::fmax(
                    std::fmin((ros::Time::now() - last_updt_focal).toSec(), (ros::Time::now() - last_meas_focal).toSec()), 0.0);
                filter->A = A_dt(new_dt);
                
                filter_state_focal = filter->predict(filter_state_focal, u, Qq, new_dt);

                if (std::isnan(filter_state_focal.x(0)))
                {
                    ROS_ERROR("Filter error on line 328");
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

                vert_est_output.range = filter_state_focal.x(0);
                pub_vert_estimator_output.publish(vert_est_output);

                geometry_msgs::Point velo;
                velo.z = filter_state_focal.x(1);
                pub_vert_estimator_output_fcu.publish(velo);

                // std::string output_frame = estimation_frame;

                // xxx
                
                ROS_INFO_THROTTLE(0.25, "H: %f, H_dt: %f, H_ddt: %f", filter_state_focal.x(0), filter_state_focal.x(1),
                                  filter_state_focal.x(2));
            }
        }
        //}

        /* Filter matrices update //{ */

        A_t A_dt(double dt)
        {

            A << 1, dt, dt * dt / 2,
                0, 1, dt,
                0, 0, 1;
            return A;
        }

        H_t H_n(int type)
        {
            if (type == 0)
            {

                H << 1, 0, 0;
            }
            else if (type == 1)
            {

                H << 0, 1, 0;
            }
            else if (type == 2)
            {
                H << 0, 0, 1;
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "Wrong measurement matrix type");
            }
            return H;
        }

        //}

        /* Covariance to eigen method //{ */

        Eigen::MatrixXd rosCovarianceToEigen(const boost::array<double, 9> input) /*for 3x3 matrices*/
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

        Eigen::MatrixXd rosCovarianceToEigen2(const boost::array<double, 36> input) /*for 6x6 matrices*/
        {
            Eigen::MatrixXd output2(6, 6);

            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    output2(j, i) = input[6 * j + i];
                }
            }
            return output2;
        }
        //}

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

            u_t u = u_t::Zero();

            double new_dt = std::fmax(std::fmin(std::fmin((ros::Time::now() - agents[nb_index].last_updt).toSec(),
                                                          (ros::Time::now() - agents[nb_index].last_meas_u).toSec()),
                                                (ros::Time::now() - agents[nb_index].last_meas_s).toSec()),
                                      0.0);
            filter->A = A_dt(new_dt);
            agents[nb_index].filter_state = filter->predict(agents[nb_index].filter_state, u, Qq, new_dt);
            if (std::isnan(filter_state_focal.x(0)))
            {
                ROS_ERROR("Filter error on line 453");
            }
            agents[nb_index].last_updt = ros::Time::now();

            try
            {
                filter->H = H_n(Ve);
                R_t R;
                R = Eigen::MatrixXd::Identity(1, 1) * 1;
                Eigen::VectorXd z(1);

                z(0) = odom_msg->twist.twist.linear.z;

                agents[nb_index].filter_state = filter->correct(agents[nb_index].filter_state, z, R);
                if (std::isnan(filter_state_focal.x(0)))
                {
                    ROS_ERROR("Filter error on line 468");
                }
                agents[nb_index].last_meas_s = ros::Time::now();
            }
            catch ([[maybe_unused]] std::exception e)
            {
                ROS_ERROR("LKF failed: %s", e.what());
            }

            // try {
            //     filter->H = H_n(Po);
            //     R_t R;
            //     R = Eigen::MatrixXd::Identity(1,1) * 1;
            //     Eigen::VectorXd z(1);

            //     z(0) = odom_msg->pose.pose.position.z;

            //     agents[nb_index].filter_state = filter->correct(agents[nb_index].filter_state, z, R);
            //     if (std::isnan(filter_state_focal.x(0)))
            //     {
            //         ROS_ERROR("Filter error on ine 496");
                    
            //     }
            //     agents[nb_index].last_meas_s = ros::Time::now();
            // }
            // catch ([[maybe_unused]] std::exception e) {
            //     ROS_ERROR("LKF failed: %s", e.what());
            // }


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
                    poseCovB = rosCovarianceToEigen2(res_b_.value().pose.covariance);
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
                        agents[aid].quat = res_b_.value().pose.pose.orientation;

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
                                    geometry_msgs::Point A;
                                    A.x = nb.pfcu.x;
                                    A.y = nb.pfcu.y;
                                    A.z = nb.filter_state.x(0);
                                    geometry_msgs::Point B;
                                    B.x = A.x + cos(nb_hdg + focal_heading);
                                    B.y = A.y + sin(nb_hdg + focal_heading);
                                    B.z = A.z + sin(Quat2Eul(nb.quat));
                                    // B.z = A.z;
                                    geometry_msgs::Point C;
                                    C.x = agents[aid].pfcu.x;
                                    C.y = agents[aid].pfcu.y;
                                    C.z = agents[aid].filter_state.x(0); /*need to calculate actual value*/
                                    geometry_msgs::Point D;
                                    D.x = C.x + cos(agents[aid].angle_z + focal_heading);
                                    D.y = C.y + sin(agents[aid].angle_z + focal_heading);
                                    D.z = C.z + sin(Quat2Eul(agents[aid].quat));
                                    // D.z = C.z;

                                    double a1 = B.y - A.y;
                                    double b1 = A.x - B.x;
                                    double c1 = B.z - A.z;
                                    double d1 = a1 * A.x + b1 * A.y + c1 * A.z;

                                    double a2 = D.y - C.y;
                                    double b2 = C.x - D.x;
                                    double c2 = D.z - C.z;
                                    double d2 = a2 * C.x + b2 * C.y + c2 * C.z;

                                    // Line AB: A.x + t * (B.x - A.x) = X
                                    // double a1 = B.x - A.x;
                                    // double b1 = -(D.x - C.x);
                                    // double c1 = C.x - A.x;

                                    // Line AB: A.y + t * (B.y - A.y) = Y
                                    // double a2 = B.y - A.y;
                                    // double b2 = -(D.y - C.y);
                                    // double c2 = C.y - A.y;

                                    // Line AB: A.z + t * (B.z - A.z) = Z
                                    // double a3 = B.z - A.z;
                                    // double b3 = -(D.z - C.z);
                                    // double c3 = C.z - A.z;               

                                    // geometry_msgs::Point dirAB, dirCD;
                                    // dirAB.x = B.x - A.x;
                                    // dirAB.y = B.y - A.y;
                                    // dirAB.z = B.z - A.z;
                                    // dirCD.x = D.x - C.x;
                                    // dirCD.y = D.y - C.y;
                                    // dirCD.z = D.z - C.z;

                                    // geometry_msgs::Point crossProduct;
                                    // crossProduct.x = dirAB.y * dirCD.z - dirAB.z * dirCD.y;
                                    // crossProduct.y = dirAB.z * dirCD.x - dirAB.x * dirCD.z;
                                    // crossProduct.z = dirAB.x * dirCD.y - dirAB.y * dirCD.x;

                                    // double det = a1 * (b2 * c3 - b3 * c2) - a2 * (b1 * c3 - b3 * c1) + a3 * (b1 * c2 - b2 * c1);

                                    // double crossProductMagnitude = std::sqrt(crossProduct.x * crossProduct.x + crossProduct.y * crossProduct.y + crossProduct.z * crossProduct.z);

                                    // if (crossProductMagnitude == 0.0) {
                                    //     ROS_ERROR("The line segments AB and CD are parallel. They do not intersect.");
    
                                    // }

                                    //  double t = ((C.x - A.x) * crossProduct.x + (C.y - A.y) * crossProduct.y + (C.z - A.z) * crossProduct.z) / crossProductMagnitude;
                                    //  double u = ((A.x - C.x) * crossProduct.x + (A.y - C.y) * crossProduct.y + (A.z - C.z) * crossProduct.z) / crossProductMagnitude;
    
                                    //  // Check if the line segments intersect within their domains
                                    // if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
                                    //     new_int.x = A.x + t * dirAB.x;
                                    //     new_int.y = A.y + t * dirAB.y;
                                    //     new_int.z = A.z + t * dirAB.z;
                                    // } else {
                                    //     ROS_ERROR("The line segments AB and CD do not intersect within their domains.");
                                    // }

                                    double det = a1 * (b2 * c1 - b1 * c2) - a2 * (b1 * c1 - a1 * c2) + b1 * (a2 * c2 - b2 * c1);

                                   

                                    if (abs(det) < 0.0001)
                                    {
                                        ROS_ERROR("Intersection not found, det: %f", det);
                                    }
                                    else
                                    {   
                                        // double t1 = (c1 * (b2 * c3 - b3 * c2) - c2 * (b1 * c3 - b3 * c1) + c3 * (b1 * c2 - b2 * c1)) / det;
                                        // double t2 = (a1 * (c2 * D.z - c3 * D.y) - a2 * (c1 * D.z - c3 * D.x) + a3 * (c1 * D.y - c2 * D.x)) / det;
                                        // double t3 = (a1 * (b2 * D.z - b3 * D.y) - a2 * (b1 * D.z - b3 * D.x) + a3 * (b1 * D.y - b2 * D.x)) / det;

                                        // Calculate the intersection point coordinates
                                        // double intersectionX = A.x + t1 * (B.x - A.x);
                                        // double intersectionY = A.y + t1 * (B.y - A.y);
                                        // double intersectionZ = A.z + t1 * (B.z - A.z);

                                        // Create a geometry_msgs::Point object for the intersection point
                                        
                                        // new_int.x = intersectionX;
                                        // new_int.y = intersectionY;
                                        // new_int.z = intersectionZ;



                                        double detx = d1 * (b2 * c1 - b1 * c2) - d2 * (b1 * c1 - a1 * c2) + b1 * (d2 * c2 - d1 * c1);
                                        double dety = a1 * (d2 * c2 - d1 * c1) - a2 * (d1 * c1 - d2 * c1) + d1 * (a2 * c2 - b2 * c1);
                                        double detz = a1 * (b2 * d1 - b1 * d2) - a2 * (b1 * d1 - a1 * d2) + b1 * (a2 * d2 - b2 * d1);

                                        new_int.x = detx / det;
                                        new_int.y = dety / det;
                                        new_int.z = detz / det;
                                    }
                                    
                                    geometry_msgs::Point uvdar_pos_check;
                                    uvdar_pos_check.x = new_int.x;
                                    uvdar_pos_check.y = new_int.y;
                                    uvdar_pos_check.z = new_int.z;
                                    pub_uvdar_pos_check.publish(uvdar_pos_check);

                                    double gt_dist = sqrt(pow(focal_position.x - new_int.x, 2) + pow(focal_position.y - new_int.y, 2) +
                                                          pow(focal_position.z - new_int.z, 2));
                                    if (abs(det) > 0.0001 && gt_dist < 5.0)
                                    // if(gt_dist < 5.0)
                                    {
                                        ROS_INFO(
                                            "D: %f, h_diff: %f, gap: %f, det: %f, nb_dt: %f, eb1: %f, eb2: %f, eb3: %f, ea1: %f, ea2: %f, "
                                            "ea3: %f",
                                            gt_dist, hdg_diff, gap, det, nb_dt, nb.eigens.x, nb.eigens.y, nb.eigens.z, agents[aid].eigens.x,
                                            agents[aid].eigens.y, agents[aid].eigens.z);

                                        sum_of_ints.ints.x += new_int.x;
                                        sum_of_ints.ints.y += new_int.y;
                                        sum_of_ints.ints.z += new_int.z; /*calculate this correctly, not sure about current implementation*/
                                        sum_of_ints.ints_count += 1;
                                        

                                        if (!filter_init_focal)
                                        {
                                            Eigen::VectorXd poseVec(3);
                                            poseVec(0) = new_int.z; /*important part to figure out. Intersection of all poses to find actual
                                                                       coordinates*/
                                            poseVec(1) = 0;
                                            poseVec(2) = 0;
                                            Eigen::MatrixXd poseCov(3, 3);
                                            poseCov << Eigen::MatrixXd::Identity(3, 3);

                                            ROS_INFO("LKF initialized for the focal UAV at %f of GPS frame.", poseVec(0));

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
                                                filter->H = H_n(Po);

                                                R_t R;
                                                R = Eigen::MatrixXd::Identity(1, 1) * 0.1;
                                                Eigen::VectorXd z(1);
                                                z(0) = new_int.z; /* need intersection point of all poses*/

                                                filter_state_focal = filter->correct(filter_state_focal, z, R);
                                                if (std::isnan(filter_state_focal.x(0)))
                                                {
                                                    ROS_ERROR("Filter error on line 666");
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
                    poseCov = rosCovarianceToEigen2(res_l_.value().pose.covariance);
                    Eigen::EigenSolver<Eigen::Matrix3d> es(poseCov.topLeftCorner(3, 3));
                    auto eigvals = es.eigenvalues();

                    if (!agents[aid].filter_init)
                    {
                        Eigen::VectorXd poseVec(3);
                        poseVec(0) = res_l_.value().pose.pose.position.z;
                        poseVec(1) = 0;
                        poseVec(2) = 0;
                        Eigen::MatrixXd poseCov(3, 3);
                        poseCov << Eigen::MatrixXd::Identity(3, 3);

                        ROS_INFO("LKF initialized for UAV%d with UVDAR reloc at %f, %f, %f of gps frame.", agents[aid].name_id,
                                 poseVec(0), poseVec(1), poseVec(2));

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
                        {
                            try
                            {
                                filter->H = H_n(Po);

                                R_t R;
                                R = Eigen::MatrixXd::Identity(1, 1) * 0.00001;
                                Eigen::VectorXd z(1);
                                z(0) = res_l_.value().pose.pose.position.z;
                                agents[aid].filter_state = filter->correct(agents[aid].filter_state, z, R);
                                if (std::isnan(filter_state_focal.x(0)))
                                {
                                    ROS_ERROR("Filter error on line 738");
                                }
                                agents[aid].last_meas_u = ros::Time::now();

                                ROS_WARN("Agent state updated uav%d", agents[aid].name_id);
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
                    ROS_INFO_STREAM("[VerticalEstimator]: Failed to get transformation for tf2lf measurement, returning.");
                }
            }
            if (sum_of_ints.ints_count > 0.0)

            {
                sum_of_ints.ints.x = sum_of_ints.ints.x / sum_of_ints.ints_count;
                sum_of_ints.ints.y = sum_of_ints.ints.y / sum_of_ints.ints_count;
                sum_of_ints.ints.z = sum_of_ints.ints.z / sum_of_ints.ints_count;

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
                    u_vel.z = u_vel.z / 10.0;
                    ROS_WARN("Velocity z: %f", u_vel.z);

                    if (filter_init_focal)
                    {
                        try
                        {
                            filter->H = H_n(Ve);

                            R_t R;
                            R = Eigen::MatrixXd::Identity(1, 1) * 3.0;
                            Eigen::VectorXd z(1);
                            z(0) = u_vel.z;
                            filter_state_focal = filter->correct(filter_state_focal, z, R);
                            if (std::isnan(filter_state_focal.x(0)))
                            {
                                ROS_ERROR("Filter error on line 803");
                            }
                            last_meas_focal = ros::Time::now();
                        }
                        catch ([[maybe_unused]] std::exception e)
                        {
                            ROS_ERROR("LKF failed: %s", e.what());
                        }
                    }
                    geometry_msgs::Point vel;
                    vel.z = u_vel.z;
                    pub_velocity_uvdar_fcu.publish(vel);
                }
            }
        }

        void MainOdom(const nav_msgs::Odometry &msg)
        {
            focal_heading = mrs_lib::AttitudeConverter(msg.pose.pose.orientation).getHeading();
            focal_height = msg.pose.pose.position.z;

            focal_position = msg.pose.pose.position;
        }

        void ImuOdom(const sensor_msgs::Imu& msg){
            double az = msg.linear_acceleration.z;

            double dt = (ros::Time::now() - last_imu_update).toSec();

            velocity_imu_focal.z = velocity_imu_focal.z + az * dt;
            last_imu_update = ros::Time::now();

            if((ros::Time::now() - last_imu_correction).toSec() > 0.1 && (ros::Time::now() - last_imu_meas).toSec() > 0.25){
                if(filter_init_focal){
                    try {
                        filter->H = H_n(Ve);

                        R_t R;

                        R = Eigen::MatrixXd::Identity(1,1) * 1.0;
                        Eigen::VectorXd z(1);
                        z(0) = velocity_imu_focal.z;
                        filter_state_focal = filter->correct(filter_state_focal, z, R);
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
                            filter->H = H_n(Ac);

                            R_t R;
                            R = Eigen::MatrixXd::Identity(1,1) * 1.0;
                            Eigen::VectorXd z(1);
                            z(0) = az;
                            filter_state_focal = filter->correct(filter_state_focal, z, R);
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
                velocity_imu_focal.z = msg.velocity.linear.z;
                last_imu_correction = ros::Time::now();
                last_imu_update = ros::Time::now();
                velocity_imu_initialized = true;

            } else {
                if((ros::Time::now() - last_imu_correction).toSec() > 1.0){
                    velocity_imu_focal.z = (velocity_imu_focal.z + msg.velocity.linear.z)/2;
                    last_imu_correction = ros::Time::now();
                    last_imu_update = ros::Time::now();
                }
            }
        }

        void
        GarminRange(const sensor_msgs::Range &rangemsg)
        {   
            sensor_msgs::Range rmsg = rangemsg;
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
            // return;


            // try
            // {
            //     filter->H = H_n(Po);
            //     R_t R;
            //     R = Eigen::MatrixXd::Identity(1, 1) * 0.1;
            //     Eigen::VectorXd z(1);
            //     z(0) = rmsg.range;
            //     filter_state_focal = filter->correct(filter_state_focal, z, R);
            //     if (std::isnan(filter_state_focal.x(0)))
            //     {
            //         ROS_ERROR("Filter error on line 835");
            //     }
            //     last_meas_focal = ros::Time::now();
            // }
            // catch ([[maybe_unused]] std::exception e)
            // {
            //     ROS_ERROR("LKF failed: %s", e.what());
            // }
        }
        //}
        //}

        double Quat2Eul(const geometry_msgs::Quaternion quat){

            geometry_msgs::Quaternion quat_local = quat;
           
            double mag = sqrt(pow(quat.x,2) + pow(quat.y,2) + pow(quat.z,2) + pow(quat.w,2));

            quat_local.x = quat_local.x/mag;
            quat_local.y = quat_local.y/mag;
            quat_local.z = quat_local.z/mag;
            quat_local.w = quat_local.y/mag;
            
            q2e.roll = atan2(2 * (quat_local.w * quat_local.x + quat_local.y * quat_local.z), 1 - 2 * (pow(quat_local.x,2) + pow(quat_local.y,2))); //around x axis
            q2e.pitch = asin(2 * (quat_local.w * quat_local.y - quat_local.z * quat_local.x)); //around y axis
            q2e.yaw = atan2(2 * (quat_local.w * quat_local.z + quat_local.x * quat_local.y), 1 - 2 * (pow(quat_local.y,2) + pow(quat_local.z,2))); //around z axis

            return q2e.roll;
        }

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
        ros::Publisher pub_vert_estimator_output_fcu;

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

        using nb_state_callback = boost::function<void(const nav_msgs::OdometryConstPtr &)>;
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

        bool vio_frame = false;

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

            double angle_z;

            geometry_msgs::Quaternion quat;
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

        // geometry_msgs::Point new_int;

        ros::Publisher pub_uvdar_pos_check;

        
        struct EulAng
        {
            double roll;
            double pitch;
            double yaw;
        };

        EulAng q2e;
        // Eigen::MatrixXd output(3,3);
        //}
        //}
    };

} // namespace vertical_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vertical_estimator::VerticalEstimator, nodelet::Nodelet)
