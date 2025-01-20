//
// Created by gura on 24-1-16.
//

#include "livox_localization/GlobalLocalization.h"
#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
// #include <teaser/matcher.h>
// #include <teaser/registration.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>
#include <tf2_ros/buffer.h>
#include "functional"

#include <rm_msgs/ClientMapSendData.h>
#include <std_msgs/String.h>

class GlobalLocalization
{
public:
  GlobalLocalization() : tf_listener_{ tf_buffer_ }
  {
    ros::NodeHandle nh("~");
    ROS_INFO("Localization Node Inited...");
    global_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // publisher
    pub_map_to_odom_ = nh.advertise<nav_msgs::Odometry>("/map_to_odom", 1);
    sentry_state_pub_ = nh.advertise<std_msgs::String>("/custom_info", 1);

    sub_cloud_registered =
        nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 1, &GlobalLocalization::cbSaveCurScan, this);
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, &GlobalLocalization::cbSaveCurOdom, this);
    sub_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
                                                                      &GlobalLocalization::cbRobotPose, this);

    //  ros::Subscriber sub_need_localization =
    //      nh.subscribe<std_msgs::Bool>("/need_localization", 1,
    //                                   cbNeedLocalization);
    if (!nh.getParam("map_file_path", map_file_path_))
      ROS_ERROR("map_file_path is not set");
    nh.param("scan_voxel_size", scan_voxel_size_, 0.1);
    nh.param("map_voxel_size", map_voxel_size_, 0.1);
    nh.param("freq_localization", freq_localization_, 0.5);
    nh.param("localization_th", localization_th_, 0.10);

    nh.param("sac_ia_min_sample_distance", sac_ia_min_sample_distance_, 0.05);
    nh.param("sac_ia_max_correspondence_distance", sac_ia_max_correspondence_distance_, 0.3);
    nh.param("sac_ia_nr_iterations", sac_ia_nr_iterations_, 50);

    nh.param("icp_max_correspondence_distance", icp_max_correspondence_distance_, 1.0);
    nh.param("icp_nr_iterations", icp_nr_iterations_, 30);
    nh.param("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-5);

    nh.param("ndt_resolution", ndt_resolution_, 1.0);
    nh.param("ndt_step_size", ndt_step_size_, 0.1);
    nh.param("ndt_nr_iterations", ndt_nr_iterations_, 30);
    nh.param("ndt_transformation_epsilon", ndt_transformation_epsilon_, 1e-3);

    nh.param("icp_nl_max_correspondence_distance", icp_nl_max_correspondence_distance_, 1.0);
    nh.param("icp_nl_nr_iterations", icp_nl_nr_iterations_, 30);
    nh.param("icp_nl_transformation_epsilon", icp_nl_transformation_epsilon_, 1e-5);

    nh.param("front_end_optimization", front_end_optimization_, false);
    nh.param("localization_failed_limit", localization_failed_limit_, 3);

    // 初始化
    double pose_x, pose_y, pose_z, rot_x, rot_y, rot_z, rot_w;
    nh.param("pose_x", pose_x, 0.0);
    nh.param("pose_y", pose_y, 0.0);
    nh.param("pose_z", pose_z, 0.0);
    nh.param("rot_x", rot_x, 0.0);
    nh.param("rot_y", rot_y, 0.0);
    nh.param("rot_z", rot_z, 0.0);
    nh.param("rot_w", rot_w, 1.0);
    global_localization_client_ = nh.serviceClient<livox_localization::GlobalLocalization>("/global_localization");

    geometry_msgs::PoseWithCovariance init_pose;
    init_pose.pose.position.x = pose_x;
    init_pose.pose.position.y = pose_y;
    init_pose.pose.position.z = pose_z;
    tf2::Quaternion quat;
    quat.setValue(rot_x, rot_y, rot_z, rot_w);
    quat.normalize();
    init_pose.pose.orientation.x = quat.getX();
    init_pose.pose.orientation.y = quat.getY();
    init_pose.pose.orientation.z = quat.getZ();
    init_pose.pose.orientation.w = quat.getW();

    T_map_to_odom_ = poseToMat(init_pose);

    // 初始化全局地图
    ROS_INFO("Waiting for global map......");
    initGlobalMap(map_file_path_);

    nav_msgs::Odometry map_to_odom;
    map_to_odom.pose.pose.position.x = pose_x;
    map_to_odom.pose.pose.position.y = pose_y;
    map_to_odom.pose.pose.position.z = pose_z;
    map_to_odom.pose.pose.orientation.x = rot_x;
    map_to_odom.pose.pose.orientation.y = rot_y;
    map_to_odom.pose.pose.orientation.z = rot_z;
    map_to_odom.pose.pose.orientation.w = rot_w;
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.header.frame_id = "map";
    pub_map_to_odom_.publish(map_to_odom);

    ROS_INFO("start");
    timer_ = nh.createTimer(ros::Duration(1 / freq_localization_),
                            std::bind(&GlobalLocalization::threadLocalization, this), false, true);
  }

  void publishPointCloud(ros::Publisher& publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)
  {
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pcl::toROSMsg(*pc, msg);
    publisher.publish(msg);
  }

  Eigen::Matrix4f poseToMat(const geometry_msgs::PoseWithCovariance& pose_msg)
  {
    tf::StampedTransform transform_base_link;
    tf::poseMsgToTF(pose_msg.pose, transform_base_link);
    Eigen::Affine3d affine;
    tf::transformTFToEigen(transform_base_link, affine);
    Eigen::Matrix4f base_link_matrix = affine.matrix().cast<float>();

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform *= base_link_matrix;
    return transform;
  }

  tf::StampedTransform matToTf(const Eigen::Matrix4f& mat)
  {
    Eigen::Affine3d affine(mat.cast<double>());
    tf::StampedTransform transform;
    tf::transformEigenToTF(affine, transform);
    return transform;
  }

  void print4x4Matrix(const Eigen::Matrix4f& matrix)
  {
    tf::StampedTransform tf = matToTf(matrix);
    ROS_INFO_STREAM("tran :" << tf.getOrigin().getX() << " " << tf.getOrigin().getY() << " " << tf.getOrigin().getZ());
    double roll, pitch, yaw;                                   // 定义存储r\p\y的容器
    tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);  // 进行转换
    ROS_INFO_STREAM("rot :" << roll << " " << pitch << " " << yaw);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr msgToPcl(const sensor_msgs::PointCloud2& pc_msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(pc_msg, *pc);
    return pc;
  }

  // teaser::PointCloud pclToTeaser(const pcl::PointCloud<pcl::PointXYZI>::Ptr
  // &pc) {
  //   teaser::PointCloud teaser_pc;
  //   for (auto &p : pc->points) {
  //     teaser_pc.push_back({p.x, p.y, p.z});
  //   }
  //   return teaser_pc;
  // }
  //
  // std::pair<Eigen::Matrix4f, double> teaserRegistration(
  //     const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_scan,
  //     const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_map,
  //     const pcl::registration::TransformationEstimation<
  //         pcl::PointXYZI, pcl::PointXYZI>::Matrix4 &pose_estimation) {
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr pc_scan_transformed(
  //       new pcl::PointCloud<pcl::PointXYZI>);
  //   pcl::transformPointCloud(*pc_scan, *pc_scan_transformed, pose_estimation);
  //   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  //   ros::Time start = ros::Time::now();
  //
  //   teaser::RobustRegistrationSolver::Params params;
  //   params.noise_bound = 0.1;
  //   params.cbar2 = 1;
  //   params.estimate_scaling = false;
  //   params.rotation_max_iterations = 100;
  //   params.rotation_gnc_factor = 1.4;
  //   //  params.rotation_estimation_algorithm =
  //   //
  //   teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  //   params.rotation_estimation_algorithm =
  //       teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
  //   params.inlier_selection_mode =
  //       teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
  //   params.rotation_cost_threshold = 0.005;
  //   teaser::RobustRegistrationSolver solver(params);
  //   teaser::PointCloud src_cloud = pclToTeaser(pc_scan_transformed);
  //   teaser::PointCloud tgt_cloud = pclToTeaser(pc_map);
  //   //  teaser::FPFHEstimation fpfh;
  //   //  auto obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, 0.02, 0.04);
  //   //  auto scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, 0.02,
  //   0.04); FeatureCloud fpfh; auto obj_descriptors =
  //       fpfh.computeLocalFeaturesDirect(pc_scan_transformed, 0.2, 0.3);
  //   auto scene_descriptors = fpfh.computeLocalFeaturesDirect(pc_map, 0.2, 0.3);
  //   teaser::Matcher matcher;
  //   auto correspondences = matcher.calculateCorrespondences(
  //       src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, false,
  //       true, false, 0.95);
  //
  //   solver.solve(pclToTeaser(pc_scan), pclToTeaser(pc_map), correspondences);
  //   auto solution = solver.getSolution();
  //   ROS_INFO_STREAM("Estimated rotation: " << solution.rotation);
  //   ROS_INFO_STREAM("Estimated translation: " << solution.translation);
  //   ROS_INFO_STREAM("Estimated time: " << (ros::Time::now() - start).toSec());
  //
  //   teaser::RobustRegistrationSolver solver2(params);
  //   int N = src_cloud.size();
  //   // Convert the point cloud to Eigen
  //   Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  //   for (size_t i = 0; i < N; ++i) {
  //     src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
  //   }
  //   Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
  //   for (size_t i = 0; i < N; ++i) {
  //     tgt.col(i) << tgt_cloud[i].x, tgt_cloud[i].y, tgt_cloud[i].z;
  //   }
  //   solver2.solve(src, tgt);
  //   auto solution2 = solver2.getSolution();
  //   ROS_INFO_STREAM("Estimated rotation: " << solution2.rotation);
  //   ROS_INFO_STREAM("Estimated translation: " << solution2.translation);
  //
  //   transform_2.translation() = solution.translation.cast<float>();
  //   transform_2.rotate(solution.rotation.cast<float>());
  //   return {transform_2.matrix(), 0.0};
  // }

  // std::pair<Eigen::Matrix4f, double> sac_iaRegistration(
  //     const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_scan,
  //     const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_map,
  //     const pcl::registration::TransformationEstimation<
  //         pcl::PointXYZI, pcl::PointXYZI>::Matrix4 &pose_estimation) {
  //   pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI,
  //                                        pcl::FPFHSignature33>
  //       sac_ia;
  //
  //   sac_ia.setMinSampleDistance(static_cast<float>(sac_ia_min_sample_distance_));
  //   sac_ia.setMaxCorrespondenceDistance(
  //       static_cast<float>(sac_ia_max_correspondence_distance_));
  //   sac_ia.setMaximumIterations(static_cast<float>(sac_ia_nr_iterations_));
  //   sac_ia.setTransformationEpsilon(0.0001);
  //   FeatureCloud target_cloud, source_cloud;
  //   target_cloud.setInputCloud(pc_map);
  //   source_cloud.setInputCloud(pc_scan);
  //
  //   sac_ia.setInputTarget(target_cloud.getPointCloud());
  //   sac_ia.setTargetFeatures(target_cloud.getLocalFeatures());
  //   sac_ia.setInputSource(source_cloud.getPointCloud());
  //   sac_ia.setSourceFeatures(source_cloud.getLocalFeatures());
  //
  //   pcl::PointCloud<pcl::PointXYZI> final;
  //   sac_ia.align(final, pose_estimation);
  //
  //   if (!sac_ia.hasConverged()) {
  //     ROS_ERROR("sac_ia not Converged");
  //   }
  //   pcl::Registration<pcl::PointXYZI, pcl::PointXYZI, float>::Matrix4
  //       transformation = sac_ia.getFinalTransformation();
  //   double score = sac_ia.getFitnessScore();
  //   print4x4Matrix(transformation);
  //
  //   return {transformation, score};
  // }

  std::pair<Eigen::Matrix4f, double> icpRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_scan,
                                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_map,
                                                     const Eigen::Matrix4f& initial)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp.setMaximumIterations(icp_nr_iterations_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setInputSource(pc_scan);
    icp.setInputTarget(pc_map);

    pcl::PointCloud<pcl::PointXYZI> final;
    icp.align(final, initial);

    if (!icp.hasConverged())
    {
      ROS_ERROR("icp not Converged");
    }
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI, float>::Matrix4 transformation = icp.getFinalTransformation();
    double score = icp.getFitnessScore();
    //  print4x4Matrix(transformation);
    return { transformation, score };
  }

  std::pair<Eigen::Matrix4f, double> ndtRegistration(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_scan, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_map,
      const pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Matrix4& pose_estimation)
  {
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setMaximumIterations(ndt_nr_iterations_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(ndt_resolution_);
    ndt.setInputSource(pc_scan);
    ndt.setInputTarget(pc_map);

    pcl::PointCloud<pcl::PointXYZI> final;
    ndt.align(final, pose_estimation);

    if (!ndt.hasConverged())
    {
      ROS_ERROR("ndt not Converged");
    }
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI, float>::Matrix4 transformation = ndt.getFinalTransformation();
    double score = ndt.getFitnessScore();
    //  print4x4Matrix(transformation);
    return { transformation, score };
  }

  std::pair<Eigen::Matrix4f, double> icp_nlRegistration(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_scan, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_map,
      const pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Matrix4& initial)
  {
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZI, pcl::PointXYZI> icp_nl;
    icp_nl.setMaxCorrespondenceDistance(icp_nl_max_correspondence_distance_);
    icp_nl.setMaximumIterations(icp_nl_nr_iterations_);
    icp_nl.setTransformationEpsilon(icp_nl_transformation_epsilon_);
    icp_nl.setInputSource(pc_scan);
    icp_nl.setInputTarget(pc_map);

    pcl::PointCloud<pcl::PointXYZI> final;
    icp_nl.align(final, initial);

    if (!icp_nl.hasConverged())
    {
      ROS_ERROR("icp_nl not Converged");
    }
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI, float>::Matrix4 transformation = icp_nl.getFinalTransformation();
    double score = icp_nl.getFitnessScore();
    //  print4x4Matrix(transformation);
    return { transformation, score };
  }

  std::pair<Eigen::Matrix4f, double> orientationRegistration(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_scan, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_map,
      const pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Matrix4& pose_estimation)
  {
    std::vector<Eigen::Matrix4f> initial_transforms;
    std::vector<float> angles;
    for (float angle = 0.0; angle < 360.0; angle += 5.0)
    {
      Eigen::Affine3f transform(pose_estimation);
      transform.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitZ()));

      initial_transforms.push_back(transform.matrix());
      angles.push_back(angle * M_PI / 180.0);
    }

    std::vector<float> fitness_scores;
    std::vector<Eigen::Matrix4f> Transformation_sources;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloud_sources;

    for (int i = 0; i < initial_transforms.size(); i++)
    {
      auto registration_result = ndtRegistration(pc_scan, pc_map, initial_transforms[i]);
      Transformation_sources.push_back(registration_result.first);
      fitness_scores.push_back(registration_result.second);
    }

    int best_index = 0;
    float best_score = fitness_scores[0];
    for (int i = 1; i < fitness_scores.size(); i++)
    {
      if (fitness_scores[i] < best_score)
      {
        best_index = i;
        best_score = fitness_scores[i];
      }
    }
    return { Transformation_sources[best_index], best_score };
  }

  Eigen::Matrix4f inverseSE3(const Eigen::Matrix4f& trans)
  {
    Eigen::Matrix4f trans_inverse = Eigen::Matrix4f::Identity();
    tf::StampedTransform trans_tf;
    Eigen::Affine3d ori_affine(trans.cast<double>());
    tf::transformEigenToTF(ori_affine, trans_tf);
    Eigen::Affine3d tran_affine;
    tf::transformTFToEigen(trans_tf.inverse(), tran_affine);
    trans_inverse *= tran_affine.matrix().cast<float>();
    return trans_inverse;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cropGlobalMapInFOV(const pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map,
                                                          const Eigen::Matrix4f& pose_estimation,
                                                          const nav_msgs::Odometry::ConstPtr& cur_odom)
  {
    // 当前scan原点的位姿
    Eigen::Matrix4f T_odom_to_base_link = poseToMat(cur_odom->pose);
    Eigen::Matrix4f T_map_to_base_link = pose_estimation * T_odom_to_base_link;
    Eigen::Matrix4f T_base_link_to_map = inverseSE3(T_map_to_base_link);

    // 把地图转换到lidar系下
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_in_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*global_map, *global_map_in_map, T_base_link_to_map);

    return global_map_in_map;
  }

  bool globalLocalization(const Eigen::Matrix4f& pose_estimation)
  {
    //  publishPointCloud(pub_map_, global_map_);
    ROS_INFO("in");
    // TODO 这里注意线程安全
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_tobe_mapped(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f transformation = pose_estimation;
    double fitness;
    print4x4Matrix(pose_estimation);

    pcl::copyPointCloud(*cur_scan_, *scan_tobe_mapped);
    if (relocate_times_ >= localization_failed_limit_)
      need_global_localization_ = true;

    if (need_global_localization_)
    {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*scan_tobe_mapped, msg);
      tf2::Transform odom2base;
      odom2base.setOrigin(tf2::Vector3(cur_odom_.pose.pose.position.x, cur_odom_.pose.pose.position.y,
                                       cur_odom_.pose.pose.position.z));
      odom2base.setRotation(tf2::Quaternion(cur_odom_.pose.pose.orientation.x, cur_odom_.pose.pose.orientation.y,
                                            cur_odom_.pose.pose.orientation.z, cur_odom_.pose.pose.orientation.w));
      msg.header.stamp = ros::Time::now();
      livox_localization::GlobalLocalization srv;
      srv.request.cloud_msg = msg;
      if (global_localization_client_.call(srv))
      {
        ROS_INFO_STREAM("pose" << srv.response.pose);
        tf2::Transform map2base, map2odom;
        map2base.setOrigin(tf2::Vector3(srv.response.pose.pose.position.x, srv.response.pose.pose.position.y,
                                        srv.response.pose.pose.position.z));
        map2base.setRotation(tf2::Quaternion(srv.response.pose.pose.orientation.x, srv.response.pose.pose.orientation.y,
                                             srv.response.pose.pose.orientation.z,
                                             srv.response.pose.pose.orientation.w));
        map2odom = map2base * odom2base.inverse();
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.transform = tf2::toMsg(map2odom);
        geometry_msgs::PoseWithCovariance pose_msg;
        pose_msg.pose.position.x = tf_msg.transform.translation.x;
        pose_msg.pose.position.y = tf_msg.transform.translation.y;
        pose_msg.pose.position.z = tf_msg.transform.translation.z;
        pose_msg.pose.orientation.x = tf_msg.transform.rotation.x;
        pose_msg.pose.orientation.y = tf_msg.transform.rotation.y;
        pose_msg.pose.orientation.z = tf_msg.transform.rotation.z;
        pose_msg.pose.orientation.w = tf_msg.transform.rotation.w;
        transformation = poseToMat(pose_msg);
        need_global_localization_ = false;
      }
      else
        ROS_ERROR("Failed");
    }

    ros::Time tic = ros::Time::now();
    //  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_in_fov =
    //      cropGlobalMapInFOV(global_map_, transformation, cur_odom_);

    // 粗配准
    //  std::tie(transformation, fitness) =
    //      sac_iaRegistration(scan_tobe_mapped, global_map_, transformation);
    // 一致性初始配准

    //  ROS_INFO_STREAM("sac_ia fitness: " << fitness);
    //  std::tie(transformation, fitness) =
    //      teaserRegistration(scan_tobe_mapped, global_map_, transformation);
    // 精配准

    std::tie(transformation, fitness) = ndtRegistration(scan_tobe_mapped, global_map_, transformation);

    ROS_INFO("ndt Time: %f", (ros::Time::now() - tic).toSec());
    ROS_INFO_STREAM("ndt fitness: " << fitness);
    tic = ros::Time::now();

    std::tie(transformation, fitness) = icp_nlRegistration(scan_tobe_mapped, global_map_, transformation);

    ROS_INFO("icp_nl Time: %f", (ros::Time::now() - tic).toSec());
    ROS_INFO_STREAM("icp_nl fitness: " << fitness);

    //  if (fitness > localization_th_) {
    //    tic = ros::Time::now();
    //    std::tie(transformation, fitness) =
    //        orientationRegistration(scan_tobe_mapped, global_map_,
    //        transformation);
    //  }
    //  ROS_INFO("orientation find Time: %f", (ros::Time::now() - tic).toSec());

    std_msgs::String data;
    // 当全局定位成功时才更新map2odom
    if (fitness < localization_th_)
    {
      T_map_to_odom_ = transformation;

      // 发布map_to_odom
      nav_msgs::Odometry map_to_odom;
      tf::StampedTransform map2odom = matToTf(T_map_to_odom_);
      map_to_odom.pose.pose.position.x = map2odom.getOrigin().x();
      map_to_odom.pose.pose.position.y = map2odom.getOrigin().y();
      map_to_odom.pose.pose.position.z = map2odom.getOrigin().z();
      map_to_odom.pose.pose.orientation.x = map2odom.getRotation().x();
      map_to_odom.pose.pose.orientation.y = map2odom.getRotation().y();
      map_to_odom.pose.pose.orientation.z = map2odom.getRotation().z();
      map_to_odom.pose.pose.orientation.w = map2odom.getRotation().w();
      map_to_odom.header.stamp = ros::Time::now();
      map_to_odom.header.frame_id = "map";

      pub_map_to_odom_.publish(map_to_odom);
      relocate_times_ = 0;
      ROS_INFO("Success! The fitness: %f", fitness);
      data.data = "Suc!" + std::to_string(fitness);
      sentry_state_pub_.publish(data);
      return true;
    }
    else
    {
      ROS_WARN("Not match!!! fitness score:%f", fitness);
      data.data = "Fail" + std::to_string(fitness);
      sentry_state_pub_.publish(data);
      relocate_times_ += 1;
      return false;
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxelDownSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd,
                                                       double voxel_size)
  {
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(pcd);
    sor.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size), static_cast<float>(voxel_size));
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    sor.filter(*pcd_downsampled);
    return pcd_downsampled;
  }

  void initializeGlobalMap(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
  {
    global_map_ = msgToPcl(*pc_msg);
    global_map_ = voxelDownSample(global_map_, map_voxel_size_);
    ROS_INFO("Global map received.");
  }

  bool initGlobalMap(const std::string& map_pcd_path)
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_pcd_path, *global_map_) == -1)
    {
      ROS_ERROR("Failed to load PCD file");
      return false;
    }
    else
    {
      ROS_INFO("Global map received.");
      return true;
    }
  }

    void cbSaveCurOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
      cur_odom_ = *odom_msg;
    }

  void cbSaveCurScan(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
  {
    // 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    // 转换为pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc_msg, *pc);
    cur_scan_ = voxelDownSample(pc, scan_voxel_size_);
    first_scan_ = true;
  }

  void cbRobotPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
  {
    geometry_msgs::TransformStamped camera_init2map;
    try
    {
      camera_init2map = tf_buffer_.lookupTransform("camera_init", "map", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    geometry_msgs::PoseStamped input, output;
    input.pose = msg->pose.pose;
    tf2::doTransform(input, output, camera_init2map);

    ROS_INFO_STREAM("world2sensor x:" << output.pose.position.x << "  Y:" << output.pose.position.y
                                      << "  Z:" << output.pose.position.z << "rot_x" << output.pose.orientation.x
                                      << " y" << output.pose.orientation.y << "z:" << output.pose.orientation.z
                                      << "w:" << output.pose.orientation.w);
    geometry_msgs::PoseWithCovariance initial_pose;
    initial_pose.pose=output.pose;
    T_map_to_odom_ = Eigen::Matrix4f::Identity();
    robot_pose_ = poseToMat(initial_pose);
    ROS_INFO_STREAM("init_X:" << msg->pose.pose.position.x << "Y:" << msg->pose.pose.position.y
                              << "Z:" << msg->pose.pose.position.z);
  }

  void threadLocalization()
  {
    if (first_scan_)
    {
      if (front_end_optimization_)
        globalLocalization(Eigen::Matrix4f::Identity());
      else
        globalLocalization(robot_pose_ * T_map_to_odom_);
    }
    else
      ROS_INFO("waiting for initialized.");
  }

private:
  tf2::Transform world2sensor_;
  ros::Subscriber sub_cloud_registered, sub_odom, sub_pose;
  tf2::Transform world2odom_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
  bool initialized_ = false;
  Eigen::Matrix4f T_map_to_odom_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f robot_pose_ = Eigen::Matrix4f::Identity();
  nav_msgs::Odometry cur_odom_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan_;
  int relocate_times_ = 0;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Constants
  double map_voxel_size_ = 0.4;
  double scan_voxel_size_ = 0.1;
  double freq_localization_ = 0.5;
  double localization_th_ = 0.10;

  // sac_ia
  double sac_ia_min_sample_distance_;
  double sac_ia_max_correspondence_distance_;
  int sac_ia_nr_iterations_;

  // icp
  double icp_max_correspondence_distance_{ 1.0 };
  int icp_nr_iterations_{ 30 };
  double icp_transformation_epsilon_ = 1e-5;

  // ndt
  double ndt_resolution_{ 1.0 };
  double ndt_step_size_{ 0.1 };
  int ndt_nr_iterations_{ 30 };
  double ndt_transformation_epsilon_ = 1e-3;

  // icp_nl
  double icp_nl_max_correspondence_distance_{ 1.0 };
  int icp_nl_nr_iterations_{ 30 };
  double icp_nl_transformation_epsilon_ = 1e-5;

  ros::Publisher pub_map_to_odom_;
  ros::Publisher pub_map_;
  ros::Publisher pub_cran_;
  ros::Publisher sentry_state_pub_;
  ros::ServiceClient global_localization_client_;
  bool need_global_localization_ = false;

  bool front_end_optimization_ = false;

  bool first_scan_ = false;
  int localization_failed_limit_ = 3;

  std::string map_file_path_;
  ros::Timer timer_;
};

class FeatureCloud
{
public:
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZI> SearchMethod;

  FeatureCloud() : search_method_xyz_(new SearchMethod), normal_radius_(1.2f), feature_radius_(1.21f)
  {
  }

  ~FeatureCloud()
  {
  }

  void setInputCloud(PointCloud::Ptr xyz)
  {
    xyz_ = xyz;
    processInput();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr getPointCloud()
  {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr getSurfaceNormals()
  {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr getLocalFeatures()
  {
    return (features_);
  }

  LocalFeatures::Ptr computeLocalFeaturesDirect(PointCloud::Ptr xyz, double normal_search_radius,
                                                double fpfh_search_radius)
  {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    LocalFeatures::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(xyz);
    normalEstimation.setRadiusSearch(normal_search_radius);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // Estimate FPFH
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(xyz);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(kdtree);
    fpfh_est.setRadiusSearch(fpfh_search_radius);
    fpfh_est.compute(*descriptors);

    return descriptors;
  }

protected:
  // Compute the surface normals and local features
  void processInput()
  {
    computeSurfaceNormals();
    computeLocalFeatures();
  }

  // Compute the surface normals
  void computeSurfaceNormals()
  {
    // 创建表面法向量
    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

    // 计算表面法向量
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
    norm_est.setInputCloud(xyz_);
    norm_est.setSearchMethod(search_method_xyz_);
    norm_est.setRadiusSearch(normal_radius_);
    norm_est.compute(*normals_);
  }

  void computeLocalFeatures()
  {
    features_ = LocalFeatures::Ptr(new LocalFeatures);

    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(xyz_);
    fpfh_est.setInputNormals(normals_);
    fpfh_est.setSearchMethod(search_method_xyz_);
    fpfh_est.setRadiusSearch(feature_radius_);
    fpfh_est.compute(*features_);
  }

private:
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;  // 快速点特征直方图
  SearchMethod::Ptr search_method_xyz_;

  float normal_radius_;
  float feature_radius_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast_lio_localization");
  GlobalLocalization global_localization;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  // 开始定期全局定位
  //  std::thread localization_th_read(threadLocalization);
  //  localization_th_read.join();

  return 0;
}
