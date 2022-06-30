/*
 * @Description: ICP 匹配模块
 * @Author: QZ
 * @Date: 2022-06-30 10:29:27
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_MANUAL_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_MANUAL_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "se3.hpp"
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_localization
{
class ICPGaussRegistration : public RegistrationInterface
{
public:
    ICPGaussRegistration(const YAML::Node &node);
    ICPGaussRegistration(float max_correspond_dis, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                   const Eigen::Matrix4f &predict_pose,
                   CloudData::CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4f &result_pose) override;

private:
    bool SetRegistrationParam(float max_correspond_dis, int max_iter);
    void calculateTrans(const CloudData::CLOUD_PTR &input_cloud);

private:
    CloudData::CLOUD_PTR target_cloud_;
    pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_ptr_;
    float max_disttance_;
    int max_iterator_;

    Eigen::Matrix3f rotation_matrix_;
    Eigen::Vector3f translation_;
    Eigen::Matrix4f transformation_;
};
} // namespace lidar_localization

#endif