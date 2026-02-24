#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

const std::string NODE_NAME = "pointcloud_objects_node";

class PCObjects : public rclcpp::Node {
public:
    PCObjects() : rclcpp::Node(NODE_NAME)
    {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            PC_TOPIC_NAME, QOS1, std::bind(&PCObjects::PointCloudCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    /**
     * @brief 判断是否当前包是否存在Camera 和 Robot 坐标系转换关系
     *
     * @param msg `PC_TOPIC_NAME` 中发布的信息包
     * @return true 当前包存在 Camera 和 Robot 坐标系转换关系
     * @return false 当前包不存在 Camera 和 Robot 坐标系转换关系
     */
    bool IsTransAvailable(sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        return tf_buffer_->canTransform(BASE_FRAME_ID, msg->header.frame_id, msg->header.stamp);
    }

    /**
     * @brief 从Camera 坐标系转换到 Robot坐标系
     *
     * @param msg PC_TOPIC_NAME` 中发布的信息包
     * @return sensor_msgs::msg::PointCloud2::SharedPtr PointCloud2 在 Robot坐标系下的坐标
     */
    sensor_msgs::msg::PointCloud2::SharedPtr CamToBaseFrame(sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        sensor_msgs::msg::PointCloud2 pc_footprint;
        pcl_ros::transformPointCloud(BASE_FRAME_ID, *msg, pc_footprint, *tf_buffer_);
        return std::make_shared<sensor_msgs::msg::PointCloud2>(pc_footprint);
    }

    /**
     * @brief 将点云数据格式从 ROS2 转换到 PCL
     *
     * @param pc_footprint Robot 坐标系下的点云数据（ROS2格式）
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Robot 坐标系下的点云数据（PCL格式）
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr RosToPclFormat(sensor_msgs::msg::PointCloud2::SharedPtr& pc_footprint)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*pc_footprint, *cloud_src);
        return cloud_src;
    }

    /**
     * @brief 高通滤波
     *
     * @param cloud_src Robot 坐标系下的点云数据（PCL格式）
     */
    void ApplyPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;

        pass.setInputCloud(cloud_src->makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.5, 1.5);
        pass.filter(*cloud_src);

        pass.setInputCloud(cloud_src->makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.5, 0.5);
        pass.filter(*cloud_src);

        pass.setInputCloud(cloud_src->makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.5, 1.5);
        pass.filter(*cloud_src);
    }

    /**
     * @brief 提取桌面的点
     *
     * @param cloud_src Robot 坐标系下的点云数据（PCL格式）
     * @param planeIndices 桌面点在点云数据的索引
     */
    void FilterOutPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src, pcl::PointIndices::Ptr& planeIndices)
    {
        if (planeIndices->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No plane detected");
            return;
        }

        int point_num = planeIndices->indices.size();
        float point_z_sum = 0;
        for (int i = 0; i < point_num; ++i) {
            int point_index = planeIndices->indices[i];
            point_z_sum += cloud_src->points[point_index].z;
        }
        float plane_height = point_z_sum / point_num;
        RCLCPP_INFO(this->get_logger(), "plane_height=%.2f", plane_height);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_src->makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(plane_height + 0.2, 1.5);
        pass.filter(*cloud_src);
    }

    /**
     * @brief 提取平面
     *
     * @param cloud_src Robot 坐标系下的点云数据（PCL格式）
     */
    void ApplySegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;

        segmentation.setInputCloud(cloud_src->makeShared());
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.05);
        segmentation.setOptimizeCoefficients(true);

        pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
        segmentation.segment(*planeIndices, *coefficients);

        FilterOutPlane(cloud_src, planeIndices);
    }

    /**
     * @brief 计算分割出的物体的质心
     *
     * @param cloud_src Robot 坐标系下的点云数据（PCL格式）
     * @param cluster_indices 被分割出物体在点云数据中的索引
     */
    void GetCenteroid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src, std::vector<pcl::PointIndices>& cluster_indices)
    {
        int object_num = cluster_indices.size();
        for (int i = 0; i < object_num; ++i) {
            int point_num = cluster_indices[i].indices.size();
            float points_x_sum = 0;
            float points_y_sum = 0;
            float points_z_sum = 0;
            for (int j = 0; j < point_num; ++j) {
                int point_index = cluster_indices[i].indices[j];
                points_x_sum += cloud_src->points[point_index].x;
                points_y_sum += cloud_src->points[point_index].y;
                points_z_sum += cloud_src->points[point_index].z;
            }
            float centeroid_x = points_x_sum / point_num;
            float centeroid_y = points_y_sum / point_num;
            float centeroid_z = points_z_sum / point_num;
            RCLCPP_INFO(this->get_logger(), "object %d pos=(%.2f, %.2f, %.2f)", i, centeroid_x, centeroid_y,
                        centeroid_z);
        }
        RCLCPP_INFO(this->get_logger(), "-----------");
    }

    /**
     * @brief 使用Kd 树算法进行聚类分割
     *
     * @param cloud_src Robot 坐标系下的点云数据（PCL格式）
     */
    void ApplyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_src->makeShared());

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud_src->makeShared());
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setClusterTolerance(0.1);
        ec.setSearchMethod(tree);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        int object_num = cluster_indices.size();
        RCLCPP_INFO(this->get_logger(), "object_num=%d", object_num);

        GetCenteroid(cloud_src, cluster_indices);
    }

    /**
     * @brief `PC_TOPIC_NAME` 话题 Subscription的回调函数
     *
     * @param msg `PC_TOPIC_NAME` 发布的数据包
     */
    void PointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        if (!IsTransAvailable(msg)) {
            return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr pc_footprint = CamToBaseFrame(msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src = RosToPclFormat(pc_footprint);
        ApplyPassThroughFilter(cloud_src);
        ApplySegmentation(cloud_src);
        ApplyClustering(cloud_src);
    }

    const std::string PC_TOPIC_NAME = "/kinect2/sd/points";
    const std::string BASE_FRAME_ID = "base_footprint";
    const int QOS1 = 1;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PCObjects>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}