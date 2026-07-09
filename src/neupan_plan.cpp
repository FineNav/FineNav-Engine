// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_planner_neupan/neupan_plan.hpp"

namespace finenav {

void neupan_plan::configure(const ConfigType& params) {
    config_ = params;
    configured_ = false;

    if (config_.config_yaml.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("neupan_plan"), "config_yaml is empty");//弹出错误提示，配置文件路径不能为空
        return;
    }

    if (!py_guard_) {
        py_guard_ = std::make_unique<py::scoped_interpreter>();
    }//确保Python解释器已初始化且只初始化一次

    py::gil_scoped_acquire gil;//py全局解释器锁,确保在当前线程中安全地调用Python代码
    try {
        py::module_::import("sys").attr("path").attr("append")(config_.python_path);
        py::object module = py::module_::import(config_.module_name.c_str());
        py::object cls = module.attr(config_.class_name.c_str());
        planner_ = cls.attr("init_from_yaml")(config_.config_yaml);
        /*相当于:
        import sys
        sys.path.append(config_.python_path)  # 把你的 neupan 算法所在目录加到系统环境变量，不然找不到模块
        import neupan.neupan as module          # 导入模块 (假设 config_.module_name 是 "neupan.neupan")
        cls = module.neupan                   # 获取模块里的 neupan 类 (假设 config_.class_name 是 "neupan")
        planner_ = cls.init_from_yaml(config_.config_yaml) # 调用类方法，用传进来的 yaml 文件初始化一个规划器实例
        */
        configured_ = true;
    } catch (const py::error_already_set& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("neupan_plan"), "Python init failed: %s", ex.what());
        configured_ = false;
    }
}

neupan_plan::Result neupan_plan::plan(const Context& ctx, const IMapView& map_view) {
    if (!configured_) {
        RCLCPP_ERROR(rclcpp::get_logger("neupan_plan"), "Planner not configured");
        return std::nullopt;
    }//检查规划器是否已成功配置，如果没有，记录错误并返回空结果

    py::gil_scoped_acquire gil;
    OutputDataT<neupan_plan> output;

    const auto& pose = ctx.robot_state.pose;
    const double yaw = getYawFromPose(pose);

    py::array_t<double> state({3});
    auto st = state.mutable_unchecked<1>();
    st(0) = pose.position.x;
    st(1) = pose.position.y;
    st(2) = yaw;//只需要x,y坐标和朝向角度
    
//获取地图点云数据，并根据配置文件中的 max_points 参数进行降采样，最终转换为适合传递给 Python 规划器的 numpy 数组格式。
    auto cloud = map_view.getPointCloud();
    if (config_.max_points > 0 &&
        static_cast<int>(cloud.size()) > config_.max_points) {
        const size_t stride = static_cast<size_t>(
            std::ceil(static_cast<double>(cloud.size()) / config_.max_points));
        std::vector<Eigen::Vector3d> reduced;
        reduced.reserve(config_.max_points);
        for (size_t i = 0; i < cloud.size(); i += stride) {
            reduced.push_back(cloud[i]);
        }
        cloud.swap(reduced);//进行降采样，减少点云数量以提高规划效率，同时保持环境信息的代表性
    }

    const ssize_t count = static_cast<ssize_t>(cloud.size());
    py::array_t<double> points({2, count});
    auto pts = points.mutable_unchecked<2>();
    for (ssize_t i = 0; i < count; ++i) {
        pts(0, i) = cloud[i].x();
        pts(1, i) = cloud[i].y();
    }

    py::object result = planner_.attr("forward")(state, points, py::none());
    py::array_t<double> vel_np = result.cast<py::array_t<double>>();

    auto buf = vel_np.request();
    if (buf.ndim != 2 || buf.shape[0] < 2 || buf.shape[1] < 1) {
        RCLCPP_ERROR(rclcpp::get_logger("neupan_plan"), "Unexpected output shape");
        return std::nullopt;
    }

    auto vel = vel_np.unchecked<2>();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vel(0, 0);
    cmd.angular.z = vel(1, 0);
    output.twists.push_back(cmd);

    return output;
}

double neupan_plan::getYawFromPose(const geometry_msgs::msg::Pose& pose) const {
    const auto& q = pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

} // namespace finenav
