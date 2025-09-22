#include <memory>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <optional>
#include <sstream>
#include <cstdlib>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "arm_kinematics/srv/compute_ik.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

using ComputeIK = arm_kinematics::srv::ComputeIK;

class IkServiceCpp : public rclcpp::Node {
public:
  IkServiceCpp() : rclcpp::Node("ik_service_cpp") {
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("tip_link", "End-Coupler-v1");
    this->declare_parameter<std::string>("robot_description", "");

    service_ = this->create_service<ComputeIK>(
      "/compute_ik",
      std::bind(&IkServiceCpp::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "ik_service_cpp ready on /compute_ik");
  }

private:
  struct ChainCacheKey {
    std::string base;
    std::string tip;
    bool operator<(const ChainCacheKey & other) const {
      return base < other.base || (base == other.base && tip < other.tip);
    }
  };

  std::optional<std::string> get_urdf_xml() {
    // Prefer parameter robot_description
    std::string robot_description;
    this->get_parameter("robot_description", robot_description);
    if (!robot_description.empty()) {
      return robot_description;
    }

    // Fallback: run xacro on arm_description/urdf/arm.urdf.xacro
    try {
      const std::string share_dir = ament_index_cpp::get_package_share_directory("arm_description");
      const std::string xacro_path = share_dir + "/urdf/arm.urdf.xacro";
      // Use tempfile path in /tmp
      const std::string tmp_path = "/tmp/arm_urdf_from_xacro.urdf";
      std::stringstream cmd;
      cmd << "xacro " << xacro_path << " > " << tmp_path;
      int ret = std::system(cmd.str().c_str());
      if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "xacro invocation failed: %d", ret);
        return std::nullopt;
      }
      // Read file
      std::ifstream ifs(tmp_path);
      if (!ifs) {
        RCLCPP_ERROR(this->get_logger(), "failed to open URDF temp file");
        return std::nullopt;
      }
      std::stringstream buffer;
      buffer << ifs.rdbuf();
      return buffer.str();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception loading URDF: %s", e.what());
      return std::nullopt;
    }
  }

  std::optional<KDL::Chain> get_chain(const std::string & base, const std::string & tip) {
    ChainCacheKey key{base, tip};
    auto it = chain_cache_.find(key);
    if (it != chain_cache_.end()) {
      return it->second;
    }

    auto urdf_xml_opt = get_urdf_xml();
    if (!urdf_xml_opt.has_value()) {
      RCLCPP_ERROR(this->get_logger(), "failed to build KDL chain");
      return std::nullopt;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml_opt.value(), tree)) {
      RCLCPP_ERROR(this->get_logger(), "failed to build KDL chain");
      return std::nullopt;
    }

    KDL::Chain chain;
    if (!tree.getChain(base, tip, chain)) {
      RCLCPP_ERROR(this->get_logger(), "failed to build KDL chain");
      return std::nullopt;
    }

    // Log joint order for diagnostics
    unsigned int nj = chain.getNrOfJoints();
    RCLCPP_INFO(this->get_logger(), "Built KDL chain %s -> %s with %u joints", base.c_str(), tip.c_str(), nj);
    unsigned int idx = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      const auto & seg = chain.getSegment(i);
      const auto & j = seg.getJoint();
      if (j.getType() != KDL::Joint::None) {
        RCLCPP_INFO(this->get_logger(), "  joint[%u]: %s", idx, j.getName().c_str());
        idx++;
      }
    }

    chain_cache_.emplace(key, chain);
    return chain;
  }

  static KDL::Frame pose_to_frame(const geometry_msgs::msg::Pose & p) {
    auto qx = p.orientation.x;
    auto qy = p.orientation.y;
    auto qz = p.orientation.z;
    auto qw = p.orientation.w;
    const double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm > 0.0) {
      qx /= norm; qy /= norm; qz /= norm; qw /= norm;
    } else {
      qw = 1.0; qx = qy = qz = 0.0;
    }
    KDL::Rotation R = KDL::Rotation::Quaternion(qx, qy, qz, qw);
    return KDL::Frame(R, KDL::Vector(p.position.x, p.position.y, p.position.z));
  }

  void handle_request(const std::shared_ptr<ComputeIK::Request> req,
                      std::shared_ptr<ComputeIK::Response> res) {
    // Defaults from params, overridden by request if non-empty
    std::string base = this->get_parameter("base_link").as_string();
    std::string tip = this->get_parameter("tip_link").as_string();
    if (!req->base_link.empty()) base = req->base_link;
    if (!req->tip_link.empty()) tip = req->tip_link;

    auto chain_opt = get_chain(base, tip);
    if (!chain_opt.has_value()) {
      res->success = false;
      res->message = "failed to build KDL chain";
      res->solution_positions.clear();
      return;
    }
    KDL::Chain chain = chain_opt.value();

    const unsigned int nj = chain.getNrOfJoints();
    if (req->joint_names.size() != req->seed_positions.size() || req->seed_positions.size() != nj) {
      res->success = false;
      res->message = "joint arrays mismatch";
      res->solution_positions.clear();
      return;
    }

    std::vector<std::vector<double>> seed_candidates;
    seed_candidates.push_back(std::vector<double>(req->seed_positions.begin(), req->seed_positions.end()));
    seed_candidates.push_back(std::vector<double>(nj, 0.0));
    seed_candidates.push_back(std::vector<double>(nj, 0.2));
    seed_candidates.push_back(std::vector<double>(nj, -0.2));
    seed_candidates.push_back(std::vector<double>(nj, 0.4));
    seed_candidates.push_back(std::vector<double>(nj, -0.4));

    KDL::ChainIkSolverPos_LMA solver(chain, 1e-6, 2000);
    KDL::Frame target_frame = pose_to_frame(req->target.pose);
    KDL::JntArray q_out(nj);
    for (const auto & seed_vec : seed_candidates) {
      KDL::JntArray q_seed(nj);
      for (unsigned int i = 0; i < nj; ++i) q_seed(i) = seed_vec[i];
      int status = solver.CartToJnt(q_seed, target_frame, q_out);
      if (status >= 0) {
        res->solution_positions.resize(nj);
        for (unsigned int i = 0; i < nj; ++i) res->solution_positions[i] = q_out(i);
        res->success = true;
        res->message = "";
        return;
      }
    }

    // Fallback: Newton-Raphson IK
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::ChainIkSolverVel_pinv ik_vel(chain);
    KDL::ChainIkSolverPos_NR ik_nr(chain, fk, ik_vel, 2000, 1e-6);
    for (const auto & seed_vec : seed_candidates) {
      KDL::JntArray q_seed(nj);
      for (unsigned int i = 0; i < nj; ++i) q_seed(i) = seed_vec[i];
      int status = ik_nr.CartToJnt(q_seed, target_frame, q_out);
      if (status >= 0) {
        res->solution_positions.resize(nj);
        for (unsigned int i = 0; i < nj; ++i) res->solution_positions[i] = q_out(i);
        res->success = true;
        res->message = "";
        return;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "IK failed after multiple seeds (nj=%u): target[x=%.3f y=%.3f z=%.3f]",
                 nj,
                 req->target.pose.position.x,
                 req->target.pose.position.y,
                 req->target.pose.position.z);
    res->success = false;
    res->message = "IK failed";
    res->solution_positions.clear();
    return;
  }

  rclcpp::Service<ComputeIK>::SharedPtr service_;
  std::map<ChainCacheKey, KDL::Chain> chain_cache_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkServiceCpp>());
  rclcpp::shutdown();
  return 0;
}


