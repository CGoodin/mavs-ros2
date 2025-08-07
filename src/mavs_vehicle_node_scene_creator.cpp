#include "mavs_vehicle_node_scene_creator.h"
#include <rclcpp/rclcpp.hpp>
#include "mavs_core/data_path.h"

#include <fstream>
#include <cmath>
#include <filesystem>
#include <numeric>      // for std::accumulate
#include <iomanip>      // for std::setw
#include <ctime>        // for std::time

//namespace fs = std::filesystem;

MavsVehicleNodeSceneCreator::MavsVehicleNodeSceneCreator() : MavsNode()
{
    dt_ = GetFloatParam("dt", 0.01f);
    nsteps_ = 0;
    elapsed_time_ = 0.0f;

    throttle_ = 0.0f;
    steering_ = 0.0f;
    braking_ = 0.0f;

    render_debug_ = GetBoolParam("debug_camera", false);
    use_human_driver_ = GetBoolParam("use_human_driver", false);
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();

    // New parameters for scene creator modes:
    mode_ = GetStringParam("mode", "automated"); // "manual" or "automated"
    iterations_ = GetIntParam("iterations", 100);
    ditch_depth_start_ = GetFloatParam("ditch_depth_start", 0.0f);
    ditch_width_start_ = GetFloatParam("ditch_width_start", 2.5f);
    results_path_ = GetStringParam("results_path", "/tmp/mavs_results");

    // Ensure results directory exists
    if (!std::filesystem::is_directory(results_path_)) {
        std::filesystem::create_directories(results_path_);
    }

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&MavsVehicleNodeSceneCreator::TwistCallback, this, std::placeholders::_1));

    odom_true_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry_true", 10);
    anim_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("anim_poses", 10);

    if (use_sim_time_) {
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1);
    }

    // Prepare simulation variables based on mode
    PrepareSimulationParams();

    // Start timer callback for sim steps
    render_steps_ = std::max(1, (int)(0.1f / dt_));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000.0f)),
        std::bind(&MavsVehicleNodeSceneCreator::TimerCallback, this));
}

void MavsVehicleNodeSceneCreator::TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Map Twist message to throttle, steering, braking
    throttle_ = msg->linear.x;
    steering_ = msg->angular.z;
    braking_ = msg->linear.y;
}

void MavsVehicleNodeSceneCreator::LoadVehicleAndScene()
{
    //const char* env_p = std::getenv("MAVS_DATA_PATH");
    //if (!env_p) {
    //    throw std::runtime_error("MAVS_DATA_PATH environment variable not set!");
    //}
    //std::string data_path(env_p);
    mavs::MavsDataPath dp;
    std::string data_path = dp.GetPath();

    // Load the vehicle
    std::string rp3d_vehicle_file = GetStringParam("rp3d_vehicle_file", "mrzr4_tires_low_gear.json");
    mavs_veh_.Load(data_path + "vehicles/rp3d_vehicles/"+rp3d_vehicle_file);

    // Load a default scene (update path if different)
    //scene_.Load(data_path + "/scenes/sample_scene.json");

    //env_.SetRaytracer(&scene_);
}


void MavsVehicleNodeSceneCreator::PrepareSimulationParams()
{
    ditch_depths_ = GenerateRange(0.0f, 4.0f, 0.2f);
    top_widths_ = GenerateRange(2.5f, 9.0f, 0.5f);

    if (mode_ == "manual") {
        selected_depths_.clear();
        selected_widths_.clear();
        for (auto d : ditch_depths_) {
            if (d >= ditch_depth_start_)
                selected_depths_.push_back(d);
        }
        for (auto w : top_widths_) {
            if (w >= ditch_width_start_)
                selected_widths_.push_back(w);
        }
        max_iterations_ = iterations_;
    } else {
        selected_depths_ = ditch_depths_;
        selected_widths_ = top_widths_;
        max_iterations_ = std::min(iterations_, (int)(selected_depths_.size() * selected_widths_.size()));
    }

    current_iteration_ = 0;
    depth_index_ = 0;
    width_index_ = 0;

    LoadVehicleAndScene();
}

std::vector<float> MavsVehicleNodeSceneCreator::GenerateRange(float start, float end, float step)
{
    std::vector<float> vals;
    for (float v = start; v <= end + 1e-4f; v += step)
        vals.push_back(std::round(v * 10.0f) / 10.0f);
    return vals;
}

void MavsVehicleNodeSceneCreator::TimerCallback()
{
    if (current_iteration_ >= max_iterations_) {
        RCLCPP_INFO(this->get_logger(), "Completed all iterations (%d). Shutting down.", max_iterations_);
        rclcpp::shutdown();
        return;
    }

    float current_depth = selected_depths_[depth_index_];
    float current_width = selected_widths_[width_index_];
    float base_width = current_width + 6.0f;

    SetupTerrain(current_width, base_width, current_depth);
    ResetVehicleState();

    std::vector<float> speed_history = RunSimulation();
    LogResults(current_depth, current_width, speed_history);

    current_iteration_++;
    width_index_++;
    if (width_index_ >= (int)selected_widths_.size()) {
        width_index_ = 0;
        depth_index_++;
        if (depth_index_ >= (int)selected_depths_.size()) {
            depth_index_ = 0;
        }
    }
}

void MavsVehicleNodeSceneCreator::SetupTerrain(float top_width, float base_width, float depth)
{
    mavs::terraingen::TerrainCreator terrain;
    terrain.AddTrapezoid(top_width, base_width, depth, 20.0f);
    terrain.CreateTerrain(-50.0f, -25.0f, 200.0f, 25.0f, 0.5f);
    scene_ = *terrain.GetScenePointer();

    env_.SetRaytracer(&scene_);
    env_.SetGlobalSurfaceProperties("dry", 6894.76f * 250.0f);
}

void MavsVehicleNodeSceneCreator::ResetVehicleState()
{
    glm::vec3 initial_position(0.0f, 0.0f, scene_.GetSurfaceHeight(0.0f, 0.0f) + 0.25f);
    glm::quat initial_orientation(1.0f, 0.0f, 0.0f, 0.0f);

    mavs_veh_.SetPosition(initial_position.x, initial_position.y, initial_position.z);
    mavs_veh_.SetOrientation(initial_orientation.w, initial_orientation.x, initial_orientation.y, initial_orientation.z);
    mavs_veh_.Update(&env_, 0.0f, 0.0f, 0.0f, 0.00001f);

    throttle_ = 0.3f;
    steering_ = 0.0f;
    braking_ = 0.0f;

    elapsed_time_ = 0.0f;
    nsteps_ = 0;
}

std::vector<float> MavsVehicleNodeSceneCreator::RunSimulation()
{
    constexpr float sim_duration = 15.0f;
    float local_elapsed = 0.0f;
    int local_steps = 0;

    std::vector<float> speed_history;
    //bool ditch_entered = false;
    //float desired_speed = 2.0f;

    //float prev_z = mavs_veh_.GetPosition().z;

    while (local_elapsed < sim_duration)
    {
        mavs_veh_.Update(&env_, throttle_, steering_, braking_, dt_);

        if (local_steps % 4 == 0) {
            //glm::vec3 pos = mavs_veh_.GetPosition();
            // CTG: need to figure out how we want to manage vehicle control
            //if (!ditch_entered && pos.z < prev_z - 0.01f) {
            //    ditch_entered = true;
            //    desired_speed = 4.0f;
            //    throttle_ = 0.8f;
            //}
            //prev_z = pos.z;

            float speed = mavs_veh_.GetSpeed();
            speed_history.push_back(speed);
        }

        local_elapsed += dt_;
        local_steps++;
    }

    return speed_history;
}

void MavsVehicleNodeSceneCreator::LogResults(float ditch_depth, float ditch_width, const std::vector<float>& speeds)
{
    float avg_speed = 0.0f;
    if (!speeds.empty()) {
        avg_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0f) / speeds.size();
    }

    std::string filename = results_path_ + "/run_" + std::to_string(current_iteration_) + ".json";

    std::ofstream file(filename);
    if (file.is_open()) {
        file << "{\n";
        file << "  \"iteration\": " << current_iteration_ << ",\n";
        file << "  \"ditch_depth\": " << ditch_depth << ",\n";
        file << "  \"ditch_width\": " << ditch_width << ",\n";
        file << "  \"average_speed\": " << avg_speed << ",\n";
        file << "  \"timestamp\": " << std::time(nullptr) << "\n";
        file << "}\n";

        RCLCPP_INFO(this->get_logger(), "Saved results to %s", filename.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to open file for saving results: %s", filename.c_str());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavsVehicleNodeSceneCreator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

