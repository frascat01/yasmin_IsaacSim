#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "yasmin/blackboard.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"
#include "yasmin_ros/get_parameters_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Pose = geometry_msgs::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
// Constants for state outcomes
const std::string HAS_NEXT = "has_next"; ///< Indicates there are more waypoints
const std::string END = "end";           ///< Indicates no more waypoints
enum RecoveryAttempts {
    NONE,
    FIRST,
    SECOND,
    THIRD
};
/**
 * @class Nav2State
 * @brief ActionState for navigating to a specified pose using ROS 2 Navigation.
 */
class Nav2State : public yasmin_ros::ActionState<NavigateToPose> {
    public:
        /**
         * @brief Constructs a Nav2State object.
         *
         * Initializes the action state with the NavigateToPose action type,
         * action name, and goal creation callback.
         */
        Nav2State()
            : yasmin_ros::ActionState<NavigateToPose>(
                    "/navigate_to_pose",
                    std::bind(&Nav2State::create_goal_handler, this, _1)) {}

        /**
         * @brief Creates a goal for navigation based on the current pose in the
         * blackboard.
         *
         * @param blackboard Shared pointer to the blackboard instance holding current
         * state data.
         * @return NavigateToPose::Goal The constructed goal for the navigation
         * action.
         */
        NavigateToPose::Goal create_goal_handler(
            std::shared_ptr<yasmin::Blackboard> blackboard) {
            NavigateToPose::Goal goal;
            goal.pose.pose = blackboard->get<Pose>("pose");
            goal.pose.header.frame_id = "map"; // Set the reference frame to 'map'
            return goal;
        }
};
/**
 * @brief Recoverystate for handling recovery behavior when navigation fails.
 * 
 * @param blackboard Shared pointer to the blackboard instance holding current
 * state data.
 */
class RecoveryState : public yasmin::State {
public:
    RecoveryState(): yasmin::State({"recovering","failed"}){};


    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        int attempts = blackboard->get<int>("recovery_attempts");
        switch (attempts) {
            case RecoveryAttempts::NONE:{
                YASMIN_LOG_INFO("Trying first recovery behavior: adding offset to current pose.");
                Pose current_pose = blackboard->get<Pose>("pose");
                current_pose.position.x += 0.5; // Add offset
                current_pose.position.y += 0.5;
                blackboard->set<Pose>("pose", current_pose);
                attempts += 1;
                blackboard->set<int>("recovery_attempts", attempts);
                return "recovering";
            }
            case RecoveryAttempts::FIRST:{
                YASMIN_LOG_INFO("Trying second recovery behavior: moving to the next waypoint.");
                attempts += 1;
                blackboard->set<int>("recovery_attempts", attempts);
                return "recovering";
            }
            case RecoveryAttempts::SECOND:{
                YASMIN_LOG_INFO("Trying third recovery behavior: returning to home pose.");
                Pose home_pose = blackboard->get<Pose>("home_pose");
                blackboard->set<Pose>("pose", home_pose);
                attempts += 1;
                blackboard->set<int>("recovery_attempts", attempts);
                return "recovering";
            }
            case RecoveryAttempts::THIRD:{
                YASMIN_LOG_ERROR("All recovery attempts failed. Aborting mission.");
                return "failed";
            }
        }
    };
};

/**
 * @class IdleState
 * @brief IdleState for monitoring incoming pose requests.
 * 
 * This state checks for new pose requests and transitions based on the number
 * of times it has been called.
 */
class IdleState : 
    public yasmin_ros::MonitorState<Pose> {
public:

    IdleState():
        yasmin_ros::MonitorState<Pose>(
        "pose",
        {"outcome1"},
        std::bind(&IdleState::check_idle,this,_1,_2),
        10,
        10,
        10
    ) {

    };

    std::string check_idle(std::shared_ptr<yasmin::Blackboard> blackboard,
                            std::shared_ptr<Pose> msg){
        (void)blackboard;

        YASMIN_LOG_INFO("Request to move to new pose received.");
        YASMIN_LOG_INFO("Pose - x: %.2f, y: %.2f", msg->position.x, msg->position.y);
        YASMIN_LOG_INFO("Orientation - z: %.2f, w: %.2f", msg->orientation.z, msg->orientation.w);                            

        return "outcome1";
    };
};

/**
 * @class CharginState 
 * @brief ChargingState for monitoring level of battery. 
 */
class ChargingState: public yasmin::State {
public:
    
    int ChargingFlow;
    ChargingState(): yasmin::State({"charged", "not_charged"}), ChargingFlow(800) {};

    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        double battery_level = blackboard->get<double>("battery_level");
        battery_level += ChargingFlow;

        blackboard->set<double>("battery_level", battery_level);

        if (battery_level >= 3000.0) {
            YASMIN_LOG_INFO("Battery fully charged.");
            return "charged"; // Battery is fully charged
        } else {
            YASMIN_LOG_INFO("Charging in progress.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return "not_charged"; // Continue charging
        }
    }
};

/**
 * @class BatteryState
 * @brief BatteryState for monitoring battery levels and activating coming home
 * behavior.
 * 
 * This state checks the battery level from the blackboard and determines if
 * it is below a specified threshold. If the battery level is low, it triggers
 * the coming home behavior.
 */
class BatteryState : public yasmin::State {
private:
    double battery_capacity_;
    double consumption_per_cycle_;

public:
    BatteryState(double capacity = 45000.0, double consumption = 550.0)  // 45Ah = 45000mAh
        : yasmin::State({"battery_ok", "battery_low", "battery_critical"}),
          battery_capacity_(capacity),
          consumption_per_cycle_(consumption) {}

    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        double battery_level = blackboard->get<double>("battery_level");
        
        // Soglie basate sulla capacità totale
        double critical_threshold = 0.10 * battery_capacity_;  // 10%
        double low_threshold = 0.20 * battery_capacity_;       // 20%
        
        // Simula consumo
        battery_level -= consumption_per_cycle_;
        blackboard->set<double>("battery_level", battery_level);
        
        // Controllo livelli con if-else (non switch)
        if (battery_level <= critical_threshold) {
            RCLCPP_WARN(rclcpp::get_logger("battery"), 
                "Battery CRITICAL: %.0f mAh (%.1f%%)", 
                battery_level, (battery_level / battery_capacity_) * 100);
            return "battery_critical";
        } 
        else if (battery_level <= low_threshold) {
            RCLCPP_WARN(rclcpp::get_logger("battery"), 
                "Battery LOW: %.0f mAh (%.1f%%)", 
                battery_level, (battery_level / battery_capacity_) * 100);
            return "battery_low";
        } 
        else {
            RCLCPP_INFO(rclcpp::get_logger("battery"), 
                "Battery OK: %.0f mAh (%.1f%%)", 
                battery_level, (battery_level / battery_capacity_) * 100);
            return "battery_ok";
        }
    }
};

/**
 * @brief Initializes waypoints in the blackboard for navigation.
 *
 * @param blackboard Shared pointer to the blackboard instance to store
 * waypoints.
 * @return std::string Outcome indicating success (SUCCEED).
 */
std::string
create_waypoints(std::shared_ptr<yasmin::Blackboard> blackboard) {
  std::map<std::string, std::vector<double>> waypoints = {
      {"entrance", {1.25, 6.30, -0.78, 0.67}},
      {"bathroom", {4.89, 1.64, 0.0, 1.0}},
      {"livingroom", {1.55, 4.03, -0.69, 0.72}},
      {"kitchen", {3.79, 6.77, 0.99, 0.12}},
      {"bedroom", {7.50, 4.89, 0.76, 0.65}}};
  blackboard->set<std::map<std::string, std::vector<double>>>("waypoints",
                                                              waypoints);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @brief Selects a random set of waypoints from the available waypoints.
 *
 * @param blackboard Shared pointer to the blackboard instance to store random
 * waypoints.
 * @return std::string Outcome indicating success (SUCCEED).
 */
std::string take_random_waypoint(
    std::shared_ptr<yasmin::Blackboard> blackboard) {
  auto waypoints =
      blackboard->get<std::map<std::string, std::vector<double>>>("waypoints");
  int waypoints_num = blackboard->get<int>("waypoints_num");

  std::vector<std::string> waypoint_names;
  for (const auto &pair : waypoints) {
    waypoint_names.push_back(pair.first);
  }

  // Randomly select waypoints_num waypoints
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(waypoint_names.begin(), waypoint_names.end(), g);
  std::vector<std::string> random_waypoints(
      waypoint_names.begin(), waypoint_names.begin() + waypoints_num);

  blackboard->set<std::vector<std::string>>("random_waypoints",
                                            random_waypoints);
  return yasmin_ros::basic_outcomes::SUCCEED;
}
// /**
//  * @brief Handles the coming home behavior by retrieving the home pose.
//  *
//  * @param blackboard Shared pointer to the blackboard instance holding current
//  * state data.
//  * @return Pose The home pose for navigation.
//  */
// std::string coming_home_pose(
//     std::shared_ptr<yasmin::Blackboard> blackboard){
//     blackboard -> set<bool>("navigating", false);

//     return (blackboard -> get<bool>("navigating")) ? yasmin_ros::basic_outcomes::SUCCEED : yasmin_ros::basic_outcomes::ABORT;
// }



/**
 * @brief HomeState for navigating to the home position.
 * 
 * @param blackboard Shared pointer to the blackboard instance holding current
 * state data.
 * @return std::string Outcome indicating success (SUCCEED),abort (ABORT) and cancel
 * (CANCEL).
 */
class HomeState : public yasmin_ros::ActionState<NavigateToPose> {
public:
    /**
     * @brief Constructs a HomeState object.
     *
     * Initializes the action state with the NavigateToPose action type,
     * action name, and goal creation callback.
     */
    HomeState()
        : yasmin_ros::ActionState<NavigateToPose>(
                "/navigate_to_pose",
                std::bind(&HomeState::home_handler, this, _1)) {}

    /**
     * @brief Creates a goal for navigation based on the current pose in the
     * blackboard.
     *
     * @param blackboard Shared pointer to the blackboard instance holding current
     * state data.
     * @return NavigateToPose::Goal The constructed goal for the navigation
     * action.
     */
    NavigateToPose::Goal home_handler(
        std::shared_ptr<yasmin::Blackboard> blackboard) {
        NavigateToPose::Goal goal;
        goal.pose.pose = blackboard->get<Pose>("home_pose");
        goal.pose.header.frame_id = "map"; // Set the reference frame to 'map'
        return goal;
    }
};

/**
 * @brief Retrieves the next waypoint from the list of random waypoints.
 *
 * Updates the blackboard with the pose of the next waypoint.
 *
 * @param blackboard Shared pointer to the blackboard instance holding current
 * state data.
 * @return std::string Outcome indicating whether there is a next waypoint
 * (HAS_NEXT) or if navigation is complete (END).
 */
class NextWaypointState: public yasmin::State {
public:
    NextWaypointState(): yasmin::State({END, HAS_NEXT}) {}; 
    
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override{
        if (!(blackboard -> get<bool>("navigating"))){
            Pose home_pose = blackboard -> get<Pose>("home_pose");
            if (blackboard->get<Pose>("pose")==home_pose) return END;
            blackboard -> set<Pose>("pose", home_pose);
            blackboard -> set<std::string>("text", "Returning to home position.");
            return HAS_NEXT;
        } else {
            auto random_waypoints =
                blackboard->get<std::vector<std::string>>("random_waypoints");
            auto waypoints =
                blackboard->get<std::map<std::string, std::vector<double>>>("waypoints");

            if (random_waypoints.empty()) {
                return END;
            }

            std::string wp_name = random_waypoints.back();
            random_waypoints.pop_back();
            blackboard->set<std::vector<std::string>>("random_waypoints",
                                                        random_waypoints);

            auto wp = waypoints.at(wp_name);

            Pose pose;
            pose.position.x = wp[0];
            pose.position.y = wp[1];
            pose.orientation.z = wp[2];
            pose.orientation.w = wp[3];

            blackboard->set<Pose>("pose", pose);
            blackboard->set<std::string>("text", "I have reached waypoint " + wp_name);

            return HAS_NEXT;
        }
    }
};
    /**
 * @class EmergencyState
 * @brief EmergencyState for stopping the robot in emergency situations.
 * 
 * This state publishes a zero-velocity command to halt the robot immediately.
 */
class EmergencyState: public yasmin_ros::PublisherState<Twist>{
public:
    EmergencyState(): yasmin_ros::PublisherState<Twist>("cmd_vel",
                                                        std::bind(&EmergencyState::stopping_message,this,_1)
                                                ) {};

    Twist stopping_message(std::shared_ptr<yasmin::Blackboard> blackboard){
        YASMIN_LOG_INFO("Emergency stop activated. Robot halted.");
        Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
        return msg;
    };
};
class CheckingReasonState: public yasmin::State {
    public:
    CheckingReasonState(): yasmin::State({"is_emergency","new_goal"}){};



    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        bool emergency = blackboard->get<bool>("emergency_stop");
        if (emergency) {
            YASMIN_LOG_WARN("Emergency stop requested.");
            blackboard->set<bool>("emergency_stop", false); // Reset flag
            return "is_emergency";
        } else {
            YASMIN_LOG_INFO("New goal received, proceeding with navigation.");
            return "new_goal";
        }
    };
};

int main(int argc, char *argv[]) {

    YASMIN_LOG_INFO("yasmin_nav2_demo");
    rclcpp::init(argc, argv);

    // Set ROS 2 logs
    yasmin_ros::set_ros_loggers();

    // Create state machines
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                                            yasmin_ros::basic_outcomes::ABORT,
                                            yasmin_ros::basic_outcomes::CANCEL},
        true);
    auto nav_sm = std::make_shared<yasmin::StateMachine>(
        std::initializer_list<std::string>{"Exiting from Navigation",
                                            yasmin_ros::basic_outcomes::ABORT,
                                            yasmin_ros::basic_outcomes::CANCEL});
    // Add states to the state machine
    sm->add_state("IDLE", std::make_shared<IdleState>(),
                        {
                            {"outcome1", "CREATING_WAYPOINTS"},
                            {yasmin_ros::basic_outcomes::TIMEOUT, yasmin_ros::basic_outcomes::ABORT},
                        });
    // sm-> add_state("COMING_HOME",
    //                 std::make_shared<yasmin::CbState>(
    //                     std::initializer_list<std::string>{
    //                         yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::ABORT},
    //                     coming_home_pose),
    //                 std::map<std::string, std::string>{
    //                     {yasmin_ros::basic_outcomes::SUCCEED,"CHARGING"},
    //                     {yasmin_ros::basic_outcomes::ABORT, "EMERGENCY_STOP"},
    //                 });

    sm->add_state(
        "COMING_HOME", std::make_shared<HomeState>(),
        std::map<std::string, std::string>{
            {yasmin_ros::basic_outcomes::SUCCEED, "CHARGING"},
            {yasmin_ros::basic_outcomes::CANCEL,
            yasmin_ros::basic_outcomes::CANCEL},
            {yasmin_ros::basic_outcomes::ABORT,
            "EMERGENCY_STOP"}});
    sm->add_state(
        "CREATING_WAYPOINTS",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED},
            create_waypoints),
        std::map<std::string, std::string>{
            {yasmin_ros::basic_outcomes::SUCCEED, "TAKING_RANDOM_WAYPOINTS"}});
    sm->add_state("TAKING_RANDOM_WAYPOINTS",
                    std::make_shared<yasmin::CbState>(
                        std::initializer_list<std::string>{
                            yasmin_ros::basic_outcomes::SUCCEED},
                        take_random_waypoint),
                    std::map<std::string, std::string>{
                        {yasmin_ros::basic_outcomes::SUCCEED, "NAVIGATING"}});
    
    sm->add_state("EMERGENCY_STOP",
                    std::make_shared<EmergencyState>(),
                    {
                        {yasmin_ros::basic_outcomes::SUCCEED,"IDLE"},
                    });
    sm->add_state("CHARGING",std::make_shared<ChargingState>(),
                    std::map<std::string, std::string>{
                        {"charged", "IDLE"},
                        {"not_charged", "CHARGING"}});
    // nav_sm->add_state(
    //     "BATTERY_MONITORING", std::make_shared<BatteryState>(45000.0, 550.0),
    //     std::map<std::string, std::string>{
    //         {"battery_ok", "Finished"},
    //         {"battery_low", yasmin_ros::basic_outcomes::CANCEL},
    //         {"battery_critical", yasmin_ros::basic_outcomes::ABORT}});       
    
    // nav_sm->add_state(
    //     "GETTING_NEXT_WAYPOINT",
    //     std::make_shared<yasmin::CbState>(
    //         std::initializer_list<std::string>{END, HAS_NEXT}, get_next_waypoint),
    //     std::map<std::string, std::string>{
    //         {END, "Finished"},
    //         {HAS_NEXT, "NAVIGATING"}});
    
    auto battery_state = std::make_shared<BatteryState>(45000.0, 550.0);
    auto get_next_wp_state = std::make_shared<NextWaypointState>();

    auto concurrent_state = std::make_shared<yasmin::Concurrence>(
        std::map<std::string,std::shared_ptr<yasmin::State>>{{"BATTERY_MONITORING", battery_state},
                                                            {"GETTING_NEXT_WAYPOINT", get_next_wp_state}},
        "Critical_conditions",
        yasmin::Concurrence::OutcomeMap{
            {"Continue",
            yasmin::Concurrence::StateOutcomeMap{{"BATTERY_MONITORING", "battery_ok"},
                                                {"GETTING_NEXT_WAYPOINT", HAS_NEXT}}},
            {"Finished",
            yasmin::Concurrence::StateOutcomeMap{{"GETTING_NEXT_WAYPOINT", END},
                                                {"BATTERY_MONITORING", "battery_ok"}}},
            {"Coming_Home",
            yasmin::Concurrence::StateOutcomeMap{{"BATTERY_MONITORING", "battery_low"}}}
        }
    );

    
    nav_sm ->add_state("Concurrence", concurrent_state,
                        {
                            {"Continue", "NAVIGATING"},
                            {"Finished", "Exiting from Navigation"},
                            {"Coming_Home", "Exiting from Navigation"},
                            {"Critical_conditions", yasmin_ros::basic_outcomes::ABORT},
                        });
    
    nav_sm->add_state(
        "NAVIGATING", std::make_shared<Nav2State>(),
        std::map<std::string, std::string>{
            {yasmin_ros::basic_outcomes::SUCCEED, "Concurrence"},
            {yasmin_ros::basic_outcomes::CANCEL,
            "CHECKING_REASON"},
            {yasmin_ros::basic_outcomes::ABORT,
            "RECOVERY"}});
    nav_sm -> add_state(
        "RECOVERY", std::make_shared<RecoveryState>(),
        {
            {"recovering", "Concurrence"},
            {"failed", yasmin_ros::basic_outcomes::ABORT},
        }
    );
    nav_sm -> add_state(
        "CHECKING_REASON", std::make_shared<CheckingReasonState>(),
        {
            {"is_emergency", yasmin_ros::basic_outcomes::ABORT},
            {"new_goal", "NAVIGATING"},
        }
    );
    sm->add_state(
        "NAVIGATING", nav_sm,
        std::map<std::string, std::string>{{"Exiting from Navigation",
                                            "COMING_HOME"},
                                            {yasmin_ros::basic_outcomes::ABORT,
                                            "EMERGENCY_STOP"}});

    // Publish FSM information for visualization
    yasmin_viewer::YasminViewerPub yasmin_pub(sm, "YASMIN_NAV2_DEMO");
    yasmin_viewer::YasminViewerPub nav_pub(nav_sm, "NAVIGATION_SUBSTATE");
    // Execute the state machine
    auto blackboard = std::make_shared<yasmin::Blackboard>();
    blackboard->set<int>("waypoints_num",
                        2); // Set the number of waypoints to navigate
    blackboard ->set<double>("battery_level", 45000.0); // Initial battery level
    blackboard -> set<bool>("navigating", true); // Navigation flag
    blackboard -> set<Pose>("home_pose", Pose()); // Home pose at origin
    blackboard -> set<int>("recovery_attempts", 0); // Recovery attempts
    blackboard -> set<bool>("emergency_stop", false); // Emergency stop flag
    try {
        std::string outcome = (*sm.get())(blackboard);
        YASMIN_LOG_INFO(outcome.c_str());
        // std::string outcome_battery = (*battery_sm.get())(blackboard);
        // YASMIN_LOG_INFO(outcome_battery.c_str());
    } catch (const std::exception &e) {
        YASMIN_LOG_WARN(e.what());
    }

    rclcpp::shutdown();

    return 0;
}