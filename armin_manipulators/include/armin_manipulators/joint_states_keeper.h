#ifndef JOINT_STATES_KEEPER_H
#define JOINT_STATES_KEEPER_H
#include <config.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <boost/scoped_ptr.hpp>

#include <chained_ik.h>
#include "armin_manipulators/SetPlanningOption.h"
#include "custom_types.h"

struct axis_state {
    int x, y;
};


class JointStatesKeeper
{
    bool wasInitialized;
    sensor_msgs::JointState lastJointStatesMessage;
    int activeJoint;
    struct axis_state ax[3], ax_previous[3];
    int buttonsLast[16];
    enum operationMode {
        MODE_JOINTS,
        MODE_3D
    } opMode;

    // setting flags
    bool allowOrientationChange; //!< - running in 3D moves with this flag set to TRUE results in calculating position without regarding to orientation preservation. Default is FALSE
    bool allowLongMovements; //!< - setting this flag results in execution of potentially long Cartesian distance. Default is FALSE
    bool allowBigJointChanges; //!< - setting this flag results in potential execution of trajectories involving big angular moves of joints. Default is FALSE
    bool considerLimits; //!< - setting this flag forces usage of KDL Joint Limited version of calls. Default is TRUE
    bool checkSelfCollisions; //!< - setting flag to TRUE enables self-collision checks during solution sanity checks. Default is TRUE
    bool checkWorldCollisions; //!< - setting flag to TRUE enables checking collision with objects in the world. Default os FALSE

    bool sanityCheckLongMovementWillOccure(const trajectory_msgs::JointTrajectory::_points_type &points, const KDL::Frame &ee_frame_pos_To, const KDL::Frame &ee_frame_pos_FROM);
    bool sanityCheckBigJointChangeWillOccure(const trajectory_msgs::JointTrajectory::_points_type &points);
    bool sanityCheckSelfCollision(const trajectory_msgs::JointTrajectory::_points_type &points);
    bool sanityCheckWorldCollision(const trajectory_msgs::JointTrajectory::_points_type &points);


    static const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group_int;
    //moveit::planning_interface::MoveGroupInterface move_group_rot;

    struct timeval last_update, now;
    bool zero_delta_was_sent;
    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac;
    int goal_id;

    ros::NodeHandle & node_handle;
    robot_model::RobotModelPtr robot_model;
    robot_state::RobotStatePtr robot_state;
    const robot_state::JointModelGroup* joint_model_group;
    planning_scene::PlanningScenePtr planning_scene;
    planning_interface::PlannerManagerPtr planner_instance;
    bool zeroVelocityWasServed;

    bool openRequested;
    bool closeRequested;

public:

    bool noPresencePreviously;
    double optoSensorBase[3];
    double optoSensorLatest[3];
    KDL::Rotation baseOrientation;
    void scaleOptoSensor(double&, bool, int = -1);

    static const double AVERAGE_BETWEEN_MOVE_DURATION; //meter per sec
    static const double ACCELERATION; //meter per sec
    static const double DECELERATION; //meter per sec
    static const double VITESSE_MAXIMALE; //meter per sec
    static const ros::Duration calculatePositionEstimatedDuration;

#ifdef orocos_kdl_FOUND
    InverseSolverFromKDL kdlChain;
#endif
    trajectory_msgs::JointTrajectory lastSentJointTrajectory;

    void updateKDLJointArrayFromTimeAndVelocity(KDL::JntArray &, const ros::Time &, const sensor_msgs::JointState &, const trajectory_msgs::JointTrajectory&);

    static double getMaxRequiredTimeToReachPositionOnDeceleration(const std::vector<double> &, const std::vector<double> &, const std::vector<double> &);
    static double gt4s(double S, double v, double vM, double acc, double dec);

  public:
    JointStatesKeeper(ros::NodeHandle & nh_);

    ~JointStatesKeeper();

    void jointStatesCallback(sensor_msgs::JointStatePtr jStates);
    bool setPlanningOptionCallback(armin_manipulators::SetPlanningOption::Request &req, armin_manipulators::SetPlanningOption::Response &resp);
    bool givePlanningOptionCallback(armin_manipulators::SetPlanningOption::Request &req, armin_manipulators::SetPlanningOption::Response &resp);

    bool isInitialized() const { return wasInitialized;};

    void updateAxesState(const struct axis_state axes[3]);
    void publishJointsCommand(ros::Publisher & chat);
    void publishSphereJointsCommand(ros::Publisher &, bool = false);

    trajectory_msgs::JointTrajectory getJointsCommand();

    trajectory_msgs::JointTrajectory getJointsCommand_Singled() const;

    trajectory_msgs::JointTrajectory getTrajectoryForMove();

    moveit_msgs::RobotTrajectory getMoveItMove(double &fraction);

    void updateButtons(int butNum, int value);

    void publishGripCmd(ros::Publisher & p);

    void move();

    void keepMoving(ros::Publisher & chat);

    bool solvePlanningToResponse(moveit_msgs::MotionPlanResponse &response, const geometry_msgs::PoseStamped &pose);

    double accumulateJogDistances(const std::vector<trajectory_msgs::JointTrajectoryPoint> &points);

    void outputCoordinatedInfo( const moveit_msgs::MotionPlanResponse &response);

    double maxDurationToPosition(const std::vector<double> &targetPosition,
            const std::vector<double> &startPosition,const std::vector<double> &startVelocities, bool decel);

    double maxDecelerationToPosition(const std::vector<double> &targetPosition,
            const std::vector<double> &startPosition,const std::vector<double> &startVelocities);

    std::vector<double> calculateVitesseFinale(
        const std::vector<double> &distance,
        const std::vector<double> &inV, const double &d, bool braking);

    static bool isZeroVelocityReached(const std::vector<double> &l);

    bool hasPivotPoints( std::vector<trajectory_msgs::JointTrajectoryPoint> traj_points);

    void serverRotation(ros::Publisher & chat);
    void serveRotationKDL(ros::Publisher & chat);

    bool solvePlanningToResponse(trajectory_msgs::JointTrajectory &, const KDL::Frame &, const KDL::Frame &, const KDL::JntArray &, bool = false);
    bool solveRotationPlanningToResponse(trajectory_msgs::JointTrajectory &joint_trajectory, const struct axis_state ax[3],  const KDL::Frame &ee_frame_pos_FROM, const KDL::JntArray jointpositions);
    bool solveRotationPlanning(trajectory_msgs::JointTrajectory &, const KDL::Frame &, const KDL::JntArray, bool);
    bool solveRelativeRotationPlanningToResponse(trajectory_msgs::JointTrajectory &joint_trajectory, const struct axis_state ax[3],  const KDL::Frame &ee_frame_pos_FROM, const KDL::JntArray jointpositions);

    double fillVelocitiesOfTrajectory(trajectory_msgs::JointTrajectory::_points_type & points, const sensor_msgs::JointState::_velocity_type &);
    double fillVelocitiesOfStoppingTrajectory(trajectory_msgs::JointTrajectory::_points_type & points, const sensor_msgs::JointState::_velocity_type &);

    bool sanityChecksFailed(const trajectory_msgs::JointTrajectory::_points_type &, const KDL::Frame &, const KDL::Frame &);

    bool getLatestPosition_1over10_mm(int16_t &x, int16_t &y, int16_t &z);

    void setCloseRq(bool v) { closeRequested = v;};
    void setOpenRq(bool v) { openRequested = v;};

    void updateSensorsAndRest(const struct data &, bool = false);

    void publishStop(ros::Publisher &, const std::string & = "");
    void setAllowOrientationChange(bool);

    static const double LINEAR_VELOCITY_MAXIMAL; //meter per sec
};

#endif //JOINT_STATES_KEEPER_H
