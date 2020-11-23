#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <signal.h>
#include <errno.h>
#include <sstream>

#include <sys/time.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <joint_states_keeper.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <tf/tf.h>

template<typename a> a sign(a v){
    if (v > 0)
        return 1;
    if ( v < 0)
        return -1;
    return 0;
};

double operator-(const timeval &a, const timeval &b) {
    return (a.tv_sec - b.tv_sec) + 1.e-6 * (a.tv_usec - b.tv_usec);
};

std::ostream & operator<<(std::ostream & o_str, const std::vector<double> &v)
{
    o_str<<" [";
    for(auto const &j: v ) { o_str<<j<<"; ";};
    o_str<<"]";
}

JointStatesKeeper::JointStatesKeeper(ros::NodeHandle & nh_):
        wasInitialized(false)
        , activeJoint(0) , opMode(MODE_JOINTS)
        , allowOrientationChange(false)
        , allowLongMovements(false)
        , allowBigJointChanges(false)
        , considerLimits(true)
        , checkSelfCollisions(true)
        , checkWorldCollisions(false)

        , move_group_int(PLANNING_GROUP)
        //, move_group_rot(PLANNING_GROUP)
        , last_update{0,0}, now{0,0}
        , ac("/execute_trajectory")
        , goal_id(0)
        , node_handle(nh_)
        , zeroVelocityWasServed(true)
        , openRequested(false)
        , closeRequested(false)
        , noPresencePreviously(false)
{
        ax[0] = {0,0}; ax[1] = {0,0}; ax[2] = {0,0};
        for(auto i=0; i < 16; ++i) buttonsLast[i] = 0;

        ROS_INFO("Waiting for execute traj Action Server become available");
        ac.waitForServer();
        ROS_INFO("Server appeared. Going on");

            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            robot_model = robot_model_loader.getModel();
            /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
            robot_state = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model));
            joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
  // that maintains the state of the world (including the robot).
            planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));

  // Configure a valid robot state  !!!! Should be done each time we re-plan next move
  //planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  std::string planner_plugin_name;
    // hard-coding planner for now. refer to motion_planning_api_tutorial.cpp
    // to see how this should be done properly
    planner_plugin_name = "ompl_interface/OMPLPlanner";
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
};

JointStatesKeeper::~JointStatesKeeper()
{
}

void JointStatesKeeper::jointStatesCallback(sensor_msgs::JointStatePtr jStates)
{
    lastJointStatesMessage = *jStates;
    wasInitialized = true;
}

void JointStatesKeeper::updateAxesState(const struct axis_state axes[3])
{
        // special check for changes in ax2.y
        if (ax[2].y != 0 && axes[2].y == 0) {
            activeJoint += sign(ax[2].y);
            std::cout<<"Changed active axes to "<<activeJoint<<std::endl;
            if (activeJoint < 0)
                activeJoint = 0;
            if (activeJoint >= lastJointStatesMessage.position.size())
                activeJoint = lastJointStatesMessage.position.size()- 1;

            std::cout<<"after correction "<<activeJoint<<std::endl;
        }
        for(auto i=0; i < 3; ++i) ax[i] = axes[i];
        //
}

void JointStatesKeeper::publishJointsCommand(ros::Publisher & chat)
{
        switch (opMode) {
            case MODE_JOINTS:
                chat.publish(getJointsCommand_Singled());
                break;
            case MODE_3D:
                keepMoving(chat);
                break;
        }
}

trajectory_msgs::JointTrajectory JointStatesKeeper::getJointsCommand()
{
        switch (opMode) {
            case MODE_JOINTS:
                return getJointsCommand_Singled();
                break;
            case MODE_3D:
                return getTrajectoryForMove();
                break;
        }
}

trajectory_msgs::JointTrajectory JointStatesKeeper::getJointsCommand_Singled() const
{
        trajectory_msgs::JointTrajectory result {};
        result.header = std_msgs::Header {};
        result.joint_names = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint",
                "a4_joint", "a5_joint", "a6_joint" };

        trajectory_msgs::JointTrajectoryPoint point;
        for( auto const &i : lastJointStatesMessage.position) {
            point.positions.push_back(i);
        }
        if( activeJoint >= 0  && activeJoint <  point.positions.size()) {
            // TODO add account of latest sent data, so that
            // no "returning" movement occures
            point.positions[activeJoint] += 0.5 *ax[0].x / 32000.; // 320000 means 1 rad
        }
        point.time_from_start = ros::Duration(2);

        result.points.push_back(point);

        return result;
};

trajectory_msgs::JointTrajectory JointStatesKeeper::getTrajectoryForMove()
{
        trajectory_msgs::JointTrajectory result {};
        result.header = std_msgs::Header {};
        result.joint_names = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint",
                "a4_joint", "a5_joint", "a6_joint" };
        double fraction;
        moveit_msgs::RobotTrajectory trajectory = getMoveItMove(fraction);
        if (fraction > 0.3)
            return trajectory.joint_trajectory;
        return result;
}

moveit_msgs::RobotTrajectory JointStatesKeeper::getMoveItMove(double &fraction)
{
        move_group_int.setStartStateToCurrentState();
        geometry_msgs::Pose target_pose3 = move_group_int.getCurrentPose().pose;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose3);
        // joystick ax0 and ax1 returns vertical direction negative if stick moved from self
        target_pose3.position.x -= 0.1 * ax[1].y / 32000 ; //32000 -> 0.01 cm @ 200 Hz rate
        target_pose3.position.y += 0.1 * ax[0].x / 32000 ; //32000 -> 0.01 cm @ 200 Hz rate
        target_pose3.position.z -= 0.1 * ax[0].y / 32000 ; //32000 -> 0.01 cm @ 200 Hz rate
        waypoints.push_back(target_pose3);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        fraction = move_group_int.computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);
        //std::cout<<"Moving from "<<waypoints[0]<< " to "<<waypoints[1]<<"\n"
        //    <<"  Prodused a result : "<<fraction<<" and trajectory "<<
        //        trajectory<<std::endl;

        return trajectory;
}

void JointStatesKeeper::updateButtons(int butNum, int value)
{
        if(butNum == 4 && value == 0 && buttonsLast[butNum] != 0) {
            opMode = MODE_3D;
            std::cout<<"operation mode set to 3D"<<std::endl;
        } 
        if(butNum == 3 && value == 0 && buttonsLast[butNum] != 0) {
            opMode = MODE_JOINTS;
            std::cout<<"operation mode set to joint-based"<<std::endl;
        } 
        if(butNum == 1 && value == 0 && buttonsLast[butNum] != 0) {
            allowOrientationChange = ! allowOrientationChange;
            std::cout<<"Orientation change during path search is set to"<<
                allowOrientationChange<<std::endl;
        }
        if(butNum == 0 && value == 0 && buttonsLast[butNum] != 0) {
            allowLongMovements = !allowLongMovements;
            std::cout<<"Long movements allowence is set to"<<
                allowLongMovements<<std::endl;
        }
        if(butNum == 8 && value !=0 && buttonsLast[butNum] == 0) {
            openRequested = true;
        }
        if(butNum == 9 && value != 0 && buttonsLast[butNum] == 0) {
            closeRequested = true;
        }

ROS_INFO_STREAM("   setting button #"<<butNum<<"  to value="<<value);

        buttonsLast[butNum] = value;
};

void JointStatesKeeper::publishGripCmd(ros::Publisher & p)
{
        std_msgs::String msg;
        if(openRequested) {
            msg.data = "open";
            p.publish(msg);
            openRequested = false;
        }
        if(closeRequested) {
            msg.data = "close";
            p.publish(msg);
            closeRequested = false;
        }
}

void JointStatesKeeper::move()
{
        if (opMode != MODE_3D)
            return;

        gettimeofday(&now, nullptr);
        if( now - last_update < 2.)
            return;

        if( ax[0].x ==0 && ax[1].y == 0 && ax[0].y == 0)
        {  if (zero_delta_was_sent)
            return;
            zero_delta_was_sent = true;
        } else {
            zero_delta_was_sent = false;
        }

        last_update = now;

        move_group_int.setStartStateToCurrentState();
        geometry_msgs::Pose target_pose3 = move_group_int.getCurrentPose().pose;
        target_pose3.position.x -= 0.05 * ax[1].y / 32000 ; //32000 -> 0.01 cm @ 200 Hz rate
        target_pose3.position.y += 0.05 * ax[0].x / 32000 ; //32000 -> 0.01 cm @ 200 Hz rate
        target_pose3.position.z -= 0.05 * ax[0].y / 32000 ; //32000 -> 0.01 cm @ 200 Hz rate

        move_group_int.setPoseTarget(target_pose3);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_int.plan(my_plan) 
                            == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s   computed in %g secs",
                success ? "" : "FAILED", my_plan.planning_time_);

        ROS_INFO_STREAM("Plan contains "<<my_plan.trajectory_.joint_trajectory.points.size()<<"  points ");
        //ROS_INFO_STREAM("Points are: "<<my_plan.trajectory_.joint_trajectory );
        ROS_INFO("Sending new goal to AC ");
        moveit_msgs::ExecuteTrajectoryGoal goal;
        goal.trajectory = my_plan.trajectory_;

        ac.sendGoal(goal);

        if (success) {
      //ROS_INFO("before moveGroup::move");
            //move_group_int.execute(my_plan);
      //      ROS_INFO_STREAM("async resulted in "<<move_group_int.asyncExecute(my_plan));
      //ROS_INFO("after moveGroup::move");
        }
}

#ifdef USE_MOVEIT
void JointStatesKeeper::keepMoving(ros::Publisher & chat)
{
        // TODO добавить историю уже отправленных заданий на перемещение.
        // пока работаем с текущим положением и ??

        //forcing rotation
        if (buttonsLast[6])
        {
            serverRotation(chat);
            return;
        }

        geometry_msgs::PoseStamped pose;
        geometry_msgs::Pose poseNow;
        pose.header.frame_id = "world";
        pose.pose = move_group_int.getCurrentPose().pose;
        poseNow = move_group_int.getCurrentPose().pose;

        bool noMovementState = ((ax[1].y == 0) && (ax[0].x  == 0) && (ax[0].y  == 0) ); 

        if (noMovementState) {
            zeroVelocityWasServed = true;
            if (zeroVelocityWasServed) {
                //there's nothing to be done. we already sent
                // stopping sequence and can do nothing
                return;
            }
            pose.pose.position.x -= LINEAR_VELOCITY_MAXIMAL * ax_previous[1].y / 32000;
            pose.pose.position.y += LINEAR_VELOCITY_MAXIMAL * ax_previous[0].x / 32000;
            pose.pose.position.z -= LINEAR_VELOCITY_MAXIMAL * ax_previous[0].y / 32000;
            
        }
        //how fast can we stop?
        // stop path is known for each joint. maximal path defines total stopping time
        // TODO determine position which can be reached while each joint is stopping
        //
        //pose.pose.position.x -= 0;
        //pose.pose.position.y += 0;
        //pose.pose.position.z -= 0;
        // which velocity do we already have?
        // TODO determine cartesian velocity
        // and based on it, estimate next position (which we can reach in 2-3 time quants)
        else {
            pose.pose.position.x -= LINEAR_VELOCITY_MAXIMAL * ax[1].y / 32000;
            pose.pose.position.y += LINEAR_VELOCITY_MAXIMAL * ax[0].x / 32000;
            pose.pose.position.z -= LINEAR_VELOCITY_MAXIMAL * ax[0].y / 32000;

            zeroVelocityWasServed = false;
        }
        ax_previous[0] = ax[0];
        ax_previous[1] = ax[1];
        ax_previous[2] = ax[2];

        // A tolerance of 0.001 m is specified in position
        // and 0.001 radians in orientation
        ros::Time start = ros::Time::now();
        /* Check that the planning was successful */
        moveit_msgs::MotionPlanResponse response;

        if(! solvePlanningToResponse(response, pose))
        {
            ROS_ERROR("Could not compute plan successfully");
            return;
        }

        // OK вот тут у нас есть список точек, через которые нужно пройти, для достижения
        // запрошенной позиции.
        // ещё мы знаем последние мгновенные скорости на узлах
        // в будущем ограничения выдернуть из сервера параметров, пока хард-кодим:
        //   максимальное ускорение, максимальную скорость на моторе

        //  временно введём ограничение на максимальный путь проходимый "инструментом"
        double path_by_this_trajectory = accumulateJogDistances(
                        response.trajectory.joint_trajectory.points);

        if (path_by_this_trajectory > 0.1) {
            ROS_INFO_STREAM("Received track with path "<<path_by_this_trajectory<<
                    " while distance between points is "<<
                std::sqrt( (poseNow.position.x - pose.pose.position.x) *
                            (poseNow.position.x - pose.pose.position.x) +
         (poseNow.position.y - pose.pose.position.y) *
                            (poseNow.position.y - pose.pose.position.y) +
         (poseNow.position.z - pose.pose.position.z) *
                            (poseNow.position.z - pose.pose.position.z)));
        }
        if (path_by_this_trajectory / LINEAR_VELOCITY_MAXIMAL > 3. && ! allowLongMovements) {
            ROS_INFO("Long distance occured but it's not allowed for execution");
            zeroVelocityWasServed = true;
            return;
        }

        if (hasPivotPoints(response.trajectory.joint_trajectory.points))
        {
            ROS_INFO_STREAM(" Solution have pivots"<<std::endl<<response);
        }
        response.trajectory.joint_trajectory.header.stamp = ros::Time::now();

      if (buttonsLast[7])
      {
            response.trajectory.joint_trajectory.points[0] = response.trajectory.joint_trajectory.points.back();
            response.trajectory.joint_trajectory.points.resize(1);
            double duration = 3200 / sqrt(ax[1].y* ax[1].y + ax[0].x*ax[0].x + ax[0].y*ax[0].y);
            double lf, itgr;
            lf = modf(duration, &itgr);
            response.trajectory.joint_trajectory.points[0].time_from_start = ros::Duration( static_cast<int>(itgr), static_cast<int>(lf * 1000000000));

      }
      else{

        auto prev_position = response.trajectory.joint_trajectory.points.front().positions;
        auto vitesseFinale = jointVelocities;
    //DIRTY for manual debug only
        if (vitesseFinale.size() < 6) vitesseFinale.resize(6);
        double momentBefore = 0.;
        for(auto &i: response.trajectory.joint_trajectory.points) {
            double maxDuration = maxDurationToPosition(i.positions, prev_position, vitesseFinale, noMovementState);
            auto diff = i.positions;
            auto k = prev_position.cbegin();
            auto j = diff.begin();
            for(; j != diff.end(); ++k, ++j) { *j -= *k;};

            vitesseFinale = calculateVitesseFinale(diff, vitesseFinale, maxDuration, noMovementState);
            // based on actual velocity of joints,
            // we can calculate how fast each joint will reach new position
            // maximal duration - is time_from_start for this waypoint
            // velocities are not used by ROS-controller, so we don't generate it
            // for the moment
            i.velocities = vitesseFinale;
            double frac, sol;
            momentBefore += maxDuration;
            frac = modf(momentBefore, &sol);
            i.time_from_start = ros::Duration(static_cast<int>(sol), static_cast<int>( frac * 1e9));
            prev_position = i.positions;
            if (noMovementState && isZeroVelocityReached(i.velocities))
                break;
        }
        if (noMovementState) {
            auto i = response.trajectory.joint_trajectory.points.size();
            auto m = i;
            for(i=0; i < m; ++i)
                if( isZeroVelocityReached(response.trajectory.joint_trajectory.points[i].velocities))
                {
                    i++;
                    break;
                }
            response.trajectory.joint_trajectory.points.resize(i);
            /*auto i = response.trajectory.joint_trajectory.points.begin();
            while( ! isZeroVelocityReached(i->velocities) &&
                i != response.trajectory.joint_trajectory.points.end()) ++i;
            if( i != response.trajectory.joint_trajectory.points.end()) ++i;

            auto c = std::vector< trajectory_msgs::JointTrajectoryPoint > {};
            c.assign(
                    response.trajectory.joint_trajectory.points.begin(), i);
            response.trajectory.joint_trajectory.points = c;
            */
            zeroVelocityWasServed = true;
        }

        response.trajectory.joint_trajectory.points.erase(
            response.trajectory.joint_trajectory.points.begin());

        // if we are here - we definitely have what to publish
        ROS_INFO_STREAM("Resulted in "<<response.trajectory.joint_trajectory.points.size()
            <<"  steps, taking "<<momentBefore<<" seconds to execute");
        ROS_INFO_STREAM("sending trajectory "<<std::endl<<response);
        ros::Time end = ros::Time::now();
        auto dura = end - start;
        ROS_INFO_STREAM("My settings of time stamps takes "<<dura);
      }

        chat.publish( response.trajectory.joint_trajectory);
}
#else
//forcing usage of KDL
void JointStatesKeeper::keepMoving(ros::Publisher & chat)
{
        // TODO добавить историю уже отправленных заданий на перемещение.
        // пока работаем с текущим положением и ??

    KDL::Frame ee_frame_pos_To, ee_frame_pos_FROM;
    KDL::JntArray jointpositions = KDL::JntArray(kdlChain.getJointsCount());
    KDL::JntArray jpNext = KDL::JntArray(kdlChain.getJointsCount());
    //ATT assumed that jointPositions array is of the same size
    // as KDL's jointpositions
    trajectory_msgs::JointTrajectory joint_trajectory;
    // установка момента приводит к появлению ошибок вида
//[ WARN] [1594631963.014984367]: Dropping first 1 trajectory point(s) out of 6, as they occur before the current time.
//First valid point will be reached in 0.544s.
//[ WARN] [1594631963.015271621]: Dropping first 1 trajectory point(s) out of 6, as they occur before the current time.
//First valid point will be reached in 0.480s.
//[ WARN] [1594631963.059127649]: Dropping first 1 trajectory point(s) out of 6, as they occur before the current time.
//First valid point will be reached in 0.653s.
//[ WARN] [1594631963.059456104]: Dropping first 1 trajectory point(s) out of 6, as they occur before the current time.
//First valid point will be reached in 0.383s.
//[ WARN] [1594631963.103056069]: Dropping first 1 trajectory point(s) out of 6, as they occur before the current time.
//First valid point will be reached in 0.409s.

    //joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "world";
    joint_trajectory.joint_names = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint", "a4_joint", "a5_joint", "a6_joint"};
    ros::Time now = ros::Time::now();
    updateKDLJointArrayFromTimeAndVelocity(jointpositions, now, lastJointStatesMessage, lastSentJointTrajectory);
    if( !kdlChain.convertJointsToFrame(ee_frame_pos_FROM, jointpositions))
    {
        ROS_FATAL_STREAM("failed to convert to Frame from joints "<<lastJointStatesMessage);
        return;
    }
    ee_frame_pos_To = ee_frame_pos_FROM;

    bool noMovementState = ((ax[1].y == 0) && (ax[0].x  == 0) && (ax[0].y  == 0) ); 

    if (noMovementState) {
        if (zeroVelocityWasServed) {
                //there's nothing to be done. we already sent
                // stopping sequence and can do nothing
            return;
        }
        trajectory_msgs::JointTrajectoryPoint lastPosition;
        lastPosition.positions = lastJointStatesMessage.position;
        lastPosition.time_from_start = ros::Duration(0, 40000000);
        joint_trajectory.points.push_back(lastPosition);
        lastSentJointTrajectory = joint_trajectory;
        chat.publish(joint_trajectory);
        zeroVelocityWasServed = true;
        ROS_INFO_STREAM("sending trajectory "<<std::endl<<joint_trajectory);
        return;
    }

        //how fast can we stop?
        // stop path is known for each joint. maximal path defines total stopping time
        // TODO determine position which can be reached while each joint is stopping
        //
        //pose.pose.position.x -= 0;
        //pose.pose.position.y += 0;
        //pose.pose.position.z -= 0;
        // which velocity do we already have?
        // TODO determine cartesian velocity
        // and based on it, estimate next position (which we can reach in 2-3 time quants)
    else {
        ee_frame_pos_To.p.data[0] -= LINEAR_VELOCITY_MAXIMAL * ax[1].y / 32000;
        ee_frame_pos_To.p.data[1] += LINEAR_VELOCITY_MAXIMAL * ax[0].x / 32000;
        ee_frame_pos_To.p.data[2] -= LINEAR_VELOCITY_MAXIMAL * ax[0].y / 32000;

        zeroVelocityWasServed = false;
    }
    ax_previous[0] = ax[0];
    ax_previous[1] = ax[1];
    ax_previous[2] = ax[2];

    ROS_INFO_STREAM("Started keepMoving call with latest known:"<<lastJointStatesMessage);

    bool planningResult;
    // A tolerance of 0.001 m is specified in position
    // and 0.001 radians in orientation
    ros::Time start = ros::Time::now();

    if ( buttonsLast[6])
        planningResult = solveRotationPlanningToResponse(joint_trajectory, ax,
            ee_frame_pos_FROM, jointpositions);
    else
        planningResult = solvePlanningToResponse(joint_trajectory, ee_frame_pos_To,
            ee_frame_pos_FROM, jointpositions, allowOrientationChange);

    if(! planningResult)
    {
        ROS_ERROR("Could not compute plan successfully");
        //TODO rework simple "return" statement to case with noMovementState (which is
        // stopping case
        return;
    }

        // OK вот тут у нас есть список точек, через которые нужно пройти, для достижения
        // запрошенной позиции.
        // ещё мы знаем последние мгновенные скорости на узлах
        // в будущем ограничения выдернуть из сервера параметров, пока хард-кодим:
        //   максимальное ускорение, максимальную скорость на моторе
        //  временно введём ограничение на максимальный путь проходимый "инструментом"
    bool shouldExitFunction = sanityChecksFailed(joint_trajectory.points,
                ee_frame_pos_To, ee_frame_pos_FROM);
    if (shouldExitFunction)
    {
        // TODO sanity check can inform that joint limits would be also reached.
        // preferable way to perform is to behave as for noMovementState case
        // make fastest possible STOP
        return;
    }

    if (buttonsLast[7])
      {
            joint_trajectory.points[0] = joint_trajectory.points.back();
            joint_trajectory.points.resize(1);
            double duration = 3200 / sqrt(ax[1].y* ax[1].y + ax[0].x*ax[0].x + ax[0].y*ax[0].y);
            double lf, itgr;
            lf = modf(duration, &itgr);
            joint_trajectory.points[0].time_from_start = ros::Duration( static_cast<int>(itgr), static_cast<int>(lf * 1000000000));

      }
    else {
        double momentBefore;
        if (noMovementState) {
            momentBefore = fillVelocitiesOfStoppingTrajectory(joint_trajectory.points,
                lastJointStatesMessage.velocity);
            zeroVelocityWasServed = true;
        }
        else {
            momentBefore = fillVelocitiesOfTrajectory(joint_trajectory.points,
                lastJointStatesMessage.velocity);
        }
            // if we are here - we definitely have what to publish
            ROS_INFO_STREAM("Resulted in "<<joint_trajectory.points.size()
                <<"  steps, taking "<<momentBefore<<" seconds to execute");
            ROS_INFO_STREAM("sending trajectory "<<std::endl<<joint_trajectory);
            ros::Time end = ros::Time::now();
            auto dura = end - start;
            ROS_INFO_STREAM("My settings of time stamps takes "<<dura);
    }

    lastSentJointTrajectory = joint_trajectory;
    chat.publish(joint_trajectory);
}

#endif

bool JointStatesKeeper::solvePlanningToResponse(moveit_msgs::MotionPlanResponse &response, const geometry_msgs::PoseStamped &pose)
{
        std::vector<double> tolerance_pose(3, 0.001);
        std::vector<double> tolerance_angle(3, 0.001);
        // if orientation change is allowed which tolerances will be?

        // We will create the request as a constraint using a helper function available
        // from the
        // `kinematic_constraints`_
        // package.
        //
        // .. _kinematic_constraints:
        //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
        moveit_msgs::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints("a6_link", pose,
                tolerance_pose, tolerance_angle);

        robot_state->setJointGroupPositions(joint_model_group, lastJointStatesMessage.position);
        robot_state->update();

        planning_interface::MotionPlanRequest req;
        req.group_name = PLANNING_GROUP;
        // req.start_state = 
            sensor_msgs::JointState js;
            js.position = lastJointStatesMessage.position;
            js.name = { "a1_joint", "a2_joint", "a3_joint", "a4_joint", "a5_joint", "a6_joint"};
        req.start_state.joint_state = js;
        req.goal_constraints.push_back(pose_goal);
    //req.path_constraints =
    //req.planner_id = 
    //req.num_planning_attempts = 

        planning_interface::MotionPlanResponse res;
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(
            planning_scene, req, res.error_code_);
        planning_scene->setCurrentState(*robot_state.get());
/*
        std::vector< std::string > algs;
        planner_instance->getPlanningAlgorithms(algs);
        for(auto &i: algs) {
            std::cout<<i<<std::endl;
        }
        
        auto r = planner_instance->getPlannerConfigurations();
        for( auto i: r) {
            std::cout<<i.first<<"->"<<i.second.group<<"     "<<i.second.name<<std::endl;
            for(auto j: i.second.config) {
                std::cout<<"\t\t"<<j.first<<"    "<<j.second<<std::endl;
            }
        }
*/

    /* Call the Planner */
        context->solve(res);

        if (res.error_code_.val != res.error_code_.SUCCESS)
        {
            return false;
        }
        res.getMessage(response);
        return true;
}

double JointStatesKeeper::accumulateJogDistances(const std::vector<trajectory_msgs::JointTrajectoryPoint> &points)
{
        double sum = 0.;
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
        //Eigen::Matrix<double, 3, 1> analizedPosition;
        kinematic_state->setVariablePositions(points.front().positions);
        kinematic_state->update();
        Eigen::Affine3d fPosi =
            kinematic_state->getGlobalLinkTransform("a6_link");
        auto analizedPosition = fPosi.translation();

        for( auto const &i: points) {
            kinematic_state->setVariablePositions(i.positions);
            kinematic_state->update();

            const Eigen::Affine3d &end_effector_state =
                    kinematic_state->getGlobalLinkTransform("a6_link");
            auto nextPosition = end_effector_state.translation();
            sum += std::sqrt(
            (nextPosition(0,0)- analizedPosition(0,0)) *(nextPosition(0,0) - analizedPosition(0,0)) +
            (nextPosition(1,0)- analizedPosition(1,0)) *(nextPosition(1,0) - analizedPosition(1,0)) +
            (nextPosition(2,0)- analizedPosition(2,0)) *(nextPosition(2,0) - analizedPosition(2,0)));
            analizedPosition = nextPosition;
        }
        return sum;
}

void JointStatesKeeper::outputCoordinatedInfo( const moveit_msgs::MotionPlanResponse &response)
{
        bool needInforming = false;
    if (needInforming ) {
        ROS_INFO_STREAM("  response contains:\n"<<response.trajectory_start.joint_state);
        ROS_INFO_STREAM("    "<<response.trajectory.joint_trajectory.points.size()<<"  points");
        ROS_INFO_STREAM("  first point "<<response.trajectory.joint_trajectory.points[0]<<
            "\n last one "<<response.trajectory.joint_trajectory.points.back() );
    }

        geometry_msgs::PoseStamped joint_global_frame_pose_stamped;
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));

        kinematic_state->setVariablePositions(
            response.trajectory.joint_trajectory.points.back().positions);
        kinematic_state->update();

        const Eigen::Affine3d &end_effector_state =
                    kinematic_state->getGlobalLinkTransform("a6_link");

        ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
        ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());
        Eigen::Quaterniond q( end_effector_state.rotation());
        ROS_INFO_STREAM("Quaternion: \n" << q.x()<<"  "<<q.y()<<"  "<<q.z()<<"  "<<q.w());

        ROS_INFO("For first point same computation:");
        kinematic_state->setVariablePositions(
            response.trajectory.joint_trajectory.points[0].positions);
        kinematic_state->update();

        const Eigen::Affine3d &end_effector_state2 =
            kinematic_state->getGlobalLinkTransform("a6_link");
        ROS_INFO_STREAM("Translation: \n" << end_effector_state2.translation());
        Eigen::Quaterniond q2(end_effector_state2.rotation());
        ROS_INFO_STREAM("Rotation: \n" << end_effector_state2.rotation() );
        ROS_INFO_STREAM("Quaternion: \n" << q2.x()<<"  "<<q2.y()<<"  "<<q2.z()<<"  "<<q2.w());
}

double JointStatesKeeper::maxDurationToPosition(const std::vector<double> &targetPosition,
            const std::vector<double> &startPosition,const std::vector<double> &startVelocities, bool decel)
{
        if (decel)
            return maxDecelerationToPosition(targetPosition, startPosition, startVelocities);
        double t = 0.;
        auto i = targetPosition.size();
        auto m = i;
        for(i=0; i < m; ++i) {
            auto angular_distance = targetPosition[i] - startPosition[i];
            if (std::abs(angular_distance) < 1e-6) continue;
            auto neededDirection = angular_distance / std::abs(angular_distance);
            double fullSpeedReachTime = std::abs(
                (neededDirection * VITESSE_MAXIMALE - startVelocities[i]) / ACCELERATION);

            double det_squared =  startVelocities[i] * startVelocities[i]
                        + (neededDirection * ACCELERATION) * angular_distance;
            double s1 = (- startVelocities[i] - std::sqrt( det_squared)) / (neededDirection * ACCELERATION);
            double s2 = (- startVelocities[i] + std::sqrt( det_squared)) / (neededDirection * ACCELERATION);
            double tempRequis = (s2 > 0)?s2:s1;
            if (tempRequis > fullSpeedReachTime) {
                double fullAccPath = ACCELERATION * fullSpeedReachTime*fullSpeedReachTime/2. +
                        std::abs(startVelocities[i]) * fullSpeedReachTime;
                tempRequis = fullSpeedReachTime + (angular_distance - fullAccPath) / VITESSE_MAXIMALE;
            }
            if( tempRequis > t) t = tempRequis;
        }
        return t;
}

double JointStatesKeeper::maxDecelerationToPosition(const std::vector<double> &targetPosition,
            const std::vector<double> &startPosition,const std::vector<double> &startVelocities)
{
        double t = 0.;
        auto i = targetPosition.size();
        auto m = i;
        // we know that we need to stop. how long will we stop on each joint?
        for(i=0; i < m; ++i) {
            auto decel_time_needed = std::abs(startVelocities[i]) / ACCELERATION;
            // а сколько мы проедем за это время?
            auto angular_distance = targetPosition[i] - startPosition[i];
            if (std::abs(angular_distance) < 1e-6) continue;
            auto neededDirection = angular_distance / std::abs(angular_distance);
            double stopDistance = - neededDirection * ACCELERATION * decel_time_needed * decel_time_needed/2. +
                    startVelocities[i] * decel_time_needed;
            if (std::abs(angular_distance) < std::abs(stopDistance)) {
                // TODO
            }
            if(t < decel_time_needed) t = decel_time_needed;
        }
        return t;
}

std::vector<double> JointStatesKeeper::calculateVitesseFinale(
        const std::vector<double> &distance,
        const std::vector<double> &inV, const double &d, bool braking)
{
        std::vector<double> rv = inV;
        int idx = 0;
        for( auto &i: rv) {
            double dir = 0.;
            if (distance[idx] >=0) dir = 1;
            else dir = -1;
            if (braking) {
                auto delta = -d * dir * ACCELERATION;
                if ( (i + delta) * i < 0)
                    i = 0;
                else
                    i += delta;
            } else {
                i += d * dir * ACCELERATION;
                if (std::abs(i) > VITESSE_MAXIMALE)
                    i = i / std::abs(i) * VITESSE_MAXIMALE;
            }
            idx++;
        }
        return rv;
}

bool JointStatesKeeper::isZeroVelocityReached(const std::vector<double> &l)
{
        for( auto &i: l) {
            if( std::abs(i) > 1e-7)
                return false;
        }
        return true;
}

bool JointStatesKeeper::hasPivotPoints( std::vector<trajectory_msgs::JointTrajectoryPoint> traj_points)
{
        for(auto i= 0; i < traj_points[0].positions.size(); ++i)
        {
            double initial_direction = traj_points[1].positions[i] - traj_points[0].positions[i];
            for(auto j = 1; j < traj_points.size(); ++j)
            {
                double step_direction = traj_points[j].positions[i] -
                    traj_points[j-1].positions[i];
                if (step_direction * initial_direction < 0 )
                    return false;
            }
        }
}

void JointStatesKeeper::serverRotation(ros::Publisher & chat)
{

        //ROS_INFO("entered serverRotations");
        geometry_msgs::Pose pose;
        pose = move_group_int.getCurrentPose().pose;

        bool noMovementState = ((ax[1].y == 0) && (ax[0].x  == 0) && (ax[0].y  == 0) ); 
        if (noMovementState)
            return;

        tf::Quaternion q(pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        roll += 0.3 *  ax[1].y / 32000;
        pitch += 0.3 *  ax[0].x / 32000;
        yaw += 0.3 * ax[0].y / 32000;

        q.setRPY(roll, pitch, yaw);
        pose.orientation.x = q[0];
        pose.orientation.y = q[1];
        pose.orientation.z = q[2];
        pose.orientation.w = q[3];

        move_group_int.setStartStateToCurrentState();
        move_group_int.setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_int.plan(my_plan) 
                            == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_STREAM("orientation change request returned "<<success);

        if ( not success)
            return;

        chat.publish(my_plan.trajectory_.joint_trajectory);
}

bool JointStatesKeeper::solvePlanningToResponse(trajectory_msgs::JointTrajectory &response, const KDL::Frame &finalFrame, const KDL::Frame &initialFrame, const KDL::JntArray &startJoints, bool canChangeOrientation)
{
    KDL::JntArray fromJoints = startJoints;
    
    const int STEPS_FOR_JOG = 5;
    for(auto i =0; i <= STEPS_FOR_JOG; ++i)
    {
        KDL::JntArray toJoints = fromJoints;
        KDL::Frame stepFrame = initialFrame;
        for(auto j=0; j < 3; ++j) {
            stepFrame.p.data[j] += (finalFrame.p.data[j] - initialFrame.p.data[j])/STEPS_FOR_JOG * i;
        }
        bool r;
        if (canChangeOrientation)
        {
            r = kdlChain.getJointForFrameStartingJointsDismissOrientation(toJoints, stepFrame, fromJoints);
        } else {
            r = kdlChain.getJointForFrameStartingJoints(toJoints, stepFrame, fromJoints);
        }
        if (! r)
        {
            ROS_FATAL_STREAM("Failed to calculate sub-movement on step "<<i<<" of movement ...");
            return false;
        }
        trajectory_msgs::JointTrajectoryPoint tPoint;
        for(auto j =0; j < kdlChain.getJointsCount(); ++j) {
            tPoint.positions.push_back(toJoints(j));
        }
        response.points.push_back( tPoint);
        fromJoints = toJoints;
    }
    //
    return true;
}

double JointStatesKeeper::fillVelocitiesOfTrajectory(trajectory_msgs::JointTrajectory::_points_type & points, const sensor_msgs::JointState::_velocity_type &veloc)
{
    bool noMovementState = false;
    auto prev_position = points.front().positions;
    auto vitesseFinale = lastJointStatesMessage.velocity;
    //DIRTY for manual debug only
    if (vitesseFinale.size() < 6) vitesseFinale.resize(6);
    double momentBefore = 0.;
    for(auto &i: points) {
        double maxDuration = maxDurationToPosition(i.positions, prev_position, vitesseFinale, noMovementState);
        auto diff = i.positions;
        auto k = prev_position.cbegin();
        auto j = diff.begin();
        for(; j != diff.end(); ++k, ++j) { *j -= *k;};

        vitesseFinale = calculateVitesseFinale(diff, vitesseFinale, maxDuration, noMovementState);
            // based on actual velocity of joints,
            // we can calculate how fast each joint will reach new position
            // maximal duration - is time_from_start for this waypoint
            // velocities are not used by ROS-controller, so we don't generate it
            // for the moment
        i.velocities = vitesseFinale;
        double frac, sol;
        momentBefore += maxDuration;
        frac = modf(momentBefore, &sol);
        i.time_from_start = ros::Duration(static_cast<int>(sol), static_cast<int>( frac * 1e9));
        prev_position = i.positions;
        if (noMovementState && isZeroVelocityReached(i.velocities))
                break;
    }
    return momentBefore;
}

double JointStatesKeeper::fillVelocitiesOfStoppingTrajectory(trajectory_msgs::JointTrajectory::_points_type & points, const sensor_msgs::JointState::_velocity_type &vels)
{
    auto initialVelocityOnSegment = vels;
    auto initialPositions = points.front().positions;
    int i;
    double waypointTimestamp = 0.;
    for( auto p = std::begin( points); p != std::end(points); ++p)
    {
        std::vector<double> timesToReach(initialPositions.size(), 0.);
        std::vector<bool> willBeStopped(initialPositions.size(), false);
        int completeStopCount = 0;
        for(i=0; i < p->positions.size(); ++i)
        {
            auto S_i =  p->positions[i] - initialPositions[i];
            // нам нужно сменить направление движения?
            if ( S_i * initialVelocityOnSegment[i] < 0) {
                // pivot case. consider separately
                ROS_FATAL_STREAM("pivot for joint "<<i<<" with distance "<<S_i<<" and speed "
                    <<initialVelocityOnSegment[i]);
                double stopTime = std::abs(initialVelocityOnSegment[i]) / DECELERATION;
            } else {
                // как долго мы можем оттормаживаться?
                // или пока скорость станет 0, тогда t = abs(V)/ dec
                // или пока мы пройдём всю возможную дистанцию
                // т.е. дистанция S_i < V_i ^2 / (2 dec)
                auto det = initialVelocityOnSegment[i] * initialVelocityOnSegment[i] - 2. * S_i * DECELERATION;
                if (det >= 0.) {
                    timesToReach[i] = ( std::abs(initialVelocityOnSegment[i]) -
                        std::sqrt( det)) / DECELERATION;
                    willBeStopped[i] = false;
                } else {
                    timesToReach[i] = std::abs(initialVelocityOnSegment[i]) / DECELERATION;
                    willBeStopped[i] = true;
                    completeStopCount++;
                }
            }
        }
        p->velocities = std::vector<double> (initialPositions.size(), 0.); // filled with 0-s
        double thisSegmentDuration = 0.;
        // первый, самый простой случай: все могут оттормозиться
        if (completeStopCount == p->positions.size())
        {
            thisSegmentDuration =
                getMaxRequiredTimeToReachPositionOnDeceleration(p->positions, initialPositions,
                    initialVelocityOnSegment);
            points.erase(p+1);
        }
        // случай чуть сложнее - ни один не может оттормозиться
        else if ( completeStopCount == 0 ) 
        {
            //сложность ситуации в том, что время у нас ограничено самым быстрым суставом.
            // как только он достигнет своей целевой позиции, у остальных нет времени доводиться
            // до целей. остаётся только посчитать скорости к заданному моменту, исходя из того
            // что выполняется максимальное оттормаживание
            thisSegmentDuration = 1e6;
            for(auto j = 0; j < p->positions.size(); ++j)
            {
                if( timesToReach[j] < thisSegmentDuration)
                    thisSegmentDuration = timesToReach[j];
            }
            for(auto j =0; j < p->velocities.size(); ++j)
            {
                p->velocities[j] = initialVelocityOnSegment[j]
                        - DECELERATION * thisSegmentDuration * std::abs(initialVelocityOnSegment[j] )/initialVelocityOnSegment[j];

            }
        }
        // промежуточное состояние - часть может остановиться, другая всё ещё будет двигаться
        else {
            // используем тот же код, что и для частичной остановки с отсечкой по скорости
            thisSegmentDuration = 1e6;
            for(auto j = 0; j < p->positions.size(); ++j)
            {
                if( timesToReach[j] < thisSegmentDuration)
                    thisSegmentDuration = timesToReach[j];
            }
            for(auto j =0; j < p->velocities.size(); ++j)
            {
                p->velocities[j] = initialVelocityOnSegment[j]
                        - DECELERATION * thisSegmentDuration * std::abs(initialVelocityOnSegment[j] )/initialVelocityOnSegment[j];
                if (p->velocities[j] * initialVelocityOnSegment[j] < 0) // знак скорости сменился
                    p->velocities[j] = 0.;
            }
        }
        double frac, sol;
        waypointTimestamp += thisSegmentDuration;
        frac = modf(waypointTimestamp, &sol);
        p->time_from_start = ros::Duration(static_cast<int>(sol), static_cast<int>( frac * 1e9));
        initialPositions = p->positions;
        initialVelocityOnSegment = p->velocities;
    }
}

bool JointStatesKeeper::sanityChecksFailed(const trajectory_msgs::JointTrajectory::_points_type &points, const KDL::Frame &ee_frame_pos_To, const KDL::Frame &ee_frame_pos_FROM)
{
    bool result = false;
    if (not allowLongMovements)
        result |= sanityCheckLongMovementWillOccure(points, ee_frame_pos_To, ee_frame_pos_FROM);
    if (not allowBigJointChanges)
        result |= sanityCheckBigJointChangeWillOccure(points);
    if (checkSelfCollisions)
        result |= sanityCheckSelfCollision(points);
    if (checkWorldCollisions)
        result |=sanityCheckWorldCollision(points);

    return result;


        if (hasPivotPoints(points))
        {
            std::stringstream ss;
            ss<<"[ ";
            for(auto & i: points) { ss<<i<<"; "; }
            ss<<" ] ";
            ROS_INFO_STREAM(" Solution have pivots"<<std::endl<<ss.str());
        }
    return false;
}

void JointStatesKeeper::updateKDLJointArrayFromTimeAndVelocity(KDL::JntArray &arr,
    const ros::Time &to_moment, const sensor_msgs::JointState & lastJS, const trajectory_msgs::JointTrajectory&)
{
    auto lastVels = lastJS.velocity;
    if (lastVels.size() < 6)
        lastVels.resize(6);
    for(auto i = 0; i < lastJS.position.size(); ++i)
    {
        // TODO ATT this place should be reverted eventually
        arr(i) = lastJointStatesMessage.position[i] ;//+
                //(to_moment + calculatePositionEstimatedDuration - lastJS.header.stamp).toSec() * lastVels[i];
    }
}

double JointStatesKeeper::getMaxRequiredTimeToReachPositionOnDeceleration(
    const std::vector<double> & targetPos, const std::vector<double> &startingPos, const std::vector<double> &startVel)
{
    double rv = 0.;
    for(int i=0; i < targetPos.size(); ++i)
    {
        double S_i = targetPos[i] - startingPos[i];
        double vel_i = startVel[i];
        double thisJointTime = 0.;
        if ( vel_i * S_i < 0) {
            auto beforePivotDistance = vel_i * std::abs(vel_i) /2. / DECELERATION;
            S_i -= beforePivotDistance;
            vel_i = 0;
            thisJointTime += std::abs(vel_i) / DECELERATION;
        }
        double timeForSegment = gt4s( S_i, vel_i, VITESSE_MAXIMALE, ACCELERATION, DECELERATION);
        thisJointTime += timeForSegment;
        if (thisJointTime > rv) {
            rv = thisJointTime;
        }
    }
    return rv;
}

double JointStatesKeeper::gt4s(double S, double v, double vMAX, double acc, double dec)
{
    double rv = -1;
    double AD = acc + dec;
    double det = v*v * AD *AD - acc * AD * (v*v - 2. * dec * S);
    if (det < 0)
        return rv;

    rv = (-v*AD  + std::sqrt(det))/ ( AD * acc);
    if (v + rv * acc > vMAX)
    {
        // TODO
        ROS_ERROR_STREAM("while caculating gt4s with S ="<<S<<" v="<<v<<
            " v_max="<<vMAX<<"  acc="<<acc<<"  dec="<<dec);
        //recalculation required
    }
    return rv;
}

void JointStatesKeeper::scaleOptoSensor(double &v, bool stick, int idx)
{
    if (not stick)
    {
        v *= 2.4e-3;
        return;
    }
    static const double multi[]=
        {
            1. / 127. *2.4e-3,
            1.39/170. *2.4e-3,
            3.
        };
    if (idx < 0)
        v *= 2.4e-3;
    else {
        v *= multi[idx];
    }
}

bool JointStatesKeeper::solveRotationPlanning(trajectory_msgs::JointTrajectory &traj,
        const KDL::Frame &frame_from, const KDL::JntArray joints, bool stick)
{
    struct axis_state ax[3];
    // prepare for solveRotationPlanningToResponse  call:
    //   mimic ax[] array, so that rollMax, pitchMax and yawMax
    // are equivalent to differences from frame_from to baseOrientation + opto-Shifts
    double optoDeltas[3];
    for(auto i=0; i < 3; ++i) optoDeltas[i] = optoSensorLatest[i] - optoSensorBase[i];
    // 0 - Rall, 1 - Pitch, 2 - Yaw
    for(auto i=0; i < 3; ++i) scaleOptoSensor( optoDeltas[i], stick, i);

    if (stick) {
    ax[1].y = 32000./0.3 * optoDeltas[2];
    ax[0].x = 32000./0.3 * optoDeltas[1];  // ATT manual coefficient estimation
    ax[0].y = 32000./0.3 * optoDeltas[0];  // ATT manual coefficient estimation
    } else {
	    //final - roll taken from "yaw"
	    ax[1].y = 3200./2./0.3 * optoDeltas[0];
	    ax[0].x = 3200./2./0.3 * optoDeltas[2];
	    ax[0].y = 3200./2./0.3 * optoDeltas[1];
    }
ROS_INFO_STREAM("called solveRotationPlanningToResponse with roll="<<0.3 *  ax[1].y / 32000 <<"   pitch="<<0.3 *  ax[0].x / 32000<<"  yaw="<<0.3 * ax[0].y / 32000);

    return solveRelativeRotationPlanningToResponse(traj, ax, frame_from, joints);
}

bool JointStatesKeeper::solveRelativeRotationPlanningToResponse(
        trajectory_msgs::JointTrajectory &joint_trajectory,
        const struct axis_state ax[3],
        const KDL::Frame &initialFrame, const KDL::JntArray startJoints)
{
    double rollMax = 0.3 *  ax[1].y / 32000;
    double pitchMax = 0.3 *  ax[0].x / 32000;
    double yawMax = 0.3 * ax[0].y / 32000;
    const int STEPS_FOR_JOG = 5;
    KDL::JntArray fromJoints = startJoints;

    auto rqRota = KDL::Rotation::EulerZYX(yawMax, rollMax, pitchMax);

    KDL::Rotation finalRotation = baseOrientation * KDL::Rotation::EulerZYX(yawMax, rollMax, pitchMax);
    auto needToTurn = (initialFrame.M.Inverse() * finalRotation).GetRot();

    ROS_INFO_STREAM("Entered solveRotationPlanningToResponse with\nbaseorientatio = "<<baseOrientation<<"\nroll ="<<rollMax<<"   pitch="<<pitchMax<<"   yaw="<<yawMax<<"\ninitialFrame="<<initialFrame);

    ROS_INFO_STREAM("Found that need rotation: \n"<<needToTurn<<"while KDL rotation is:\n"<<rqRota.GetRot());
    ROS_INFO_STREAM("baseOrientation and initialFrame differs as :\n"<<(initialFrame.M.Inverse() * baseOrientation).GetRot());
    
    auto MaxAngle = needToTurn.Norm();
    if ( MaxAngle< 1e-3)
    {
        ROS_INFO_STREAM("Rotation by roll= "<<rollMax<<"  pitch="<<pitchMax<<"   yaw="<<yawMax<<"   resulted in vector with norm = "<<needToTurn.Norm()<<"     returning false");
        return false;
    }

    for(auto i=0; i <= STEPS_FOR_JOG; ++i)
    {
        KDL::JntArray toJoints = fromJoints;
        KDL::Frame stepFrame = initialFrame;

        stepFrame.M =  stepFrame.M * KDL::Rotation::Rot(needToTurn, MaxAngle/STEPS_FOR_JOG *i);
        bool r = kdlChain.getJointForRotationStartingJoints(toJoints, stepFrame, fromJoints, 10000, 1e-4);
        if (! r)
        {
            ROS_FATAL_STREAM("Failed to calculate sub-movement on step "<<i<<" of movement ...");
            return false;
        }
        trajectory_msgs::JointTrajectoryPoint tPoint;
        for(auto j =0; j < kdlChain.getJointsCount(); ++j) {
            tPoint.positions.push_back(toJoints(j));
        }
        joint_trajectory.points.push_back( tPoint);
        fromJoints = toJoints;
    }
    return true;
}

bool JointStatesKeeper::solveRotationPlanningToResponse(
        trajectory_msgs::JointTrajectory &joint_trajectory,
        const struct axis_state ax[3],
        const KDL::Frame &initialFrame, const KDL::JntArray startJoints)
{
    double rollMax = 0.3 *  ax[1].y / 32000;
    double pitchMax = 0.3 *  ax[0].x / 32000;
    double yawMax = 0.3 * ax[0].y / 32000;
    const int STEPS_FOR_JOG = 5;
    KDL::JntArray fromJoints = startJoints;

    for(auto i=0; i <= STEPS_FOR_JOG; ++i)
    {
        KDL::JntArray toJoints = fromJoints;
        KDL::Frame stepFrame = initialFrame;

        stepFrame.M = stepFrame.M *KDL::Rotation::EulerZYX(yawMax* i/STEPS_FOR_JOG,
                                                rollMax* i/STEPS_FOR_JOG,
                                                pitchMax* i/STEPS_FOR_JOG);
        bool r = kdlChain.getJointForRotationStartingJoints(toJoints, stepFrame, fromJoints, 1000, 1e-4);
        if (! r)
        {
            ROS_FATAL_STREAM("Failed to calculate sub-movement on step "<<i<<" of movement ...");
            return false;
        }
        trajectory_msgs::JointTrajectoryPoint tPoint;
        for(auto j =0; j < kdlChain.getJointsCount(); ++j) {
            tPoint.positions.push_back(toJoints(j));
        }
        joint_trajectory.points.push_back( tPoint);
        fromJoints = toJoints;
    }
    return true;
}

bool JointStatesKeeper::setPlanningOptionCallback(armin_manipulators::SetPlanningOption::Request &req, armin_manipulators::SetPlanningOption::Response &resp)
{
    bool rv = true;
    ROS_INFO_STREAM("setPlanningOptionCallback called");
    resp.result = req.value;
    switch (req.setting.command)
    {
        case armin_manipulators::OptionSetting::ENABLE_LONG_DISTANCE:
            allowLongMovements = req.value;
            break;
        case armin_manipulators::OptionSetting::PRESERVE_ORIENTATION:
            allowOrientationChange = ! req.value;
            break;
        case armin_manipulators::OptionSetting::CONSIDER_LIMITS:
            considerLimits = req.value;
// debug part
            if (req.value) {
                opMode = MODE_3D;
            } else {
                opMode = MODE_JOINTS;
            }
            break;
        case armin_manipulators::OptionSetting::USE_BOOSTED_MOVE:
            buttonsLast[7] = req.value;
            break;
        case armin_manipulators::OptionSetting::ONLY_ROTATION:
            buttonsLast[6] = req.value;
            break;
        default:
            resp.result = false;
            rv = false;
            break;
    }
    return rv;
}

bool JointStatesKeeper::givePlanningOptionCallback(armin_manipulators::SetPlanningOption::Request &req, armin_manipulators::SetPlanningOption::Response &resp)
{
    resp.result = false;
    bool rv = true;
    //ROS_INFO_STREAM("givePlanningOptionCallback called");
    switch (req.setting.command)
    {
        case armin_manipulators::OptionSetting::ENABLE_LONG_DISTANCE:
            resp.result = allowLongMovements;
            break;
        case armin_manipulators::OptionSetting::PRESERVE_ORIENTATION:
            resp.result = ! allowOrientationChange;
            break;
        case armin_manipulators::OptionSetting::CONSIDER_LIMITS:
            resp.result = considerLimits;
            break;
        default:
            rv = false;
    }
    return rv;
}

bool JointStatesKeeper::sanityCheckLongMovementWillOccure(const trajectory_msgs::JointTrajectory::_points_type &points, const KDL::Frame &ee_frame_pos_To, const KDL::Frame &ee_frame_pos_FROM)
{
    double path_by_this_trajectory = accumulateJogDistances(points);

    if (path_by_this_trajectory > 0.1) {
            double a = 0.;
            for(auto j =0; j < 3;++j) { double t = ee_frame_pos_To.p.data[j] - ee_frame_pos_FROM.p.data[j]; a += t * t; }

            ROS_INFO_STREAM("Received track with path "<<path_by_this_trajectory<<
                    " while distance between points is "<<
                std::sqrt( a));
        }
        if (path_by_this_trajectory / LINEAR_VELOCITY_MAXIMAL > 3. && ! allowLongMovements) {
            ROS_INFO("Long distance occured but it's not allowed for execution");
            return true;
        }
    return false;
}

bool JointStatesKeeper::sanityCheckBigJointChangeWillOccure(const trajectory_msgs::JointTrajectory::_points_type &points)
{
    bool result = false;
    auto prev = std::begin(points);
    auto accumulated = std::vector<double>((*prev).positions.size());
    for(auto i = std::begin(points); i != std::end(points); ++i)
    {
        for(auto j=0; j < i->positions.size(); ++j)
        {
            accumulated[j] += std::abs((*i).positions[j] - (*prev).positions[j]);
        }
        prev = i;
    }
    // How much will we allow to rotate? What is it depend on?
    double MaxRotation = M_PI / 3.;
    for(auto j =0; j < accumulated.size(); ++j)
    {
        if (accumulated[j] > MaxRotation)
            result = true;
    }
    if (result)
    {
        ROS_INFO_STREAM("sanityCheckBigJointChangeWillOccure found big joint change. Rejecting solution \n"<<points[0]<<"\n"<<points[1]<<"\n"<<points[2]<<"\n"<<points[3]<<"\n"<<points[4]<<"\n"<<points[5]);
    }
    return false;
    return result;
}

bool JointStatesKeeper::sanityCheckSelfCollision(const trajectory_msgs::JointTrajectory::_points_type &points)
{
    bool result = false;

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    //planning_scene->checkSelfCollision(collision_request, collision_result);
    robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();

    for(auto i = std::begin(points); i != std::end(points); ++i)
    {
        current_state.setVariablePositions(i->positions);
        collision_result.clear();
        planning_scene->checkSelfCollision(collision_request, collision_result);
        result |= collision_result.collision;
        if (collision_result.collision) {
            ROS_INFO_STREAM("Self collision will occure on step "<<
                static_cast<int>(std::distance(std::begin(points), i))<<
                "   Containing joint states: "<<*i);
        }
    }
    return result;
}

bool JointStatesKeeper::sanityCheckWorldCollision(const trajectory_msgs::JointTrajectory::_points_type &points)
{
    //TODO need implementation
    return false;
}

bool JointStatesKeeper::getLatestPosition_1over10_mm(int16_t &x, int16_t &y, int16_t &z)
{
    KDL::Frame  ee_frame_pos_FROM;
    KDL::JntArray jointpositions = KDL::JntArray(kdlChain.getJointsCount());
    KDL::JntArray jpNext = KDL::JntArray(kdlChain.getJointsCount());
    for(auto i = 0; i < lastJointStatesMessage.position.size(); ++i)
    {
        jointpositions(i) = lastJointStatesMessage.position[i];
    }
    if( !kdlChain.convertJointsToFrame(ee_frame_pos_FROM, jointpositions))
    {
        ROS_INFO_STREAM("failed to convert joint positions to tool position in getLatestPosition1o5mm");
        return false;
    }

    x = static_cast<int16_t>(ee_frame_pos_FROM.p[0] * 1000 * 10);
    y = static_cast<int16_t>(ee_frame_pos_FROM.p[1] * 1000 * 10);
    z = static_cast<int16_t>(ee_frame_pos_FROM.p[2] * 1000 * 10);
    return true;
}

void JointStatesKeeper::publishSphereJointsCommand(ros::Publisher & topicName, bool stickManipulator)
{
    // similar to publishJointsCommand -> keepMoving as we know that it's always in 3D mode
    // TODO DRY: this is semi-copy of keepMoving of KDL-based solver

    KDL::Frame ee_frame_pos_To, ee_frame_pos_FROM;
    KDL::JntArray jointpositions = KDL::JntArray(kdlChain.getJointsCount());
    KDL::JntArray jpNext = KDL::JntArray(kdlChain.getJointsCount());
    trajectory_msgs::JointTrajectory joint_trajectory;
    //joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "world";
    joint_trajectory.joint_names = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint", "a4_joint", "a5_joint", "a6_joint"};
    ros::Time now = ros::Time::now();
    updateKDLJointArrayFromTimeAndVelocity(jointpositions, now, lastJointStatesMessage, lastSentJointTrajectory);
    if( !kdlChain.convertJointsToFrame(ee_frame_pos_FROM, jointpositions))
    {
        ROS_FATAL_STREAM("failed to convert to Frame from joints "<<lastJointStatesMessage);
        return;
    }
    ee_frame_pos_To = ee_frame_pos_FROM;

        ee_frame_pos_To.p.data[0] -= LINEAR_VELOCITY_MAXIMAL * ax[1].y / 32000;
        ee_frame_pos_To.p.data[1] += LINEAR_VELOCITY_MAXIMAL * ax[0].x / 32000;
        ee_frame_pos_To.p.data[2] -= LINEAR_VELOCITY_MAXIMAL * ax[0].y / 32000;

        zeroVelocityWasServed = false;

    ax_previous[0] = ax[0];
    ax_previous[1] = ax[1];
    ax_previous[2] = ax[2];

    //ROS_INFO_STREAM("Started keepMoving call with latest known:"<<lastJointStatesMessage);

    bool planningResult;
    // A tolerance of 0.001 m is specified in position
    // and 0.001 radians in orientation
    ros::Time start = ros::Time::now();

    if ( buttonsLast[6])
        planningResult = solveRotationPlanning(joint_trajectory, ee_frame_pos_FROM, jointpositions, stickManipulator);
    else
        planningResult = solvePlanningToResponse(joint_trajectory, ee_frame_pos_To,
            ee_frame_pos_FROM, jointpositions, allowOrientationChange);

    if(! planningResult)
    {
        ROS_ERROR("Could not compute plan successfully");
        //TODO rework simple "return" statement to case with noMovementState (which is
        // stopping case
            //ROS_INFO_STREAM("planning failed   sending trajectory "<<std::endl<<joint_trajectory);
        //publishStop(topicName, "   :   failed planning request");
        return;
    }

        // OK вот тут у нас есть список точек, через которые нужно пройти, для достижения
        // запрошенной позиции.
        // ещё мы знаем последние мгновенные скорости на узлах
        // в будущем ограничения выдернуть из сервера параметров, пока хард-кодим:
        //   максимальное ускорение, максимальную скорость на моторе
        //  временно введём ограничение на максимальный путь проходимый "инструментом"
    bool shouldExitFunction = sanityChecksFailed(joint_trajectory.points,
                ee_frame_pos_To, ee_frame_pos_FROM);
    if (shouldExitFunction)
    {
        // TODO sanity check can inform that joint limits would be also reached.
        // preferable way to perform is to behave as for noMovementState case
        // make fastest possible STOP
            ROS_INFO_STREAM("shouldExitFunction flag raised   sending STOP command ");
        publishStop(topicName);
        return;
    }

    if (buttonsLast[7])
      {
            joint_trajectory.points[0] = joint_trajectory.points.back();
            joint_trajectory.points.resize(1);
            double duration = 3200 / sqrt(ax[1].y* ax[1].y + ax[0].x*ax[0].x + ax[0].y*ax[0].y);
            double lf, itgr;
            lf = modf(duration, &itgr);
            joint_trajectory.points[0].time_from_start = ros::Duration( static_cast<int>(itgr), static_cast<int>(lf * 1000000000));

      }
    else {
        double momentBefore;
        
            momentBefore = fillVelocitiesOfTrajectory(joint_trajectory.points,
                lastJointStatesMessage.velocity);
            // if we are here - we definitely have what to publish
            ROS_INFO_STREAM("Resulted in "<<joint_trajectory.points.size()
                <<"  steps, taking "<<momentBefore<<" seconds to execute");
            //ROS_INFO_STREAM("sending trajectory "<<std::endl<<joint_trajectory);
            ros::Time end = ros::Time::now();
            auto dura = end - start;
            ROS_INFO_STREAM("My settings of time stamps takes "<<dura);
    }

    lastSentJointTrajectory = joint_trajectory;
    topicName.publish(joint_trajectory);
}

double scaleOriZ(double v)
{
    if (v >= 1600 && v <= 1865)
        return 0;
    if (v > 1865) {
        return M_PI/180. * 15. * ( v - 1865)/(3600. - 1865.); 
    }
    if ( v < 1600) {
        return M_PI/180. * 15 * (v - 1600.)/(1600. - 0.);
    }
}

void JointStatesKeeper::updateSensorsAndRest(const struct data &fetchedData, bool stick)
{
    //
    if (noPresencePreviously && IS_USER_PRESENT(fetchedData))
    {
        if (not isInitialized())
        {
            //don't know what to do. TODO
            return;
        }
        // to determine required rotation we fix orientation of instrument
        // + latest position from 
      if( not stick) {
        optoSensorBase[0] = fetchedData.sensor_x;
        optoSensorBase[1] = fetchedData.sensor_y;
        optoSensorBase[2] = fetchedData.sensor_z;
      } else {
        optoSensorBase[0] = ax[1].y;
        optoSensorBase[1] = ax[0].x;
        optoSensorBase[2] = scaleOriZ(fetchedData.sensor_z);
      }
        //NOTE TODO: this part of code is for KDL-based path building.
        // require creation of similar thing for pure MoveIt based solutions
        // TODO DRY
        KDL::Frame ee_frame_pos_FROM;
        KDL::JntArray jointpositions = KDL::JntArray(kdlChain.getJointsCount());
        for(auto i = 0; i < lastJointStatesMessage.position.size(); ++i)
            { jointpositions(i) = lastJointStatesMessage.position[i]; }
        //
        kdlChain.convertJointsToFrame(ee_frame_pos_FROM, jointpositions);
        baseOrientation = ee_frame_pos_FROM.M;
    }
  if (not stick) {
    optoSensorLatest[0] = fetchedData.sensor_x;
    optoSensorLatest[1] = fetchedData.sensor_y;
    optoSensorLatest[2] = fetchedData.sensor_z;
  } else {
    optoSensorLatest[0] = ax[1].y;
    optoSensorLatest[1] = ax[0].x;
    optoSensorLatest[2] = scaleOriZ(fetchedData.sensor_z);
   }
    

    noPresencePreviously = not IS_USER_PRESENT(fetchedData);
}

void JointStatesKeeper::publishStop(ros::Publisher & chat, const std::string &from)
{
    trajectory_msgs::JointTrajectory joint_trajectory;
    //joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "world";
    joint_trajectory.joint_names = std::vector<std::string> { "a1_joint", "a2_joint", "a3_joint", "a4_joint", "a5_joint", "a6_joint"};
    chat.publish(joint_trajectory);
    ROS_INFO_STREAM("Published stop command from:"<<from);
}

void JointStatesKeeper::setAllowOrientationChange(bool v)
{
    allowOrientationChange = v;
}

//const std::string JointStatesKeeper::PLANNING_GROUP{ "manipulator"};
const std::string JointStatesKeeper::PLANNING_GROUP{ "base"};
const double JointStatesKeeper::LINEAR_VELOCITY_MAXIMAL = 0.15; //meter per sec
const double JointStatesKeeper::AVERAGE_BETWEEN_MOVE_DURATION = 0.1;
const double JointStatesKeeper::ACCELERATION = 100. * 2.*M_PI/ 60. * 1e-3; // 100 rpm/s * rpm_to_rads_per_sec * A1_transmission
const double JointStatesKeeper::DECELERATION = 1000. * 2.*M_PI/ 60. * 1e-3; // 1000 rpm/s * rpm_to_rads_per_sec * A1_transmission
const double JointStatesKeeper::VITESSE_MAXIMALE = 4000. * 2.*M_PI/60. * 1e-3 ; // 4000 rpm * rpm_to_rads_per_sec * A1_transmission
const ros::Duration JointStatesKeeper::calculatePositionEstimatedDuration { 0, 30000000}; // all keepMoving call for KDL-based estimation was taking about 3ms
                                                                                            // but setting timestamp for query results in ROS-Controller informing that message is "bla-bla" microseconds behind
                                                                                            // thus, setting forward time to +30ms

