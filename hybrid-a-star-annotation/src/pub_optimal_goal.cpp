#include "pub_optimal_goal.h"
#include "unistd.h"

using namespace HybridAStar;

#define PI 3.1415926

OptimalGoal::OptimalGoal()
{
    subMap = nh.subscribe("/local_map_occ", 1, &OptimalGoal::setMap, this);
    subGoal = nh.subscribe("/visual_follow_move/navigation_goal", 1, &OptimalGoal::setGoal, this);
    pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pubDetectedBorder = nh.advertise<geometry_msgs::PolygonStamped>("/obs_detected_border", 1);

    optimal_goal.header.frame_id = Constants::map_frame;
    optimal_goal.pose.position.z = 0;
    optimal_goal.pose.orientation.x = 0.707107;
    optimal_goal.pose.orientation.y = 0.707107;
    optimal_goal.pose.orientation.z = 0.0;
    optimal_goal.pose.orientation.w = 0.0;

    goal_time = ros::Time::now().toSec();

    x0 = 5.0;
    y0 = 4.6;
    r2 = 2.5;
};

void OptimalGoal::calcCrossPointsOfTwoCircles(double x1, double y1, double r1, double x2, double y2, double r2)
{
    double t = atan2(x1 - x2, y1 - y2);
    double a = acos(fminl(fmaxl(pow(r1, 2) / (2 * r1 * r2), -1.0), 1.0));

    // left
    x3 = x1 - r1 * sin(t + a);
    y3 = y1 - r1 * cos(t + a);
    // right
    x4 = x1 - r1 * sin(t - a);
    y4 = y1 - r1 * cos(t - a);
}

bool OptimalGoal::isNearObstacle(int x, int y)
{
    for (int i = -4; i < 4; i++)
    {
        for (int j = -4; j < 4; j++)
        {
            if (grid->data[x + j + (y + i) * 100])
            {
                std::cout << "Danger grid in map[" << (x + j) * Constants::cellSize << ", " << (y + i) * Constants::cellSize << "]." << std::endl;
                return true;
            }
        }
    }
    std::cout << "Safe." << std::endl;
    return false;
}

void OptimalGoal::showBorder(double x, double y)
{
    detectedBorder.polygon.points.clear();

    geometry_msgs::Point32 point;
    point.x = x - 0.6;
    point.y = y - 0.6;
    point.z = 0;
    detectedBorder.polygon.points.push_back(point);
    point.x = x - 0.6;
    point.y = y + 0.6;
    point.z = 0;
    detectedBorder.polygon.points.push_back(point);
    point.x = x + 0.6;
    point.y = y + 0.6;
    point.z = 0;
    detectedBorder.polygon.points.push_back(point);
    point.x = x + 0.6;
    point.y = y - 0.6;
    point.z = 0;
    detectedBorder.polygon.points.push_back(point);

    detectedBorder.header.frame_id = Constants::map_frame;
    pubDetectedBorder.publish(detectedBorder);
}

void OptimalGoal::setGoal(const geometry_msgs::PoseStamped goal_msg)
{
    goal_time = ros::Time::now().toSec();
    std::cout << "goal_time is " << std::setprecision(10) << goal_time << std::endl;
    return;
}

void OptimalGoal::setMap(const nav_msgs::OccupancyGrid::Ptr map)
{
    grid = map;
    time1 = map->header.stamp;
    // if (listener.canTransform(Constants::map_frame, "nav_goal", ros::Time(0)))
    if (abs(ros::Time::now().toSec() - goal_time) < 0.5)  // default:1s
    {
        std::cout << "goal has updated." << std::endl;
        tf::StampedTransform tf_circle;
        try
        {
            listener.waitForTransform(Constants::map_frame, "nav_goal", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(Constants::map_frame, "nav_goal", ros::Time(0), tf_circle);
        }
        catch (tf::TransformException &ex)
        {
            // ROS_ERROR("%s", ex.what());
            std::cout << "Cannot tramsform from nav_goal to local_map." << std::endl;
            return;
        }
        std::cout << "Goal is detected." << std::endl;
        x1 = tf_circle.getOrigin().getX();
        y1 = tf_circle.getOrigin().getY();
        cx1 = round(x1 / Constants::cellSize);
        cy1 = round(y1 / Constants::cellSize);
        if (!isNearObstacle(cx1, cy1))
        {
            std::cout << "x1:" << x1 << " ,y1:" << y1 << std::endl;
            showBorder(x1, y1);
            optimal_goal.pose.position.x = x1;
            optimal_goal.pose.position.y = y1;
            q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI/2 - atan((x1-x0)/(y1-y0)));
            optimal_goal.pose.orientation.x = q.x;
            optimal_goal.pose.orientation.y = q.y;
            optimal_goal.pose.orientation.z = q.z;
            optimal_goal.pose.orientation.w = q.w;
            if ((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1) <= 3.0)
            {
                return;
            }
            pubGoal.publish(optimal_goal);
            return;
        }
        else
        {
            tf::StampedTransform tf_target;
            try
            {
                listener.waitForTransform(Constants::map_frame, Constants::target_frame, ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform(Constants::map_frame, Constants::target_frame, ros::Time(0), tf_target);
            }
            catch (tf::TransformException &ex)
            {
                // ROS_ERROR("%s", ex.what());
                std::cout << "Cannot tramsform from target to local_map." << std::endl;
                return;
            }
            x2 = tf_target.getOrigin().getX();
            y2 = tf_target.getOrigin().getY();
            for (r1 = 0.2; r1 < r2; r1 += 0.2)
            {
                std::cout << "r1:" << r1 << std::endl;
                calcCrossPointsOfTwoCircles(x1, y1, r1, x2, y2, r2);

                if (x1 > x2)  // target is on the right-front side of robot
                {
                    cx3 = round(x3 / Constants::cellSize);
                    cy3 = round(y3 / Constants::cellSize);
                    if (!isNearObstacle(cx3, cy3))
                    {
                        std::cout << "x3:" << x3 << " ,y3:" << y3 << std::endl;
                        showBorder(x3, y3);
                        optimal_goal.pose.position.x = x3;
                        optimal_goal.pose.position.y = y3;
                        q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI/2 - atan((x3-x0)/(y3-y0)));
                        optimal_goal.pose.orientation.x = q.x;
                        optimal_goal.pose.orientation.y = q.y;
                        optimal_goal.pose.orientation.z = q.z;
                        optimal_goal.pose.orientation.w = q.w;
                        if ((x0-x3)*(x0-x3) + (y0-y3)*(y0-y3) <= 3.0)
                        {
                            return;
                        }
                        pubGoal.publish(optimal_goal);
                        return;
                    }
                }

                else  // target is on the left-front side of robot
                {
                    cx4 = round(x4 / Constants::cellSize);
                    cy4 = round(y4 / Constants::cellSize);
                    if (!isNearObstacle(cx4, cy4))
                    {
                        std::cout << "x4:" << x4 << " ,y4:" << y4 << std::endl;
                        showBorder(x4, y4);
                        optimal_goal.pose.position.x = x4;
                        optimal_goal.pose.position.y = y4;
                        q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI/2 - atan((x4-x0)/(y4-y0)));
                        optimal_goal.pose.orientation.x = q.x;
                        optimal_goal.pose.orientation.y = q.y;
                        optimal_goal.pose.orientation.z = q.z;
                        optimal_goal.pose.orientation.w = q.w;
                        if ((x0-x4)*(x0-x4) + (y0-y4)*(y0-y4) <= 3.0)
                        {
                            return;
                        }
                        pubGoal.publish(optimal_goal);
                        return;
                    }
                }
            }
        }
    }
    else
    {
        std::cout << "goal is disappear." << std::endl;
        listener.clear();
        // showBorder(5.0, 5.0);
        optimal_goal.pose.position.x = 5.0;
        optimal_goal.pose.position.y = 4.6;
        pubGoal.publish(optimal_goal);
        return;
    }
}

int main(int argc, char **argv)
{
    // initiate the broadcaster
    ros::init(argc, argv, "pub_optimal_goal");

    HybridAStar::OptimalGoal og;

    sleep(1.0);

    ros::Rate r(10);
    while (ros::ok())
    {

        ros::spinOnce();

        r.sleep();
    }
}
