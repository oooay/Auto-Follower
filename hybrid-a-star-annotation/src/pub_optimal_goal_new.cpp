#include "pub_optimal_goal_new.h"
#include "unistd.h"

using namespace HybridAStar;

#define PI 3.1415926

OptimalGoal::OptimalGoal()
{
    subMap = nh.subscribe("/local_map_occ", 1, &OptimalGoal::setMap, this);
    subGoal = nh.subscribe("/visual_follow_move/navigation_goal", 1, &OptimalGoal::setGoal, this);
    pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pubDetectedBorder = nh.advertise<geometry_msgs::PolygonStamped>("/obs_detected_border", 1);
    pubLine = nh.advertise<visualization_msgs::Marker>("/line_from_robot_to_target", 1);
    pubIntersections = nh.advertise<visualization_msgs::Marker>("/intersections", 1);

    intersections_successed.header.frame_id = intersections_failed.header.frame_id = robot_to_target.header.frame_id = Constants::map_frame;
    intersections_successed.header.stamp = intersections_failed.header.stamp = robot_to_target.header.stamp = ros::Time::now();
    intersections_successed.action = intersections_failed.action = robot_to_target.action = visualization_msgs::Marker::ADD;
    intersections_successed.type = intersections_failed.type = visualization_msgs::Marker::POINTS;
    robot_to_target.type = visualization_msgs::Marker::LINE_STRIP;
    intersections_successed.id = 0;
    intersections_failed.id = 1;
    robot_to_target.id = 2;
    intersections_successed.scale.x = intersections_failed.scale.x = 0.1;
    intersections_successed.scale.y = intersections_failed.scale.y = 0.1;
    intersections_successed.color.g = 0.6;
    intersections_successed.color.r = 0.2;
    intersections_successed.color.b = 0.2;
    intersections_failed.color.r = 1.0;
    intersections_successed.color.a = intersections_failed.color.a = 1.0;
    robot_to_target.scale.x = 0.05;
    robot_to_target.color.b = 1.0;
    robot_to_target.color.a = 1.0;

    optimal_goal.header.frame_id = Constants::map_frame;
    optimal_goal.pose.position.z = 0;
    optimal_goal.pose.orientation.x = 0.707107;
    optimal_goal.pose.orientation.y = 0.707107;
    optimal_goal.pose.orientation.z = 0.0;
    optimal_goal.pose.orientation.w = 0.0;

    goal_time = ros::Time::now().toSec();

    x0 = 5.0;
    y0 = 4.6;
    r2 = 1.5;
    dxy2 = 0.0;
};

void OptimalGoal::calcCrossPointsOfTwoCircles(double x1, double y1, double r1, double x2, double y2, double r2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    dxy2 = pow(dx, 2) + pow(dy, 2);
    // std::cout << "dx:" << dx << " ,dy:" << dy << " ,dxy2:" << dxy2 << std::endl;
    double theta = atan2(dx, dy);
    double alpha = acos(fminl(fmaxl((pow(r1, 2) + dxy2 - pow(r2, 2)) / (2 * r1 * sqrt(dxy2)), -1.0), 1.0));
    // std::cout << "theta:" << theta << " rad ,alpha:" << alpha << " rad" << std::endl;

    // left
    x3 = x1 + r1 * sin(theta + alpha);
    y3 = y1 + r1 * cos(theta + alpha);
    // right
    x4 = x1 + r1 * sin(theta - alpha);
    y4 = y1 + r1 * cos(theta - alpha);
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

void OptimalGoal::showLine(double x, double y)
{
    robot_to_target.points.clear();

    geometry_msgs::Point point;
    point.x = x0;
    point.y = y0;
    point.z = 0;
    robot_to_target.points.push_back(point);
    point.x = x;
    point.y = y;
    point.z = 0;
    robot_to_target.points.push_back(point);

    pubLine.publish(robot_to_target);
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
    if (abs(ros::Time::now().toSec() - goal_time) < 0.5) // default:1s
    {
        std::cout << "goal has updated." << std::endl;

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

        intersections_successed.points.clear();
        intersections_failed.points.clear();

        x2 = tf_target.getOrigin().getX();
        y2 = tf_target.getOrigin().getY();
        std::cout << "x2:" << x2 << " ,y2:" << y2 << std::endl;

        showLine(x2, y2);

        geometry_msgs::Point p;

        // intersections_failed.points.push_back(p);
        for (r2 = 1.0; r2 < 5.0; r2 += 0.4)
        {
            x1 = x2 - (x2 - x0) * r2 / sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2));
            y1 = y2 - (y2 - y0) * r2 / sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2));

            std::cout << "x1:" << x1 << " ,y1:" << y1 << std::endl;
            cx1 = round(x1 / Constants::cellSize);
            cy1 = round(y1 / Constants::cellSize);

            p.x = x1;
            p.y = y1;
            p.z = 0;

            if (!isNearObstacle(cx1, cy1))
            {
                showBorder(x1, y1);
                if ((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) <= 3.0)
                {
                    return;
                }
                optimal_goal.pose.position.x = x1;
                optimal_goal.pose.position.y = y1;
                q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI / 2 - atan((x1 - x0) / (y1 - y0)));
                optimal_goal.pose.orientation.x = q.x;
                optimal_goal.pose.orientation.y = q.y;
                optimal_goal.pose.orientation.z = q.z;
                optimal_goal.pose.orientation.w = q.w;
                pubGoal.publish(optimal_goal);

                intersections_successed.points.push_back(p);
                pubIntersections.publish(intersections_failed);
                pubIntersections.publish(intersections_successed);
                return;
            }
            else
            {
                intersections_failed.points.push_back(p);
            }

            for (r1 = 0.2; r1 < 1.414 * r2; r1 += 0.2)
            {
                std::cout << "r1:" << r1 << std::endl;
                calcCrossPointsOfTwoCircles(x1, y1, r1, x2, y2, r2);

                std::cout << "x3:" << x3 << " ,y3:" << y3 << std::endl;
                cx3 = round(x3 / Constants::cellSize);
                cy3 = round(y3 / Constants::cellSize);

                p.x = x3;
                p.y = y3;
                p.z = 0;

                if (!isNearObstacle(cx3, cy3))
                {
                    showBorder(x3, y3);
                    if ((x0 - x3) * (x0 - x3) + (y0 - y3) * (y0 - y3) <= 3.0)
                    {
                        return;
                    }
                    optimal_goal.pose.position.x = x3;
                    optimal_goal.pose.position.y = y3;
                    q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI / 2 - atan((x3 - x0) / (y3 - y0)));
                    optimal_goal.pose.orientation.x = q.x;
                    optimal_goal.pose.orientation.y = q.y;
                    optimal_goal.pose.orientation.z = q.z;
                    optimal_goal.pose.orientation.w = q.w;
                    pubGoal.publish(optimal_goal);

                    intersections_successed.points.push_back(p);
                    pubIntersections.publish(intersections_failed);
                    pubIntersections.publish(intersections_successed);
                    return;
                }
                else
                {
                    intersections_failed.points.push_back(p);
                }

                std::cout << "x4:" << x4 << " ,y4:" << y4 << std::endl;
                cx4 = round(x4 / Constants::cellSize);
                cy4 = round(y4 / Constants::cellSize);

                p.x = x4;
                p.y = y4;
                p.z = 0;

                if (!isNearObstacle(cx4, cy4))
                {
                    showBorder(x4, y4);
                    if ((x0 - x4) * (x0 - x4) + (y0 - y4) * (y0 - y4) <= 3.0)
                    {
                        return;
                    }
                    optimal_goal.pose.position.x = x4;
                    optimal_goal.pose.position.y = y4;
                    q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI / 2 - atan((x4 - x0) / (y4 - y0)));
                    optimal_goal.pose.orientation.x = q.x;
                    optimal_goal.pose.orientation.y = q.y;
                    optimal_goal.pose.orientation.z = q.z;
                    optimal_goal.pose.orientation.w = q.w;
                    pubGoal.publish(optimal_goal);

                    intersections_successed.points.push_back(p);
                    pubIntersections.publish(intersections_failed);
                    pubIntersections.publish(intersections_successed);
                    return;
                }
                else
                {
                    intersections_failed.points.push_back(p);
                }
            }
        }
        pubIntersections.publish(intersections_failed);
        pubIntersections.publish(intersections_successed);
    }
    else
    {
        std::cout << "goal is disappear." << std::endl;
        listener.clear();
        optimal_goal.pose.position.x = x0;
        optimal_goal.pose.position.y = y0;
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
