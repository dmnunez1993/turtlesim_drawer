#include <boost/bind.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>

#include <turtlesim_drawer/Stat.h>
#include <turtlesim_drawer/Shape.h>

#define PI 3.141592

using namespace std;

//Star Path definition
double star_distance_array[] = {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
vector<double> star_distances(star_distance_array, star_distance_array + sizeof(star_distance_array) / sizeof(star_distance_array[0]));
double star_angle_array[] = {0.628, 5.026, 2.513, 5.026, 2.513, 5.026, 2.513, 5.026, 2.513, 5.026, 2.513};
vector<double> star_angles(star_angle_array, star_angle_array + sizeof(star_angle_array) / sizeof(star_angle_array[0]));

//Triangle Path definition
double triangle_distance_array[] = {0, 2, 2, 2};
vector<double> triangle_distances(triangle_distance_array, triangle_distance_array + sizeof(triangle_distance_array) / sizeof(triangle_distance_array[0]));
double triangle_angle_array[] = {0, PI/3*2, PI/3*2, PI/3*2};
vector<double> triangle_angles(triangle_angle_array, triangle_angle_array + sizeof(triangle_angle_array) / sizeof(triangle_angle_array[0]));

//Square Path definition
double square_distance_array[] = {0, 2, 2, 2, 2};
vector<double> square_distances(square_distance_array, square_distance_array + sizeof(square_distance_array) / sizeof(square_distance_array[0]));
double square_angle_array[] = {0, PI/2, PI/2, PI/2, PI/2};
vector<double> square_angles(square_angle_array, square_angle_array + sizeof(square_angle_array) / sizeof(square_angle_array[0]));

enum State
{
    FORWARD,
    STOP_FORWARD,
    TURN,
    STOP_TURN,
    IDLE,
    PAUSE
};

int angle_to_degrees(double angle)  {
    return int(angle * (180 / PI));
}

double normalize_angle(double angle)    {
    double normalized_angle;

    if (angle < 0)   {
        normalized_angle = 2*PI + angle;
    }   else    {
        normalized_angle = angle;
    }

    return normalized_angle;
}

class Path  {
    private:
        vector<double> distances;
        vector<double> angles;
        int current_goal;
        int amount_of_goals;
    public:
        Path();
        Path(vector<double>, vector<double>);
        double getCurrentDistance(void);
        double getCurrentAngle(void);
        void updateGoal(void);
        void reset(void);
        bool finished(void);
};

Path::Path()    {};

Path::Path(vector<double> path_distances, vector<double> path_angles)    {
    distances = path_distances;
    angles = path_angles;
    current_goal = 0;
    amount_of_goals = distances.size();
}

double Path::getCurrentDistance(void)  {
    return distances[current_goal];
}

double Path::getCurrentAngle(void) {
    return angles[current_goal];
}

void Path::updateGoal(void) {
    current_goal++;
}

bool Path::finished(void)   {
    return current_goal >= amount_of_goals;
}

void Path::reset(void)  {
    current_goal = 0;
    amount_of_goals = distances.size();
}

class Drawer    {
    private:
        ros::NodeHandle nh;
        State d_state;
        State d_last_state;
        bool d_first_goal_set;
        turtlesim::PoseConstPtr d_pose;
        turtlesim::Pose d_goal;
        Path path;
    public:
        Drawer(ros::NodeHandle);
        void setPath(Path);
        void setupDrawer(void);
        void poseCallback(const turtlesim::PoseConstPtr&);
        bool hasReachedGoal(void);
        bool hasStopped(void);
        bool isPaused(void);
        bool isRunning(void);
        bool isStarted(void);
        bool rotateClockwise(double, double);
        void printGoal(void);
        void commandTurtle(ros::Publisher, float, float);
        void publishStat(ros::Publisher);
        void stopForward(ros::Publisher);
        void stopTurn(ros::Publisher);
        void forward(ros::Publisher);
        void turn(ros::Publisher);
        void pause(ros::Publisher);
        void stop(ros::Publisher);
        void reset(void);
        void timerCallback(const ros::TimerEvent&, ros::Publisher, ros::Publisher);
        bool startCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
        bool stopCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
        bool pauseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
        bool resumeCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
        bool shapeCallback(turtlesim_drawer::Shape::Request&, turtlesim_drawer::Shape::Response&);
        void run(void);
};

Drawer::Drawer(ros::NodeHandle node_handle)   {
    nh = node_handle;
    d_state = FORWARD;
    d_last_state = FORWARD;
    d_first_goal_set = false;
}

void Drawer::setPath(Path new_path) {
    path = new_path;
}

void Drawer::setupDrawer()  {
    path.reset();
    d_first_goal_set = true;
    d_state = FORWARD;
    d_goal.x = cos(d_pose->theta) * path.getCurrentDistance() + d_pose->x;
    d_goal.y = sin(d_pose->theta) * path.getCurrentDistance() + d_pose->y;
    d_goal.theta = d_pose->theta;
    printGoal();
}

void Drawer::poseCallback(const turtlesim::PoseConstPtr& pose)  {
    d_pose = pose;

}

bool Drawer::hasReachedGoal()   {
    double d_current_theta_normalized, d_goal_theta_normalized;
    d_current_theta_normalized = normalize_angle(d_pose->theta);
    d_goal_theta_normalized = normalize_angle(d_goal.theta);

    if (d_goal.theta < 0)   {
        d_goal_theta_normalized = 2*PI + d_goal.theta;
    }   else    {
        d_goal_theta_normalized = d_goal.theta;
    }
    return fabsf(d_pose->x - d_goal.x) < 0.1 && fabsf(d_pose->y - d_goal.y) < 0.1 && fabsf(d_current_theta_normalized - d_goal_theta_normalized) < 0.01;
}

bool Drawer::hasStopped()   {
    return d_pose->angular_velocity < 0.00005 && d_pose->linear_velocity < 0.00005;
}

bool Drawer::isPaused() {
    return d_state == PAUSE;
}

bool Drawer::isRunning()    {
    return d_state != PAUSE;
}

bool Drawer::isStarted()  {
    return d_state != IDLE;
}

bool Drawer::rotateClockwise(double goal, double source) {
    if ((angle_to_degrees(goal) - angle_to_degrees(source) + 360) % 360 < 180)   {
        return true;
    }   else    {
        return false;
    }
}

void Drawer::printGoal()    {
    ROS_INFO("New goal [%f %f, %f]", d_goal.x, d_goal.y, d_goal.theta);
}

void Drawer::commandTurtle(ros::Publisher twist_pub, float linear, float angular)   {
    geometry_msgs::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_pub.publish(twist);
}

void Drawer::publishStat(ros::Publisher stat_pub)  {
    turtlesim_drawer::Stat stat;
    stat.paused = isPaused();
    stat.started = isStarted();
    stat_pub.publish(stat);
}

void Drawer::stopForward(ros::Publisher twist_pub)  {
    if (hasStopped())   {
        ROS_INFO("Reached goal");
        d_state = TURN;
        d_goal.x = d_pose->x;
        d_goal.y = d_pose->y;
        d_goal.theta = fmod(d_pose->theta + path.getCurrentAngle(), 2*PI);
        printGoal();
    }   else    {
        commandTurtle(twist_pub, 0, 0);
    }
}

void Drawer::stopTurn(ros::Publisher twist_pub) {
    if (hasStopped())   {
        ROS_INFO("Reached goal");
        d_state = FORWARD;
        path.updateGoal();
        if (path.finished())    {
            ROS_INFO("DONE");
            ROS_INFO("READY");
            //ros::shutdown();
            d_state = IDLE;
            return;
        }
        d_goal.x = cos(d_pose->theta) * path.getCurrentDistance() + d_pose->x;
        d_goal.y = sin(d_pose->theta) * path.getCurrentDistance() + d_pose->y;
        d_goal.theta = d_pose->theta;
        printGoal();
    }   else    {
        commandTurtle(twist_pub, 0, 0);
    }
}

void Drawer::forward(ros::Publisher twist_pub)  {
    if (hasReachedGoal())   {
        d_state = STOP_FORWARD;
        commandTurtle(twist_pub, 0, 0);
    }   else    {
        commandTurtle(twist_pub, 1.0, 0.0);
    }
}

void Drawer::turn(ros::Publisher twist_pub) {
    if (hasReachedGoal())   {
        d_state = STOP_TURN;
        commandTurtle(twist_pub, 0, 0);
    }   else    {
        double d_current_theta_normalized, d_goal_theta_normalized;
        d_current_theta_normalized = normalize_angle(d_pose->theta);
        d_goal_theta_normalized = normalize_angle(d_goal.theta);

        if (rotateClockwise(d_goal_theta_normalized, d_current_theta_normalized))   {
            commandTurtle(twist_pub, 0.0, 0.8);
        }   else    {
            commandTurtle(twist_pub, 0.0, -0.8);
        }
    }
}

void Drawer::pause(ros::Publisher twist_pub)    {
    if (isPaused())   {
        commandTurtle(twist_pub, 0, 0);
    }
}

void Drawer::stop(ros::Publisher twist_pub) {
    if (!isStarted())   {
        commandTurtle(twist_pub, 0, 0);
    }
}

void Drawer::reset()    {
    ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");
    std_srvs::Empty empty;
    reset.call(empty);
}

void Drawer::timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub, ros::Publisher stat_pub)    {
    if (!d_pose)    {
        return;
    }

    if (d_state == FORWARD) {
        forward(twist_pub);
    }   else if (d_state == STOP_FORWARD)   {
        stopForward(twist_pub);
    }   else if (d_state == TURN)   {
        turn(twist_pub);
    }   else if (d_state == STOP_TURN)  {
        stopTurn(twist_pub);
    }   else if (d_state == PAUSE)  {
        pause(twist_pub);
    }   else if (d_state == IDLE)   {
        stop(twist_pub);
    }

    publishStat(stat_pub);
}

bool Drawer::startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)  {
    if (!isStarted())    {
        ROS_INFO("Starting...");
        reset();
        ros::Duration(1).sleep(); //Waits for pose update
        setupDrawer();
        d_last_state = d_state;
        d_state = FORWARD;
    }   else    {
        ROS_INFO("Already started!!");
    }

    return true;
}

bool Drawer::stopCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)  {
    if (isStarted())    {
        d_last_state = d_state;
        d_state = IDLE;
        ROS_INFO("Stopping...");
        ROS_INFO("READY");
    }   else    {
        ROS_INFO("Already stoped!!");
    }

    return true;
}

bool Drawer::pauseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)  {
    if (isStarted())    {
        if (isRunning())    {
            d_last_state = d_state;
            d_state = PAUSE;
            ROS_INFO("Pausing...");
        }   else    {
            ROS_INFO("Already paused!!");
        }
    }   else    {
        ROS_INFO("Drawer not running!!");
    }

    return true;
}

bool Drawer::resumeCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)   {
    if (isStarted())    {
        if (isPaused()) {
            d_state = d_last_state;
            d_last_state = PAUSE;
            ROS_INFO("Resuming...");
        }   else    {
            ROS_INFO("Already running!!");
        }
    }   else    {
        ROS_INFO("Drawer not running!!");
    }

    return true;
}

bool Drawer::shapeCallback(turtlesim_drawer::Shape::Request& request, turtlesim_drawer::Shape::Response& response)  {
    if (request.shape=="star")  {
        Path path(star_distances, star_angles);
        setPath(path);
        response.current_shape = "star";
    }   else if (request.shape=="square") {
        Path path(square_distances, square_angles);
        setPath(path);
        response.current_shape = "square";
    }   else if (request.shape=="triangle") {
        Path path(triangle_distances, triangle_angles);
        setPath(path);
        response.current_shape = "triangle";
    }

    return true;
}

void Drawer::run() {
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, &Drawer::poseCallback, this);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    ros::Publisher stat_pub = nh.advertise<turtlesim_drawer::Stat>("turtlesim_drawer/stat", 1);
    ros::ServiceServer start_service = nh.advertiseService("turtlesim_drawer/start", &Drawer::startCallback, this);
    ros::ServiceServer stop_service = nh.advertiseService("turtlesim_drawer/stop", &Drawer::stopCallback, this);
    ros::ServiceServer pause_service = nh.advertiseService("turtlesim_drawer/pause", &Drawer::pauseCallback, this);
    ros::ServiceServer resume_service = nh.advertiseService("turtlesim_drawer/resume", &Drawer::resumeCallback, this);
    ros::ServiceServer shape_service = nh.advertiseService("turtlesim_drawer/shape", &Drawer::shapeCallback, this);
    ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(&Drawer::timerCallback, this, _1, twist_pub, stat_pub));
    reset();
    ROS_INFO("READY");
    d_state = IDLE;
    ros::spin();
}

int main(int argc, char** argv)  {
    ros::init(argc, argv, "drawer_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Drawer drawer(nh);
    Path path(star_distances, star_angles);
    drawer.setPath(path);
    drawer.run();
}
