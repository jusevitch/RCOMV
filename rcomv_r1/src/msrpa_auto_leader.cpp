#include <ros/ros.h>
#include <string>
#include <vector>
#include <deque>

#include <rcomv_r1/MSRPA.h>

class Auto_Leader
{
  public:
    Auto_Leader();
    ~Auto_Leader();

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_;

    // Parameters
    int number_of_configurations; // Configuration = Trajectory + Formation
    std::vector<std::string> trajectory_types;
    std::vector<double> flattened_trajectory_list; // 1D array with all trajectory vectors
    std::vector<double> flattened_formation_list;  // 1D array with all formation vectors
    std::vector<double> time_list;

    // Publishers
    ros::Publisher pub;

    // Subscribers
    ros::Subscriber sub;

    // Other objects
    ros::Timer pubTimer;
    std::vector<rcomv_r1::MSRPA> trajectories;
    std::deque<std::deque<double> > master_trajectory_list;
    std::deque<std::deque<double> > master_formation_list;
    int configuration_counter;
    double t, t0; // Initial time
    double prev_switching_time, next_switching_time;

    // Functions
    void timerCallback(const ros::TimerEvent& event);
    std::deque<std::deque<double> > parse_trajectory_list(std::deque<double> d);
    std::deque<std::deque<double> >  parse_formation_list(std::deque<double> d);
    std::deque<double> vec2deque(const std::vector<double> &vec);
    std::vector<double> deque2vec(const std::deque<double> &deq);



};


Auto_Leader::Auto_Leader()
    : nh_private_("~")
{

    // Parameters
    nh_private_.param<int>("number_of_configurations", number_of_configurations, 0);
    nh_private_.param<std::vector<std::string> >("trajectory_types", trajectory_types, std::vector<std::string>());
    nh_private_.param<std::vector<double> >("flattened_trajectory_list", flattened_trajectory_list, std::vector<double>());
    nh_private_.param<std::vector<double> >("flattened_formation_list", flattened_formation_list, std::vector<double>());
    nh_private_.param<std::vector<double> >("time_list", time_list, std::vector<double>()); // Length must be equal to number_of_configurations

    // Parse trajectories and Formations
    master_trajectory_list = parse_trajectory_list(vec2deque(flattened_trajectory_list));
    master_formation_list = parse_trajectory_list(vec2deque(flattened_formation_list));

    // Publishers
    pub = nh.advertise<rcomv_r1::MSRPA>("/leader_cmd", 1);

    // Subscribers
    

    // Other
    pubTimer = nh.createTimer(ros::Duration(0.05), &Auto_Leader::timerCallback, this);

    configuration_counter = 0;
    t0 = ros::Time::now().toSec();
    prev_switching_time = t0;
    next_switching_time = t0 + time_list[0];

}

Auto_Leader::~Auto_Leader()
{
    // Publish a NaN message / some other message to make things stop
    rcomv_r1::MSRPA msg;
    msg.type = "NaN";
    msg.trajectory.resize(7);
    msg.formation.resize(7);
    pub.publish(msg);
    ros::shutdown();
}

void Auto_Leader::timerCallback(const ros::TimerEvent& event)
{
    t = ros::Time::now().toSec() - t0;
    if (t > next_switching_time){
        configuration_counter++;
        prev_switching_time = next_switching_time;
        next_switching_time = t0 + time_list[configuration_counter % number_of_configurations];
    }

    rcomv_r1::MSRPA msg;
    msg.type = trajectory_types[configuration_counter];
    msg.trajectory = deque2vec(master_trajectory_list[configuration_counter]);
    msg.formation = deque2vec(master_formation_list[configuration_counter]);

    pub.publish(msg);

}

std::deque<std::deque<double> > Auto_Leader::parse_trajectory_list(std::deque<double> d)
{
    // Format of the trajectory list:
    // Each trajectory t_i is formatted as [length_of_traj param1 param2 ...]
    // flattened_trajectory_list is formatted as [t0 t1 ...]
    
    std::deque<std::deque<double> > out_deque;
    for (int ii = 0; ii < number_of_configurations; ii++)
    {
        int temp_num_entries = static_cast<int>(d[0]);
        out_deque.push_back(std::deque<double>(temp_num_entries));
        for (int jj = 1; jj <= temp_num_entries; jj++)
        {
            out_deque[ii][jj - 1] = d[jj];
        }
        d.erase(d.begin(), d.begin() + temp_num_entries + 1);
    }

    if (d.size() > 0)
    {
        ROS_INFO("WARNING: Not all entries turned into trajectories.");
    }

    return out_deque;
}

std::deque<std::deque<double> > Auto_Leader::parse_formation_list(std::deque<double> d)
{
    std::deque<std::deque<double> > out_deque;
    for (int ii = 0; ii < number_of_configurations; ii++)
    {
        int temp_num_entries = static_cast<int>(d[0]);
        out_deque.push_back(std::deque<double>(temp_num_entries));
        for (int jj = 1; jj <= temp_num_entries; jj++)
        {
            out_deque[ii][jj - 1] = d[jj];
        }
        d.erase(d.begin(), d.begin() + temp_num_entries + 1);
    }

    if (d.size() > 0)
    {
        ROS_INFO("WARNING: Not all entries turned into trajectories.");
    }

    return out_deque;
}

std::deque<double> Auto_Leader::vec2deque(const std::vector<double> &vec)
{
    std::deque<double> out_deque;
    for (int ii = 0; ii < vec.size(); ii++)
    {
        out_deque[ii] = vec[ii];
    }
    return out_deque;
}

std::vector<double> Auto_Leader::deque2vec(const std::deque<double> &deq){
    std::vector<double> out_vector;
    for (int ii = 0; ii < deq.size(); ii++)
    {
        out_vector[ii] = deq[ii];
    }
    return out_vector;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "msrpa_auto_leader");

    Auto_Leader auto_leader;

    ros::spin();

    return 0;
}