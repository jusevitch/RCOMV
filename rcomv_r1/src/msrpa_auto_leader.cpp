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
    std::vector<double> time_duration_list;

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
    double current_time, t0; // Initial time
    double prev_switching_time, next_switching_time;

    // Functions
    void timerCallback(const ros::TimerEvent& event);
    std::deque<std::deque<double> > parse_trajectory_list(std::deque<double> d, int size);
    std::deque<std::deque<double> >  parse_formation_list(std::deque<double> d, int size);
    std::deque<double> vec2deque(const std::vector<double> &vec, int size);
    std::vector<double> deque2vec(const std::deque<double> &deq, int size);



};


Auto_Leader::Auto_Leader()
    : nh_private_("~")
{

    // Parameters
    nh_private_.param<int>("number_of_configurations", number_of_configurations, 0);
    nh_private_.param<std::vector<std::string> >("trajectory_types", trajectory_types, std::vector<std::string>());
    nh_private_.param<std::vector<double> >("flattened_trajectory_list", flattened_trajectory_list, std::vector<double>());
    nh_private_.param<std::vector<double> >("flattened_formation_list", flattened_formation_list, std::vector<double>());
    nh_private_.param<std::vector<double> >("time_duration_list", time_duration_list, std::vector<double>()); // Length must be equal to number_of_configurations

    // Parse trajectories and Formations
    master_trajectory_list = parse_trajectory_list(vec2deque(flattened_trajectory_list, flattened_trajectory_list.size()), flattened_trajectory_list.size());
    master_formation_list = parse_trajectory_list(vec2deque(flattened_formation_list, flattened_formation_list.size()), flattened_formation_list.size());

    for(int ii = 0; ii < master_trajectory_list.size(); ii++)
    {
        for(int jj = 0; jj < master_trajectory_list[ii].size(); jj++)
        {
            ROS_INFO("mtraject[ii][jj]: %lf", master_trajectory_list[ii][jj]);
        }
        ROS_INFO("\n");
        
    }
    ROS_INFO("\n\n");

    for(int ii = 0; ii < master_formation_list.size(); ii++)
    {
        for(int jj = 0; jj < master_formation_list[ii].size(); jj++)
        {
            ROS_INFO("mform[ii][jj]: %lf", master_formation_list[ii][jj]);
        }
        ROS_INFO("\n");
    }
    

    // Publishers
    pub = nh.advertise<rcomv_r1::MSRPA>("/leader_cmd", 1);

    // Subscribers
    

    // Other
    pubTimer = nh.createTimer(ros::Duration(0.05), &Auto_Leader::timerCallback, this);

    configuration_counter = 0;
    ros::spinOnce();
    t0 = ros::Time::now().toSec();
    prev_switching_time = t0;
    next_switching_time = t0 + time_duration_list[0];

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

    current_time = ros::Time::now().toSec();
    if (current_time > next_switching_time){
        configuration_counter = (configuration_counter + 1) % number_of_configurations;
        prev_switching_time = next_switching_time;
        next_switching_time += time_duration_list[configuration_counter];
    }

    rcomv_r1::MSRPA msg;
    msg.type = trajectory_types[configuration_counter];
    msg.trajectory = deque2vec(master_trajectory_list[configuration_counter], master_trajectory_list[configuration_counter].size());
    msg.formation = deque2vec(master_formation_list[configuration_counter], master_formation_list[configuration_counter].size());

    if (msg.trajectory.size() > 0) {
    ROS_INFO("trajectory.size : %lu", msg.trajectory.size());
    ROS_INFO("trajectory: [%lf, %lf, %lf]", msg.trajectory[0], msg.trajectory[1], msg.trajectory[2]);

    pub.publish(msg);
    } else {
        ROS_INFO("WARNING: trajectory.size == 0 at time %lf", current_time);
    }

}

std::deque<std::deque<double> > Auto_Leader::parse_trajectory_list(std::deque<double> d, int size)
{
    // Format of the trajectory list:
    // Each trajectory t_i is formatted as [length_of_traj param1 param2 ...]
    // flattened_trajectory_list is formatted as [t0 t1 ...]
    
    int idx = 0;

    std::deque<std::deque<double> > out_deque;
    ROS_INFO("size: %d", size);
    for (int ii = 0; ii < number_of_configurations; ii++)
    {
        ROS_INFO("\n temp_num_entries before: %lf", d[idx]);
        int temp_num_entries = static_cast<int>(d[idx]);
        ROS_INFO("\ntemp_num_entries: %d", temp_num_entries);
        idx += 1;
        ROS_INFO("idx: %d", idx);
        out_deque.push_back(std::deque<double>(temp_num_entries));
        for (int jj = idx; jj < idx + temp_num_entries; jj++)
        {
            out_deque[ii][jj - idx] = d[jj];
            ROS_INFO("jj: %d", jj);
            ROS_INFO("entry: %lf", d[jj]);
        }
        idx += temp_num_entries;
    }

    // if (d.size() > 0)
    // {
    //     ROS_INFO("d.size(): %lu", d.size());
    //     for(int ii = 0; ii < d.size(); ii++)
    //     {
    //         ROS_INFO("d[ii]: %lf", d[ii]);
    //     }
        
    //     ROS_INFO("WARNING: Not all entries turned into trajectories.");
    // }

    ROS_INFO("\n");
    return out_deque;
}

std::deque<std::deque<double> > Auto_Leader::parse_formation_list(std::deque<double> d, int size)
{

    int idx = 0;

    std::deque<std::deque<double> > out_deque;
    ROS_INFO("size: %d", size);
    for (int ii = 0; ii < number_of_configurations; ii++)
    {
        ROS_INFO("\n temp_num_entries before: %lf", d[idx]);
        int temp_num_entries = static_cast<int>(d[idx]);
        ROS_INFO("temp_num_entries after: %d", temp_num_entries);
        idx += 1;
        ROS_INFO("idx: %d", idx);
        out_deque.push_back(std::deque<double>(temp_num_entries));
        for (int jj = idx; jj < idx + temp_num_entries; jj++)
        {
            out_deque[ii][jj - idx] = d[jj];
            ROS_INFO("jj: %d", jj);
            ROS_INFO("entry: %lf", d[jj]);
        }
        idx += temp_num_entries;
    }

    // if (d.size() > 0)
    // {
    //     ROS_INFO("d.size(): %lu", d.size());
    //     for(int ii = 0; ii < d.size(); ii++)
    //     {
    //         ROS_INFO("d[ii]: %lf", d[ii]);
    //     }
        
    //     ROS_INFO("WARNING: Not all entries turned into trajectories.");
    // }

    ROS_INFO("\n");
    return out_deque;
}

std::deque<double> Auto_Leader::vec2deque(const std::vector<double> &vec, int size)
{
    std::deque<double> out_deque(size);
    for (int ii = 0; ii < size; ii++)
    {
        out_deque[ii] = vec[ii];
    }
    return out_deque;
}

std::vector<double> Auto_Leader::deque2vec(const std::deque<double> &deq, int size){
    std::vector<double> out_vector(size);
    for (int ii = 0; ii < size; ii++)
    {
        // ROS_INFO("deq[ii]: %lf", deq[ii]);
        out_vector[ii] = deq[ii];
        // ROS_INFO("outvector[ii]: %lf", out_vector[ii]);
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