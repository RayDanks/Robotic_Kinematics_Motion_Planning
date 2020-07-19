#include "ros/ros.h"
#include "cw2q4/youbotKine.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "boost/foreach.hpp"

trajectory_msgs::JointTrajectoryPoint traj_pt;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_cw2");
    youbot_kinematic youbot;
    youbot.init();
    int checkpoint_data = 2
    int dt = 1;

    if (checkpoint_data == 1)
    {
        rosbag::Bag bag1;
        ros::NodeHandle nh;
        bag1.open(MY_BAG_A, rosbag::bagmode::Read); //MY BAG A from the CMakeLists.txt

        trajectory_msgs::JointTrajectory my_traj;
        trajectory_msgs::JointTrajectoryPoint my_pt;

        trajectory_msgs::JointTrajectoryPoint my_pt1; //THese numbers refer to which checkpoint they are relevant to.
        trajectory_msgs::JointTrajectoryPoint my_pt2;
        trajectory_msgs::JointTrajectoryPoint my_pt3;
        trajectory_msgs::JointTrajectoryPoint my_pt4;
        trajectory_msgs::JointTrajectoryPoint my_pt5;

        trajectory_msgs::JointTrajectoryPoint my_pt_middle;
        trajectory_msgs::JointTrajectoryPoint my_pt_split;
        trajectory_msgs::JointTrajectoryPoint my_pt_to_be_split;
        trajectory_msgs::JointTrajectoryPoint my_pt_difference;
        trajectory_msgs::JointTrajectoryPoint my_pt_previous;
        trajectory_msgs::JointTrajectoryPoint my_pt_initial;

        std::vector<std::string> topics;
        topics.push_back(std::string("joint_data")); //topic gained from the bag file (opened in text editor).
        //This topic gives us the joint angles which are desired.


        //Using the linear method, we need to progress through the joint_data read from the bag at 0.1*joint_data per second.
        //This is so that after ten seconds, the robot is at the final joint position.

        ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

        //There are 5 joints, therefore:

        //Double checked these names with gazebo and they are correct
        my_traj.header.stamp = ros::Time::now();
        my_traj.joint_names.push_back("arm_joint_1");
        my_traj.joint_names.push_back("arm_joint_2");
        my_traj.joint_names.push_back("arm_joint_3");
        my_traj.joint_names.push_back("arm_joint_4");
        my_traj.joint_names.push_back("arm_joint_5");

        my_pt.positions.resize(5);
        my_pt.velocities.resize(5);

        my_pt1.positions.resize(5);
        my_pt1.velocities.resize(5);

        my_pt2.positions.resize(5);
        my_pt2.velocities.resize(5);

        my_pt3.positions.resize(5);
        my_pt3.velocities.resize(5);

        my_pt3.positions.resize(5);
        my_pt3.velocities.resize(5);

        my_pt4.positions.resize(5);
        my_pt4.velocities.resize(5);

        my_pt5.positions.resize(5);
        my_pt5.velocities.resize(5);

        my_pt_split.positions.resize(5);
        my_pt_split.velocities.resize(5);

        my_pt_to_be_split.positions.resize(5);
        my_pt_to_be_split.velocities.resize(5);

        my_pt_difference.positions.resize(5);
        my_pt_difference.velocities.resize(5);

        my_pt_previous.positions.resize(5);
        my_pt_previous.velocities.resize(5);

        my_pt_initial.positions.resize(5);
        my_pt_initial.velocities.resize(5);

        rosbag::View view(bag1, rosbag::TopicQuery(topics)); //THis is creating our view

        int tfs = 10; //begins 10 seconds in
        int ForLoop = 0;
        int MainLoop = 0;
        int ForLoop2 = 0;
        int ForLoop3 = 0;

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
            if (J != NULL) {
                if (J->position.size() != 0) {
                    MainLoop++; //Use this main loop variable as a subsitute for the presence of an "i" in the BOOST_FOREACH loop !!
                    //Main Loop shoots off 5 times (for 5 positions) - changes positions every 5 loops

                    my_pt.time_from_start.sec = tfs; //at every second we want the robot to move 10% of the way
                    my_pt_split.time_from_start.sec = tfs; //at every second we want the robot to move 10% of the way

                    for (int i = 0; i < 5; i++) {
                        ForLoop++; //shoots off 25 times - 5 times for each position
                        my_pt.positions.at(i) = J->position.at(i); //This defines the actual points and velocities
                        my_pt.velocities.at(i) = J->velocity.at(i);
                    }
                    //Now we have a full my_pt vector:
                    if (MainLoop == 1) {
                        my_pt1 = my_pt;
                    } else if (MainLoop == 2) {
                        my_pt2 = my_pt;
                    } else if (MainLoop == 3) {
                        my_pt3 = my_pt;
                    } else if (MainLoop == 4) {
                        my_pt4 = my_pt;
                    } else if (MainLoop == 5) {
                        my_pt5 = my_pt;
                    } else {
                        std::cout << "ERROR IN BOOST_FOREACH LOOP " << std::endl;
                    }
                }
            }
        }
        //Now we have 5 different point vectors which define each checkpoint, which won't be deleted by the loop

////////////This splitting loop MUST be outside the BOOST_FOREACH loop, since that boost loop only executes the first command/loop it comes across.
        for (int h = 1; h < 6; h++) {
            if (h == 1) {
                my_pt_previous = my_pt_initial;
                my_pt_to_be_split = my_pt1;
            }
            else if (h == 2) {
                my_pt_previous = my_pt1;
                my_pt_to_be_split = my_pt2;
            }
            else if (h == 3) {
                my_pt_previous = my_pt2;
                my_pt_to_be_split = my_pt3;
            }
            else if (h == 4) {
                my_pt_previous = my_pt3;
                my_pt_to_be_split = my_pt4;
            }
            else if (h == 5) {
                my_pt_previous = my_pt4;
                my_pt_to_be_split = my_pt5;
            }
            else {
                std::cout << "ERROR IN SPLITTING LOOP \n" << std::endl;
            }


            for (int init = 0; init < 5; init++) {
                my_pt_initial.positions.at(init) = 0;
                my_pt_initial.velocities.at(init) = 0;
            }

            for (int g = 0; g < 10; g++) {
                ForLoop3++;
                for (int r = 0; r < 5; r++) { //Need this 5 loop to access all 5 joint angles

                    my_pt_difference.positions.at(r) = my_pt_to_be_split.positions.at(r) - my_pt_previous.positions.at(r); //this creates the difference between the two
                    my_pt_difference.velocities.at(r) = my_pt_to_be_split.velocities.at(r) - my_pt_previous.velocities.at(r); //this creates the difference between the two

                    my_pt_split.positions.at(r) = my_pt_previous.positions.at(r) + my_pt_difference.positions.at(r) * ((float)g / 10); //eventually, the entire difference between previous and
                    my_pt_split.velocities.at(r) = my_pt_difference.velocities.at(r); //See slides - this is the correct velocity gradient for straight line path !!

                    //current point should be reached, hence the previous point should become the current point
                    //Note above that we must convert g from int to float since joint trajectory points use float64.
                }

                my_pt_split.time_from_start.sec = tfs;

                my_traj.points.push_back(my_pt_split); //This adds 10 points per original point; each point is occuring a second after the other
                //Therefore 10 seconds per original point

                tfs = tfs + 1; //dt = 1s

                //This is included so that we can have the 0,0,0,0,0 instantiation at 10 seconds AND have the final execution at 60 seconds too - ideal!
                if (tfs == 60) {
                    my_pt_split.time_from_start.sec = tfs;

                    for (int ra = 0; ra < 5; ra++) {
                        my_pt_split.positions.at(ra) = my_pt_previous.positions.at(ra) + my_pt_difference.positions.at(ra); //eventually, the entire difference between previous and
                        my_pt_split.velocities.at(ra) = my_pt_difference.velocities.at(ra); //eventually, the entire difference between previous and
                    }
                    my_traj.points.push_back(my_pt_split);
                }
            }
        }
        std::cout << "FINAL TRAJECTORIES \n" << my_traj << std::endl;
        ///////Thewse should be the same as the trajectory at 20, 30, 40, 50, 60 seconds respectively.
        //////////////////Checked - they are correct and trajectory is correct too !///////////////////
        std::cout << "my_pt1: \n" << my_pt1 << std::endl; //trajectory at 20 seconds
        std::cout << "my_pt2: \n" << my_pt2 << std::endl; //trajectory at 30 seconds
        std::cout << "my_pt3: \n" << my_pt3 << std::endl; //trajectory at 40 seconds
        std::cout << "my_pt4: \n" << my_pt4 << std::endl; //trajectory at 50 seconds
        std::cout << "my_pt5: \n" << my_pt5 << std::endl; //trajectory at 60 seconds

        traj_pub.publish(my_traj); //THis should make the robot move.

        sleep(5); //THis is machine-dependent
        bag1.close();

    }

    else if (checkpoint_data == 2)
    {
        ////////////////////////////////////////////////////Note that the below code is coopied from Q5a, as it is not the focus of this question//////////////////////////
        rosbag::Bag bag1;

        ros::NodeHandle nh;


        bag1.open(MY_BAG_A, rosbag::bagmode::Read); //MY BAG A from the CMakeLists.txt


        trajectory_msgs::JointTrajectory my_traj;
        trajectory_msgs::JointTrajectoryPoint my_pt;


        trajectory_msgs::JointTrajectoryPoint my_pt1; //THese numbers refer to which checkpoint they are relevant to.
        trajectory_msgs::JointTrajectoryPoint my_pt2;
        trajectory_msgs::JointTrajectoryPoint my_pt3;
        trajectory_msgs::JointTrajectoryPoint my_pt4;
        trajectory_msgs::JointTrajectoryPoint my_pt5;


        trajectory_msgs::JointTrajectoryPoint my_pt_middle;
        trajectory_msgs::JointTrajectoryPoint my_pt_split;
        trajectory_msgs::JointTrajectoryPoint my_pt_to_be_split;
        trajectory_msgs::JointTrajectoryPoint my_pt_difference;
        trajectory_msgs::JointTrajectoryPoint my_pt_previous;
        trajectory_msgs::JointTrajectoryPoint my_pt_initial;

        std::vector<std::string> topics;
        topics.push_back(std::string("joint_data"));

        ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

        my_traj.header.stamp = ros::Time::now();
        my_traj.joint_names.push_back("arm_joint_1");
        my_traj.joint_names.push_back("arm_joint_2");
        my_traj.joint_names.push_back("arm_joint_3");
        my_traj.joint_names.push_back("arm_joint_4");
        my_traj.joint_names.push_back("arm_joint_5");

        my_pt.positions.resize(5);
        my_pt.velocities.resize(5);

        my_pt1.positions.resize(5);
        my_pt1.velocities.resize(5);

        my_pt2.positions.resize(5);
        my_pt2.velocities.resize(5);

        my_pt3.positions.resize(5);
        my_pt3.velocities.resize(5);

        my_pt3.positions.resize(5);
        my_pt3.velocities.resize(5);

        my_pt4.positions.resize(5);
        my_pt4.velocities.resize(5);

        my_pt5.positions.resize(5);
        my_pt5.velocities.resize(5);

        my_pt_split.positions.resize(5);
        my_pt_split.velocities.resize(5);

        my_pt_to_be_split.positions.resize(5);
        my_pt_to_be_split.velocities.resize(5);

        my_pt_difference.positions.resize(5);
        my_pt_difference.velocities.resize(5);

        my_pt_previous.positions.resize(5);
        my_pt_previous.velocities.resize(5);

        my_pt_initial.positions.resize(5);
        my_pt_initial.velocities.resize(5);

        rosbag::View view(bag1, rosbag::TopicQuery(topics));

        int tfs = 10;
        int ForLoop = 0;
        int MainLoop = 0;
        int ForLoop2 = 0;
        int ForLoop3 = 0;

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
            if (J != NULL) {
                if (J->position.size() != 0) {
                    MainLoop++; //Use this main loop as a subsitute for the presence of an "i" in the BOOST_FOREACH loop !!
                    std::cout << " Main Loop: \n" << MainLoop
                              << std::endl; //Shoots off 5 times (for 5 positions) - changes positions every 5 loops

                    my_pt.time_from_start.sec = tfs; //at every second we want the robot to move 10% of the way
                    my_pt_split.time_from_start.sec = tfs; //at every second we want the robot to move 10% of the way

                    for (int i = 0; i < 5; i++) //THis goes through each of the
                    {
                        ForLoop++; //shoots off 25 times - 5 times for each position
                        std::cout << "First For Loop: \n" << ForLoop << std::endl;
                        my_pt.positions.at(i) = J->position.at(i); //This defines the actual points and velocities
                        my_pt.velocities.at(i) = J->velocity.at(i);
                    }
                    //NOw we have a full my_pt vector:
                    ///////////////////////////////////tested printing these and they work fine///////////////////////////
                    if (MainLoop == 1) {
                        my_pt1 = my_pt;
                    } else if (MainLoop == 2) {
                        my_pt2 = my_pt;
                    } else if (MainLoop == 3) {
                        my_pt3 = my_pt;
                    } else if (MainLoop == 4) {
                        my_pt4 = my_pt;
                    } else if (MainLoop == 5) {
                        my_pt5 = my_pt;
                    } else {
                        std::cout << "ERROR IN BOOST_FOREACH LOOP " << std::endl;
                    }
                }
            }
        }

        for (int init = 0;init<5;init++) { //starting angles all zero
            my_pt_initial.positions.at(init) = 0;
            my_pt_initial.velocities.at(init) = 0;
        }


        //Now we have 5 different point vectors which define each checkpoint, which won't be deleted by the loop

        //NOw, we have all five checkpoints to work with, which can be printed here:
        std::cout << "my_pt_initial: \n" << my_pt_initial << std::endl;
        std::cout << "my_pt1: \n" << my_pt1 << std::endl;
        std::cout << "my_pt2: \n" << my_pt2 << std::endl;
        std::cout << "my_pt3: \n" << my_pt3 << std::endl;
        std::cout << "my_pt4: \n" << my_pt4 << std::endl;
        std::cout << "my_pt5: \n" << my_pt5 << std::endl;

        //Now the pertinent part of this question is that each of these checkpoints should be organised to find the quickest route between them


        //THe first thing to do is to use the joint angles to work out where these individual poses/checkpoints are
        //we should do this using forward kinematics:

        //Firstly need to put these joint angles into double arrays, so that the functions will accept them as joint values:
        double initialAngles[] = {my_pt_initial.positions.at(0),my_pt_initial.positions.at(1),my_pt_initial.positions.at(2),my_pt_initial.positions.at(3),my_pt_initial.positions.at(4)}; //can also change initial angles here.
        double point1Angles[] = {my_pt1.positions.at(0),my_pt1.positions.at(1),my_pt1.positions.at(2),my_pt1.positions.at(3),my_pt1.positions.at(4)};
        double point2Angles[] = {my_pt2.positions.at(0),my_pt2.positions.at(1),my_pt2.positions.at(2),my_pt2.positions.at(3),my_pt2.positions.at(4)};
        double point3Angles[] = {my_pt3.positions.at(0),my_pt3.positions.at(1),my_pt3.positions.at(2),my_pt3.positions.at(3),my_pt3.positions.at(4)};
        double point4Angles[] = {my_pt4.positions.at(0),my_pt4.positions.at(1),my_pt4.positions.at(2),my_pt4.positions.at(3),my_pt4.positions.at(4)};
        double point5Angles[] = {my_pt5.positions.at(0),my_pt5.positions.at(1),my_pt5.positions.at(2),my_pt5.positions.at(3),my_pt5.positions.at(4)};

        std::cout << "InitialAngles: " << initialAngles[0] << " , " << initialAngles[1] << " , "<< initialAngles[2] << " , "<< initialAngles[3] << " , "<< initialAngles[4] << std::endl;
        std::cout << "Point1Angles: " << point1Angles[0] << " , " << point1Angles[1] << " , "<< point1Angles[2] << " , "<< point1Angles[3] << " , "<< point1Angles[4] << std::endl;
        std::cout << "Point2Angles: " << point2Angles[0] << " , "<< point2Angles[1] << " , "<< point2Angles[2] << " , "<< point2Angles[3] << " , "<< point2Angles[4] << std::endl;
        std::cout << "Point3Angles: " << point3Angles[0] << " , "<< point3Angles[1] << " , "<< point3Angles[2] << " , "<< point3Angles[3] << " , "<< point3Angles[4] << std::endl;
        std::cout << "Point4Agles: " << point4Angles[0] << " , "<< point4Angles[1] << " , "<< point4Angles[2] << " , "<< point4Angles[3] << " , "<< point4Angles[4] << std::endl;
        std::cout << "Point5Angles: " << point5Angles[0] << " , "<< point5Angles[1] << " , "<< point5Angles[2] << " , "<< point5Angles[3] << " , "<< point5Angles[4] << std::endl;

        //Now that we have the angles in double arrays, we should input these into the forward kinematics function

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
        MatrixXd jacobian(6, 5);
        MatrixXd IK_closed;
        double* IK_ite;
        bool singularity;

        youbot_kinematic object;
        object.init();

        Matrix4d InitialPoint = object.forward_kine_offset(initialAngles, 5);
        Matrix4d Point1 = object.forward_kine_offset(point1Angles, 5);
        Matrix4d Point2 = object.forward_kine_offset(point2Angles, 5);
        Matrix4d Point3 = object.forward_kine_offset(point3Angles, 5);
        Matrix4d Point4 = object.forward_kine_offset(point4Angles, 5);
        Matrix4d Point5 = object.forward_kine_offset(point5Angles, 5);

        std::cout << "Initial Transform \n" << InitialPoint << std::endl;
        std::cout << "Point 1 transform: \n" << Point1 << std::endl;
        std::cout << "Point 2 transform: \n" << Point2 << std::endl;
        std::cout << "Point 3 transform: \n" << Point3 << std::endl;
        std::cout << "Point 4 transform: \n" << Point4 << std::endl;
        std::cout << "Point 5 transform: \n" << Point5 << std::endl;

        //Now we have the 4x4 transformation matrices which describe the position of the end effector for each checkpoint

        //First off, let's not worry about rotation - Therefore, from these 4x4 pose matrices, 3x1 Vectord3d parts can be extracted

        Vector3d InitialPointXYZ;
        Vector3d Point1XYZ;
        Vector3d Point2XYZ;
        Vector3d Point3XYZ;
        Vector3d Point4XYZ;
        Vector3d Point5XYZ;

        InitialPointXYZ << InitialPoint(0,3),InitialPoint(1,3),InitialPoint(2,3);
        Point1XYZ << Point1(0,3),Point1(1,3),Point1(2,3);
        Point2XYZ << Point2(0,3),Point2(1,3),Point2(2,3);
        Point3XYZ << Point3(0,3),Point3(1,3),Point3(2,3);
        Point4XYZ << Point4(0,3),Point4(1,3),Point4(2,3);
        Point5XYZ << Point5(0,3),Point5(1,3),Point5(2,3);

        std::cout << "Initial Point XYZ: \n" << InitialPointXYZ << std::endl;
        std::cout << "Point 1 XYZ: \n" << Point1XYZ << std::endl;
        std::cout << "Point 2 XYZ: \n" << Point2XYZ << std::endl;
        std::cout << "Point 3 XYZ: \n" << Point3XYZ << std::endl;
        std::cout << "Point 4 XYZ: \n" << Point4XYZ << std::endl;
        std::cout << "Point 5 XYZ: \n" << Point5XYZ << std::endl;


        //Now we can begin to implement the shortest path algorithm

        //A simple algorithm would be to take the shortest path every single time, however, this is likely very sub optimal.

        //Solution: THere are only 120 possible combinations (5! = 120), we could just try them all.
        //We do lose the ability to go back to the same node, however it is still likely to be efficient.
        //Travelling salesman problem model - visit each node only one time !
        //Realistically, for such a small sample, this is the most efficient method - let's go !

        //THe first thing to do is to create a 5x5 matrix which encompasses all the distances between each checkpoint:
        MatrixXd Distances(6,6);

        //The below matrix is necessary so as to be able to index the actual point vectors (ie have them in the same matrix)
        MatrixXd AllPoints(6,3);
        AllPoints << InitialPointXYZ(0), InitialPointXYZ(1), InitialPointXYZ(2),
                     Point1XYZ(0), Point1XYZ(1), Point1XYZ(2),
                     Point2XYZ(0), Point2XYZ(1), Point2XYZ(2),
                     Point3XYZ(0), Point3XYZ(1), Point3XYZ(2),
                     Point4XYZ(0), Point4XYZ(1), Point4XYZ(2),
                     Point5XYZ(0), Point5XYZ(1), Point5XYZ(2);

        //Below is the distances loop
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                Vector3d BaseTempPoint;
                BaseTempPoint << AllPoints(i,0), AllPoints(i,1), AllPoints(i,2); //TRavel through all points per i cycle

                Vector3d DistalTempPoint;
                DistalTempPoint << AllPoints(j,0), AllPoints(j,1), AllPoints(j,2); //Travel through all points per j cycle

                Vector3d DifferenceTempPoint;
                DifferenceTempPoint = DistalTempPoint-BaseTempPoint; //order does not matter as it will be normalised

                double DistanceElementTemporary;
                DistanceElementTemporary = DifferenceTempPoint.norm();

                Distances(i,j) = DistanceElementTemporary;
            }
        }


        std::cout << "Distance Matrix: \n" << Distances << std::endl;
        //To read this matrix: Each row number is the base/starting point and each column number is the distance between that row's starting point and that column's ending point
        double Checkpoints[] = {1,2,3,4,5};
        MatrixXd OverallDistanceMatrix(120,6); //INstantiating the overall distance matrix - first column is distance and next five are the actual checkpoints
        double Combinations = 0; //THis will update once; every single combination has been achieved; eventually it should be 120 - print at end to check !!


        for (int Loop1 = 0; Loop1 < 5; Loop1++) {
            double FirstCheckPoint = Checkpoints[Loop1]; //This shows which ACTUAL checkpoint we're going for first (starting from zero)
            double ZeroToFirst = Distances(0,Loop1+1);


            for (int Loop2 = 0; Loop2 < 4; Loop2++) {
                double RemainingCheckpoints1[4];
                //say we started at 1, we want this to loop through 2, 3, 4 and 5
                if (FirstCheckPoint == Checkpoints[0]) {
                    double RemainingCheckpoints1[] = {2,3,4,5}; //Use this in the next loop too !!
                }
                else if (FirstCheckPoint == Checkpoints[1])
                {
                    double RemainingCheckpoints1[] = {1, 3, 4, 5};
                }
                else if (FirstCheckPoint == Checkpoints[2]) {
                    double RemainingCheckpoints1[] = {1, 2, 4, 5};
                }
                else if (FirstCheckPoint == Checkpoints[3]) {
                    double RemainingCheckpoints1[] = {1, 2, 3, 5};
                }
                else if (FirstCheckPoint == Checkpoints[4]) {
                    double RemainingCheckpoints1[] = {1, 2, 3, 4};
                }
                else {
                    std::cout << "ERROR IN LOOP2 \n" << std::endl;
                }
                double SecondCheckpoint = RemainingCheckpoints1[Loop2];
                double FirstToSecond = Distances(FirstCheckPoint,SecondCheckpoint); //We can use real checkpoint number because we included zero in distance matrix


                for (int Loop3 = 0; Loop3 < 3; Loop3++) {
                    double RemainingCheckpoints2[3];
                    if (SecondCheckpoint == RemainingCheckpoints1[0]) {
                        double RemainingCheckpoints2[] = {RemainingCheckpoints1[1],RemainingCheckpoints1[2],RemainingCheckpoints1[3]};
                    }
                    else if (SecondCheckpoint == RemainingCheckpoints1[1]) {
                        double RemainingCheckpoints2[] = {RemainingCheckpoints1[0],RemainingCheckpoints1[2],RemainingCheckpoints1[3]};
                    }
                    else if (SecondCheckpoint == RemainingCheckpoints1[2]) {
                        double RemainingCheckpoints2[] = {RemainingCheckpoints1[0],RemainingCheckpoints1[1],RemainingCheckpoints1[3]};
                    }
                    else if (SecondCheckpoint == RemainingCheckpoints1[3]) {
                        double RemainingCheckpoints2[] = {RemainingCheckpoints1[0],RemainingCheckpoints1[1],RemainingCheckpoints1[2]};
                    }
                    else {
                        std::cout << "ERROR IN LOOP3 \n" << std::endl;
                    }
                    double ThirdCheckpoint = RemainingCheckpoints2[Loop3];
                    double SecondToThird = Distances(SecondCheckpoint, ThirdCheckpoint);


                    for (int Loop4 = 0; Loop4 < 2; Loop4++) {
                        double RemainingCheckpoints3[2];
                        if (ThirdCheckpoint == RemainingCheckpoints2[0]) {
                            double RemainingCheckpoints3[] = {RemainingCheckpoints2[1],RemainingCheckpoints2[2]};
                        }
                        else if (ThirdCheckpoint == RemainingCheckpoints2[1]) {
                            double RemainingCheckpoints3[] = {RemainingCheckpoints2[0], RemainingCheckpoints2[2]};
                        }
                        else if (ThirdCheckpoint == RemainingCheckpoints2[2]) {
                            double RemainingCheckpoints3[] = {RemainingCheckpoints2[0], RemainingCheckpoints2[1]};
                        }
                        else {
                            std::cout << "ERROR IN LOOP4 \n" << std::endl;
                        }
                        double FourthCheckpoint = RemainingCheckpoints3[Loop4];
                        double ThirdToFourth = Distances(ThirdCheckpoint, FourthCheckpoint);


                        //No fifth loop needed, just if statements


                        double FifthCheckpoint;
                        if (FourthCheckpoint == RemainingCheckpoints3[0]) {
                            FifthCheckpoint = RemainingCheckpoints3[1];
                        }
                        else if (FourthCheckpoint == RemainingCheckpoints3[1]) {
                            FifthCheckpoint = RemainingCheckpoints3[0];
                        }
                        else{
                            std::cout << "ERROR IN FIFTH CHECKPOINT \n" << std::endl;
                        }

                        double FourthToFifth = Distances(FourthCheckpoint,FifthCheckpoint);

                        double OverallDistance = ZeroToFirst+FirstToSecond+SecondToThird+ThirdToFourth+FourthToFifth;

                        OverallDistanceMatrix(Combinations,0) = OverallDistance;
                        OverallDistanceMatrix(Combinations,1) = FirstCheckPoint; //Beware the unorthodox syntax on FirstCheckPoint (capital P).
                        OverallDistanceMatrix(Combinations,2) = SecondCheckpoint;
                        OverallDistanceMatrix(Combinations,3) = ThirdCheckpoint;
                        OverallDistanceMatrix(Combinations,4) = FourthCheckpoint;
                        OverallDistanceMatrix(Combinations,5) = FifthCheckpoint;

                        Combinations++; //Update Combinations afterwards for easier syntax on the OverallDistanceMatrix
                    }
                }
            }
        }
        //outside the for loops now
        std::cout << "Number of Combinations Considered: " << Combinations << std::endl; //This SHOULD be 120 at the end !!
        std::cout << "Overall Distance Matrix: \n" << OverallDistanceMatrix << std::endl;

        //The above code adequately finds the overal distance matrix

        //The next step is to find the minimum distance travelled out of all of the combinations

        VectorXd OverallDistanceVector(120);
        for (int ODV = 0; ODV < 120; ODV++) {
            OverallDistanceVector(ODV) = OverallDistanceMatrix(ODV,0);
        }
        std::cout << "Overall Distance Vector: \n" << OverallDistanceVector << std::endl; //THis should be the same as the first column of the overall distance matrix
        VectorXd::Index MinimumIndex;
        double MinimumOverallDistance = OverallDistanceVector.minCoeff(&MinimumIndex);
        std::cout << "Minimum Overall DIstance: " << MinimumOverallDistance << std::endl;
        std::cout << "Minimum Index: " << MinimumIndex << std::endl;

        VectorXd QuickestCombination(6);
        QuickestCombination << OverallDistanceMatrix(MinimumIndex,0),
                               OverallDistanceMatrix(MinimumIndex,1),
                               OverallDistanceMatrix(MinimumIndex,2),
                               OverallDistanceMatrix(MinimumIndex,3),
                               OverallDistanceMatrix(MinimumIndex,4),
                               OverallDistanceMatrix(MinimumIndex,5);

        std::cout << "Quickest Combination: " << QuickestCombination << std::endl;


        //The above works, the results are as follows:
        //Shortest Distance Path is 1.35826, with Checkpoint order: 3 -> 1 -> 5 -> 4 -> 2  - NOTE THAT THIS IS FOR STARTING AT ALL JOINT ANGLES = 0

        //Now we need to figure out a way to associate this optimal checkpoint order with the my_pt1,2,3,4,5 !

        for (int PointLoop = 1; PointLoop < 6; PointLoop++) {
            double PointLoopElement = QuickestCombination(PointLoop); //returns the current checkpoint
            if (PointLoop == 1) {
                my_pt_previous = my_pt_initial;
            }
            double PreviousPoint = QuickestCombination(PointLoop-1); //returns the number of the previous checkpoint
            if (PreviousPoint == 1) {
                my_pt_previous = my_pt1;
            }
            else if (PreviousPoint == 2) {
                my_pt_previous = my_pt2;
            }
            else if (PreviousPoint == 3) {
                my_pt_previous = my_pt3;
            }
            else if (PreviousPoint == 4) {
                my_pt_previous = my_pt4;
            }
            else if (PreviousPoint == 5) {
                my_pt_previous = my_pt5;
            }


            if (PointLoopElement == 1) {
                //Now we need to push this to the trajectory !
                my_pt_to_be_split = my_pt1; //Current point - to be split into ten pieces
            }
            else if (PointLoopElement == 2) {
                my_pt_to_be_split = my_pt2;
            }
            else if (PointLoopElement == 3) {
                my_pt_to_be_split = my_pt3;
            }
            else if (PointLoopElement == 4) {
                my_pt_to_be_split = my_pt4;
            }
            else if (PointLoopElement == 5) {
                my_pt_to_be_split = my_pt5;
            }

            for (int g= 0; g<10; g++) {
                for (int r = 0; r < 5; r++) { //constructing five elements of points
                    my_pt_difference.positions.at(r) = my_pt_to_be_split.positions.at(r)-my_pt_previous.positions.at(r);
                    my_pt_difference.velocities.at(r) = my_pt_to_be_split.velocities.at(r)-my_pt_previous.velocities.at(r);

                    my_pt_split.positions.at(r) = my_pt_previous.positions.at(r)+my_pt_difference.positions.at(r)*((float)g / 10);
                    my_pt_split.velocities.at(r) = my_pt_difference.velocities.at(r); //See slides - this is the correct velocity gradient for straight line path !!
                }
                my_pt_split.time_from_start.sec = tfs;

                my_traj.points.push_back(my_pt_split);

                tfs = tfs + 1; //dt = 1s

                if (tfs == 60) { //allows us to have the initial point (0,0,0,0,0) at 10 seconds and the final point at 60 seconds - ideal!
                    my_pt_split.time_from_start.sec = tfs;

                    for (int ra = 0; ra < 5; ra++) {
                        my_pt_split.positions.at(ra) = my_pt_previous.positions.at(ra) + my_pt_difference.positions.at(ra); //eventually, the entire difference between previous and
                        my_pt_split.velocities.at(ra) = my_pt_difference.velocities.at(ra); //eventually, the entire difference between previous and
                    }
                    my_traj.points.push_back(my_pt_split);
                }
            }
        }

        //Printed out and tested my_traj and it is all in the correct order, at the correct time (one checkpoint every 10 seconds), using straight line trajectory planning

        //Now my_traj contains all of the points for the checkpoint in the desired order
        //The actual improvement on speed is gained from the optimisation algorithm (order of checkpoints).

        std::cout << "Final, optimised trajectories: " << my_traj << std::endl;

        std::cout << "my_pt1: \n" << my_pt1 << std::endl; //trajectory at 20 seconds
        std::cout << "my_pt2: \n" << my_pt2 << std::endl; //trajectory at 30 seconds
        std::cout << "my_pt3: \n" << my_pt3 << std::endl; //trajectory at 40 seconds
        std::cout << "my_pt4: \n" << my_pt4 << std::endl; //trajectory at 50 seconds
        std::cout << "my_pt5: \n" << my_pt5 << std::endl; //trajectory at 60 seconds

        //Confirmed that the order of my_traj is correct, hence the robot should move as quick as it can through the specified points order

        traj_pub.publish(my_traj); //Publish my_traj to the robot
        sleep(5);
        bag1.close();

    }


    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10);
    }

    return 123;
}
