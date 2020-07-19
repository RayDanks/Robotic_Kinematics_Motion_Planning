#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include "cw2q4/youbotKine.h"
//#include "../../cw3/cw3q2/include/cw3q2/iiwa14Kine.h" //Navigating from one path to the other, since cw3q5 is non executable
#include "../../kdl_kine/include/kdl_kine/kdl_kine_solver.h"
#include <string>


void iiwa14_kinematics() {
    std::cout << "start of kine checker \n" << std::endl;

    Eigen::Matrix4d A;
    double curr_joint[7];

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix4d T_com = Eigen::Matrix4d::Identity(4, 4);

    MatrixXd jacobian(6, 7);
    MatrixXd jacobian_cm(6, 7);
    MatrixXd IK_closed;
    VectorXd IK_ite;
    bool singularity;
    MatrixXd B;
    MatrixXd C;
    VectorXd G;
    youbot_kinematic object;
    object.init();


    robot_kinematic object1; //KDL
    KDL::Frame KDL_FK;
    KDL::Jacobian KDL_JACOB;
    object1.init(2); //QUESTION NUMBER - I ADJUSTED THE FUNCTION!

    //feel free to change the values. but make sure they are inside the robot's workspace
    VectorXd pose1_values(7), pose2_values(7), pose3_values(7), pose4_values(7), pose5_values(7), pose0_values(7), pose7_values(7), pose8_values(7), Joint_Velocities_1(7);



    pose1_values(0) = -0.411883; pose1_values(1) = 1.30864; pose1_values(2) = -1.96052; pose1_values(3) = -1.07369; pose1_values(4) = -1.16671; pose1_values(5) = 0; pose1_values(6) = 0;
    pose2_values(0) = -1.66061; pose2_values(1) = 1.78931; pose2_values(2) = -2.54818; pose2_values(3) = -1.33999; pose2_values(4) = 1.70616; pose2_values(5) = 0; pose2_values(6) = 0;
    pose3_values(0) = -0.268307;  pose3_values(1) = 1.48098; pose3_values(2) = -2.04278; pose3_values(3) = -0.859284; pose3_values(4) = -2.2123; pose3_values(5) = 0; pose3_values(6) = 0;
    pose4_values(0) = -1.5747; pose4_values(1) = 1.0566; pose4_values(2) = -1.99377; pose4_values(3) = -2.25452; pose4_values(4) = 0.975057; pose4_values(5) = 0; pose4_values(6) = 0;
    pose5_values(0) = 1.785; pose5_values(1) = 2.0513; pose5_values(2) = -2.54818; pose5_values(3) = -1.34124; pose5_values(4) = 0.472335; pose5_values(5) = 0; pose5_values(6) = 0;
    pose0_values(0) = 0; pose0_values(1) = 0; pose0_values(2) = 0; pose0_values(3) = 0; pose0_values(4) = 0; pose0_values(5) = 0; pose0_values(6) = 0;
    pose7_values(0) = 0.4; pose7_values(1) = 2; pose7_values(2) = 1.57; pose7_values(3) = 0.77; pose7_values(4) = -1; pose7_values(5) = 1.23; pose7_values(6) = 0.83;
    pose8_values(0) = -2.1234; pose8_values(1) = 0.262683; pose8_values(2) = -3.09916; pose8_values(3) = -0.596647; pose8_values(4) = 0.220901; pose8_values(5) = -0.523965; pose8_values(6) = -1.26206;




    Joint_Velocities_1(0) = 0.1; Joint_Velocities_1(1) = 0.1; Joint_Velocities_1(2) = 0.1; Joint_Velocities_1(3) = 0.1; Joint_Velocities_1(4) = 0.1; Joint_Velocities_1(5) = 0.1; Joint_Velocities_1(6) = 0.1;


    std::cout << "after pose values \n" << std::endl;
//	//JOINT1
	T = object.forward_kine(pose1_values, 5);
	jacobian = object.get_jacobian(pose1_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose1_values);
	singularity = object.check_singularity(pose1_values);
	std::cout << "///// JOINT 1 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose1_values[0] << ", " << pose1_values[1] << ", " << pose1_values[2] << ", " << pose1_values[3] << ", " << pose1_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity checker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

//	//JOINT2
	T = object.forward_kine(pose2_values, 5);
	jacobian = object.get_jacobian(pose2_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose2_values);
	singularity = object.check_singularity(pose2_values);
	std::cout << "///// JOINT 2 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose2_values[0] << ", " << pose2_values[1] << ", " << pose2_values[2] << ", " << pose2_values[3] << ", " << pose2_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity checker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

//	//JOINT3
	T = object.forward_kine(pose3_values, 5);
	jacobian = object.get_jacobian(pose3_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose3_values);
    singularity = object.check_singularity(pose3_values);
	std::cout << "///// JOINT 3 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose3_values[0] << ", " << pose3_values[1] << ", " << pose3_values[2] << ", " << pose3_values[3] << ", " << pose3_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity checker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;

    //JOINT4

    T = object.forward_kine(pose4_values, 7);
    std::cout << "My FK: \n" << T << std::endl;
    for (int i = 1;i < 8; i++) {
        T_com = object.forward_kine_cm(pose7_values, i);
        std::cout << "my T_com: \n" << T_com << std::endl;
    }
    jacobian = object.get_jacobian(pose7_values);
    std::cout << "My Jacobian: \n" << jacobian << std::endl;

    for (int i = 1;i < 8;i++) {
        jacobian_cm = object.get_jacobian_cm(pose7_values, i);
        std::cout << "My jacobian_cm: \n" << jacobian_cm << std::endl;
    }
    IK_closed = object.inverse_kine_closed_form(T);
    IK_ite = object.inverse_kine_ite(T, pose7_values);
    singularity = object.check_singularity(pose4_values);
    KDL::JntArray KDL_Joints;
    KDL_Joints.resize(7);


    for (int i = 0;i<7;i++) {
        KDL_Joints(i) = pose4_values(i);
    }

    std::cout << "KDL JOINTS: " << KDL_Joints.data << std::endl;


    KDL_FK = object1.KDLfkine(KDL_Joints);
    KDL_JACOB = object1.KDLjacob(KDL_Joints);

    KDL::JntArray IK_CLOSED_KDL = object1.inverse_kinematics_closed(KDL_FK);
    std::cout << "KDL IK_CLOSED: " << IK_CLOSED_KDL.data << std::endl;
    MatrixXd KDL_B = object1.getB(pose4_values);
    VectorXd KDL_C = object1.getC(pose4_values, Joint_Velocities_1);
    VectorXd KDL_G = object1.getG(pose4_values);
    std::cout << "KDL B: \n" << KDL_B << std::endl;
    std::cout << "KDL C: \n" << KDL_C << std::endl;
    std::cout << "KDL G: \n" << KDL_G << std::endl;


    std::cout << "KDL FK: \n" << KDL_FK << std::endl;
    std::cout << "KDL JACOBIAN \n" << KDL_JACOB.data << std::endl;
//    ROS_INFO(KDL_FK);



    std::cout << "///// JOINT 4 /////" << std::endl;
    std::cout << "The input joint values are: \n" << "[" << pose4_values[0] << ", " << pose4_values[1] << ", " << pose4_values[2] << ", " << pose4_values[3] << ", " << pose4_values[4] << "]" << "\n" << std::endl;
    std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
    std::cout << "Closed form IK, NaN represents an invalid solution (eg past joint limits): \n"<< IK_closed << "\n" << std::endl;
    std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
    std::cout << "B Matrix: \n" << B << std::endl;
    std::cout << "Singularity checker: \n"<< singularity << "\n" << std::endl;
    std::cout << "\n" << std::endl;

//	//JOINT5
	T = object.forward_kine(pose5_values, 5);
	jacobian = object.get_jacobian(pose5_values);
	IK_closed = object.inverse_kine_closed_form(T);
	IK_ite = object.inverse_kine_ite(T, pose5_values);
    singularity = object.check_singularity(pose5_values);
	std::cout << "///// JOINT 5 /////" << std::endl;
	std::cout << "The input joint values are: \n" << "[" << pose5_values[0] << ", " << pose5_values[1] << ", " << pose5_values[2] << ", " << pose5_values[3] << ", " << pose5_values[4] << "]" << "\n" << std::endl;
	std::cout << "Jacobian: \n"<< jacobian << "\n" << std::endl;
	std::cout << "Closed form IK: \n"<< IK_closed << "\n" << std::endl;
	std::cout << "Iterative form IK: \n"<< IK_ite << std::endl;
	std::cout << "Singularity checker: \n"<< singularity << "\n" << std::endl;
	std::cout << "\n" << std::endl;
}











int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_checker_node");
    ros::NodeHandle nh;

    iiwa14_kinematics();
    ros::spin();
}
