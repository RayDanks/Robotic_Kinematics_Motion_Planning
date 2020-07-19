#include <cw2q4/youbotKine.h>


void youbot_kinematic::init()
{
    DH_params[0][0] = -0.033;   DH_params[1][0] = 0.155;  DH_params[2][0] = 0.135; DH_params[3][0] = -0.002;  DH_params[4][0] = 0.0;
    DH_params[0][1] = M_PI_2;   DH_params[1][1] = 0.0;    DH_params[2][1] = 0.0;   DH_params[3][1] = M_PI_2;  DH_params[4][1] = M_PI;
    DH_params[0][2] = 0.145;    DH_params[1][2] = 0.0;    DH_params[2][2] = 0.0;   DH_params[3][2] = 0.0;     DH_params[4][2] = -0.185;
    DH_params[0][3] = M_PI;     DH_params[1][3] = M_PI_2; DH_params[2][3] = 0.0;   DH_params[3][3] = -M_PI_2; DH_params[4][3] = M_PI;

    joint_offset[0] = 170*M_PI/180;
    joint_offset[1] = 65*M_PI/180;
    joint_offset[2] = -146*M_PI/180;
    joint_offset[3] = 102.5*M_PI/180;
    joint_offset[4] = 167.5*M_PI/180;

    joint_limit_min[0] = -169*M_PI/180;
    joint_limit_min[1] = -65*M_PI/180;
    joint_limit_min[2] = -150*M_PI/180;
    joint_limit_min[3] = -102.5*M_PI/180;
    joint_limit_min[4] = -167.5*M_PI/180;

    joint_limit_max[0] = 169*M_PI/180;
    joint_limit_max[1] = 90*M_PI/180;
    joint_limit_max[2] = 146*M_PI/180;
    joint_limit_max[3] = 102.5*M_PI/180;
    joint_limit_max[4] = 167.5*M_PI/180;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &youbot_kinematic::joint_state_callback, this);
}


void youbot_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for(int i = 0; i < 5; i++)
        current_joint_position[i] = q->position.at(i);

    current_pose = forward_kine(current_joint_position, 5);
    broadcast_pose(current_pose);
}

Matrix4d youbot_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Matrix4d A;
    A(3, 3) = 1.0;
    A(3, 2) = 0.0;
    A(3, 1) = 0.0;
    A(3, 0) = 0.0;

    A(0, 0) = cos(theta);
    A(0, 1) = -sin(theta)*cos(alpha);
    A(0, 2) = sin(theta)*sin(alpha);
    A(0, 3) = a * cos(theta);

    A(1, 0) = sin(theta);
    A(1, 1) = cos(theta)*cos(alpha);
    A(1, 2) = -cos(theta)*sin(alpha);
    A(1, 3) = a * sin(theta);

    A(2, 0) = 0.0;
    A(2, 1) = sin(alpha);
    A(2, 2) = cos(alpha);
    A(2, 3) = d;

    return A;
}


Matrix4d youbot_kinematic::forward_kine(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_val[i] + DH_params[i][3]);

        T = T * A;
    }

    return T;
}

Matrix4d youbot_kinematic::forward_kine_offset(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        if (i == 0)
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_offset[i] - (joint_val[i] + DH_params[i][3]));
        else
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], (joint_val[i] + DH_params[i][3]) - joint_offset[i]);

        T = T * A;
    }

    return T;
}

void youbot_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "base_link";
    T.child_frame_id = "arm_end_effector";

    pose_br.sendTransform(T);
}

MatrixXd youbot_kinematic::get_jacobian(double joint_val[])
{
    youbot_kinematic::init(); //gets the DH parameters

    //First thing to do is to specify the transformation matrices:
    Matrix4d ZeroTOne = youbot_kinematic::forward_kine(joint_val, 1); //Frame is 1. Joint_val is how joint rotations change - entire matrix is used by forward_kine function.
    Matrix4d ZeroTTwo = youbot_kinematic::forward_kine(joint_val, 2);
    Matrix4d ZeroTThree = youbot_kinematic::forward_kine(joint_val, 3);
    Matrix4d ZeroTFour = youbot_kinematic::forward_kine(joint_val, 4);
    Matrix4d ZeroTFive = youbot_kinematic::forward_kine(joint_val, 5);

    //Now the Z values must be derived/assigned:

    //Z0 - Origin!
    Vector3d Z0;
    Z0(0) = 0;
    Z0(1) = 0;
    Z0(2) = 1; //No rotation

    //Z1
    Vector3d Z1;
    Z1(0) = ZeroTOne(0,2);
    Z1(1) = ZeroTOne(1,2);
    Z1(2) = ZeroTOne(2,2);

    //Z2
    Vector3d Z2;
    Z2(0) = ZeroTTwo(0,2);
    Z2(1) = ZeroTTwo(1,2);
    Z2(2) = ZeroTTwo(2,2);

    //Z3
    Vector3d Z3;
    Z3(0) = ZeroTThree(0,2);
    Z3(1) = ZeroTThree(1,2);
    Z3(2) = ZeroTThree(2,2);

    //Z4
    Vector3d Z4;
    Z4(0) = ZeroTFour(0,2);
    Z4(1) = ZeroTFour(1,2);
    Z4(2) = ZeroTFour(2,2);

    //Z5 - This actually isn't needed due to Zi-1 being used and never hence reaching Z5!
    Vector3d Z5;
    Z5(0) = ZeroTFive(0,2);
    Z5(1) = ZeroTFive(1,2);
    Z5(2) = ZeroTFive(2,2);


    //Now all the O values must be defined

    //O0 - Origin!
    Vector3d O0;
    O0(0) = 0;
    O0(1) = 0;
    O0(2) = 0;

    //O1
    Vector3d O1;
    O1(0) = ZeroTOne(0,3);
    O1(1) = ZeroTOne(1,3);
    O1(2) = ZeroTOne(2,3);

    //O2
    Vector3d O2;
    O2(0) = ZeroTTwo(0,3);
    O2(1) = ZeroTTwo(1,3);
    O2(2) = ZeroTTwo(2,3);

    //O3
    Vector3d O3;
    O3(0) = ZeroTThree(0,3);
    O3(1) = ZeroTThree(1,3);
    O3(2) = ZeroTThree(2,3);

    //O4
    Vector3d O4;
    O4(0) = ZeroTFour(0,3);
    O4(1) = ZeroTFour(1,3);
    O4(2) = ZeroTFour(2,3);

    //O5 == Pe !!
    Vector3d O5;
    O5(0) = ZeroTFive(0,3);
    O5(1) = ZeroTFive(1,3);
    O5(2) = ZeroTFive(2,3);

    //Now formally define the equivalency between O5 and Pe
    Vector3d Pe = O5; //This is the only time 05 is used, due to Oi-1 being used in the Jacobian element equation.


    //Now we must define the Jpi Jacobian elements

    //Matrix 4d and Vector 3d are part of the eigen library so we can use eigen cross!
    //Matrix 4d = matrix of four doubles etc.

    Vector3d Jp1 = Z0.cross((Pe-O0));
    Vector3d Jp2 = Z1.cross((Pe-O1));
    Vector3d Jp3 = Z2.cross((Pe-O2));
    Vector3d Jp4 = Z3.cross((Pe-O3));
    Vector3d Jp5 = Z4.cross((Pe-O4));

    //The Joi terms need not be defined as they are simple substitution.

    //Now, these vectors must be put together to form the final Jacobian Matrix

    MatrixXd J(6,5); //Number of values, ie not starting from 0.

    for (int j1 = 0; j1 < 3 ;j1++) { //Loop for susbtituting in J1
        J(j1, 0) = Jp1[j1];
    }
    for (int j2 = 0;j2<3;j2++) { //Loop for susbtituting in J2
        J(j2,1) = Jp2[j2];
    }
    for (int j3 = 0;j3<3;j3++) { //Loop for susbtituting in J3
        J(j3,2) = Jp3[j3];
    }
    for (int j4 = 0;j4<3;j4++) { //Loop for susbtituting in J4
        J(j4,3) = Jp4[j4];
    }
    for (int j5 = 0;j5<3;j5++) { //Loop for susbtituting in J5
        J(j5,4) = Jp5[j5];
    }

    //Now that the Jpi terms are substituted in, the Jpo terms must be too.
    //Note that Oi = Zi-1 (for revolute joints)

    for (int k1 = 3; k1 < 6 ;k1++) { //Loop for susbtituting in O1
        J(k1, 0) = Z0[k1-3];
    }
    for (int k2 = 3;k2<6;k2++) { //Loop for susbtituting in O2
        J(k2,1) = Z1[k2-3];
    }
    for (int k3 = 3;k3<6;k3++) { //Loop for susbtituting in O3
        J(k3,2) = Z2[k3-3];
    }
    for (int k4 = 3;k4<6;k4++) { //Loop for susbtituting in O4
        J(k4,3) = Z3[k4-3];
    }
    for (int k5 = 3;k5<6;k5++) { //Loop for susbtituting in O5
        J(k5,4) = Z4[k5-3];
    }

    //Now all of the values should have been substituted in, so the Jacobian is complete.

    return J;

}

MatrixXd youbot_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    std::cout << "TEST IK Closed Form Method \n" <<std::endl;
    //TODO: Fill in this function to complete the question 3c
    double r11 = pose(0,0);
    double r12 = pose(0,1);
    double r13 = pose(0,2);
    double r21 = pose(1,0);
    double r22 = pose(1,1);
    double r23 = pose(1,2);
    double r31 = pose(2,0);
    double r32 = pose(2,1);
    double r33 = pose(2,2);
    double pex = pose(0,3);
    double pey = pose(1,3);
    double pez = pose(2,3);
    double a1 = DH_params[0][0];
    double a2 = DH_params[1][0];
    double a3 = DH_params[2][0];
    double a4 = DH_params[3][0];
    double d1 = DH_params[0][2];
    double d5 = DH_params[4][2];

    double t1 = atan2(r13,r23);
    double t5 = atan2(r31,r32);

    double A = pez-d1+d5*r33-(a4*r23)/sin(t1);
    double B = (pey/sin(t1))+a1+a4*r33-(d5*r32)/sin(t5);

    double t3 = acos((A*A+B*B-a3*a3-a2*a2)/(2*a3*a2));

    double w1 = a3*cos(t3)+a2;
    double w2 = a3*sin(t3);

    double epsilon = atan2(w1,w2);

    double t2 = acos(B/(sqrt(w1*w1+w2*w2)))-epsilon;

    double t4 = acos(r33)-t2-t3;

    MatrixXd JointAngles(5,1); //THis could have been a vector but the function needs a MatrixXd - not sure what form this outcome was wanted in really
    JointAngles << t1,
                   t2,
                   t3,
                   t4,
                   t5;
    std::cout << "Joint Angles, from theta1 to theta5: " << JointAngles << std::endl;

    return JointAngles;
}


double* youbot_kinematic::inverse_kine_ite(Matrix4d pose, double joint_val[])

{
    //Note that the input is 4x4 pose and current joint values

    //First thing is to define a desired end effector pose
    double DEEPex = pose(0,3); //Desired End effector position along x axis.
    double DEEPey = pose(1,3);
    double DEEPez = pose(2,3);

   Matrix3d DesiredRotMat;
   DesiredRotMat << pose(0,0), pose(0,1), pose(0,2),
                    pose(1,0), pose(1,1), pose(1,2),
                    pose(2,0), pose(2,1), pose(2,2);

    AngleAxisd TargetRotation(DesiredRotMat);
    VectorXd TargetEEPose(6);

    TargetEEPose(0) = DEEPex;
    TargetEEPose(1) = DEEPey;
    TargetEEPose(2) = DEEPez;
    TargetEEPose(3) = TargetRotation.axis()(0);
    TargetEEPose(4) = TargetRotation.axis()(1);
    TargetEEPose(5) = TargetRotation.axis()(2);

    double TargetEEPoseMagnitude = TargetEEPose.norm();

    //Firstly, joint values must be initialised as zero:
    //These parameters must be created here, so that the q array may be edited throughout the while loop.
    //THis is the intialisation of the q matrix - checking where the joints are currently.
    //NOte that throughout the iterative IK process, the ACTUAL joints need not move, they just move to the right position when the final q matrix is gained.

    double k0 = joint_val[0];
    double k1 = joint_val[1];
    double k2 = joint_val[2];
    double k3 = joint_val[3];
    double k4 = joint_val[4];

    //5 thetas/joint values otherwise the q matrix would not be the correct dimensions at the end of the while loop!

    int Iterations = 0;
    while(1) {
        Iterations++;
        double* q = new double[5]; //THis is necessary since once the double array is returned, it will be decimated - with the double* array declaration, it will not.
        q[0] = k0;
        q[1] = k1;
        q[2] = k2;
        q[3] = k3;
        q[4] = k4;

        Matrix4d FKIte = youbot_kinematic::forward_kine(q, 5); //Finding the transformation between base and end effector each time - 4x4 matrix.
        MatrixXd JIte(6,5);
        JIte = youbot_kinematic::get_jacobian(q); //computing the jacobian for this respective iteration

        //We need to now get the end effector pose in a similar way to the desired end effector pose (6x1 vector).
        Matrix3d RotMat;
        RotMat <<   FKIte(0,0), FKIte(0,1), FKIte(0,2),
                    FKIte(1,0), FKIte(1,1), FKIte(1,2),
                    FKIte(2,0), FKIte(2,1), FKIte(2,2);

        //Note that if rotation was not in Axis-Angle/Rodrigues form then the updated q vectors (joint_val) would not be the right size!
        AngleAxisd CurrentRot(RotMat);

        VectorXd Xe(6);
        Xe << FKIte(0,3),
              FKIte(1,3),
              FKIte(2,3),
              CurrentRot.axis()(0),
              CurrentRot.axis()(1),
              CurrentRot.axis()(2); //Current End Effector pose, with rotation in Rodrigues Representation.

        //Now the new values for joint positions must be found (q(k) in the slides), using the Jacobian.
        //Since the Jacobian is not square, its inverse cannot be used so we are using transpose method.
        double alpha = 0.1;
        //Now the q double array must be mapped to an Eigen::Vector4d, otherwise it cannot interface with the other Eigen::Vector objects.
        VectorXd qEigen(5); //Works easier than the map function.
        qEigen << q[0],q[1],q[2],q[3],q[4];
        VectorXd q_newEigen(5);
        q_newEigen = qEigen+alpha*JIte.transpose()*(TargetEEPose - Xe);

        //Now, in order for the "new" q to be used in the next iteration of this loop, it must be reconverted into a double array.
        double q_new[] = {q_newEigen(0),q_newEigen(1),q_newEigen(2),q_newEigen(3),q_newEigen(4)};
        //This in turn updates the q array, ready for the next iteration.
        k0 = q_new[0];
        k1 = q_new[1];
        k2 = q_new[2];
        k3 = q_new[3];
        k4 = q_new[4];



        //Now, convergence criteria must be defined:
        //If the end effector is within 5% (95% accuracy) of the desired position, terminate the loop:

        //m_axis and m_angle are private so we cannot access elements of the axis-angle vector
        //Convert target rotation and current rotation to quaternions, PURELY to compare the two.


        std::cout << "k0: \n" << q_new[0] << std::endl;
        std::cout << "k1: \n" << q_new[1] << std::endl;
        std::cout << "k2: \n" << q_new[2] << std::endl;
        std::cout << "k3: \n" << q_new[3] << std::endl;
        std::cout << "k4: \n" << q_new[4] << std::endl;

    double XeMagnitude = Xe.norm();
    double convergenceCriteria = 0.01;

    if ((TargetEEPoseMagnitude-XeMagnitude) < convergenceCriteria) {
        std::cout << "q: " << *q << std::endl; //*q converts q from pointer
        std::cout << "Xe: " << Xe << std::endl;
        std::cout << "TargetEEPose: " << TargetEEPose << std::endl;
        std::cout << "Theta1: " << q[0] << std::endl;
        std::cout << "Theta2: " << q[1] << std::endl;
        std::cout << "Theta3: " << q[2] << std::endl;
        std::cout << "Theta4: " << q[3] << std::endl;
        std::cout << "Theta5: " << q[4] << std::endl;
        return q;
    }

        //Now, the situation where the target pose is outside of the robot workspace must be accounted for:
        if (q_new == q){ //If the joints are no longer moving
            double WorkspaceIterations = 0;
            WorkspaceIterations++;
            if (WorkspaceIterations == 10) {
                std::cout << "The robot is at the edge of its workspace/is no longer moving \n" <<std::endl;
                std::cout << "Iterations Required: \n" << Iterations << std::endl;
                return q;
            }
        }
    }
}


bool youbot_kinematic::check_singularity(double joint_val[])
{
    //A singularity occurs when two revolute joint's axes are fully aligned (eg when the robot is fully outstretched)
    //THis can be detected by the Jacobian matrix losing rank and becoming singular
    //When a matrix 'loses rank', its determinant is zero

    //However, our jacobian is non-square and hence the determinant is undefined.
    //IN this case, when the following equation is satisfied: abs((J^T)*J) = 0 when a singularity occurs.#
    //Note: by abs(), the determinant is meant by this - J^T*J will lead to a 6x6 matrix, where the determinant can be found.

    MatrixXd J = youbot_kinematic::get_jacobian(joint_val);
    MatrixXd JT = J.transpose();
    MatrixXd JTJ = JT*J;
    double DetJTJ = JTJ.determinant();

    if (DetJTJ == 0) {
        std::cout << "Singularity Detected";
        return true; //This syntax is used since this function is defined as a boolean.
    }
    else {
        return false;
    }
}
