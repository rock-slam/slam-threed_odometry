/** Standard libraries **/
#include <map>
#include <vector>
#include <string>

/** Library includes **/
#include <threed_odometry/KinematicKDL.hpp>

#define BOOST_TEST_MODULE KinematicModelTest
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace threed_odometry;

BOOST_AUTO_TEST_CASE( URDFModel)
{

    std::string urdf_file = boost::unit_test::framework::master_test_suite().argv[1]; // Takes path for URDF file from arguments

    /***************************************/
    /** ATTENTION: You need to change the **/
    /** following values to the right ones**/
    /** for your Robot model. Manually in **/
    /** the code.                         **/
    /***************************************/

    /** VALUES FOR EXOTER **/
    std::string str_contact_point_segments[] = {"leg0_4_slip_z",
                                                 "leg1_4_slip_z",
                                                 "leg2_4_slip_z",
                                                 "leg3_4_slip_z",
                                                 "leg4_4_slip_z",
                                                 "leg5_4_slip_z"};

    std::string str_contact_angle_segments[] = {"leg0_4_contact", "leg1_4_contact", "leg2_4_contact", "leg3_4_contact", "leg4_4_contact", "leg5_4_contact"};

    std::string str_joint_names[] = {
                                "JointHead",
                                "leg0/joint0",
                                "leg0/joint1",
                                "leg0/joint2",
                                "leg0/joint3",
                                "leg0/joint4",
                                "leg0/joint_contact",
                                "leg0/joint_slipz",
                                "leg1/joint0",
                                "leg1/joint1",
                                "leg1/joint2",
                                "leg1/joint3",
                                "leg1/joint4",
                                "leg1/joint_contact",
                                "leg1/joint_slipz",
                                "leg2/joint0",
                                "leg2/joint1",
                                "leg2/joint2",
                                "leg2/joint3",
                                "leg2/joint4",
                                "leg2/joint_contact",
                                "leg2/joint_slipz",
                                "leg3/joint0",
                                "leg3/joint1",
                                "leg3/joint2",
                                "leg3/joint3",
                                "leg3/joint4",
                                "leg3/joint_contact",
                                "leg3/joint_slipz",
                                "leg4/joint0",
                                "leg4/joint1",
                                "leg4/joint2",
                                "leg4/joint3",
                                "leg4/joint4",
                                "leg4/joint_contact",
                                "leg4/joint_slipz",
                                "leg5/joint0",
                                "leg5/joint1",
                                "leg5/joint2",
                                "leg5/joint3",
                                "leg5/joint4",
                                "leg5/joint_contact",
                                "leg5/joint_slipz"};

    std::string str_slip_joints[] = {"leg0/joint_slipz",
                                        "leg1/joint_slipz",
                                        "leg2/joint_slipz",
                                        "leg3/joint_slipz",
                                        "leg4/joint_slipz",
                                        "leg5/joint_slipz"};

    std::string str_contact_joints[] = {"leg0/joint_contact",
                                        "leg1/joint_contact",
                                        "leg2/joint_contact",
                                        "leg3/joint_contact",
                                        "leg4/joint_contact",
                                        "leg5/joint_contact"};

    /*****************************************/
    /** END OF EDITABLE PART FOR PARAMETERS **/
    /*****************************************/
    std::vector<std::string> contact_points( str_contact_point_segments, str_contact_point_segments + (sizeof(str_contact_point_segments)/sizeof(std::string)));
    std::vector<std::string> contact_angles( str_contact_angle_segments, str_contact_angle_segments + (sizeof(str_contact_angle_segments)/sizeof(std::string)));
    std::vector<std::string> joint_names( str_joint_names, str_joint_names + (sizeof(str_joint_names)/sizeof(std::string)));
    std::vector<std::string> slip_joints( str_slip_joints, str_slip_joints + (sizeof(str_slip_joints)/sizeof(std::string)));
    std::vector<std::string> contact_joints( str_contact_joints, str_contact_joints + (sizeof(str_contact_joints)/sizeof(std::string)));
    const int number_robot_joints = joint_names.size() - slip_joints.size() - contact_joints.size();
    std::cout<<"** [TEST] ROBOT KDL MODEL *********\n";
    std::cout<<"** [TEST] URDF FILE: "<<urdf_file<<"\n";
    KinematicKDL robotKDL(urdf_file, contact_points, contact_angles, number_robot_joints, slip_joints.size(), contact_joints.size());
    std::cout<<"** [TEST] ROBOT MODEL_DOF: "<< robotKDL.model_dof <<"\n";

    std::vector<double> joint_positions (robotKDL.model_dof, 0);
    std::vector<Eigen::Affine3d> fkRobotKDL; // Vector of Forward Kinematics Transformation
    std::vector<base::Matrix6d> fkCovKDL; // Vector of Covariance matrices

    /** Store the values **/
    for (register int i=0; i<static_cast<int>(robotKDL.model_dof); ++i)
    {
        joint_positions[i] = 0.00;
    }

    //joint_positions[3] = 90.00 * D2R; //FL Steer joint (ONLY EXOTER)
    //joint_positions[40] = 90.00 * D2R; //FR Steer joint (ONLY EXOTER)

    std::cout<<"** [TEST] ROBOT JOINTS VALUES\n";
    for (std::vector<double>::iterator it = joint_positions.begin() ; it != joint_positions.end(); ++it)
        std::cout << ' ' << *it;
    std::cout << '\n';

    robotKDL.fkSolver(joint_positions, contact_points, fkRobotKDL, fkCovKDL);

    std::cout<<"** [TEST] fkRobotKDL is of size: "<<fkRobotKDL.size()<<" fkCovKDL of size: "<<fkCovKDL.size()<<"\n";

    for (register unsigned int i=0; i<fkRobotKDL.size(); ++i)
    {
        std::cout<<"[TEST] fkRobotKDL chain["<<i<<"]\n"<<fkRobotKDL[i].matrix()<<"\n";
    }

    std::cout<<"**\n\n** [TEST] ROBOT KDL JACOBIAN *********\n";
    Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> JKdl;
    JKdl.resize(6*contact_points.size(), robotKDL.model_dof);

    JKdl = robotKDL.jacobianSolver(joint_names, joint_positions);

    std::cout<<"** [TEST] JACOBIAN KDL is of size "<<JKdl.rows()<<" x "<<JKdl.cols()<<"\n"<< JKdl <<"\n\n";

    /***********************************************************************************************************/

    std::vector<std::string> motion_model_joint_names = joint_names;

    Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> JMotion;
    JMotion.resize(6*contact_points.size(), robotKDL.model_dof);

    robotKDL.organizeJacobian(0, motion_model_joint_names, joint_names, JKdl, JMotion);

    std::cout<<"** [TEST] JACOBIAN KDL is of size "<<JMotion.rows()<<" x "<<JMotion.cols()<<"\n"<< JMotion <<"\n\n";

}

