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
    std::string str_contact_point_segments[] = {"fl_segment_slipz", "fr_segment_slipz", "ml_segment_slipz", "mr_segment_slipz", "rl_segment_slipz", "rr_segment_slipz"};

    std::string str_contact_angle_segments[] = {"fl_ground", "fr_ground", "ml_ground", "mr_ground", "rl_ground", "rr_ground"};

    std::string str_joint_names[] = {"left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz",
                                        "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz",
                                        "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz",
                                        "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz",
                                        "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz",
                                        "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"};

    std::string str_slip_joints[] = {"fl_slipx", "fr_slipx", "ml_slipx", "mr_slipx", "rl_slipx", "rr_slipx",
                                    "fl_slipy", "fr_slipy", "ml_slipy", "mr_slipy", "rl_slipy", "rr_slipy",
                                    "fl_slipz", "fr_slipz", "ml_slipz", "mr_slipz", "rl_slipz", "rr_slipz"};

    std::string str_contact_joints[] = {"fl_contact", "fr_contact", "ml_contact", "mr_contact", "rl_contact", "rr_contact"};
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

    /** Organized the Jacobian with the Slip and Contact Angles at the end (ONLY EXOTER) **/
    std::string str_motion_model_joint_names[] = {"left_passive",  "right_passive", "rear_passive", "fl_mimic", "fr_mimic", "ml_mimic", "mr_mimic", "rl_mimic", "rr_mimic",
                                    "fl_walking", "fr_walking", "ml_walking", "mr_walking", "rl_walking", "rr_walking",
                                    "fl_steer", "fr_steer", "rl_steer", "rr_steer",
                                    "fl_drive", "fr_drive", "ml_drive", "mr_drive","rl_drive", "rr_drive",
                                    "fl_translation", "fr_translation","ml_translation","mr_translation","rl_translation","rr_translation",
                                    "fl_slipx", "fr_slipx", "ml_slipx", "mr_slipx", "rl_slipx", "rr_slipx",
                                    "fl_slipy", "fr_slipy", "ml_slipy", "mr_slipy", "rl_slipy", "rr_slipy",
                                    "fl_slipz", "fr_slipz", "ml_slipz", "mr_slipz", "rl_slipz", "rr_slipz",
                                    "fl_contact","fr_contact","ml_contact", "mr_contact","rl_contact","rr_contact"};

    std::vector<std::string> motion_model_joint_names( str_motion_model_joint_names, str_motion_model_joint_names + (sizeof(str_motion_model_joint_names)/sizeof(std::string)));

    Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> JMotion;
    JMotion.resize(6*contact_points.size(), robotKDL.model_dof);

    robotKDL.organizeJacobian(0, motion_model_joint_names, joint_names, JKdl, JMotion);

    std::cout<<"** [TEST] JACOBIAN KDL is of size "<<JMotion.rows()<<" x "<<JMotion.cols()<<"\n"<< JMotion <<"\n\n";

}

