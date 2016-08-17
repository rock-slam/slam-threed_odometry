/**\file KinematicKDL.hpp
*
* This class has the primitive methods for the Forward kinematics and Wheel Jacobian.
* First computes the forward kinematics in order to have afterwards the wheel Jacobians using KDL.
*
* It reads a URDF file to create KDL Chains using the kdl_parse.
*
* @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
* @date December 2014.
* @version 1.0.
*/


#ifndef ODOMETRY_KINEMATIC_KDL_HPP
#define ODOMETRY_KINEMATIC_KDL_HPP

/* Base includes */
#include <assert.h> /** Assert **/
#include <sstream>
#include <string> /** This should be already in sstream **/

/** Rock Base Library **/
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base-logging/Logging.hpp>

#include <Eigen/Core>

/* KDL Parser */
#include <kdl_parser/kdl_parser.hpp>

/** KDL library **/
#include <kdl/frames_io.hpp> /** Defines KDL frames **/
#include <kdl/treefksolver.hpp> /** Solve forward kinematics **/
#include <kdl/treefksolverpos_recursive.hpp> /** Solve forward kinematics **/
#include <kdl/chainjnttojacsolver.hpp> /** For Jacobian matrix **/
#include <kdl/treejnttojacsolver.hpp> /** For Jacobian matrix **/



namespace threed_odometry
{

    class KinematicKDL
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members

    protected:
        KDL::Tree tree; /** There are as many chains in the tree as number of contact points **/
        std::string model_name; /** Name of the model class **/
        std::vector<KDL::Chain> ichains; /** Kinematics chains **/
        std::vector< std::vector<std::string> > ichains_joint_names; /** Kinematics chains **/
        std::vector<std::string> contact_point_segments; /** Number and segment names of contact points **/
        std::vector<std::string> contact_angle_segments; /** Number and segment names of contact angles **/
        int number_robot_joints, number_slip_joints, number_contact_joints; /** Number of joints per each type **/

    public:
        unsigned int model_dof; /** Complete robot model DoF **/


    public:
        KinematicKDL (const std::string &urdf_file, const std::vector<std::string> &_contact_points,
                const std::vector<std::string> &contact_angles, const int _number_robot_joints, const int _number_slip_joints, const int _number_contact_joints);
        ~KinematicKDL();

        std::string getName();
        void fkSolver(const std::vector<double> &positions, const std::vector<std::string> &tip_names, std::vector<Eigen::Affine3d> &fkTrans, std::vector<base::Matrix6d> &fkCov);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobianSolver(const std::vector<std::string> &names, const std::vector<double> &positions);
        void unpackJoints(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions,
                                const std::vector<std::string>& involved_joints, KDL::JntArray& joint_array);
        void organizeJacobian(const int chainidx, const std::vector<std::string> &joint_names, const std::vector<std::string> &involved_joints,
                                    const Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> &jacobian, Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> &J);

    };

    static void printLink(const KDL::SegmentMap::const_iterator& link, const std::string& prefix)
    {
        std::cout << prefix << "- Segment " << link->second.segment.getName() << " [Joint "<< link->second.segment.getJoint().getName()<<
            "] origin axis: "<<link->second.segment.getJoint().JointOrigin()<<" rotating axis: "<<link->second.segment.getJoint().JointAxis()<<" has " << link->second.children.size() << " children" << std::endl;
        for (unsigned int i=0; i<link->second.children.size(); ++i)
            printLink(link->second.children[i], prefix + "  ");
    };

    static std::string getContactPoint (const KDL::SegmentMap::const_iterator& link, const int childrenIdx)
    {
        if (link->second.children.size() == 0)
            return link->second.segment.getName();

        std::cout<<link->second.segment.getName() << " has "<<link->second.children.size()<<" children\n";
        return getContactPoint(link->second.children[childrenIdx], 0);
    };

};

#endif
