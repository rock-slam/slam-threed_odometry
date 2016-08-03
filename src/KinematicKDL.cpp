#include "KinematicKDL.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define EIGEN_NO_AUTOMATIC_RESIZING //To avoid automatic resizing of Dynamic Matrices

//#define DEBUG_PRINTS 1
/* URDF */
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_world/types.h>

using namespace threed_odometry;

KinematicKDL::KinematicKDL (const std::string &urdf_file, const std::vector<std::string> &_contact_point_segments,
        const std::vector<std::string> &_contact_angle_segments, const int _number_robot_joints,
        const int _number_slip_joints, const int _number_contact_joints):
        contact_point_segments(_contact_point_segments),
        contact_angle_segments(_contact_angle_segments),
        number_robot_joints(_number_robot_joints),
        number_slip_joints(_number_slip_joints),
        number_contact_joints(_number_contact_joints)
{


    /** Read URDF file **/
    std::string xml_string;
    const char * urdf_char = urdf_file.c_str();
    std::fstream xml_file(urdf_char, std::fstream::in);
    while ( xml_file.good() )
    {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
    }
    xml_file.close();

    urdf::ModelInterfaceSharedPtr robot = urdf::parseURDF(xml_string);
    if (!robot)
    {
        throw std::runtime_error("[KDL_MODEL] Constructor could not parse URDF model\n");
    }

    if (this->contact_point_segments.size() != this->contact_angle_segments.size())
    {
        throw std::runtime_error("[KDL_MODEL] Constructor, kinematic model can not have different size of contact points and contact angles segments\n");
    }

    /** Assign an information string name and complete model DoF **/
    this->model_name = robot->getName();
    this->model_dof = this->number_robot_joints + this->number_slip_joints + this->number_contact_joints;

    /** Resize vectors of chains **/
    this->ichains.resize(this->contact_point_segments.size());
    this->ichains_joint_names.resize(this->contact_point_segments.size());

    #ifdef DEBUG_PRINTS
    std::cout<<"[KDL_MODEL] number_robot_joints: "<<this->number_robot_joints<<"\n";
    std::cout<<"[KDL_MODEL] number_slip_joints: "<<this->number_slip_joints<<"\n";
    std::cout<<"[KDL_MODEL] number_contact_joints: "<<this->number_contact_joints<<"\n";
    std::cout<<"[KDL_MODEL] model_dof: "<<this->model_dof<<"\n";
    #endif

    /** Get root link*/
    urdf::LinkConstSharedPtr root_link = robot->getRoot();
    if (!root_link)
        throw std::runtime_error("[KDL_MODEL] Constructor could not find Root link\n");

    if (!kdl_parser::treeFromUrdfModel(*robot, this->tree))
        throw std::runtime_error("[KDL_MODEL] Constructor could not extract KDL tree\n");

    LOG_INFO("[KDL_MODEL] Robot name is: %s\n", this->model_name.c_str());
    LOG_INFO("[KDL_MODEL] Robot Link: %s has %d child(ren)\n", root_link->name.c_str(), root_link->child_links.size());

    /** Extract the chains in inverse order for the Motion Model Jacobian matrix **/
    KDL::SegmentMap::const_iterator root = this->tree.getRootSegment();
    std::string rootName = root->second.segment.getName();

    for (register int i=0; i<static_cast<int>(this->contact_point_segments.size()); ++i)
    {
        bool exit_value = tree.getChain(this->contact_point_segments[i], rootName, this->ichains[i]);
        if (!exit_value)
        {
            LOG_ERROR("Error extracting chain with root '%s' and tip '%s'",
                  this->contact_point_segments[i].c_str(), rootName.c_str());
            throw std::runtime_error("[KDL_MODEL] Constructor could not extract a chain in inverse order\n");
        }
    }

    /** Extract the joint names for the chains in inverse order **/
    register int chainidx = 0;
    for(std::vector<KDL::Chain>::const_iterator it = this->ichains.begin();
                it != this->ichains.end(); it++)
    {
        for(register int s=0; s<static_cast<int>((*it).segments.size()); ++s)
        {
            KDL::Segment segment = (*it).segments[s];
            std::string jname = segment.getJoint().getName();

            if(segment.getJoint().getType() == KDL::Joint::None)
            {
                LOG_DEBUG("Skipping joint %s of chain %s. Joint is fixed.",
                          jname.c_str(), segment.getName().c_str());
                continue;
            }
            #ifdef DEBUG_PRINTS
            std::cout<<"[JACOBIAN_SOLVER_TREE] Joint Name: "<<jname<<"\n";
            #endif
            this->ichains_joint_names[chainidx].push_back(jname);
        }

        chainidx++;
    }

    #ifdef DEBUG_PRINTS
    /** Walk through tree **/
    std::cout << " ======================================" << std::endl;
    std::cout << " [KDL_MODEL] Tree has " << this->tree.getNrOfSegments() << " link(s) and "<< this->tree.getNrOfJoints() <<" Joint(s)\n";
    std::cout << " ======================================" << std::endl;
    KDL::SegmentMap::const_iterator root_print = this->tree.getRootSegment();
    printLink(root_print, "");
    #endif
}

KinematicKDL::~KinematicKDL()
{
}

std::string KinematicKDL::getName ()
{
        return this->model_name;
}

void KinematicKDL::fkSolver(const std::vector<double> &positions, const std::vector<std::string> &tip_names,
        std::vector<Eigen::Affine3d> &fkTrans, std::vector<base::Matrix6d> &fkCov)
{
    /** Check if the number of values is correct **/
    if (positions.size() == this->model_dof)
    {
        /** Resize the vectors **/
        fkTrans.resize(tip_names.size());
        fkCov.resize(tip_names.size());

        /** Forward kinematic solver of the robot tree **/
        KDL::TreeFkSolverPos_recursive fksolver = KDL::TreeFkSolverPos_recursive(tree);

        /** Create joint array **/
        unsigned int nj = tree.getNrOfJoints();
        KDL::JntArray jointpositions = KDL::JntArray(nj);
        #ifdef DEBUG_PRINTS
        std::cout<<"[FORWARD_KINEMATICS_TREE] Number of Joints is: "<<nj<<"\n";
        std::cout<<"[FORWARD_KINEMATICS_TREE] Joints are: \n";
        #endif

        /** Fill the Joint Array with the position values **/
        for (register int i=0; i<static_cast<int>(positions.size()); ++i)
        {
            jointpositions(i) = positions[i];
            #ifdef DEBUG_PRINTS
            std::cout<<positions[i]<<"\n";
            #endif
        }

        /** Calculate the Forward Kinematics for all the chains of the robot **/
        for (register int i=0; i<static_cast<int>(tip_names.size()); ++i)
        {
            /** Create the frame that will contain the results **/
            KDL::Frame cartpos;

            /** Calculate forward position kinematics **/
            bool kinematics_status;
            kinematics_status = fksolver.JntToCart(jointpositions, cartpos, tip_names[i]);
            if(kinematics_status>=0)
            {
                /** Store the solution in the arguments **/
                double x,y,z,w;
                cartpos.M.GetQuaternion(x,y,z,w);
                Eigen::Quaternion<double> orientation(w,x,y,z);
                fkTrans[i] = Eigen::Affine3d(orientation);
                fkTrans[i].translation() = Eigen::Vector3d (cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
                #ifdef DEBUG_PRINTS
                std::cout<<"[FORWARD_KINEMATICS_TREE] Transformation ["<<i<<"] ("<<tip_names[i]<<")\n"<<fkTrans[i].matrix()<<"\n";
                #endif
            }

            /** No uncertainty provided **/
            fkCov[i].setZero();
        }
    }
    return;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> KinematicKDL::jacobianSolver(const std::vector<std::string> &names, const std::vector<double> &positions)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J;
    J.resize(6*this->contact_point_segments.size(), this->model_dof); J.setZero();

    /** Check if the number of values is correct **/
    if (positions.size() == this->model_dof)
    {
        /** Get Forward Kinematics of all contact points **/
        std::vector<Eigen::Affine3d> fkTrans;
        std::vector<base::Matrix6d> fkCov;
        this->fkSolver(positions, this->contact_angle_segments, fkTrans, fkCov);

        register int chainidx = 0;
        /** Calculate the Jacobian for all the inverse chains of the robot **/
        for(std::vector<KDL::Chain>::const_iterator it = this->ichains.begin();
                it != this->ichains.end(); it++)
        {
            /** Jacobian solver of the robot tree **/
            KDL::ChainJntToJacSolver jacsolver = KDL::ChainJntToJacSolver(*it);

            /** Create joint array **/
            unsigned int nj = (*it).getNrOfJoints();
            KDL::JntArray jointpositions = KDL::JntArray(nj);
            #ifdef DEBUG_PRINTS
            std::cout<<"[JACOBIAN_SOLVER_TREE] Number of Joints is: "<<nj<<"\n";
            std::cout<<"[JACOBIAN_SOLVER_TREE] Joints are: \n";
            #endif

            /** Fill the Joint Array with the position values **/
            this->unpackJoints(names, positions, this->ichains_joint_names[chainidx], jointpositions);

            /** Create KDL Jacobian **/
            KDL::Jacobian kdljacob;
            kdljacob.resize(nj);

            /** Solver for the Jacobian **/
            jacsolver.JntToJac(jointpositions, kdljacob);

            /** Get the Jacobian in Eigen Matrix class **/
            Eigen::Matrix<double, 6, Eigen::Dynamic> eigenjacob;
            eigenjacob.resize(6, nj);
            eigenjacob = kdljacob.data;
            #ifdef DEBUG_PRINTS
            std::cout<<"[JACOBIAN_SOLVER_TREE] Jacobian ("<<this->contact_point_segments[chainidx]<<") is "<<kdljacob.rows()<<" x " <<kdljacob.columns() <<std::endl;
            std::cout<<"[JACOBIAN_SOLVER_TREE] Jacobian:\n"<<eigenjacob<<std::endl;
            #endif

            /** Jacobian of inverse chain with respect to the root frame (Multiple by FK Transformations) **/
            for (register int p=0; p<static_cast<int>(eigenjacob.cols()); ++p)
            {
                Eigen::Vector3d column_vector;
                column_vector = eigenjacob.block(0, p, 3, 1);
                column_vector = fkTrans[chainidx].rotation() * column_vector;
                eigenjacob.block(0, p, 3, 1) = -column_vector;
                column_vector = eigenjacob.block(3, p, 3, 1);
                column_vector = fkTrans[chainidx].rotation() * column_vector;
                eigenjacob.block(3, p, 3, 1) = -column_vector;
            }

            this->organizeJacobian(chainidx, names, this->ichains_joint_names[chainidx], eigenjacob, J);

            chainidx++;
        }
    }
    return J;
}

void KinematicKDL::unpackJoints(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions,
                                const std::vector<std::string>& involved_joints, KDL::JntArray& joint_array)
{
    assert(joint_names.size() == joint_positions.size());
    assert(involved_joints.size() == joint_array.rows());

    for (register int i=0; i<static_cast<int>(joint_names.size()); ++i)
    {
        for (register int j=0; j<static_cast<int>(involved_joints.size()); ++j)
        {
            if (joint_names[i].compare(involved_joints[j]) == 0)
            {
                //std::cout<<"Found["<<j<<"] in ["<<i<<"]: "<<involved_joints[j]<<"\n";
                joint_array(j) = joint_positions[i];
                break;
            }
        }
    }

    return;
}

void KinematicKDL::organizeJacobian(const int chainidx, const std::vector<std::string> &joint_names, const std::vector<std::string> &involved_joints,
                                    const Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> &jacobian, Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> &J)
{
    assert(joint_names.size() == static_cast<unsigned int> (J.cols()));
    assert(involved_joints.size() == static_cast<unsigned int>(jacobian.cols()));

    for (register int i=0; i<static_cast<int>(involved_joints.size()); ++i)
    {
        for (register int j=0; j<static_cast<int>(joint_names.size()); ++j)
        {
            if (involved_joints[i].compare(joint_names[j]) == 0)
            {
                //std::cout<<"jacobian.col("<<i<<") -> J.col("<<j<<")\n";
                J.block(6*chainidx, j, jacobian.rows(), 1) = jacobian.col(i);
                break;
            }
        }
    }

}

