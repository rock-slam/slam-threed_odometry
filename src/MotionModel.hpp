/**
 * \file MotionModel.hpp
 *
 * This class provided a Motion Model solver for any mobile robot (wheel or leg)
 * with articulated joints.
 *
 * The solver is based on Weighted Least-Squares as minimizing the error to estimate the
 * resulting motion from a robot Jacobian. Robot Jacobian is understood as the sparse Jacobian
 * matrix containing one Jacobian per each Chain of the Robot.
 * Therefore, it is a minimization problem of a non-linear system which has been previously linearized
 * around the working point (robot current joint position values). This is represented in the
 * current Jacobian matrix of the robot.
 * This class requires a robot Jacobian
 * and computes the statistical motion model (estimated velocities/navigation quantities).
 *
 * Further Details at:  P.Muir et. al Kinematic Modeling of Wheeled Mobile Robots
 *                      M. Tarokh et. al Kinematics Modeling and Analyses of Articulated Rovers
 *                      J. Hidalgo et. al Kinematics Modeling of a Hybrid Wheeled-Leg Planetary Rover
 *                      J. Hidalgo, Navigation and Slip Kinematics for High Performance Motion Models
 *                      Hidalgo-Carrio, Javier and Babu, Ajish and Kirchner,
 *                      Frank, Static forces weighted Jacobian motion models
 *                      for improved Odometry
 *
 * @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
 * @date December 2014.
 * @version 1.0.
 */


#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP

#include <iostream> /** for std::cout TO-DO REMOVE **/
#include <vector> /** For std vector **/
#include <boost/shared_ptr.hpp> /** For std shared pointer **/
#include <base-logging/Logging.hpp> /** Log message **/
#include <Eigen/Geometry> /** Eigen data type for Matrix, Quaternion, etc... */
#include <Eigen/Core> /** Core methods of Eigen implementation **/
#include <Eigen/Dense> /** for the algebra and transformation matrices and accessing Matrixblock and corner among others**/
#include <Eigen/Cholesky> /** For the Cholesky decomposition **/

//#define DEBUG_PRINTS_ODOMETRY_MOTION_MODEL 1 //TO-DO: Remove this. Only for testing (master branch) purpose

namespace threed_odometry
{

    /**@class MotionModel
     *
     * Motion Model solver class
     *
     * @param _Scalar: is the typename of your desired implementation (float, double, etc..)
     *
     * Note that it is very important how you specify the number of Chains according to the contact points.
     * It is assumed that one Chain can only have one single point of contact with the ground at the same time.
     * Take <a href="http://robotik.dfki-bremen.de/en/forschung/robotersysteme/asguard-ii.html">Asguard</a> wheel
     * as an example. Each wheel is in reality five chains. However, Five points are not always in contact
     * therefor on order to determine which (and how many) points are finally making the robot motion ones
     * should use the weight matrix (Weight Least Square).
     *
     */
    template <typename _Scalar>
    class MotionModel
    {
        protected:
            int number_chains, number_robot_joints, number_slip_joints, number_contact_joints;

        public:
            unsigned int model_dof;

        protected:

            /**@brief Forms the Navigation Equations for the navigation kinematics
             *
             * It assumes that known quantities are angular rotations
             * (Cartesian), joint velocities and contact angles (depending on
             * the known_contact_angles flag). There is non-slip in the X and Y
             * axis.  Only slip in z-axis (non-holonomic constrain) The unknown
             * quantities are position of the robot in Cartesian, the contact
             * angle and the z value of the slip vector.  The system of
             * equations is unknownA * unknownx = knownB * knowny The
             * parameters are stored in the general dimension for the problem,
             * therefore unknown quantities are set to NaN in the parameters of
             * the method.
             *
             * @param[in] cartesianVelocities Eigen vector with the cartesian
             * velocities (w.r.t local body frame).
             *
             * @param[in] modelVelocities Eigen vector with the velocities of
             * the model(joint, slip and contact angle)
             *
             * @param[in] J robot jacobian matrix.
             *
             * @param[in] cartesianVelCov uncertainty in the measurement of the
             * cartesianVelocities vector.
             *
             * @param[in] modelVelCov uncertainty in the measurement of the
             * modelVelocities vector.
             *
             * @param[out] unknownA is the matrix of unknow quantities.
             * @param[out] unknownx in the vector of unknow quantities.
             * @param[out] knownB is the matrix of know quantities
             * @param[out] knowny is the matrix of know quantities
             * equation (Weighted Least-Squares).
             */
            inline void navEquations(const Eigen::Matrix <_Scalar, 6, 1> &cartesianVelocities,
                                    const Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> &modelVelocities,
                                    const Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> &J,
                                    Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> &unknownA,
                                    Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> &unknownx,
                                    Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> &knownB,
                                    Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> &knowny,
                                    bool known_contact_angles = false)
            {
                Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> spareI;
                spareI.resize(6*this->number_chains, 6);

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] navEquations\n";
                #endif

                /** Compute the composite rover equation matrix I **/
                for (register int i=0; i<this->number_chains; ++i)
                    spareI.block(i*6, 0, 6, 6) = Eigen::Matrix <_Scalar, 6, 6>::Identity();

                /** Form the unknownA matrix **/
                unknownA.block(0, 0, 6*this->number_chains, 3) = spareI.block(0, 0, 6*this->number_chains, 3);//Linear Velocities

                /** Form the unknownx vector **/
                unknownx.block(0, 0, 3, 1) = cartesianVelocities.block(0, 0, 3, 1);

                /** Non-holonomic constraint Slip vector **/
                unknownA.block(0, 3, 6*this->number_chains, this->number_chains) = -J.block(0, this->number_robot_joints+this->number_slip_joints-(this->number_chains), 6*this->number_chains, this->number_chains);
                unknownx.block(3, 0, this->number_chains, 1) = modelVelocities.block(this->number_robot_joints+this->number_slip_joints-(this->number_chains), 0, this->number_chains, 1);

                /** Contact angles per each Tree in the unknown par of the system **/
                if (known_contact_angles == false)
                {
                    unknownA.block(0, 3+this->number_chains, 6*this->number_chains, this->number_contact_joints) = -J.block(0, this->number_robot_joints+this->number_slip_joints, 6*this->number_chains, this->number_contact_joints);
                    unknownx.block(3+this->number_chains, 0, this->number_contact_joints, 1) = modelVelocities.block(this->number_robot_joints+this->number_slip_joints, 0, this->number_contact_joints, 1);
                }

                /** Form the knownB matrix **/
                knownB.block(0, 0, 6*this->number_chains, 3) = -spareI.block(0, 3, 6*this->number_chains, 3); //Angular velocities
                knownB.block(0, 3, 6*this->number_chains, this->number_robot_joints) = J.block(0, 0, 6*this->number_chains, this->number_robot_joints);//Robot joints

                /** Form the knowny vector **/
                knowny.block(0, 0, 3, 1) = cartesianVelocities.template block<3, 1> (3,0); //Angular velocities
                knowny.block(3, 0, this->number_robot_joints, 1) = modelVelocities.block(0, 0, this->number_robot_joints, 1); //Robot joints

                /** Contact angles per each Tree in the known par of the system **/
                if (known_contact_angles == true)
                {
                    knownB.block(0, 3+this->number_robot_joints, 6*this->number_chains, this->number_contact_joints) = J.block(0, this->number_robot_joints+this->number_slip_joints, 6*this->number_chains, this->number_contact_joints);//Robot joints
                    knowny.block(3+this->number_robot_joints, 0, this->number_contact_joints, 1) = modelVelocities.block(this->number_robot_joints+this->number_slip_joints, 0, this->number_contact_joints, 1);
                }

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<< "[MOTION_MODEL] spareI is of size "<<spareI.rows()<<"x"<<spareI.cols()<<"\n";
                std::cout<< "[MOTION_MODEL] The spareI matrix:\n" << spareI << std::endl;
                std::cout<< "[MOTION_MODEL] unknownA is of size "<<unknownA.rows()<<"x"<<unknownA.cols()<<"\n";
                std::cout<< "[MOTION_MODEL] The unknownA matrix:\n" << unknownA << std::endl;
                std::cout<< "[MOTION_MODEL] knownB is of size "<<knownB.rows()<<"x"<<knownB.cols()<<"\n";
                std::cout<< "[MOTION_MODEL] The knownB matrix:\n" << knownB << std::endl;
                #endif

                return;
            }

        public:

            /**@brief Constructor
             *
             * Constructs the Motion Model object with the parameters.
             *
             * @param number_chains: is the number of independent Chains connected to your desired Body Center.
             *              Trees is understood as connection of kinematics chains. As an example:
             *              <a href="http://robotik.dfki-bremen.de/en/forschung/robotersysteme/asguard-ii.html">Asguard</a>
             *              hybrid wheels model as Chains. Therefore Asguard has  five chain per Wheel,
             *              one per each foot which are potential points in contact with
             *              the ground.
             * @param number_robot_joints: Complete number of robot joints (DoF). This is every joint
             *              passive or active that your robot (or the part of your robot related to Odometry, which is the chassis)
             *              physically has. This is required for the Jacobian in a general form and to slip the Jacobian in sub-matrices.
             * @param number_slip_joints: The number of slip joints (DoF) than your robot has. For example if your robot has
             *           six contact points and 3DoF slip model per each contact point, therefore your robot has 18 slip joints.
             *           A common slip model has 3DoF (in X an Y direction and Z rotation)
             * @param number_contact_joints: This is the number of contact angles of your robot. For example, in case your robot has
             *              six contact points and 1DoF contact ange per each contact point. Your robot has 6 contact angle joints.
             *              In case it is not interesting for you model (i.e. the robot moves in indoor environment)
             *              set it to zero. Otherwise, for navigation on uneven terrains, it is normally modeled as 1DoF along the
             *              axis of the pitch angle of your robot.
             *
             */
            MotionModel (const int _number_chains, const int _number_robot_joints, const int _number_slip_joints, const int _number_contact_joints):
                            number_chains(_number_chains),
                            number_robot_joints(_number_robot_joints),
                            number_slip_joints(_number_slip_joints),
                            number_contact_joints(_number_contact_joints)
            {
                this->model_dof = number_robot_joints + number_slip_joints + number_contact_joints;

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] Constructor. Model DoF:"<<this->model_dof<<"\n";
                #endif

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"\n[MOTION_MODEL] **** END ****\n";
                #endif

                return;
            }


            /**@brief Default destructor
             */
            ~MotionModel()
            {
            }

            /**@bief Solver for the Navigation equations
             *
             * This method get the robot Jacobian matrix and computes the Navigation
             * kinematics equations to finally solve the Weighting Least-Square method.
             *
             * The Navigation kinematics equation: A * x = B * y
             * This method (navigation solver) perform the solution to the system above, solving:
             *
             *               x = (A^T * W * A)^(-1) * A^T * W * B * y
             *
             * @param[in] modelPositions, position values of the model (joints, slip and contact angle)
             * @param[in, out] modelVelocities, velocity values of the model (joints, slip and contact angle)
             * @param[in] J, The complete robot Jacobian matrix
             * @param[in, out] cartesianVelocities, velocities of the robot Cartesian space (w.r.t local body frame).
             * @param[in, out] modelVelCov, uncertainty in the measurement of the robot model velocities (joint space).
             * @param[in, out] cartesianVelCov, uncertainty in the measurement of the robot Cartesian velocities (w.r.t local body frame).
             * @param[in] Weight, trees Weighting matrix. It defines which robot's tree has more weight to the computation (0 will not have
             * contribution to the movement, no-contact).
             * @param[in] known_contact_angles, flags to set to true when the contact angles is known, false otherwise. This affect to
             * vectors and matrices dimensions.
             */
            virtual double navSolver(const Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> &modelPositions,
                                    Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> &modelVelocities,
                                    const Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> &J,
                                    Eigen::Matrix <_Scalar, 6, 1> &cartesianVelocities,
                                    Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> &modelVelCov,
                                    Eigen::Matrix <_Scalar, 6, 6> &cartesianVelCov,
                                    const Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> Weight,
                                    bool known_contact_angles = false)
            {
                double normalizedError = std::numeric_limits<double>::quiet_NaN(); //solution error of the Least-Squares

                Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> unknownA; // Nav non-sensed values matrix
                Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> unknownx; // Nav non-sensed values vector
                Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> knownB; // Nav sensed values matrix
                Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> knowny; // Nav sensed values vector

                if (known_contact_angles == false)
                {
                    /** Resize the vector and matrices to the right dimensions **/
                    unknownA.resize(6*this->number_chains, 3+this->number_chains+this->number_contact_joints);// position, z-slip and contact angles
                    unknownx.resize(3+this->number_chains+this->number_contact_joints, 1);// position, z-slip and contact angles
                    knownB.resize(6*this->number_chains, 3+this->number_robot_joints);// rotation and robot joints
                    knowny.resize(3+this->number_robot_joints, 1);// rotation and robot joints
                }
                else
                {
                    /** Resize the vector and matrices to the right dimensions **/
                    unknownA.resize(6*this->number_chains, 3+this->number_chains); // position and z-slip
                    unknownx.resize(3+this->number_chains); // position and z-slip
                    knownB.resize(6*this->number_chains, 3+this->number_robot_joints+this->number_contact_joints);// rotation, robot joints and contact angles
                    knowny.resize(3+this->number_robot_joints+this->number_contact_joints, 1);// rotation, robot joints and contact angles
                }

                /** Initialize  variables **/
                unknownA.setZero(); unknownx.setZero();
                knownB.setZero(); knowny.setZero();

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout << "[MOTION_MODEL] cartesianVelocities is of size "<<cartesianVelocities.rows()<<"x"<<cartesianVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] cartesianVelocities is \n" << cartesianVelocities<< std::endl;

                std::cout << "[MOTION_MODEL] modelVelocities is of size "<<modelVelocities.rows()<<"x"<<modelVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] modelVelocities is \n" << modelVelocities<< std::endl;
                #endif

                /** Assert vectors sizes **/
                assert(modelPositions.size() == modelVelocities.size());
                assert(modelPositions.size() == this->model_dof);
                assert(modelVelCov.cols() == modelVelCov.rows());
                assert(modelVelCov.cols() == modelVelocities.size());
                assert(Weight.cols() == 6*this->number_chains);
                assert(Weight.rows() == 6*this->number_chains);

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<< "[MOTION_MODEL] Weight is of size "<<Weight.rows()<<"x"<<Weight.cols()<<"\n";
                std::cout<< "[MOTION_MODEL] The Weight matrix:\n" << Weight << std::endl;
                std::cout<< "[MOTION_MODEL] J is of size "<<J.rows()<<"x"<<J.cols()<<"\n";
                std::cout<< "[MOTION_MODEL] The J matrix \n" << J << std::endl;
                #endif

                /** Form the Composite Navigation Equations and Noise Covariance **/
                this->navEquations (cartesianVelocities, modelVelocities, J, unknownA, unknownx, knownB, knowny, known_contact_angles);

                /** Solve the Motion Model by Least-Squares (navigation kinematics) **/
                Eigen::Matrix <_Scalar, Eigen::Dynamic, 1> knownb;
                knownb.resize(6*this->number_chains, 1);
                knownb = knownB*knowny;

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                if (known_contact_angles == false)
                {
                    /** DEBUG OUTPUT **/
                    Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> Conj;
                    Conj.resize(6*this->number_chains, 3+this->number_chains+this->number_contact_joints+1);

                    Eigen::FullPivLU<Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> > lu_decompA(unknownA);
                    std::cout << "[MOTION_MODEL] The rank of A is " << lu_decompA.rank() << std::endl;

                    Eigen::FullPivLU<Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> > lu_decompB(knownB);
                    std::cout << "[MOTION_MODEL] The rank of B is " << lu_decompB.rank() << std::endl;

                    Conj.block(0, 0, 6*this->number_chains, 3+this->number_chains+this->number_contact_joints) = unknownA;
                    Conj.block(0, 3+this->number_chains+this->number_contact_joints, 6*this->number_chains, 1) = knownb;
                    Eigen::FullPivLU< Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> > lu_decompConj(Conj);
                    std::cout << "[MOTION_MODEL] The rank of A|B*y is " << lu_decompConj.rank() << std::endl;
                    std::cout << "[MOTION_MODEL] Pseudoinverse of A\n" << (unknownA.transpose() * Weight * unknownA).inverse() << std::endl;
                }
                else
                {
                    /** DEBUG OUTPUT **/
                    Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> Conj;
                    Conj.resize(6*this->number_chains, 3+this->number_chains+1);

                    Eigen::FullPivLU<Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> > lu_decompA(unknownA);
                    std::cout << "[MOTION_MODEL] The rank of A is " << lu_decompA.rank() << std::endl;

                    Eigen::FullPivLU<Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> > lu_decompB(knownB);
                    std::cout << "[MOTION_MODEL] The rank of B is " << lu_decompB.rank() << std::endl;

                    Conj.block(0, 0, 6*this->number_chains, 3+this->number_chains) = unknownA;
                    Conj.block(0, 3+this->number_chains, 6*this->number_chains, 1) = knownb;
                    Eigen::FullPivLU< Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> > lu_decompConj(Conj);
                    std::cout << "[MOTION_MODEL] The rank of A|B*y is " << lu_decompConj.rank() << std::endl;
                    std::cout << "[MOTION_MODEL] Pseudoinverse of A\n" << (unknownA.transpose() * Weight * unknownA).inverse() << std::endl;


                }
                /*******************/
                #endif

                /** Solve the system **/
                unknownx = (unknownA.transpose() * Weight * unknownA).ldlt().solve(unknownA.transpose() * Weight * knownb);

                /** Error of the solution **/
                Eigen::Matrix<double, 1,1> squaredError = (((unknownA*unknownx - knownb).transpose() * Weight * (unknownA*unknownx - knownb)));
                if (knownb.norm() != 0.00)
                    normalizedError = sqrt(squaredError[0]) / knownb.norm();

                /** Error covariance matrix **/
                Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> errorCov;
                errorCov.resize(6*this->number_chains, 6*this->number_chains);
                errorCov = (unknownA*unknownx - knownb).asDiagonal(); errorCov *= errorCov;// L-S error covariance
                Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> uncertaintyCov; // noise cov

                /** Error in the solution. The size depends on the known_contact_angles **/
                if (known_contact_angles == false)
                {
                    uncertaintyCov.resize(3+this->number_chains+this->number_contact_joints, 3+this->number_chains+this->number_contact_joints);
                }
                else
                {
                    uncertaintyCov.resize(3+this->number_chains, 3+this->number_chains);
                }

                /** Least square error **/
                Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> errorCovinverse;
                errorCovinverse = errorCov.inverse();
                if (base::isnotnan< Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> >(errorCovinverse))
                {
                    uncertaintyCov = (unknownA.transpose() * errorCovinverse * unknownA).inverse(); // Observer
                }
                else
                {
                    uncertaintyCov = unknownA.transpose() * errorCov * unknownA; // Observer
                }

                uncertaintyCov = 0.5*(uncertaintyCov + uncertaintyCov.transpose());// Guarantee symmetry

                /** Save the results in the parameters (previous NaN values are now just known quantities) **/
                cartesianVelocities.block(0, 0, 3, 1) = unknownx.block(0, 0, 3, 1); // Linear velocities
                cartesianVelCov.block(0, 0, 3, 3) = uncertaintyCov.block(0, 0, 3,3);//Linear Velocities noise

                /** Angular velocity estimated noise (EXPERIMENTAL). Get the uncertainty of the angular velocity from the LS covariance **/
                if (cartesianVelCov.block(3, 3, 3, 3) != cartesianVelCov.block(3, 3, 3, 3))
                {
                    cartesianVelCov.block(3, 3, 3, 3).setZero();

                    /** There is one per each _RobotTrees. errorCov is a 6*_RobotTrees x 6*_RobotTrees matrix dimension **/
                    for (register size_t i=0; i<this->number_chains; ++i)
                    {
                        cartesianVelCov.block(3, 3, 3, 3) += Weight.block(3+(6*i), 3+(6*i), 3, 3) * errorCov.block(3+(6*i),3+(6*i), 3, 3);//Angular Velocities noise
                    }
                }

                /** Save the non-holonomic constraint, z-slip velocity vector **/
                modelVelocities.block(this->number_robot_joints+this->number_slip_joints-(this->number_chains), 0, this->number_chains, 1) = unknownx.block(3, 0, this->number_chains, 1);
                modelVelCov.block(this->number_robot_joints+this->number_slip_joints-(this->number_chains), this->number_robot_joints+this->number_slip_joints-(this->number_chains), this->number_chains, this->number_chains) = uncertaintyCov.block(3, 3, this->number_chains, this->number_chains);//pseudoInvUnknownA.col(3+i)[3+i]; For the time being set the error to the error in the estimation

                if (known_contact_angles == false)
                {
                    /** Save the estimated contact angle velocities per each Tree **/
                    for (register int i=0; i<this->number_contact_joints; ++i)
                    {
                        modelVelocities[this->number_robot_joints+this->number_slip_joints+i] = unknownx[3+this->number_chains+i];
                        modelVelCov.col(this->number_robot_joints+this->number_slip_joints+i)[this->number_robot_joints+this->number_slip_joints+i] = uncertaintyCov.col(3+this->number_chains+i)[3+this->number_chains+i];//pseudoInvUnknownA.col(3+_RobotTrees+i)[3+_RobotTrees+i];
                    }
                }

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout << "[MOTION_MODEL] L-S solution:\n"<<unknownx<<std::endl;

                std::cout << "[MOTION_MODEL] RESULT errorCov is of size "<<errorCov.rows()<<"x"<<errorCov.cols()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT errorCov is \n" << errorCov << std::endl;

                std::cout << "[MOTION_MODEL] RESULT uncertaintyCov is of size "<<uncertaintyCov.rows()<<"x"<<uncertaintyCov.cols()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT uncertaintyCov is \n" << uncertaintyCov << std::endl;

                std::cout << "[MOTION_MODEL] RESULT cartesianVelocities is of size "<<cartesianVelocities.rows()<<"x"<<cartesianVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT cartesianVelocities is \n" << cartesianVelocities<< std::endl;

                std::cout << "[MOTION_MODEL] RESULT cartesianVelCov is of size "<<cartesianVelCov.rows()<<"x"<<cartesianVelCov.cols()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT cartesianVelCov is \n" << cartesianVelCov<< std::endl;

                std::cout << "[MOTION_MODEL] RESULT modelVelocities is of size "<<modelVelocities.rows()<<"x"<<modelVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT modelVelocities is \n" << modelVelocities<< std::endl;

                std::cout << "[MOTION_MODEL] RESULT modelVelCov is of size "<<modelVelCov.rows()<<"x"<<modelVelCov.cols()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT modelVelCov is \n" << modelVelCov<< std::endl;

                std::cout << "[MOTION_MODEL] RESULT The absolute least squared error is:\n" << squaredError << std::endl;
                std::cout << "[MOTION_MODEL] RESULT The relative error is:\n" << normalizedError << std::endl;
                std::cout << "[MOTION_MODEL] RESULT The error vector is \n"<<(unknownA*unknownx - knownb)<<"\n";
                std::cout << "[MOTION_MODEL] RESULT The error variance is \n"<<errorCov<<"\n";
                std::cout << "[MOTION_MODEL] RESULT The error variance.inverse() is \n"<<errorCov.inverse()<<"\n";
                std::cout << "[MOTION_MODEL] RESULT The solution covariance is \n"<<cartesianVelCov<<"\n";
                #endif


                return normalizedError;
            }

    };
}

#endif //ODOMETRY_MOTION_MODEL_HPP

