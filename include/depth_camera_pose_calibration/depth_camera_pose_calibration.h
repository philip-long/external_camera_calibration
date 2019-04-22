///  @file depth_camera_pose_calibration.h
/// \class PoseCalibration
/// \brief Defines the PoseCalibration for Camera
///
///
///
/// \author Philip Long <philip.long01@gmail.com>, Irish Manufacturing Research & Northeastern Univeristy
/// \date April, 2019


#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_kdl.h>


#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <pcl_conversions/pcl_conversions.h>

#ifndef POSE_CALIB_HPP
#define POSE_CALIB_HPP


typedef std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> TransformVector;
typedef std::vector<Eigen::Matrix<double,6,Eigen::Dynamic>> JacobianVector;


/// GeometryInformation contains all geomertic information obtained from chain
struct GeometryInformation {
    std::vector<std::unique_ptr<shapes::Shape>> shapes;
    TransformVector geometry_transforms;
    JacobianVector  geometry_jacobians;


    void clear() {
        shapes.clear();
        geometry_transforms.clear();
        geometry_jacobians.clear();
    }

};


class PoseCalibration
{
private:
   ros::NodeHandle nh_;
   KDL::Tree my_tree_;
   KDL::Chain chain_;
   urdf::Model model_;
   boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
   boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver_;
    /// number of degrees of freedom
    unsigned int ndof_;
   std::string base_link_;

   
  void jointStatetoKDLJointArray ( const sensor_msgs::JointState & joint_states,
        KDL::JntArray & kdl_joint_positions );
  
      /** This function is taken from https://github.com/tu-darmstadt-ros-pkg with a tiny change to smart ptrs
    * https://github.com/tu-darmstadt-ros-pkg/robot_self_filter/blob/master/src/self_mask.cpp#L76
    **/
    static std::unique_ptr<shapes::Shape> constructShape ( const urdf::Geometry *geom );
    
        /// Convenience function to convert kdl to eigen stuff, segment=-1 returns terminal point information
    void   getKDLKinematicInformation ( const KDL::JntArray & kdl_joint_positions,
        Eigen::Affine3d & T,
        Eigen::Matrix<double,6,Eigen::Dynamic> & Jac,int segment=-1);
  
public:
  PoseCalibration ( ros::NodeHandle nh,
    std::string root,
    std::string tip,
    std::string robot_description="robot_description"
  );
  
  bool getRobotMeshModel ( sensor_msgs::JointState const & joint_state ) ;
  bool getVisualModel ( const  KDL::JntArray & kdl_joint_positions,
        GeometryInformation & geometry_information);
  
  
  // Calls fcl destructor which should destroy all objects in world
  ~PoseCalibration(){};
  
  
  
    /// Screw transform to move a twist from point a to point b, given vector L, a->b w.r.t. base frame
    static bool screwTransform ( const Eigen::Matrix<double,6,Eigen::Dynamic> &J0N_in,
                                 Eigen::Matrix<double,6,Eigen::Dynamic>& J0E_out,
                                 const Eigen::Vector3d & L );

    /// Skew symmetric matrix performing the cross product
    static bool skew ( const Eigen::Vector3d & L, Eigen::Matrix<double,3,3>& skewL ) {
        skewL ( 0,1 ) =-L ( 2 );
        skewL ( 0,2 ) =L ( 1 );
        skewL ( 1,2 ) =-L ( 0 );
        skewL ( 1,0 ) =-skewL ( 0,1 );
        skewL ( 2,0 ) =-skewL ( 0,2 );
        skewL ( 2,1 ) =-skewL ( 1,2 );
        return true;
    }
    
};
#endif
