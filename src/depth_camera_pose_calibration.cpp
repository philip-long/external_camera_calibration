/*** INCLUDE FILES ***/
#include <depth_camera_pose_calibration/depth_camera_pose_calibration.h>
// Load in KDL tree
// Convert to Mesh
// Display MESH

PoseCalibration::PoseCalibration ( ros::NodeHandle nh,
                                   std::string root,
                                   std::string tip,
                                   std::string robot_description ) :nh_ ( nh )

{
    std::string robot_desc_string;
    nh_.param ( robot_description, robot_desc_string, std::string() );
    model_.initParamWithNodeHandle ( robot_description,nh );

    if ( !kdl_parser::treeFromString ( robot_desc_string, my_tree_ ) ) {
        ROS_ERROR ( "Failed to construct kdl tree" );
    }


    base_link_=root;
    my_tree_.getChain ( root,
                        tip,
                        chain_ );
    ndof_=chain_.getNrOfJoints();


    kdl_fk_solver_.reset ( new KDL::ChainFkSolverPos_recursive ( chain_ ) );
    kdl_dfk_solver_.reset ( new KDL::ChainJntToJacSolver ( chain_ ) );
}



std::unique_ptr<shapes::Shape> PoseCalibration::constructShape ( const urdf::Geometry *geom ) {
    ROS_ASSERT ( geom );

    std::unique_ptr<shapes::Shape> result = NULL;
    switch ( geom->type ) {
    case urdf::Geometry::SPHERE: {
        result =  std::unique_ptr<shapes::Shape> ( new shapes::Sphere ( dynamic_cast<const urdf::Sphere*> ( geom )->radius ) );
        break;
    }
    case urdf::Geometry::BOX: {
        urdf::Vector3 dim = dynamic_cast<const urdf::Box*> ( geom )->dim;
        result =  std::unique_ptr<shapes::Shape> ( new shapes::Box ( dim.x, dim.y, dim.z ) );
        break;
    }
    case urdf::Geometry::CYLINDER: {
        result =  std::unique_ptr<shapes::Shape> ( new shapes::Cylinder ( dynamic_cast<const urdf::Cylinder*> ( geom )->radius,
                  dynamic_cast<const urdf::Cylinder*> ( geom )->length ) );
        break;
    }
    case urdf::Geometry::MESH: {
        const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*> ( geom );
        if ( !mesh->filename.empty() ) {
            Eigen::Vector3d scale ( mesh->scale.x, mesh->scale.y, mesh->scale.z );
            result =  std::unique_ptr<shapes::Shape> ( shapes::createMeshFromResource ( mesh->filename, scale ) );
        } else {
            ROS_WARN ( "Empty mesh filename" );
        }
        break;
    }
    default: {
        ROS_ERROR ( "Unknown geometry type: %d", ( int ) geom->type );
        break;
    }
    }
    return ( result );
}



void PoseCalibration::jointStatetoKDLJointArray ( const sensor_msgs::JointState & joint_states,
        KDL::JntArray & kdl_joint_positions ) {
    unsigned int jnt ( 0 );
    for ( int i=0; i<chain_.getNrOfSegments(); ++i ) {

        KDL::Segment seg=chain_.getSegment ( i );
        KDL::Joint kdl_joint=seg.getJoint();
        for ( int j=0; j<joint_states.name.size(); ++j ) {
            if ( kdl_joint.getName() ==joint_states.name[j] ) {
                kdl_joint_positions ( jnt ) =joint_states.position[j];
                jnt++;
            }
        }
    }
}


void   PoseCalibration::getKDLKinematicInformation ( const KDL::JntArray & kdl_joint_positions,
        Eigen::Affine3d & T,
        Eigen::Matrix<double,6,Eigen::Dynamic> & Jac,int segment ) {

    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize ( ndof_ );
    if ( segment!=-1 ) {
        kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos,segment );
        kdl_dfk_solver_->JntToJac ( kdl_joint_positions,base_J_link_origin,segment );
    } else {
        kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos );
        kdl_dfk_solver_->JntToJac ( kdl_joint_positions,base_J_link_origin );
    }
    tf::transformKDLToEigen ( cartpos,T );

    Jac=base_J_link_origin.data;
}



bool PoseCalibration::getRobotMeshModel ( sensor_msgs::JointState const & joint_state ) {

    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_state,kdl_joint_positions );
    GeometryInformation geometry_information;
    //     // Collision Link transforms
    getVisualModel ( kdl_joint_positions,geometry_information );
    pcl::PolygonMeshPtr testMesh(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud ( new pcl::PointCloud<pcl::PointXYZ> );


    for ( int i = 0; i<geometry_information.geometry_transforms.size(); i++ ) {
        std::cout<<"i = "<<i<<std::endl;
        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape ( geometry_information.shapes[i].get(),current_shape );

        if ( current_shape.which() ==1 ) {
            std::cout<<"Its a mesh"<<std::endl;
            shape_msgs::Mesh body_mesh=boost::get<shape_msgs::Mesh> ( current_shape );

	    
            for ( int k = 0; k < body_mesh.vertices.size() ; k++ ) {
                pcl::PointXYZ p ( body_mesh.vertices[k].x,
                                  body_mesh.vertices[k].y,
                                  body_mesh.vertices[k].z );
                my_cloud->points.push_back ( p );
            }
	    std::cout<<"Created a Cloud"<<std::endl;
            pcl::toPCLPointCloud2 (*my_cloud, testMesh->cloud );
	    std::cout<<"Converted a Cloud"<<std::endl;
            
            testMesh->polygons.resize ( body_mesh.triangles.size());
	    
            for ( int i = 0; i < body_mesh.triangles.size() ; i++ ) {
            testMesh->polygons[i].vertices.resize ( 3 );
                for ( int j = 0; j < 3; ++j ) {
                    testMesh->polygons[i].vertices[j]=body_mesh.triangles[i].vertex_indices[j];
                }
            }           
	
	// Now I need to sample the mesh , convert it to a message then publish it.
	
	

        } else {
            ROS_ERROR ( "Visual Geometry not support" );
        }


    }

}

bool PoseCalibration::getVisualModel ( const  KDL::JntArray & kdl_joint_positions,
GeometryInformation & geometry_information ) {

    geometry_information.clear();
    Eigen::Affine3d link_origin_T_visual_origin,base_T_link_origin,base_T_visual_origin;

    // Calculates the segement's visual geomtery
    //  The transform to the origin of the visual geometry
    //  The Jacobian matrix at the origin of the visual geometry
    for ( int i=0; i<chain_.getNrOfSegments(); ++i ) {
        KDL::Segment seg=chain_.getSegment ( i ); // Get current segment
        // Ensure visual geometry exists
        if ( model_.links_.at ( seg.getName() )->visual!= nullptr ) {

            std::unique_ptr<shapes::Shape> shape = constructShape ( model_.links_.at ( seg.getName() )->visual->geometry.get() );
            // Get Collision Origin
            Eigen::Vector3d origin_Trans_visual ( model_.links_.at ( seg.getName() )->visual->origin.position.x,
                                                  model_.links_.at ( seg.getName() )->visual->origin.position.y,
                                                  model_.links_.at ( seg.getName() )->visual->origin.position.z );
            Eigen::Quaterniond origin_Quat_visual (
                model_.links_.at ( seg.getName() )->visual->origin.rotation.w,
                model_.links_.at ( seg.getName() )->visual->origin.rotation.x,
                model_.links_.at ( seg.getName() )->visual->origin.rotation.y,
                model_.links_.at ( seg.getName() )->visual->origin.rotation.z
            );

            link_origin_T_visual_origin.translation() =origin_Trans_visual;
            link_origin_T_visual_origin.linear() =origin_Quat_visual.toRotationMatrix();

            // Finds cartesian pose w.r.t to base frame

            Eigen::Matrix<double,6,Eigen::Dynamic> base_J_visual_origin,base_J_link_origin;
            getKDLKinematicInformation ( kdl_joint_positions,base_T_link_origin,base_J_link_origin,i+1 );
            base_T_visual_origin=base_T_link_origin*link_origin_T_visual_origin;
            Eigen::Vector3d base_L_link_visual= ( base_T_link_origin.linear() * link_origin_T_visual_origin.translation() );
            // Screw transform to collision origin
            screwTransform ( base_J_link_origin,base_J_visual_origin,base_L_link_visual );

            // Push back solutions
            geometry_information.shapes.push_back ( std::move ( shape ) );
            geometry_information.geometry_transforms.push_back ( base_T_visual_origin );
            geometry_information.geometry_jacobians.push_back ( base_J_visual_origin );
        } else {
            std::cout<<seg.getName() <<" has no visual geometry"<<std::endl;
        }
    }
    return true;




}


bool PoseCalibration::screwTransform ( const Eigen::Matrix<double,6,Eigen::Dynamic> &J0N_in,
                                       Eigen::Matrix<double,6,Eigen::Dynamic>& J0E_out,
const Eigen::Vector3d & L ) {
    J0E_out.setZero();
    Eigen::Matrix<double,3,3> Lhat;
    Lhat.setZero();
    skew ( L,Lhat );
    Eigen::Matrix<double,6,6> screwL;
    screwL.setZero();
    screwL.topLeftCorner ( 3,3 ) =Eigen::Matrix<double,3,3>::Identity();
    screwL.bottomRightCorner ( 3,3 ) =Eigen::Matrix<double,3,3>::Identity();
    screwL.bottomLeftCorner ( 3,3 ) =Eigen::Matrix<double,3,3>::Zero();
    screwL.topRightCorner ( 3,3 ) =-Lhat;
    J0E_out=screwL*J0N_in;
    return true;
}



sensor_msgs::JointState joint_state;
bool joint_state_received ( false );

void jointSensorCallback ( const sensor_msgs::JointState::ConstPtr& msg ) {
    joint_state=*msg;
    joint_state_received=true;

}



int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "PoseCalibration" );
    ros::NodeHandle nh; // Create a node handle and start the node
    ROS_INFO ( "constructing" );
    PoseCalibration pose_calibration ( nh,"base_link","ee_link" );

    ros::Subscriber  joint_sub= nh.subscribe ( "/joint_states",
                                1, &jointSensorCallback );

    while ( ros::ok() ) {
        if ( joint_state_received==true ) {
            joint_state_received=false;
            pose_calibration.getRobotMeshModel ( joint_state );
            ros::Duration ( 0.1 ).sleep();
        }
        ros::spinOnce();
        ros::Duration ( 0.001 ).sleep();
    }
    return 0;
}
