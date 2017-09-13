/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Luca Muratore
 * email: luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <Playback_mat_with_DK.h>

#include <iostream>
#include <vector>

#include <matio.h>



/* Specify that the class XBotPlugin::Playback_mat_with_DK is a XBot RT plugin with name "Playback_mat_with_DK" */
REGISTER_XBOT_PLUGIN(Playback_mat_with_DK, XBotPlugin::Playback_mat_with_DK)

namespace XBotPlugin {

bool Playback_mat_with_DK::init_control_plugin( std::string path_to_config_file,
                                                XBot::SharedMemory::Ptr shared_memory,
                                                XBot::RobotInterface::Ptr robot)
{
    _root_cfg = YAML::LoadFile(path_to_config_file);
    if(!_root_cfg["mat_file_name"]) {

        DPRINTF("ERROR in %s ! Playback_mat_with_DK needs mandatory node mat_file_name in the YAML file!\n", __PRETTY_FUNCTION__);
        return false;
    }
    else{

        _mat_file_name = _root_cfg["mat_file_name"].as<std::string>();
    }
    
    // initialize mat file
    _mat = Mat_Open(_mat_file_name.c_str(), MAT_ACC_RDONLY);
    
    // initialize motor pos mat var
    if( _mat ) { // TBD if needed take the time var
        
        _motor_position_mat = Mat_VarRead(_mat, (char*)"motor_position");
        
        if( _motor_position_mat ) {
            
            // save dimensions
            for(int i=0; i<_motor_position_mat->rank; ++i)  {
                std::cout<<"\tdim["<<i<<"] == "<<_motor_position_mat->dims[i]<< std::endl;
                // NOTE expeting data as XBotCoreLogger
                if( i == 0 ) { 
                    _motor_position_rows = _motor_position_mat->dims[i];
                    _q_motor.resize(_motor_position_rows);
                }
                else if ( i == 1 ) {
                    _motor_position_cols = _motor_position_mat->dims[i];
                }
            }
            
            // save data
            _motor_position_data = static_cast<const double*>(_motor_position_mat->data) ;
        }
        else {
            DPRINTF("ERROR in %s ! Mat_VarRead on 'motor_position' failed \n", __PRETTY_FUNCTION__);
            return false;
        }
    }
    else {
        DPRINTF("ERROR in %s ! Mat_Open failed\n", __PRETTY_FUNCTION__);
        return false;
    }


    /* Save robot to a private member. */
    _robot = robot;

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/Playback_mat_with_DK_log");
    
    // fake initialization
    
     Eigen::Affine3d pose;
     _logger->add("dk_left_actual_pos", pose.translation());
     _logger->add("dk_right_actual_pos", pose.translation());
     _logger->add("dk_left_actual_or", pose.linear());
     _logger->add("dk_right_actual_or", pose.linear());
    
    return true;


}


void Playback_mat_with_DK::mat_read_q_motor( int index_col,
                                             Eigen::VectorXd& q_motor)
{

    for(int i = 0; i < _motor_position_rows; i ++) {
        _q_motor(i) = _motor_position_data[(index_col * 15) + i];
    }

}

void Playback_mat_with_DK::on_start(double time)
{
    /* Save the robot starting config to a class member */
    _start_time = time;
    
   _current_index_col = 0;

}

void Playback_mat_with_DK::on_stop(double time)
{
}


void Playback_mat_with_DK::control_loop(double time, double period)
{
         
     if(_current_index_col == (_motor_position_cols - 1)) {
         std::cout << " Finished!" << std::endl;
         return;
     }
     
      std::cout << "current index : " << _current_index_col << " over " << _motor_position_cols << std::endl;
     
     // read current sample for q motor
     mat_read_q_motor(_current_index_col, _q_motor); 
     
     std::cout << _q_motor << std::endl;
     
     // set the joint position on the internal model
     _robot->model().setJointPosition(_q_motor);
     _robot->model().update(true, false, false);
     
     // TBD do it generic
     Eigen::Affine3d left_pose, right_pose;
    _robot->model().getPose("arm1_8",  _robot->model().chain("torso").getTipLinkName(), left_pose);
    _robot->model().getPose("arm2_7",  _robot->model().chain("torso").getTipLinkName(), right_pose);
    
    _logger->add("dk_left_actual_pos", left_pose.translation());
    _logger->add("dk_right_actual_pos", right_pose.translation());
    
    _logger->add("dk_left_actual_or", left_pose.linear());
    _logger->add("dk_right_actual_or", right_pose.linear());
     
     _current_index_col++;
     
     // show the movement
     _robot->setReferenceFrom(_robot->model(), XBot::Sync::Position);
     _robot->move();


}

bool Playback_mat_with_DK::close()
{
    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}
