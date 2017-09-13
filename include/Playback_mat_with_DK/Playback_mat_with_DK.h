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

#ifndef _Playback_mat_with_DK_H_
#define _Playback_mat_with_DK_H_

#include <XCM/XBotControlPlugin.h>


namespace XBotPlugin {

/**
 * @brief Playback_mat_with_DK XBot RT Plugin
 *
 **/
class Playback_mat_with_DK : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(std::string path_to_config_file,
                                     XBot::SharedMemory::Ptr shared_memory,
                                     XBot::RobotInterface::Ptr robot);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);

protected:

    virtual void control_loop(double time, double period);

private:
    
    void mat_read_q_motor(int index_col,
                          Eigen::VectorXd& q_motor);

    XBot::RobotInterface::Ptr _robot;

    double _start_time;

    XBot::MatLogger::Ptr _logger;
    
    Eigen::VectorXd _q_motor;
    std::string _mat_file_name;
    YAML::Node _root_cfg;
    mat_t* _mat;
    matvar_t* _motor_position_mat;
    const double* _motor_position_data;
    long unsigned int _motor_position_rows, _motor_position_cols;
    int _current_index_col;

};

}

#endif // _Playback_mat_with_DK_H_
