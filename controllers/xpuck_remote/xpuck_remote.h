

#ifndef XPUCK_REMOTE_H
#define XPUCK_REMOTE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>


#include <zmq.hpp>


#define ZMQPORT_CONTROL_SENSORS     "5700"
#define ZMQPORT_CONTROL_ACTUATORS   "5701"
//https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
#include <memory>
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] );
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

using namespace argos;

class Xpuck_remote : public CCI_Controller
{

public:

    Xpuck_remote();

    virtual ~Xpuck_remote()
    {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><epuck_obstacleavoidance_controller> section.
     */
    virtual void Init(TConfigurationNode &t_node);

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();

    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Reset()
    {}

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy()
    {}

private:

    CCI_DifferentialSteeringActuator    *actuator_wheels;
    CCI_ProximitySensor                 *sensor_proximity;
    CCI_PositioningSensor               *sensor_position;

    /*
     * The following variables are used as parameters for the
     * algorithm. You can set their value in the <parameters> section
     * of the XML configuration file, under the
     * <controllers><epuck_obstacleavoidance_controller> section.
     */
    /* Wheel speed. */
    float wheel_velocity;
    float wheel_left, wheel_right;


    zmq::context_t  context;
    zmq::socket_t   pub_sensors;
    zmq::socket_t   sub_actuators;
    std::string     id;

};

#endif
