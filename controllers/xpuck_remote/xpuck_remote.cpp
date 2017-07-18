#include "xpuck_remote.h"
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

Xpuck_remote::Xpuck_remote() :
        actuator_wheels(NULL),
        sensor_proximity(NULL),
        wheel_velocity(2.5f),
        // Create ZMQ context and sockets
        context         (1),
        pub_sensors     (context, ZMQ_PUB),
        sub_actuators   (context, ZMQ_SUB)

{

}

/****************************************/
/****************************************/

void Xpuck_remote::Init(TConfigurationNode &t_node)
{
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "differential_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><epuck_obstacleavoidance><actuators> and
     * <controllers><epuck_obstacleavoidance><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */

    actuator_wheels     = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    sensor_proximity    = GetSensor<CCI_ProximitySensor>("proximity");
    sensor_position     = GetSensor<CCI_PositioningSensor>("positioning");
    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    GetNodeAttributeOrDefault(t_node, "velocity", wheel_velocity, wheel_velocity);

    // Get the ID and turn it into that recognised by the rest of the system.
    // Given a prefix "xp" argos names the instances "xp%d" but we need then as "xp%02d"
    id = GetId();
    int i = atoi(id.substr(2).c_str());
    id = string_format("xp%02d", i);

    // Publisher to send out sensor data
    pub_sensors.connect("tcp://localhost:" ZMQPORT_CONTROL_SENSORS);
    // Subscriber to listen to returning actuator commands
    sub_actuators.connect("tcp://localhost:" ZMQPORT_CONTROL_ACTUATORS);
    // subscribe to everything which has our ID
    sub_actuators.setsockopt(ZMQ_SUBSCRIBE, id.c_str(), id.size());
    // Throw away stale data as new items arrive
    const int confl = true;
    sub_actuators.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));

    wheel_left = 0.0;
    wheel_right = 0.0;

}

/****************************************/
/****************************************/

void Xpuck_remote::ControlStep()
{
    // Get the sensor and position readings from the simulator
    std::vector<double> prox            = sensor_proximity->GetReadings();
    CCI_PositioningSensor::SReading pos = sensor_position->GetReading();

    // Convert prox readings to something like what the real epuck produces
    // Argos does e^-dist. Since the range is 0 to 0.04m, this gives
    // 1 at 0m, 0.96 at 0.04m, 0 at out of range
    //
    // from my README in metis/sensor_characterisation:

    //# The proximity sensors work like radar, and the received signal power varies
    //# in proportion to inverse 4th power.
    //# Actual measurement of multiple epuck sensors with data cleaning shows that the
    //# close to the sensor (< ~ 5mm) the return signal increases very slowly (this
    //# may be due to saturation of the electronics).
    //#
    //#
    //# A pretty good curve fit is:
    //#
    //# y' = g * ((ax + d)^-4 + o)
    //# y =   0                   if y' < 0
    //#       y'                  if y' < g
    //#       sqrt(y' - g) + g    otherwise
    //#
    //# where
    //# g = 3500
    //# a = 40
    //# d = 0.7
    //# o = -0.06

#define MAX_RANGE 0.04f
    int pr[8];
    float dist[8];
    const float g = 3500.0f;
    const float a = 40.0f;
    const float d = 0.7f;
    const float o = -0.06f;
    for (int i = 0; i < 8; i++)
    {
        // Turn readings back into distances
        dist[i] = prox[i] == 0.0 ? MAX_RANGE : -(float)log(prox[i]);
        float y = (float)(g * (pow(a * dist[i] + d, -4) + o));
        if (y < 0)
            y = 0;
        else if (y >= g)
            y = (float)(sqrt(y - g) + g);
        pr[i] = (int)y;
    }

    // Send out the sensor data
    CRadians theta;
    CVector3 cv;
    // Orientation is a quaternion, but the epuck is always oriented on z-axis, so
    // convert to angle axis, flip angle if necessary, then wrap
    pos.Orientation.ToAngleAxis(theta, cv);
    float th = theta.GetValue() * cv.GetZ();
    // wrap angle to -pi:pi
    th = atan2(sin(th), cos(th));
//    printf("%s x:%9f y:%9f th:%9f %9f %9f %9f %9f %9f %9f %9f %9f\n", id.c_str(), pos.Position.GetX(), pos.Position.GetY(),
//           th, dist[0],dist[1],dist[2],dist[3],dist[4],dist[5],dist[6],dist[7]);
    std::string buf = string_format("%s %8f %8f %8f %4d %4d %4d %4d %4d %4d %4d %4d", id.c_str(),
                                    pos.Position.GetX(), pos.Position.GetY(), th,
           pr[0],pr[1],pr[2],pr[3],pr[4],pr[5],pr[6],pr[7]);
    zmq::message_t msg(buf.size());
    memcpy((char*) msg.data(), &buf[0], buf.size());
    pub_sensors.send(msg);

    // See if there is any actuator messages
    zmq::message_t actm;
    bool empty = false;
    while (!empty)
    {
        sub_actuators.recv(&actm, ZMQ_NOBLOCK);
        if (actm.size())
        {
            // We got something
            std::string s = std::string(static_cast<char *>(actm.data()), actm.size());
            //printf("Got message:%s\n", s.c_str());
            if (s.substr(0, 4) == id)
            {
                sscanf(s.substr(4).c_str(), "%f %f", &wheel_left, &wheel_right);
                //printf("message for me %f %f\n", wheel_left, wheel_right);
            }
        }
        else
            empty = true;
    }

    // Velocities from real robot are in m/s, Argos uses cm/s so multiply
    // up
    actuator_wheels->SetLinearVelocity(wheel_left*100, wheel_right*100);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(Xpuck_remote, "xpuck_remote_controller")
