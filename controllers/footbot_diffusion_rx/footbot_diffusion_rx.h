#ifndef FOOTBOT_DIFFUSION_RX_H
#define FOOTBOT_DIFFUSION_RX_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
// Logger library
#include <argos3/core/utility/logging/argos_log.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

using namespace argos;

class CFootBotRX : public CCI_Controller
{
  public:
    struct FollowingParams
    {
        Real dist;
        CRadians ang;
    };

    struct SWheelTurningParams
    {
        /*
         * The turning mechanism.
         * The robot can be in three different turning states.
         */
        enum ETurningMechanism
        {
            NO_TURN = 0,    // go straight
            SOFT_TURN,      // both wheels are turning forwards, but at different speeds
            HARD_TURN,      // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
         * Angular thresholds to change turning state.
         */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    /* Class constructor. */
    CFootBotRX();

    /* Class destructor. */
    virtual ~CFootBotRX() {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><footbot_diffusion_controller> section.
     */
    virtual void Init(TConfigurationNode& t_node);

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
    virtual void Reset();

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy() {}

  protected:
    /* Creates an object repulsion vector */
    CVector2 ReceiveMasterPosition(Real& masterOrient);
    /* Creates an object repulsion vector */
    CVector2 ReadProxSensor();
    /* Creates an object repulsion vector, in global coordinates */
    CVector2 ObjectRepulsion(const CVector2 obstacle, const CRadians orient);
    /* Acquire formation position, in local coordinates */
    CVector2 ObjectRepulsionLocal(const CVector2 obstacle);
    /* Acquire formation position*/
    bool AcquirePosition();
    /* Calculates the vector to the closest light. */
    CVector2 VectorToLight();
    /* Gets a direction vector as input and transforms it into wheel actuation. */
    void SetWheelSpeedsFromVector(const CVector2& c_heading);

  private:
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the foot-bot proximity sensor */
    CCI_FootBotProximitySensor* m_pcProximity;
    // Pointer to led actuator
    CCI_LEDsActuator* m_leds;
    /* Pointer to the foot-bot light sensor */
    CCI_FootBotLightSensor* m_pcLight;
    // Pointer to positioning sensor
    CCI_PositioningSensor* m_pos;
    // Pointer to the range and bearing actuator
    CCI_RangeAndBearingActuator* m_pcTx;
    // Pointer to the range and bearing sensor
    CCI_RangeAndBearingSensor* m_pcRx;
    /* Pointer to the omnidirectional camera sensor */
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;

    /* Maximum tolerance for the angle between
     * the robot heading direction and
     * the closest obstacle detected. */
    CDegrees m_cAlpha;

    /* Maximum tolerance for the proximity reading between
     * the robot and the closest obstacle.
     * The proximity reading is 0 when nothing is detected
     * and grows exponentially to 1 when the obstacle is
     * touching the robot.
     */
    Real m_fDelta;

    /* Wheel speed. */

    Real m_fWheelVelocity;

    /* Angle tolerance range to go straight.
     * It is set to [-alpha,alpha]. */
    CRange<CRadians> m_cGoStraightAngleRange;

    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;

    /* The following parameters */
    FollowingParams m_FollowingParams;

    /* Actuation gains */
    Real kMantainFormation;
    Real kFollowLight;
    Real kAvoidObstacle;

    /* Global Light Vectors */
    CVector2 light;
    CVector2 deduced_light;

    /* Robot orientation last time light was seen*/
    CRadians angle_var_ref;

    /* Light Vector deduction mode*/
    bool light_mode;

    /* ID detection variables*/
    std::string id_string;
    bool id_detected;
};

#endif
