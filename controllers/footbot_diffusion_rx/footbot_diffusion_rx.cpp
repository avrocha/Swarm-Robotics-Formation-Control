/* Include the controller definition */
#include "footbot_diffusion_rx.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* STL string library*/
#include <sstream>
#include <string>

void CFootBotRX::SWheelTurningParams::Init(TConfigurationNode& t_node)
{
    try
    {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

CFootBotRX::CFootBotRX()
    : m_pcCamera(NULL),
      m_pcRx(NULL),
      m_pcTx(NULL),
      m_pcLight(NULL),
      m_leds(NULL),
      m_pcWheels(NULL),
      m_pcProximity(NULL),
      m_cAlpha(10.0f),
      m_fDelta(0.5f),
      m_fWheelVelocity(2.5f),
      m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)),
      kAvoidObstacle(2.5),
      kFollowLight(0.06),
      kMantainFormation(0.5),
      light(),
      deduced_light(),
      angle_var_ref(ToRadians(CDegrees(0))),
      id_detected(0)
{}

void CFootBotRX::Init(TConfigurationNode& t_node)
{
    m_leds = GetActuator<CCI_LEDsActuator>("leds");
    m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pos = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcTx = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRx = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");

    /* Parse the configuration file */
    GetNodeAttributeOrDefault(GetNode(t_node, "base"), "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(GetNode(t_node, "base"), "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(GetNode(t_node, "base"), "velocity", m_fWheelVelocity, m_fWheelVelocity);

    // Init sWheelTurning parameters
    try
    {
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
}

void CFootBotRX::Reset() { m_pcTx->ClearData(); }

void CFootBotRX::ControlStep()
{
    if(!id_detected)
    {
        AcquirePosition();
    }
    else
    {   
        // master orientation offset
        Real masterOrientOffset;

        // master position in global coordinates
        const CVector2 masterPos = ReceiveMasterPosition(masterOrientOffset);

        // check self position and orientation, in global coordinates
        const CCI_PositioningSensor::SReading& pos = m_pos->GetReading();

        //global orientation
        CRadians angle_aux;
        CVector3 vec_aux;
        pos.Orientation.ToAngleAxis(angle_aux, vec_aux);
    
        /* Caculate Formation Control vector */

        // calculate fixed following position vector, in local coordinates
        CVector2 desired = CVector2(m_FollowingParams.dist / 100, ToRadians(ToDegrees(m_FollowingParams.ang) + CDegrees(masterOrientOffset) - CDegrees(45)));

        // calculate vector to master from its actual position, in local coordinates
        CVector2 actual = CVector2(masterPos.GetX() - pos.Position[0], masterPos.GetY() - pos.Position[1]);

        // calculate vector to following position, in local coordinates
        CVector2 goToFormation = CVector2(actual.GetX() - desired.GetX(), actual.GetY() - desired.GetY());

        goToFormation = CVector2(goToFormation.Length(), goToFormation.Angle() - vec_aux[2] * angle_aux);

        /* Detect objects and create an object repulsion vector */

        // object position in local coordinates
        CVector2 objectPos = ReadProxSensor();

        // obstacle inverse vector, in local coordinates
        CVector2 objectRep = ObjectRepulsionLocal(objectPos);


        /* Calculate light vector, if unseen use last known coordinates, adjusted to new orientation */

        if(VectorToLight().Length() != 0)
        {
            light = VectorToLight();
            light_mode = 0;
            angle_var_ref = vec_aux[2] * angle_aux;
        }
        else
        {
            deduced_light = CVector2(light.Length(), light.Angle() + (angle_var_ref - vec_aux[2] * angle_aux));
            light_mode = 1;
        }

        // resultant vector
        auto res = goToFormation * kMantainFormation + objectRep * kAvoidObstacle +
                   (1 - light_mode) * light * kFollowLight + light_mode * deduced_light * kFollowLight;

        if(objectRep.Length() == 0)
            res *= 0.38 * m_sWheelTurningParams.MaxSpeed;
        else
            res *= 0.08 * m_sWheelTurningParams.MaxSpeed;

        // resultant actuation from sum of vectors
        SetWheelSpeedsFromVector(res);

        // debug - temp
        argos::LOG << "SLAVE:" << std::endl;
        argos::LOG << "desired:  " << desired.Length() << " | " << ToDegrees(desired.Angle()) << std::endl;
        argos::LOG << "f_ctrl:  " << goToFormation.Length() << " | " << ToDegrees(goToFormation.Angle()) << std::endl;
        argos::LOG << "obj_rep: " << objectRep.Length() << "|" << ToDegrees(objectRep.Angle()) << std::endl;
        argos::LOG << "light:   " << light.Length() << "|" << ToDegrees(light.Angle()) << std::endl;
        argos::LOG << "d_light: " << deduced_light.Length() << "|" << ToDegrees(deduced_light.Angle()) << std::endl;
        argos::LOG << "res:     " << res.Length() << "|" << ToDegrees(res.Angle()) << std::endl;
        argos::LOG << "res2:    " << res.Length() << "|" << ToDegrees(res.Angle() - vec_aux[2] * angle_aux)
                   << std::endl;
    }
}

void CFootBotRX::AcquirePosition()
{
    Real ID_Tx, distance_Tx, angle_Tx;

    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRx->GetReadings();

    std::stringstream id_s(GetId());
    int id;
    id_s >> id;

    if(id == tPackets[0].Data[0])
    {
        m_FollowingParams.dist =
            1000 * tPackets[0].Data[1] + 100 * tPackets[0].Data[2] + 10 * tPackets[0].Data[3] + 1 * tPackets[0].Data[4];
        m_FollowingParams.ang = ToRadians(CDegrees(1000 * tPackets[0].Data[5] + 100 * tPackets[0].Data[6] +
                                                   10 * tPackets[0].Data[7] + 1 * tPackets[0].Data[8]));

        argos::LOG << "Slave " << id << " position decoded: " << m_FollowingParams.dist << "|"
                   << ToDegrees(m_FollowingParams.ang) << std::endl;
        id_detected = 1;
    }
}

CVector2 CFootBotRX::ReceiveMasterPosition(Real &masterOrient)
{
    Real x, y;

    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRx->GetReadings();

    x = tPackets[0].Data[0] + 0.1 * tPackets[0].Data[1] + 0.01 * tPackets[0].Data[2];
    y = tPackets[0].Data[3] + 0.1 * tPackets[0].Data[4] + 0.01 * tPackets[0].Data[5];

    masterOrient = 100*tPackets[0].Data[6] + 10 * tPackets[0].Data[7] + tPackets[0].Data[8] + 0.1*tPackets[0].Data[9];

    return CVector2(x, y);
}

CVector2 CFootBotRX::VectorToLight()
{
    /* Get light readings */
    const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();

    /* Calculate a normalized vector that points to the closest light */
    CVector2 cAccum;

    for(size_t i = 0; i < tReadings.size(); ++i)
    {
        cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
    }
    if(cAccum.Length() > 0.0f)
    {
        cAccum.Normalize();
    }
    return cAccum;
}

CVector2 CFootBotRX::ReadProxSensor()
{
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

    /* Sum them together */
    CVector2 cAccumulator;

    for(auto reading : tProxReads)
    {
        cAccumulator += CVector2(reading.Value, reading.Angle);
    }

    cAccumulator /= tProxReads.size();

    return cAccumulator;
}

CVector2 CFootBotRX::ObjectRepulsion(const CVector2 obstacle, const CRadians orient)
{
    CRadians aux = obstacle.Angle() + orient;

    // aux tolerance to avoid blocking
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(aux))
        aux = ToRadians(CDegrees(0));

    if(obstacle.Length() != 0)
        return CVector2(1 / (obstacle.Length() + 0.5), aux + ToRadians(CDegrees(180)));

    else
        return CVector2(obstacle.Length(), aux + ToRadians(CDegrees(180)));
}

CVector2 CFootBotRX::ObjectRepulsionLocal(const CVector2 obstacle)
{
    CRadians aux = obstacle.Angle();

    // aux tolerance to avoid blocking
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(aux))
        aux = ToRadians(CDegrees(0));

    return CVector2(obstacle.Length(), aux + ToRadians(CDegrees(180)));
}

void CFootBotRX::SetWheelSpeedsFromVector(const CVector2& c_heading)
{
    Real SpeedLim = 0;
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    // if (fHeadingLength < 5) {diff = CRadians(0);}
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Max<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    /* State transition logic */
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN)
    {
        if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold)
        {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN)
    {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
        {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold)
        {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN)
    {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
        {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold)
        {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }

    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(m_sWheelTurningParams.TurningMechanism)
    {
        case SWheelTurningParams::NO_TURN:
        {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
        case SWheelTurningParams::SOFT_TURN:
        {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                                m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
        case SWheelTurningParams::HARD_TURN:
        {
            fSpeed1 = -0.8 * m_sWheelTurningParams.MaxSpeed;
            fSpeed2 = 0.8 * m_sWheelTurningParams.MaxSpeed;
            break;
        }
    }
    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cHeadingAngle > CRadians::ZERO)
    {
        /* Turn Left */
        fLeftWheelSpeed = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else
    {
        /* Turn Right */
        fLeftWheelSpeed = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/* This statement notifies ARGoS of the existence of the controller.*/
REGISTER_CONTROLLER(CFootBotRX, "footbot_diffusion_rx_controller")