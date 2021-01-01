/* Include the controller definition */
#include "footbot_diffusion_tx.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <algorithm>
#include <cmath>

void CFootBotTX::SWheelTurningParams::Init(TConfigurationNode& t_node)
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

CFootBotTX::CFootBotTX()
    : m_pcRx(NULL),
      m_pcTx(NULL),
      m_pcLight(NULL),
      m_leds(NULL),
      m_pcWheels(NULL),
      m_pcProximity(NULL),
      m_cAlpha(10.0f),
      m_fDelta(0.5f),
      m_fWheelVelocity(2.5f),
      m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)),
      kAvoidObstacle(4),
      kFollowLight(0.8),
      light(),
      deduced_light(),
      angle_var_ref(ToRadians(CDegrees(0))),
      temp_ID(0),
      distanceLine({ 100, 200, 300, 400 }),
      angleLine({ 0, 0, 0, 0 }),
      distanceSquare({ (int)floor(sqrt(2) * 100 / 2), (int)floor(sqrt(2) * 100), 100, 100 }),
      angleSquare({ 0, 0, 315, 45 }),
      distanceCurve({ 100, 100, 200, 200 }),
      angleCurve({ 45, 315, 45, 315 })
{}

void CFootBotTX::Init(TConfigurationNode& t_node)
{
    m_leds = GetActuator<CCI_LEDsActuator>("leds");
    m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pos = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcTx = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRx = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_dist_sens = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
    m_dist_act = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");

    //Enable distance sensing 
    m_dist_act->Enable();

    /* Parse the configuration file */
    GetNodeAttributeOrDefault(GetNode(t_node, "base"), "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(GetNode(t_node, "base"), "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(GetNode(t_node, "base"), "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(GetNode(t_node, "formation"), "behaviour", m_behaviour, m_behaviour);
    GetNodeAttributeOrDefault(GetNode(t_node, "formation"), "num_slaves", m_num_slaves, m_num_slaves);

    // Init sWheelTurning parameters
    try
    {
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    // Turn on red LED
    m_leds->SetSingleColor(12, CColor::RED);

    // initialize ACKs vector
    ack_vec = std::vector<int>(m_num_slaves, 0);
}

void CFootBotTX::Reset() { m_pcTx->ClearData(); m_dist_act->Enable();}

void CFootBotTX::ControlStep()
{
    argos::LOG << "MASTER:" << std::endl;

    // check self position and orientation, in global coordinates
    const CCI_PositioningSensor::SReading& pos = m_pos->GetReading();

    /* Calculate light vector, if unseen use last known coordinates, adjusted to new orientation */

    CRadians angle_aux;
    CVector3 vec_aux;
    pos.Orientation.ToAngleAxis(angle_aux, vec_aux);

    if(VectorToLight().Length() != 0)
    {
        light = VectorToLight();
        light_mode = 0;
        angle_var_ref = vec_aux[2] * angle_aux;
        TransmitPosition(light.Angle(), vec_aux[2] * angle_aux);
    }
    else
    {
        deduced_light = CVector2(light.Length(), light.Angle() + (angle_var_ref - vec_aux[2] * angle_aux));
        light_mode = 1;
        TransmitPosition(deduced_light.Angle(), vec_aux[2] * angle_aux);
    }

    /* Nota temporária : Alex, quando fores retransmitir uma nova transmissão é
        importante que faças o seguinte:
        1) colocares o vetor ack_vec todo a zeros (ele está inicializado no Init())
        2) limpes os buffers de transmissão dos slaves no footbot_diffusion_rx.cpp (m_pcTx->ClearData();)
        Penso que não me esqueço de nada ...
        */
    int num_of_ACKs = CheckACK();

    if(!CreateFormation() || num_of_ACKs != m_num_slaves)
    {
        argos::LOG << "Creating formation ... " << std::endl;
    }
    else
    {
        /* Detect objects and create an object repulsion vector */

        // object position in local coordinates
        CVector2 objectPos = ReadProxSensor();

        // obstacle inverse vector, in local coordinates
        CVector2 objectRep = ObjectRepulsionLocal(objectPos);

        if(objectRep.Length() != 0)
            objectRep.Normalize();

        /* Resultant Vector */
        auto res = (1 - light_mode) * light * kFollowLight + light_mode * deduced_light * kFollowLight +
                   objectRep * kAvoidObstacle;

        if(objectRep.Length() == 0)
            res *= 0.40 * m_sWheelTurningParams.MaxSpeed;
        else
            res *= 0.08 * m_sWheelTurningParams.MaxSpeed;

        //########### EXEMPLO ################
        
        CVector2 example = GetDistanceValues();

        SetWheelSpeedsFromVector(res);

        // debug - temp
        argos::LOG << "obj_rep: " << objectRep.Length() << "|" << ToDegrees(objectRep.Angle()) << std::endl;
        argos::LOG << "light:   " << light.Length() << "|" << ToDegrees(light.Angle()) << std::endl;
        argos::LOG << "d_light: " << deduced_light.Length() << "|" << ToDegrees(deduced_light.Angle()) << std::endl;
        argos::LOG << "res:     " << res.Length() << "|" << ToDegrees(res.Angle()) << std::endl;
        argos::LOG << "res2:    " << res.Length() << "|" << ToDegrees(res.Angle() - vec_aux[2] * angle_aux)
                   << std::endl;
    }
}

void CFootBotTX::TransmitPosition(const CRadians& lightOrient, const CRadians& masterOrient)
{
    const CCI_PositioningSensor::SReading& pos = m_pos->GetReading();

    int aux;
    aux = (int)1000 * pos.Position[0];
    m_pcTx->SetData(0, aux / 1000);
    aux = aux - 1000 * (aux / 1000);
    m_pcTx->SetData(1, aux / 100);
    aux = aux - 100 * (aux / 100);
    m_pcTx->SetData(2, aux / 10);

    aux = (int)1000 * pos.Position[1];
    m_pcTx->SetData(3, aux / 1000);
    aux = aux - 1000 * (aux / 1000);
    m_pcTx->SetData(4, aux / 100);
    aux = aux - 100 * (aux / 100);
    m_pcTx->SetData(5, aux / 10);

    // light relative orientation
    int orientation = ToDegrees(10 * (lightOrient + masterOrient)).GetValue();
    argos::LOG << "master orientation sent: " << orientation << std::endl;

    for(int i = 0; i < 4; i++)
    {
        m_pcTx->SetData(9 - i, orientation % 10);
        orientation /= 10;
    }
}

bool CFootBotTX::CreateFormation()
{
    if(ReadProxSensor().Length() > 0 && changeFormation == 0 && !m_behaviour.compare("tunel"))
    {
        std::fill(ack_vec.begin(), ack_vec.end(), 0);
        m_pcTx->ClearData();
        changeFormation = 1;
        argos::LOG << "CHANGING FORMATION" << std::endl;
        temp_ID = 0;

        // CODE 9999999999 = switch formation */
        for(int i = 0; i < 10; i++)
        {
            m_pcTx->SetData(i, 9);
        }

        return 0;
    }

    if(temp_ID < m_num_slaves + 1)
    {
        // offset start comms
        if(temp_ID == 0)
        {
            AssignPosition(100, 999, 999);
            temp_ID++;
        }
        else
        {
            if(changeFormation == 0 && (!m_behaviour.compare("obstacle_square") || !m_behaviour.compare("tunel")))
            {
                argos::LOG << "Sending dist:" << distanceSquare[temp_ID - 1] << " ang:" << angleSquare[temp_ID - 1]
                           << std::endl;
                AssignPosition(temp_ID, distanceSquare[temp_ID - 1], angleSquare[temp_ID - 1]);
                temp_ID++;
            }
            else if(changeFormation == 0 && !m_behaviour.compare("obstacle_curve"))
            {
                argos::LOG << "Sending dist:" << distanceCurve[temp_ID - 1] << " ang:" << angleCurve[temp_ID - 1]
                           << std::endl;
                AssignPosition(temp_ID, distanceCurve[temp_ID - 1], angleCurve[temp_ID - 1]);
                temp_ID++;
            }
            else if(changeFormation == 1)
            {
                argos::LOG << "Sending dist:" << distanceLine[temp_ID - 1] << " ang:" << angleLine[temp_ID - 1]
                           << std::endl;
                AssignPosition(temp_ID, distanceLine[temp_ID - 1], angleLine[temp_ID - 1]);
                temp_ID++;
            }
        }
        return 0;
    }

    return 1;
}

void CFootBotTX::AssignPosition(int ID, int distance, int angle)
{
    // ID
    m_pcTx->SetData(0, ID);

    // Distance
    for(int i = 0; i < 4; i++)
    {
        m_pcTx->SetData(4 - i, distance % 10);
        distance /= 10;
    }

    // Angle
    for(int i = 0; i < 4; i++)
    {
        m_pcTx->SetData(8 - i, angle % 10);
        angle /= 10;
    }
}

int CFootBotTX::CheckACK()
{
    // Scanning info transmitted by slaves
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRx->GetReadings();

    int sum = 0;
    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        int id = tPackets[i].Data[0];
        // check if slave has already transmitted
        if(!ack_vec[id - 1])
            ack_vec[id - 1] = 1;

        sum += ack_vec[i];
    }
    return sum;
}

CVector2 CFootBotTX::VectorToLight()
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

CVector2 CFootBotTX::ReadProxSensor()
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

CVector2 CFootBotTX::GetDistanceValues(){
    const CCI_FootBotDistanceScannerSensor::TReadingsMap& tDisranceReads = m_dist_sens->GetReadingsMap();
    CVector2 cAccumulator;
    for(CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = tDisranceReads.begin(); it != tDisranceReads.end(); ++it)
    {	
        cAccumulator += CVector2(it->second, it->first);
    }
    return cAccumulator;
}

CVector2 CFootBotTX::ObjectRepulsionLocal(const CVector2 obstacle)
{
    CRadians aux = obstacle.Angle();

    // aux tolerance to avoid blocking
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(aux))
        aux = ToRadians(CDegrees(0));

    return CVector2(obstacle.Length(), aux + ToRadians(CDegrees(180)));
}

CVector2 CFootBotTX::ObjectRepulsion(const CVector2 obstacle, const CRadians orient)
{
    CRadians aux = obstacle.Angle() + orient;

    // aux tolerance to avoid blocking
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(aux))
        aux = ToRadians(CDegrees(0));

    return CVector2(obstacle.Length(), aux + ToRadians(CDegrees(180)));
}

void CFootBotTX::SetWheelSpeedsFromVector(const CVector2& c_heading)
{
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
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
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 = m_sWheelTurningParams.MaxSpeed;
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
REGISTER_CONTROLLER(CFootBotTX, "footbot_diffusion_tx_controller")
