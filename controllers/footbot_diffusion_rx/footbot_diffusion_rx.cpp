/* Include the controller definition */
#include "footbot_diffusion_rx.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


void CFootBotDiffusion::FollowingParams::Init(TConfigurationNode& t_node) {
   try {
      
      GetNodeAttribute(t_node, "distance", dist);
      CDegrees angle;
      GetNodeAttribute(t_node, "angle", angle);
      ang = ToRadians(angle);

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller following parameters.", ex);
   }
}


/****************************************/
/****************************************/

void CFootBotDiffusion::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
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
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcCamera(NULL),
   m_pcRx(NULL),
   m_pcTx(NULL),
   m_pcLight(NULL),  
   m_leds(NULL),
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_leds = GetActuator<CCI_LEDsActuator                       >("leds"); 
   m_pcLight  = GetSensor  <CCI_FootBotLightSensor             >("footbot_light");
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pos = GetSensor  <CCI_PositioningSensor                   >("positioning"); 
   m_pcTx      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRx      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(GetNode(t_node, "base"), "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(GetNode(t_node, "base"), "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(GetNode(t_node, "base"), "velocity", m_fWheelVelocity, m_fWheelVelocity);

   //Init sWheelTurning parameters
   try {
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   //Init Following parameters
   try {
      m_FollowingParams.Init(GetNode(t_node, "follow"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   //camera enable
   m_pcCamera->Enable();

}

/****************************************/
/****************************************/

void CFootBotDiffusion::Reset() {
   //Reset communications
   m_pcTx->ClearData();
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   
   CVector2 MasterPos = ReceiveMasterPosition();
   CVector2 obj_robo = ReadProxSensor();

   // calculate fixed following position vector // vd -> desired vector
      CVector2 vd = CVector2(m_FollowingParams.dist/100,m_FollowingParams.ang);

   // Check self position and orientation
      const CCI_PositioningSensor::SReading& pos = m_pos->GetReading();
  
   // camerda readings
      const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
      CVector2 vbuff = CFootBotDiffusion::VectorToRobot(sReadings); 

   // calculate vector to robot // va -> actual vector  
      CVector2 va = CVector2(MasterPos.GetX()-pos.Position[0] , MasterPos.GetY()-pos.Position[1]);
   // Orientation related operations    
      CRadians angle_aux;
      CVector3 vec_aux;
      pos.Orientation.ToAngleAxis(angle_aux,vec_aux);
   //CVector2 va = CVector2(vbuff.Length() , (vec_aux[2]*angle_aux + vbuff.Angle()));

   // calculate vector to following position // v = va-vd 
      CVector2 v = CVector2(va.GetX()-vd.GetX() , va.GetY()-vd.GetY() );
   //actuate 
   /*if (v.Length()>0.01){
      CFootBotDiffusion::SetWheelSpeedsFromVector(v,vec_aux[2]*angle_aux);
   }
   else {
      CFootBotDiffusion::SetWheelSpeedsFromVector(CVector2(0,0),vec_aux[2]*angle_aux);
   }*/

   obj_robo = ObjectRepulsion(obj_robo , vec_aux[2]*angle_aux);

   // Para testar formação  -> Parâmetros no xml 
   CFootBotDiffusion::SetWheelSpeedsFromVector(v,vec_aux[2]*angle_aux);
   
   //Para testar obstacle avoidance
   //CFootBotDiffusion::SetWheelSpeedsFromVector(CVector2(-1,0) + 1000*obj_robo ,vec_aux[2]*angle_aux);
  

   /*
   argos::LOG << "X_Tx :  " << MasterPos.GetX() << std::endl;
   argos::LOG << "Y_Tx :  " << MasterPos.GetY() << std::endl;
   argos::LOG << "VD :  " << vd.GetX() << " // " << vd.GetY() << std::endl ;
   argos::LOG << "VA :  " << va.GetX() << " // " << va.GetY() << std::endl ;
   argos::LOG << "V  :  " << v.GetX() << "  //  " << v.GetY() << std::endl ;
   //argos::LOG << "Orientation :  " << v.Angle() << std::endl;
   argos::LOG << "      " << std::endl; */

   argos::LOG << "obj_robo :  " << obj_robo.GetX() << " // " << obj_robo.GetY() << std::endl;

   
   /* DIFFUSION DEFAULT CONTROLLER
   int switch_var = 0;  
   // Get readings from proximity sensor 
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   // Sum them together 
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   // If the angle of the vector is small enough and the closest obstacle
   // is far enough, continue going straight, otherwise curve a little
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      // Go straight 
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      // switch LEDs color 
      if (switch_var){
         m_leds->SetAllColors(CColor::RED);
         switch_var = 0;
      }
      else {
         m_leds->SetAllColors(CColor::RED);
         switch_var = 1;
      }
      // Turn, depending on the sign of the angle 
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   } */
   
}

/****************************************/
/****************************************/

CVector2 CFootBotDiffusion::ReceiveMasterPosition() {
   Real X_Tx , Y_Tx;
    // Receive info from TX 
      const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRx->GetReadings();
      //Transform it 
      X_Tx = tPackets[0].Data[0] + 0.1*tPackets[0].Data[1] + 0.01*tPackets[0].Data[2];
      Y_Tx = tPackets[0].Data[3] + 0.1*tPackets[0].Data[4] + 0.01*tPackets[0].Data[5];

   return CVector2(X_Tx,Y_Tx);

}


/****************************************/
/****************************************/

CVector2 CFootBotDiffusion::VectorToLight() {
   /* Get light readings */
   const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
   /* Calculate a normalized vector that points to the closest light */
   CVector2 cAccum;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }
   if(cAccum.Length() > 0.0f) {
      /* Make the vector long as 1/4 of the max speed */
      cAccum.Normalize();
      cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   }
   return cAccum;
}

/****************************************/
/****************************************/

void CFootBotDiffusion::SetWheelSpeedsFromVector(const CVector2& c_heading , const CRadians orient) {
   Real SpeedLim = 0;
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   CRadians diff = Abs(cHeadingAngle-orient);
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   //if (fHeadingLength < 5) {diff = CRadians(0);}
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Max<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(diff <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(diff > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(diff <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(diff > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(diff > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }

   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - diff) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         if (c_heading.Length()<0) {
            SpeedLim  = m_sWheelTurningParams.MaxSpeed * 0.05;
         }
         else {
            SpeedLim  = m_sWheelTurningParams.MaxSpeed ;
         }
         fSpeed1 = -SpeedLim;
         fSpeed2 =  SpeedLim;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   LOG <<"Turn : " << m_sWheelTurningParams.TurningMechanism << std::endl;
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

CVector2 CFootBotDiffusion::VectorToRobot(const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings) {
   
   /* Go through the camera readings to calculate the vector */
   if(! sReadings.BlobList.empty()) {
      CVector2 out;
      size_t BlobsSeen = 0;

      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {

         if(sReadings.BlobList[i]->Color == CColor::RED) {
            
            out = CVector2(sReadings.BlobList[i]->Distance , sReadings.BlobList[i]-> Angle);
            BlobsSeen++;
         }
      }
      if(BlobsSeen > 0) {
         // Clamp the length of the vector to the max speed 
         /*if(out.Length() > m_sWheelTurningParams.MaxSpeed) {
            out.Normalize();
            out *= m_sWheelTurningParams.MaxSpeed;
         }*/
         argos::LOG << "1" << std::endl;
         return out;
      }
      else
         argos::LOG << "2" << std::endl;
         return CVector2();
   }
   else {
      argos::LOG << "3" << std::endl;
      return CVector2();
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotDiffusion::ReadProxSensor() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      //argos::LOG << tProxReads[i].Value<<"----"<<tProxReads[i].Angle<< std::endl;
   }
   cAccumulator /= tProxReads.size();
   
   return cAccumulator;
   
  
}

CVector2 CFootBotDiffusion::ObjectRepulsion(const CVector2 obstacle  , const CRadians orient) {
   CDegrees inv = CDegrees(180);
   CRadians aux = obstacle.Angle() + orient ;
   CVector2 out = CVector2(obstacle.Length() , aux + ToRadians(inv));
   return out;
}



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
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_rx_controller")
