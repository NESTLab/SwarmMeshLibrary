#include "swarmmesh_controller.h"

/****************************************/
/****************************************/

CSwarmMeshController::CSwarmMeshController() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRNG(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CSwarmMeshController::Init(TConfigurationNode& t_node) 
{
   /* Get sensor/actuator handles */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcBlobCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor     >("colored_blob_omnidirectional_camera"    );
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcRABA   = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   m_pcRABS   = GetSensor  <CCI_RangeAndBearingSensor  >("range_and_bearing");
   m_pcPositioning = GetSensor  <CCI_PositioningSensor  >("positioning");

   m_pcBlobCamera->Enable();

   m_pcRNG = CRandom::CreateRNG("argos");

   /* Parse the configuration file */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   ProcessOutMsgs();
}

/****************************************/
/****************************************/

void CSwarmMeshController::Diffuse() 
{
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = 
   m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
}

/****************************************/
/****************************************/

void CSwarmMeshController::ControlStep() 
{

   ProcessInMsgs();

   Diffuse();

   std::queue<SEventData> sEvents = RecordEvents();
   
   while(!sEvents.empty())
   {
      SEventData sEvent = sEvents.front();
      sEvents.pop();
      // m_cMesh->Put(sEvent);
   }

   ProcessOutMsgs();

}

/****************************************/
/****************************************/

std::queue<CSwarmMeshController::SEventData> CSwarmMeshController::RecordEvents() 
{
   std::queue<SEventData> sEvents;

   /* Get readings from blob camera */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& tBlobReads =
                m_pcBlobCamera->GetReadings();

   for(size_t i = 0; i < tBlobReads.BlobList.size(); ++i) {

      /* Assume this robot is the leader for recording the blob */
      bool bLocalLeader = true;
      /* Get blob position vector relative to this robot */
      CVector2 cEventCoord = CVector2(tBlobReads.BlobList[i]->Distance,
                                     tBlobReads.BlobList[i]->Angle);
      /* Verify that the robot is in fact the leader for recording the event */
      /* Check that the blob is within the desired sensor range 
      (must be at most half the comm range) */
      if (tBlobReads.BlobList[i]->Distance < BLOB_SENSOR_RANGE) 
      {
         /* Loop through neighbors */
         for(size_t i = 0; i < m_vecNeighbors.size(); ++i){
            /* Get neighbor position vector relative to this robot */
            CVector2 cNeighborCoord = CVector2(m_vecNeighbors[i].Distance,
                                           m_vecNeighbors[i].Bearing);
            /* Check that the neighbor is within the sensor range to the blob */
            if(Distance(cNeighborCoord, cEventCoord) < BLOB_SENSOR_RANGE)
            {
               /* If neighbor robot ID greater than mine, I am not the recording leader */
               if(m_vecNeighbors[i].RId > m_unRobotId)
               {
                  bLocalLeader = false;
                  break;
               }
            }
         }
      }
      else {bLocalLeader = false;}

      /* If leader, record the event */
      if (bLocalLeader) {
         SEventData sEvent;
         std::ostringstream stream;
         stream  << tBlobReads.BlobList[i]->Color;
         sEvent.type = stream.str();
         sEvent.payload = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
         CVector2 cPos = ComputeAbsolutePosition(cEventCoord);
         sEvent.location = std::make_pair(cPos.GetX(), cPos.GetY());
         sEvents.push(sEvent);
      }
   }
   return sEvents;
}

/****************************************/
/****************************************/


CVector2 CSwarmMeshController::ComputeAbsolutePosition(const CVector2& c_coordEvent)
{
   /* Get readings from positioning sensor */
   const CCI_PositioningSensor::SReading& tPosReads = 
   m_pcPositioning->GetReading();
   /* Position and orientation of robot in global frame */
   CVector2 cPosRobot(tPosReads.Position.GetX()*100,
                      tPosReads.Position.GetY()*100);
   CRadians cTheta, cY, cX;
   (tPosReads.Orientation).ToEulerAngles(cTheta, cY, cX);
   /* Position of event in global frame */
   CVector2 cEvet = c_coordEvent;
   Real fX = cPosRobot.GetX();
   Real fY = cPosRobot.GetY();
   CVector2 cRotated = (cEvet).Rotate(cTheta);
   CVector2 cRes(fX + cRotated.GetX(),
                 fY + cRotated.GetY());
   return cRes;
}

/****************************************/
/****************************************/

void CSwarmMeshController::ProcessInMsgs()
{
   SNeighbor sNeighbor;
   m_vecNeighbors.push_back(sNeighbor);
}

/****************************************/
/****************************************/

void CSwarmMeshController::ProcessOutMsgs()
{
      // m_unPacketSize = m_pcRABA->GetSize();
   /* Broadcast RID to Neighbors */
   // const std::string& strRobotId = GetId();
   // m_unRobotId = FromString<UInt16>(strRobotId.substr(1));
   // // m_unNodeKey = m_unRobotId;
   // CByteArray cBuffer;
   // cBuffer << m_unRobotId;
   // cBuffer << m_unRobotId;
   // // cBuffer << m_unNodeKey;
   // while(cBuffer.Size() < m_pcRABA->GetSize()) cBuffer << static_cast<UInt8>(0);
   // m_pcRABA->SetData(cBuffer);
}

/****************************************/
/****************************************/

/* This statement notifies ARGoS of the existence of the controller. */
REGISTER_CONTROLLER(CSwarmMeshController, "swarmmesh_controller")
