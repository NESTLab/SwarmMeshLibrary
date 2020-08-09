#include "swarmmesh_controller.h"

/****************************************/
/****************************************/

SEventData UnpackEventDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
   SEventData sValue;
   sValue.type = swarmmesh::UnpackString(vec_buffer, un_offset);
   sValue.payload = swarmmesh::UnpackFloat(vec_buffer, un_offset);
   sValue.location = std::make_pair(swarmmesh::UnpackFloat(vec_buffer, un_offset),
                                    swarmmesh::UnpackFloat(vec_buffer, un_offset));
   return sValue;
}

void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_event) {
   swarmmesh::PackString(vec_buffer, s_event.type);
   swarmmesh::PackFloat(vec_buffer, s_event.payload);
   swarmmesh::PackFloat(vec_buffer, s_event.location.first);
   swarmmesh::PackFloat(vec_buffer, s_event.location.second);
}

swarmmesh::SKey HashEventDataType(SEventData& s_value) {
      std::string strColor = s_value.type;

      /* Data hashing based on blob color */
      uint32_t unHash;
      if(strColor == "gray10") {unHash = 1;}
      else if(strColor == "white") {unHash = 1 + BUCKET_SIZE;}
      else if(strColor == "red") {unHash = 1 + 2 * BUCKET_SIZE;}
      else if(strColor  == "green") {unHash = 1 + 3 * BUCKET_SIZE;}
      else if(strColor  == "blue") {unHash = 1 + 4 * BUCKET_SIZE;}
      else if(strColor  == "magenta") {unHash = 1 + 5 * BUCKET_SIZE;}
      else if(strColor == "cyan") {unHash = 1 + 6 * BUCKET_SIZE;}
      else if(strColor  == "yellow") {unHash = 1 + 7 * BUCKET_SIZE;}
      else if(strColor  == "orange") {unHash = 1 + 8 * BUCKET_SIZE;}
      else if(strColor  == "brown") {unHash = 1 + 9 * BUCKET_SIZE;}
      else if(strColor  == "purple") {unHash = 1 + 10 * BUCKET_SIZE;}
      else if(strColor  ==  "gray50") {unHash = 1 + 11 * BUCKET_SIZE;}
      else  unHash = 0;

      /* Unique tuple identifier based on robot id and 
         tuple count */
      // ++unTupleCount;
      uint32_t unIdentifier = ((uint32_t) 1 << 16) +  3;
      return swarmmesh::SKey(unHash, unIdentifier);
}

// class HashEventDataType {

//    private:
//    uint16_t unRobotId = 0;
//    uint16_t unTupleCount = 0;

//    // HashEventDataType() : 
//    //    unRobotId(0),
//    //    unTupleCount(0) {}

//    public:
//    swarmmesh::SKey operator()(SEventData& s_value) const {

//       std::string strColor = s_value.type;

//       /* Data hashing based on blob color */
//       uint32_t unHash;
//       if(strColor == "gray10") {unHash = 1;}
//       else if(strColor == "white") {unHash = 1 + BUCKET_SIZE;}
//       else if(strColor == "red") {unHash = 1 + 2 * BUCKET_SIZE;}
//       else if(strColor  == "green") {unHash = 1 + 3 * BUCKET_SIZE;}
//       else if(strColor  == "blue") {unHash = 1 + 4 * BUCKET_SIZE;}
//       else if(strColor  == "magenta") {unHash = 1 + 5 * BUCKET_SIZE;}
//       else if(strColor == "cyan") {unHash = 1 + 6 * BUCKET_SIZE;}
//       else if(strColor  == "yellow") {unHash = 1 + 7 * BUCKET_SIZE;}
//       else if(strColor  == "orange") {unHash = 1 + 8 * BUCKET_SIZE;}
//       else if(strColor  == "brown") {unHash = 1 + 9 * BUCKET_SIZE;}
//       else if(strColor  == "purple") {unHash = 1 + 10 * BUCKET_SIZE;}
//       else if(strColor  ==  "gray50") {unHash = 1 + 11 * BUCKET_SIZE;}
//       else  unHash = 0;

//       /* Unique tuple identifier based on robot id and 
//          tuple count */
//       // ++unTupleCount;
//       uint32_t unIdentifier = ((uint32_t) unRobotId << 16) +  unTupleCount;
//       return swarmmesh::SKey(unHash, unIdentifier);
//    }
// };

// /* Key partitioning function by degree (D) and memory (M) */
// class PartitionDM : public CSwarmMesh<SEventData>::Partition {

//    uint16_t operator() (uint16_t un_degree, uint16_t un_mem) const{
//       if(un_degree == 0) return un_mem;
//       return un_degree * un_mem; 
//    };


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

   /* Initialize SwarmMesh variables */
   const std::string& strRobotId = GetId();
   m_unRobotId = FromString<UInt16>(strRobotId.substr(1));
   m_unTupleCount = 0; 
   m_cMySM.Init(m_unRobotId);
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

   /* Process incoming messages */
   ProcessInMsgs();

   /* Move */
   Diffuse();

   /* Record events */
   std::queue<SEventData> sEvents = RecordEvents();   
   while(!sEvents.empty())
   {
      /* Retrieve event to write in SwarmMesh */
      SEventData sEvent = sEvents.front();
      sEvents.pop();
      ++m_unTupleCount;
      /* Perform a put operation in SwarmMesh */
      m_cMySM.Put(sEvent);
   }

   /* Tell SwarmMesh to queue messages for routing data */
   m_cMySM.Route();

   /* Process outgoing messages */
   ProcessOutMsgs();

}

/****************************************/
/****************************************/

std::queue<SEventData> CSwarmMeshController::RecordEvents() 
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

   m_cMySM.ResetNeighbors();

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   for(size_t i = 0; i < tPackets.size(); ++i) {
      /* Copy packet into temporary buffer */
      CByteArray cData = tPackets[i].Data;
      /* Get robot id and update neighbor information */
      UInt16 unRobotId = cData.PopFront<uint16_t>();
      /* Record neighbors in argos */
      SNeighbor sNeighbor;
      sNeighbor.RId = unRobotId;
      sNeighbor.Distance = tPackets[i].Range;
      sNeighbor.Bearing = tPackets[i].HorizontalBearing;
      m_vecNeighbors.push_back(sNeighbor);
      /* Record neighbor to SwarmMesh */
      m_cMySM.AddNeighbor(unRobotId,
                          sNeighbor.Distance,
                          sNeighbor.Bearing.GetValue());
      /* Convert CByteArray to STL vector */
      std::vector<std::uint8_t> vecBuffer;
      while(cData.Size()) vecBuffer.push_back((uint8_t) cData.PopFront<uint8_t>());
      /* Process swarmmesh messages */
      size_t offset = 0;
      m_cMySM.Deserialize(vecBuffer, offset);
   }

}

/****************************************/
/****************************************/

void CSwarmMeshController::ProcessOutMsgs()
{

   std::vector<uint8_t> vecBuffer;

   /* Pre-pend robot id to any message*/
   swarmmesh::PackUInt16(vecBuffer, m_unRobotId);

   /* Fill buffer with swarmmesh messages */
   m_cMySM.Serialize(vecBuffer);

   /* Convert stl vector to CByteArray */
   CByteArray cBuffer;
   for (auto elem : vecBuffer) cBuffer << elem;

   /* Pad buffer with zeros to match fixed packet size */
   while(cBuffer.Size() < m_pcRABA->GetSize()) cBuffer << static_cast<UInt8>(0);

   m_pcRABA->SetData(cBuffer);

}

/****************************************/
/****************************************/

/* This statement notifies ARGoS of the existence of the controller. */
REGISTER_CONTROLLER(CSwarmMeshController, "swarmmesh_controller")
