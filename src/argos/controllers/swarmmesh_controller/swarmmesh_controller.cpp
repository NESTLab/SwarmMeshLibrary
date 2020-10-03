#include "swarmmesh_controller.h"

/****************************************/
/****************************************/

/**
 * Unpack SEventData object from the byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return SEventData The unpacked object
 */
SEventData UnpackEventDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
   SEventData sValue;
   sValue.Type = swarmmesh::UnpackString(vec_buffer, un_offset);
   sValue.Payload = swarmmesh::UnpackFloat(vec_buffer, un_offset);
   sValue.Location = {swarmmesh::UnpackFloat(vec_buffer, un_offset), 
                     swarmmesh::UnpackFloat(vec_buffer, un_offset)};
   return sValue;
}

/**
 * Serialize the given event data into the byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param s_event The event to be serialized
 */
void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_event) {
   swarmmesh::PackString(vec_buffer, s_event.Type);
   swarmmesh::PackFloat(vec_buffer, s_event.Payload);
   swarmmesh::PackFloat(vec_buffer, s_event.Location.X);
   swarmmesh::PackFloat(vec_buffer, s_event.Location.Y);
}

// swarmmesh::SKey HashEventDataType(SEventData& s_value) {
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
//       uint32_t unIdentifier = ((uint32_t) 1 << 16) +  3;
//       return swarmmesh::SKey(unHash, unIdentifier);
// }


swarmmesh::SKey CHashEventDataType::operator()(SEventData& s_value) {
   
   std::string strColor = s_value.Type;

   /* Data hashing based on blob color */
   uint16_t unHash;
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
   ++m_unTupleCount;
   uint32_t unIdentifier = ((uint32_t) m_unRobotId << 16) + m_unTupleCount;
   
   return swarmmesh::SKey(unHash, unIdentifier);
}

// /* Key partitioning function by degree (D) and memory (M) */
// class PartitionDM : public CSwarmMesh<SEventData>::Partition {

//    uint16_t operator() (uint16_t un_degree, uint16_t un_mem) const{
//       if(un_degree == 0) return un_mem;
//       return un_degree * un_mem; 
//    };

/****************************************/
/****************************************/

void CMySwarmMesh::Init(uint16_t un_robotId) {
   m_cHashEvent.Init(un_robotId);
   CSwarmMesh::Init(un_robotId, m_cHashEvent); 
}


/****************************************/
/****************************************/

std::unordered_map<std::string, std::any> CTypeFilter::GetParams() {
   std::unordered_map<std::string, std::any> mapFilterParams;
   mapFilterParams["type"] = m_strEventType;
   return mapFilterParams;
}

void CTypeFilter::Init(const std::unordered_map<std::string, std::any>& map_filterParams) {
   m_strEventType = std::any_cast<std::string>(map_filterParams.at("type"));
}

bool CTypeFilter::operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple& s_tuple) {
   return s_tuple.Value.Type == m_strEventType;
}

void CTypeFilter::Serialize(std::vector<uint8_t>& vec_buffer) {
   swarmmesh::PackString(vec_buffer, m_strEventType);
}

size_t CTypeFilter::Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) {
   m_strEventType = swarmmesh::UnpackString(vec_buffer, un_offset);
   return un_offset;
}

/****************************************/
/****************************************/

std::unordered_map<std::string, std::any> CLocationFilter::GetParams() {
   std::unordered_map<std::string, std::any> mapFilterParams;
   mapFilterParams["radius"] = m_fRadius;
   mapFilterParams["location"] = m_sEventLocation;
   return mapFilterParams;
}

void CLocationFilter::Init(const std::unordered_map<std::string, std::any>& map_filterParams) {
   m_fRadius = std::any_cast<float>(map_filterParams.at("radius"));
   m_sEventLocation = std::any_cast<SLocation>(map_filterParams.at("location"));
}

bool CLocationFilter::operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple& s_tuple) {
   CVector2 cEventCoord = CVector2(m_sEventLocation.X, m_sEventLocation.Y);
   CVector2 cTupleCoord = CVector2(s_tuple.Value.Location.X, s_tuple.Value.Location.Y);

   return SquareDistance(cEventCoord, cTupleCoord) <= m_fRadius;
}

void CLocationFilter::Serialize(std::vector<uint8_t>& vec_buffer) {
   swarmmesh::PackFloat(vec_buffer, m_fRadius);
   swarmmesh::PackFloat(vec_buffer, m_sEventLocation.X);
   swarmmesh::PackFloat(vec_buffer, m_sEventLocation.Y);
}

size_t CLocationFilter::Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) {
   m_fRadius = swarmmesh::UnpackFloat(vec_buffer, un_offset);
   m_sEventLocation = {swarmmesh::UnpackFloat(vec_buffer, un_offset), 
                        swarmmesh::UnpackFloat(vec_buffer, un_offset)};
   return un_offset;
}

/****************************************/
/****************************************/

std::unordered_map<std::string, std::any> CIdentifierFilter::GetParams() {
   std::unordered_map<std::string, std::any> mapFilterParams;
   mapFilterParams["identifier"] = m_unEventIdentifier;
   return mapFilterParams;
}

void CIdentifierFilter::Init(const std::unordered_map<std::string, std::any>& map_filterParams) {
   m_unEventIdentifier = std::any_cast<uint32_t>(map_filterParams.at("identifier"));
}

bool CIdentifierFilter::operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple& s_tuple) {
   return m_unEventIdentifier == s_tuple.Key.Identifier;
}

void CIdentifierFilter::Serialize(std::vector<uint8_t>& vec_buffer) {
   swarmmesh::PackUInt32(vec_buffer, m_unEventIdentifier);
}

size_t CIdentifierFilter::Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) {
   m_unEventIdentifier = swarmmesh::UnpackUInt32(vec_buffer, un_offset);

   return un_offset;
}

/****************************************/
/****************************************/

/**
 * Construct a new CSwarmMeshController::CSwarmMeshController object
 * 
 */
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

/**
 * Movement of robots
 * 
 */
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
   std::queue<SEventData> queueEvents = RecordEvents();   
   while(!queueEvents.empty())
   {
      /* Retrieve event to write in SwarmMesh */
      SEventData sEvent = queueEvents.front();
      queueEvents.pop();
      ++m_unTupleCount;
      /* Perform a put operation in SwarmMesh */
      m_cMySM.Put(sEvent);
   }

   /* Randomly add Filter messages */
   float fP = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
   if (fP < 0.02) {
      /* Filter type */
      int nFilterType = m_pcRNG->Uniform(CRange<UInt32>(0, 3));
      std::unordered_map<std::string, std::any> mapFilterParams;

      /* Assign parameters based on type */
      switch (nFilterType) {
      case 0:
         mapFilterParams["type"] = std::string("blue");
         break;
      case 1: {
         float fRadius = 4.5f;
         SLocation sLocation(10.0f, 11.0f);
         mapFilterParams["radius"] = fRadius;
         mapFilterParams["location"] = sLocation;
         break;
      }
      case 2:
         uint32_t unIdentifier = 32768;
         mapFilterParams["identifier"] = unIdentifier;
         break;
      
      // default: throw THROW_ARGOSEXCEPTION("Invalid filter type : ");
         // break;
      }
      m_cMySM.Filter((uint8_t) nFilterType, mapFilterParams);
   }

   /* Tell SwarmMesh to queue messages for routing data */
   m_cMySM.Route();

   /* Process outgoing messages */
   ProcessOutMsgs();

}

/****************************************/
/****************************************/

/**
 * Record all events in the current step
 * 
 * @return std::queue<SEventData> Queue of all recorded events
 */
std::queue<SEventData> CSwarmMeshController::RecordEvents() 
{
   std::queue<SEventData> queueEvents;

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
      else {
         bLocalLeader = false;
      }
      /* If leader, record the event */
      if (bLocalLeader) {
         SEventData sEvent;
         std::ostringstream stream;
         stream  << tBlobReads.BlobList[i]->Color;
         sEvent.Type = stream.str();
         sEvent.Payload = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
         CVector2 cPos = ComputeAbsolutePosition(cEventCoord);
         sEvent.Location = {static_cast<float>(cPos.GetX()), static_cast<float>(cPos.GetY())};
         queueEvents.push(sEvent);
      }
   }
   return queueEvents;
}

/****************************************/
/****************************************/

/**
 * Get absolute coordinates of event
 * 
 * @param c_coordEvent The relative coordinates of the event
 * @return CVector2 Absolute coordinates of the event
 */
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

/**
 * Process incoming messages received through the Range and Bearing Sensor
 * 
 */
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
      size_t unOffset = 0;
      m_cMySM.Deserialize(vecBuffer, unOffset);
   }

}

/****************************************/
/****************************************/

/**
 * Send outgoing messages through the range and bearing sensor
 * 
 */
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
