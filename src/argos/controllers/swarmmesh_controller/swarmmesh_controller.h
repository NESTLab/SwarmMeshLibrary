#ifndef SWARMMESH_CONTROLLER_H
#define SWARMMESH_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>

#include "swarmmesh/swarmmesh.h"

#include <queue>
#include <sstream>
#include <any>
#include <typeinfo>
#include <typeindex>

/* TODO: make this configurable */
const uint16_t BUCKET_SIZE = 10;
const argos::Real BLOB_SENSOR_RANGE = 100;

/****************************************/
/****************************************/

class CMySwarmMesh;
class CSwarmMeshController;
class CHashEventDataType;
class CTypeFilter;
class CLocationFilter;
class CIdentifierFilter;
// class CMySum;

/****************************************/
/****************************************/

/* Structure representing location */
struct SLocation {
   float X;
   float Y;

   /* Default constructor */
   SLocation() {}

   /* Parameterized constructor */
   SLocation(float f_x, float f_y) : X(f_x), Y(f_y) {}

   /* Copy operator */
      SLocation& operator=(const SLocation& s_location) {
         X = s_location.X;
         Y = s_location.Y;
         return *this;
      }
};

/* Structure representing events */
struct SEventData {
   /* Type of event encoded as a color */
   std::string Type;
   /* Value associated to the event */
   float Payload;
   /* Spatial location of the event */
   SLocation Location;
};

/****************************************/
/****************************************/

SEventData UnpackEventDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_value);

// swarmmesh::SKey HashEventDataType(SEventData& s_value);

/****************************************/
/****************************************/

class CHashEventDataType {

   private:
   uint16_t m_unRobotId = 0;
   uint16_t m_unTupleCount = 0;
   

   public:
      CHashEventDataType() : 
         m_unRobotId(0),
         m_unTupleCount(0) {}

      void Init(uint16_t un_robot_id) {m_unRobotId = un_robot_id;}
      swarmmesh::SKey operator()(SEventData& s_value);
};

class CMySwarmMesh : public swarmmesh::CSwarmMesh<SEventData> {
private:
   CHashEventDataType m_cHashEvent;

public:
   CMySwarmMesh() :
      CSwarmMesh(UnpackEventDataType,
                 PackEventDataType) {
                    RegisterFilter<CTypeFilter>(this);
                    RegisterFilter<CLocationFilter>(this);
                    RegisterFilter<CIdentifierFilter>(this);
                 }
   void Init(uint16_t un_robot_id);
   
   ~CMySwarmMesh() {
   }

};

/****************************************/
/****************************************/
/* User Defined Filters */

class CTypeFilter : public swarmmesh::CSwarmMesh<SEventData>::CFilterOperation {
   private:
      std::string m_strEventType;
   public: 
      CTypeFilter(swarmmesh::CSwarmMesh<SEventData>* pc_sm) : 
         swarmmesh::CSwarmMesh<SEventData>::CFilterOperation(pc_sm) {}
      
      ~CTypeFilter() {}
      bool operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple&) override;
      void Serialize(std::vector<uint8_t>&) override;
      size_t Deserialize(const std::vector<uint8_t>&, size_t) override;
      void Init(const std::unordered_map<std::string, std::any>&) override;
      std::unordered_map<std::string, std::any> GetParams() override;
};

class CLocationFilter : public swarmmesh::CSwarmMesh<SEventData>::CFilterOperation {
   private:
      SLocation m_sEventLocation;
      float m_fRadius;
   public: 
      CLocationFilter(swarmmesh::CSwarmMesh<SEventData>* pc_sm) : 
         swarmmesh::CSwarmMesh<SEventData>::CFilterOperation(pc_sm) {}
      
      ~CLocationFilter() {}
      bool operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple&) override;
      void Serialize(std::vector<uint8_t>&) override;
      size_t Deserialize(const std::vector<uint8_t>&, size_t) override;
      void Init(const std::unordered_map<std::string, std::any>&) override;
      std::unordered_map<std::string, std::any> GetParams() override;
};

class CIdentifierFilter : public swarmmesh::CSwarmMesh<SEventData>::CFilterOperation {
   private:
      uint32_t m_unEventIdentifier;
   public: 
      CIdentifierFilter(swarmmesh::CSwarmMesh<SEventData>* pc_sm) : 
         swarmmesh::CSwarmMesh<SEventData>::CFilterOperation(pc_sm) {}
      
      ~CIdentifierFilter() {}
      bool operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple&) override;
      void Serialize(std::vector<uint8_t>&) override;
      size_t Deserialize(const std::vector<uint8_t>&, size_t) override;
      void Init(const std::unordered_map<std::string, std::any>&) override;
      std::unordered_map<std::string, std::any> GetParams() override;
};

/****************************************/
/****************************************/

using namespace argos;

class CSwarmMeshController : public CCI_Controller {

public:

   /* Class constructor. */
   CSwarmMeshController();

   /* Class destructor. */
   //virtual ~CSwarmMeshController() {}

   /*
    * This function initializes the controller.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    */
   virtual void Destroy() {}

/** 
 * Motion related members
 **/

public:

   /* This function implements diffuse motion */
   void Diffuse();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the differential steering actuator */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcBlobCamera;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the RAB sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the RAB actuator */
   CCI_RangeAndBearingActuator* m_pcRABA;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPositioning;

   /*
    * The following variables are used as parameters for the diffusion
    * algorithm.
    */
   /* Maximum tolerance for the angle between the robot heading 
      direction and the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
      the robot and the closest obstacle. */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

/**
 * Data management related members
 **/ 

private:

   /* Structure representing neighbors */
   struct SNeighbor
   {
      UInt16 RId;
      Real Distance;
      CRadians Bearing;
      bool operator==(const SNeighbor& x) {
            return (RId == x.RId);}
   };

   std::vector<SNeighbor> m_vecNeighbors;

   /* The robot numeric id */
   UInt16 m_unRobotId;

   /* Number of recorded events */
   UInt16 m_unTupleCount;

   /* Pointer to random number generator */
   CRandom::CRNG* m_pcRNG;

   /* Data structure object */
   CMySwarmMesh m_cMySM; 

public:

   /* Returns the list of events recorded by the robot
      at the current time step */
   std::queue<SEventData> RecordEvents();

   /* Returns world coordinates for a point given its coordinate relative 
   to this robot */
   CVector2 ComputeAbsolutePosition(const CVector2& c_coordTuple);

   void ProcessInMsgs();

   void ProcessOutMsgs();

};

#endif
