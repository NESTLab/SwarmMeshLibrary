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

/* TODO: make this configurable */
const uint16_t BUCKET_SIZE = 10;
const argos::Real BLOB_SENSOR_RANGE = 100;

/****************************************/
/****************************************/

class CMySwarmMesh;
class CSwarmMeshController;
// class HashEventDataType;
// class CMyFilter;
// class CMySum;

/****************************************/
/****************************************/

/* Structure representing events */
struct SEventData {
   /* Type of event encoded as a color */
   std::string type;
   /* Value associated to the event */
   float payload;
   /* Spatial location of the event */
   std::pair<float, float> location;
};

/****************************************/
/****************************************/

SEventData UnpackEventDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_value);

swarmmesh::SKey HashEventDataType(SEventData& s_value);

/****************************************/
/****************************************/

class CMySwarmMesh : public swarmmesh::CSwarmMesh<SEventData> {
   
public:
   
   CMySwarmMesh() :
      CSwarmMesh(UnpackEventDataType,
                 PackEventDataType,
                 HashEventDataType) {}
   
   ~CMySwarmMesh() {
   }

};

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

public:

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

   /* Pointer to random number generator */
   CRandom::CRNG* m_pcRNG;

   /* Data structure object */
   CMySwarmMesh m_cMySM; 

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
