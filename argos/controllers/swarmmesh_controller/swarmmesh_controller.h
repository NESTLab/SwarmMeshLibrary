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

#include <swarmmesh/swarmmesh.h>

#include <queue>
#include <sstream>

using namespace argos;

/* TODO: make this configurable */
const uint16_t BUCKET_SIZE = 10;
const Real BLOB_SENSOR_RANGE = 100;

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

   /* Structure representing events */
   struct SEventData {
      /* Type of event encoded as a color */
      std::string type;
      /* Value associated to the event */
      float payload;
      /* Spatial location of the event */
      std::pair<float, float> location;
   };

   /* Structure representing neighbors */
   struct SNeighbor
   {
      UInt16 RId;
      Real Distance;
      CRadians Bearing;
      bool operator==(const SNeighbor& x) {
            return (RId == x.RId);}
   };

   /* The robot numeric id */
   UInt16 m_unRobotId;

   /* Pointer to random number generator */
   CRandom::CRNG* m_pcRNG;

   /* Vector of neighbors */
   std::vector<SNeighbor> m_vecNeighbors;

   /* Data hashing function */
   class HashByType {// : public CSwarmMesh<SEventData>::Hash {

      virtual uint16_t operator() (SEventData s_event) const {

         std::string strColor = s_event.type;
         /* Hashing based on blob color */
         uint16_t unPrefix;
         if(strColor == "gray10") {unPrefix = 1;}
         else if(strColor == "white") {unPrefix = 1 + BUCKET_SIZE;}
         else if(strColor == "red") {unPrefix = 1 + 2 * BUCKET_SIZE;}
         else if(strColor  == "green") {unPrefix = 1 + 3 * BUCKET_SIZE;}
         else if(strColor  == "blue") {unPrefix = 1 + 4 * BUCKET_SIZE;}
         else if(strColor  == "magenta") {unPrefix = 1 + 5 * BUCKET_SIZE;}
         else if(strColor == "cyan") {unPrefix = 1 + 6 * BUCKET_SIZE;}
         else if(strColor  == "yellow") {unPrefix = 1 + 7 * BUCKET_SIZE;}
         else if(strColor  == "orange") {unPrefix = 1 + 8 * BUCKET_SIZE;}
         else if(strColor  == "brown") {unPrefix = 1 + 9 * BUCKET_SIZE;}
         else if(strColor  == "purple") {unPrefix = 1 + 10 * BUCKET_SIZE;}
         else if(strColor  ==  "gray50") {unPrefix = 1 + 11 * BUCKET_SIZE;}
         else  unPrefix = 0;
         
         return unPrefix;
      };

   };
   HashByType m_cHash;

   /* Key partitioning function by degree (D) and memory (M) */
   class PartitionDM {//: public CSwarmMesh<SEventData>::Partition {

      virtual uint16_t operator() (uint16_t un_degree, uint16_t un_mem) const{
         if(un_degree == 0) return un_mem;
         return un_degree * un_mem; 
      };
   
   };
   PartitionDM m_cPartition;

   /* Data structure object */
   // CSwarmMesh<SEventData> *m_cMesh = new CSwarmMesh<SEventData>(); 

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
