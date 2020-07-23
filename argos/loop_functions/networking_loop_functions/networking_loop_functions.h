#ifndef NETWORKING_LOOP_FUNCTIONS_H
#define NETWORKING_LOOP_FUNCTIONS_H

/* The controller */
#include <controllers/swarmmesh_controller/swarmmesh_controller.h>

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/light_entity.h>

using namespace argos;

class CNetworkingLoopFunctions : public CLoopFunctions {

public:

   struct SBlob{
      UInt16 Id;
      CColor Color;
      CVector3 Position;
      SBlob() {}
      SBlob(const UInt32& un_id,
            const CColor& c_color,
            const CVector3& c_pos) : Id(un_id),
                                 Color(c_color),
                                 Position(c_pos) {}
   };

public:

   CNetworkingLoopFunctions();
   virtual ~CNetworkingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PreStep();
   virtual void PostStep();

private:

   bool IsExperimentFinished();
   void GenerateBlobs();
   void RemoveBlobs();

private:

   /* Vector of pointers to the robot entities */
   std::vector<CFootBotEntity*> m_pcKheperas;
   /* Vector of pointers to the robot controllers */
   std::vector<CSwarmMeshController*> m_pcControllers;
   /* Pointer to random number generator */
   CRandom::CRNG* m_pcRNG;
   /* Vector of active blobs */
   std::vector<SBlob> m_vecBlobs;
   std::vector<CLightEntity*> m_pcBlobs;
   /* Counter of generated blobs */
   UInt16 m_unBlobCounter;
   /* Mean value of blob temporal distribution */
   Real m_fLambda;
   /* Range of blob spatial distribution */
   CRange<Real> m_cSpatialRange;

   std::vector<UInt16> m_vecunTimes;

   UInt16 m_unClock;

   UInt16 m_unNumRobots;

   Real m_fMin, m_fMax;

   UInt16 m_unStorageMemory, m_unRoutingMemory, m_unTotalMemory;

   Real m_fLoadFactor;

   UInt16 m_unStamp;
};

#endif
