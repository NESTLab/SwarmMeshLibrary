#include "networking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <sstream>
#include <unistd.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

static const Real RANGE = 0.98;
static const Real RANGE_SQUARED = RANGE * RANGE;
static const CColor COLOR_TABLE[12] = {CColor::GRAY10,
                                       CColor::WHITE,
                                       CColor::RED,
                                       CColor::GREEN,
                                       CColor::BLUE,
                                       CColor::MAGENTA,
                                       CColor::CYAN,
                                       CColor::YELLOW,
                                       CColor::ORANGE,
                                       CColor::BROWN,
                                       CColor::PURPLE,
                                       CColor::GRAY50};
static const CRange<UInt32> COLOR_INDICES(0u, 12u);

/****************************************/
/****************************************/

CNetworkingLoopFunctions::CNetworkingLoopFunctions() :
   m_unBlobCounter(0), 
   m_cSpatialRange(0.0, 0.0), 
   m_unClock(0), 
   m_unStamp(0) {}

/****************************************/
/****************************************/

void CNetworkingLoopFunctions::Init(TConfigurationNode& t_node) {

   int nNumRobots; 
   Real fDensity, fRatioMI;

   /* Create the random number generator */
   m_pcRNG = CRandom::CreateRNG("argos");

   /* Parse the configuration file for parameters */
   TConfigurationNode& tNetworking = GetNode(t_node, "networking");
   GetNodeAttribute(tNetworking, "load_factor", m_fLoadFactor);
   GetNodeAttribute(tNetworking, "nb_robots", nNumRobots);
   GetNodeAttribute(tNetworking, "density", fDensity);
   GetNodeAttribute(tNetworking, "ratio_MI", fRatioMI);
   GetNodeAttribute(tNetworking, "lambda", m_fLambda);
   GetNodeAttribute(tNetworking, "min", m_fMin);
   GetNodeAttribute(tNetworking, "max", m_fMax);
   m_cSpatialRange.SetMin(m_fMin);
   m_cSpatialRange.SetMax(m_fMax);

   /* Parse the controller parameters */
   TConfigurationNode& tConfRoot = GetSimulator().GetConfigurationRoot();
   TConfigurationNode& tControllers = GetNode(tConfRoot, "controllers");
   TConfigurationNode& tController = GetNode(tControllers, "swarmmesh_controller");
   TConfigurationNode& tParams = GetNode(tController, "params");
   GetNodeAttribute(tParams, "storage", m_unStorageMemory);
   GetNodeAttribute(tParams, "routing", m_unRoutingMemory);
   m_unTotalMemory = m_unStorageMemory + m_unRoutingMemory;

   /* Get pointers to all robot entities and controllers*/
   CSpace::TMapPerType& cKheperas = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cKheperas.begin();
      it != cKheperas.end();
      ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity* cKhepera = any_cast<CFootBotEntity*>(it->second);
      CSwarmMeshController* cController = &dynamic_cast<CSwarmMeshController&>(cKhepera->GetControllableEntity().GetController());
      m_pcKheperas.push_back(cKhepera);
      m_pcControllers.push_back(cController);
   }

   m_unNumRobots = m_pcControllers.size(); 

}

/****************************************/
/****************************************/

void CNetworkingLoopFunctions::GenerateBlobs() {

   /* Select number of blobs to appear at this time step */
   UInt32 unCurNumBlobs = m_pcRNG->Poisson(m_fLambda);
   for(int i = 0; i < unCurNumBlobs; ++i)
   {
      ++m_unBlobCounter;
      UInt32 unColorIdx;
      Real fX = m_fMin-1; Real fY = m_fMin-1;
      Real fValue;

      unColorIdx = static_cast<UInt16>(m_pcRNG->Uniform(COLOR_INDICES));
      /* Generate position using a uniform distribution for x and y */
      while(fX < (m_fMin + 0.2) || fX > (m_fMax - 0.2) || 
            fY < (m_fMin + 0.2) || fY > (m_fMax - 0.2))
      {
         fX = m_pcRNG->Uniform(CRange<Real>(-RANGE, RANGE));
         fY = m_pcRNG->Uniform(CRange<Real>(-std::sqrt(RANGE_SQUARED - (fX * fX)), 
                                             std::sqrt(RANGE_SQUARED - (fX * fX))));
         /* Create the blob within range of at least one robot */
         UInt32 unRId = m_pcRNG->Uniform(CRange<UInt32>(1, m_unNumRobots + 1));
         //LOG << "robot " << unRId << std::endl;
         CFootBotEntity& fb = dynamic_cast<CFootBotEntity&>(
                                 GetSpace().GetEntity("K" + ToString(unRId)));
         CVector3 cPos = fb.GetEmbodiedEntity().GetOriginAnchor().Position;
         fX += cPos.GetX();
         fY += cPos.GetY();
      }
      /* Create blob */
      SBlob sBlob(m_unBlobCounter,
            COLOR_TABLE[unColorIdx],
            CVector3(fX, fY, 2*0.146899733f));
      /* Add blob to vector */
      m_vecBlobs.push_back(sBlob);
      //LOG << "Blob " << sBlob.Position << std::endl;
      CLightEntity* pcBlob = new CLightEntity((std::to_string(sBlob.Id)).c_str(),
                   sBlob.Position,
                   sBlob.Color,
                   1.0f);
      AddEntity(*pcBlob);
      CLEDMedium* m_pcLEDMedium = 
                     &CSimulator::GetInstance().GetMedium<CLEDMedium>("leds");
      m_pcLEDMedium->AddEntity(*pcBlob);
      m_pcBlobs.push_back(pcBlob);
      m_vecunTimes.push_back(m_unClock);
      pcBlob->Enable();
   }
}

/****************************************/
/****************************************/

bool CNetworkingLoopFunctions::IsExperimentFinished()
{
   bool bEndCond = (m_unBlobCounter >= m_fLoadFactor * m_unStorageMemory 
                  * m_unNumRobots);
   return bEndCond;
}

/****************************************/
/****************************************/

void CNetworkingLoopFunctions::RemoveBlobs(){
   size_t j = 0;
   for(size_t i = 0; i < m_pcBlobs.size(); ++i)
   {
      if(m_vecunTimes[j] < m_unClock)
      {
         m_pcBlobs[i]->Disable();
         m_pcBlobs[i]->MoveTo(CVector3(7.8, 7.8, 0),
            CQuaternion());
      }
      ++j;
   }
}

/****************************************/
/****************************************/

void CNetworkingLoopFunctions::Reset() {
   ;
}

/****************************************/
/****************************************/

void CNetworkingLoopFunctions::PreStep() {
   m_unClock = GetSpace().GetSimulationClock();
   GenerateBlobs();
}

/****************************************/
/****************************************/
void CNetworkingLoopFunctions::PostStep() {
   RemoveBlobs();
}

REGISTER_LOOP_FUNCTIONS(CNetworkingLoopFunctions, "networking_loop_functions")
