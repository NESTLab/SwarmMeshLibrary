#ifndef NETWORKING_QTUSER_FUNCTIONS_H
#define NETWORKING_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos/controllers/swarmmesh_controller/swarmmesh_controller.h>

using namespace argos;

class CNetworkingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CNetworkingQTUserFunctions();

   virtual ~CNetworkingQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);
   
};

#endif
