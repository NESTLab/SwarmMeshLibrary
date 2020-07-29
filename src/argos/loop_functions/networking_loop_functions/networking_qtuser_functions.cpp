#include "networking_qtuser_functions.h"

/****************************************/
/****************************************/

CNetworkingQTUserFunctions::CNetworkingQTUserFunctions() {
   RegisterUserFunction<CNetworkingQTUserFunctions,CFootBotEntity>(&CNetworkingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CNetworkingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   CSwarmMeshController& cController = dynamic_cast<CSwarmMeshController&>(c_entity.GetControllableEntity().GetController());
   // UInt32 unNodeKey = cController.GetNodeKey();
   // CKheperaNetworking::TStoringQueue tStoringQueue = cController.GetStoringQueue();
   // CKheperaNetworking::TRoutingQueue tRoutingQueue = cController.GetRoutingQueue();

   /* Draw RId */
   DrawText(CVector3(-0.1, 0, 0.1),   // position
            c_entity.GetId()); // text

   // /* Draw NodeKey */
   // CColor cColor;
   // UInt32 idx = unNodeKey / BUCKET_SIZE;
   // if (idx > 11) idx = 11;
   // cColor = COLOR_TABLE[idx];
   // DrawText(CVector3(-0.1, 0, 0.2),   // position
   //          (ToString(unNodeKey)).c_str(),
   //          cColor); // text

   // CVector3 cPos(0, 0, 0.3);
   // std::string strText;

   // CKheperaNetworking::STuple sTuple;
   // /* Draw Stored Tuples Queue */
   // while(!tStoringQueue.empty())
   // {
   //    sTuple = tStoringQueue.back();
   //    strText = "(" + ToString(sTuple.Key.Prefix) + "|" 
   //               + ToString(sTuple.Key.Suffix) + ", " 
   //               + ToString(sTuple.Value) + ")";
   //    cColor = COLOR_TABLE[(sTuple.Key.Prefix - 1)/BUCKET_SIZE];
   //    DrawText(cPos,
   //          strText.c_str(),
   //          cColor);
   //    cPos += CVector3(0.0, 0.0, 0.1);
   //    tStoringQueue.pop_back();
   // }
   // if(!tRoutingQueue.empty())
   // {
   //   DrawText(cPos,
   //            (ToString("_________________")).c_str(),
   //            cColor);
   //   cPos += CVector3(0.0, 0.0, 0.1);
   // }
   // /* Draw Routing Queue */
   // while(!tRoutingQueue.empty())
   // {
   //    sTuple = tRoutingQueue.back();
   //    strText = "(" + ToString(sTuple.Key.Prefix) + "|" 
   //               + ToString(sTuple.Key.Suffix) + ", " 
   //               + ToString(sTuple.Value) + ")";
   //    cColor = COLOR_TABLE[(sTuple.Key.Prefix - 1)/BUCKET_SIZE];
   //    DrawText(cPos,
   //          strText.c_str(),
   //          cColor);
   //    cPos += CVector3(0.0, 0.0, 0.1);
   //    tRoutingQueue.pop_back();
   // }

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CNetworkingQTUserFunctions, "networking_qtuser_functions")
