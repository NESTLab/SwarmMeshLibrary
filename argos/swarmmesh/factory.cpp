#include "factory.h"
#include <stdexcept>

/****************************************/
/****************************************/

template<typename TYPE>
typename CFactory<TYPE>::TTypeMap& CFactory<TYPE>::GetTypeMap() {
   static typename CFactory<TYPE>::TTypeMap tTypeMap;
   return tTypeMap;
}

/****************************************/
/****************************************/

template<typename TYPE>
void CFactory<TYPE>::Register(const std::string& str_label,
                              const std::string& str_author,
                              TCreator* pc_creator) {
   typename CFactory<TYPE>::STypeInfo* psTypeInfo = new typename CFactory<TYPE>::STypeInfo;
   psTypeInfo->Author = str_author;
   psTypeInfo->Creator = pc_creator;
   GetTypeMap()[str_label] = psTypeInfo;
}

/****************************************/
/****************************************/

template<typename TYPE>
TYPE* CFactory<TYPE>::New(const std::string& str_label) {
   typename TTypeMap::iterator it = GetTypeMap().find(str_label);
   if(it != GetTypeMap().end()) {
      return it->second->Creator();
   }
   else {
      throw std:: invalid_argument( "Symbol \"" << str_label << "\" not found" );
   }
}

/****************************************/
/****************************************/

template<typename TYPE>
bool CFactory<TYPE>::Exists(const std::string& str_label) {
   typename TTypeMap::iterator it = GetTypeMap().find(str_label);
   return(it != GetTypeMap().end());
}

/****************************************/
/****************************************/

template<typename TYPE>
void CFactory<TYPE>::Print(std::ostream& c_os) {
   typename TTypeMap::iterator it;
   c_os << "Symbols:" << std::endl;
   for(it = GetTypeMap().begin();
       it != GetTypeMap().end();
       ++it) {
      c_os << it->first << " (" << it->second->BriefDescription << ")" << std::endl;
   }
}

/****************************************/
/****************************************/

template<typename TYPE>
void CFactory<TYPE>::Destroy() {
   typename TTypeMap::iterator it;
   for(it = GetTypeMap().begin();
       it != GetTypeMap().end();
       ++it) {
      delete it->second;
   }
   GetTypeMap().clear();
}

/****************************************/
/****************************************/

