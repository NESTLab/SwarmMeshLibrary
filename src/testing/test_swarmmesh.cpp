#include "swarmmesh/swarmmesh.h"
#include <iostream>

/****************************************/
/****************************************/

class CMySwarmMesh;
class CMyFilter;
class CMySum;

/****************************************/
/****************************************/

struct SMyDataType {
   uint32_t X, Y;
};

/****************************************/
/****************************************/

SMyDataType UnpackMyDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
   SMyDataType sValue;
   sValue.X = swarmmesh::UnpackUInt32(vec_buffer, un_offset);
   sValue.Y = swarmmesh::UnpackUInt32(vec_buffer, un_offset);
   return sValue;
}


void PackMyDataType(std::vector<uint8_t>& vec_buffer, const SMyDataType& s_data) {
   swarmmesh::PackUInt32(vec_buffer, s_data.X);
   swarmmesh::PackUInt32(vec_buffer, s_data.Y);
}

swarmmesh::SKey HashMyDataType1(SMyDataType& s_value) {
   return swarmmesh::SKey(s_value.X + s_value.Y, 0);
}

/****************************************/
/****************************************/

class CMySwarmMesh : public swarmmesh::CSwarmMesh<SMyDataType> {
   
public:
   
   CMySwarmMesh() :
      CSwarmMesh(UnpackMyDataType,
                 PackMyDataType) {
      RegisterFilter<CMyFilter>(this);
      RegisterAggregateOperation<CMySum>(this);
   }
   
   ~CMySwarmMesh() {
   }

};

/****************************************/
/****************************************/

class CMyFilter : public swarmmesh::CSwarmMesh<SMyDataType>::CFilterOperation {
public:
   CMyFilter(swarmmesh::CSwarmMesh<SMyDataType>* pc_sm) :
      swarmmesh::CSwarmMesh<SMyDataType>::CFilterOperation(pc_sm) {}
   ~CMyFilter() {}
   bool operator()(const swarmmesh::CSwarmMesh<SMyDataType>::STuple& s_tuple) override {
      return s_tuple.Value.X*s_tuple.Value.X + s_tuple.Value.Y*s_tuple.Value.Y < m_unSquareRange;
   }
   void Serialize(std::vector<uint8_t>& vec_buffer) override {
      swarmmesh::PackUInt32(vec_buffer, m_unSquareRange);
   }
   size_t Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) override {
      m_unSquareRange = swarmmesh::UnpackUInt32(vec_buffer, un_offset);
      return un_offset;
   }
   void Init(const std::unordered_map<std::string, std::any>& map_filterParams) override {
      m_unSquareRange = std::any_cast<uint32_t>(map_filterParams.at("range"));
   }
   std::unordered_map<std::string, std::any> GetParams() override {
      std::unordered_map<std::string, std::any> mapFilterParams;
      mapFilterParams["range"] = m_unSquareRange;
      return mapFilterParams;
   }

private:
   uint32_t m_unSquareRange;
};

/****************************************/
/****************************************/

class CMySum : public swarmmesh::CSwarmMesh<SMyDataType>::CAggregateOperation {
public:
   CMySum(swarmmesh::CSwarmMesh<SMyDataType>* pc_sm) :
      swarmmesh::CSwarmMesh<SMyDataType>::CAggregateOperation(pc_sm) {}
   ~CMySum() {}
   SMyDataType operator()(const SMyDataType& s_aggregated,
                          const swarmmesh::CSwarmMesh<SMyDataType>::STuple& s_current) override {
      /* Search for tuple */
      for(auto t : m_vecTuples) {
         /* Already considered? */
         if(t.Key.Hash == s_current.Key.Hash &&
            t.Value.X == s_current.Value.Y &&
            t.Value.X == s_current.Value.Y) {
            return s_aggregated;
         }
      }
      /* Add it to the list */
      m_vecTuples.push_back(s_current);
      /* Sum! */
      SMyDataType sResult;
      sResult.X = s_aggregated.X + s_current.Value.X;
      sResult.Y = s_aggregated.Y + s_current.Value.Y;
      return sResult;
   }
   void Serialize(std::vector<uint8_t>&) override {}
   size_t Deserialize(const std::vector<uint8_t>&, size_t un_offset) override {
      return un_offset;
   }
private:
   std::vector<swarmmesh::CSwarmMesh<SMyDataType>::STuple> m_vecTuples;
};

/****************************************/
/****************************************/

int main() {
   CMySwarmMesh cMySM;
   std::cout << "Test OK" << std::endl;
}
