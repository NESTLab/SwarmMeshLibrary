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
                 PackMyDataType,
                 HashMyDataType1) {
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
   bool operator()(const swarmmesh::CSwarmMesh<SMyDataType>::STuple& t_tuple) override {
      return t_tuple.Value.X*t_tuple.Value.X + t_tuple.Value.Y*t_tuple.Value.Y < m_unSquareRange;
   }
   void Serialize(std::vector<uint8_t>& vec_buffer) override {
      swarmmesh::PackUInt32(vec_buffer, m_unSquareRange);
   }
   size_t Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) override {
      m_unSquareRange = swarmmesh::UnpackUInt32(vec_buffer, un_offset);
      return un_offset;
   }
   void SetThreshold(uint32_t un_square_range) {
      m_unSquareRange = un_square_range;
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
   SMyDataType operator()(const SMyDataType& t_aggregated,
                          const swarmmesh::CSwarmMesh<SMyDataType>::STuple& t_current) override {
      /* Search for tuple */
      for(auto t : m_vecTuples) {
         /* Already considered? */
         if(t.Key.Hash == t_current.Key.Hash &&
            t.Value.X == t_current.Value.Y &&
            t.Value.X == t_current.Value.Y) {
            return t_aggregated;
         }
      }
      /* Add it to the list */
      m_vecTuples.push_back(t_current);
      /* Sum! */
      SMyDataType sResult;
      sResult.X = t_aggregated.X + t_current.Value.X;
      sResult.Y = t_aggregated.Y + t_current.Value.Y;
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
