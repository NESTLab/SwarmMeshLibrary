#ifndef SWARMMESH_H
#define SWARMMESH_H

/*
 * Include some necessary headers.
 */
#include <algorithm>
#include <queue>   
#include <list> 
#include <map>
#include <unordered_map>
#include <functional>

template <typename TData> class CSwarmMesh{

    public :

    // class CReply {

    //     enum EState {
    //         WAITING,
    //         DONE
    //     };

    //     EState m_eState;

    // };

    // class CTupleReply : public CReply {

    //     std::vector<STuple<TData>*> m_vecTuples;

    // };

    // class CComputeReply : public CReply {

    //     TData m_tData;

    // };

    // class CQuery {
        
    //     uint32_t m_unId;
    //     virtual bool Match(STuple<TData>&) = 0;
    //     virtual std::vector<unsigned char> Serialize() = 0;
    //     virtual void Deserialize(const std::vector<unsigned char>&) = 0;

    // };

    // class CQueryByType : public CQuery{

    // };

    // class COperation {

    // };

    // // Key partitioning functor 
    // class Partition
    // { 
    //     public: 
    //         Partition() {}
        
    //         virtual uint16_t operator() () const;

    // }; 

    // // Data hashing functor
    // class Hash
    // {
    //     public:
    //         Hash() {}

    //         virtual uint16_t operator() () const;
    // };

    public:

    // Hash m_cHash;
    // Partition m_cPartition;

    CSwarmMesh(const Hash & c_hash, const Partition & c_partition):
    m_cHash(c_hash),
    m_cPartition(c_partition){}

    void Put(const TData& t_data);  // variant, templatized struct

    //std::future + internal state waiting/done. User cleans up when done, list of futures maintained by SwarmMesh
    // CTupleReply* Get(CQuery& c_query);
    // CComputeReply* Min(CQuery& c_query);
    // CComputeReply* Max(CQuery& c_query);
    // CComputeReply* Count(CQuery& c_query);
    // CComputeReply* Sum(CQuery& c_query);
    // CComputeReply* Average(CQuery& c_query);

    // void Erase(CQuery& c_query);

    // uint32_t RegisterQuery(CQueryByType& c_query);

    void ProcessInMsgs();

    void ProcessOutMsgs();

};


#endif
