#pragma once

#include <cinttypes>
#include <vector>
#include <string>
#include <stdexcept>
#include <sstream>
#include <functional>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <iostream>
#include <any>

const static uint16_t MEMORY_CAPACITY = 10;
const static uint16_t ROUTING_CAPACITY = 10;
const static uint8_t TUPLE_THROUGHPUT = 1;

namespace swarmmesh {

   /****************************************/
   /****************************************/

   struct SKey {

      /* Data hash 
      *  (prefix in paper) 
      */
      uint32_t Hash; 
      /* Tuple unique identifier 
      *  (suffix in paper) 
      */
      uint32_t Identifier; 

      /* Default constructor */
      SKey() {}
      
      /* Constructor */
      SKey(uint32_t un_hash, uint32_t un_identifier): 
      Hash(un_hash),
      Identifier(un_identifier) {}

      /* Copy operator */
      SKey& operator=(const SKey& s_key) {
         Hash = s_key.Hash;
         Identifier = s_key.Identifier;
         return *this;
      }
   };

   /****************************************/
   /****************************************/

   template<typename T>
   std::string MakeString(const std::string& str_sofar, T& t_arg) {
      std::ostringstream cOSS;
      cOSS << str_sofar << t_arg;
      return cOSS.str();
   }

   template<typename T, typename... Args>
   std::string MakeString(const std::string& str_sofar, T& t_arg, Args... t_rest) {
      std::ostringstream cOSS;
      cOSS << str_sofar << t_arg;
      return MakeString(cOSS.str(), t_rest...);
   }

   /****************************************/
   /****************************************/

   class CSwarmMeshException : std::exception {

   public:

      template<typename... Args>
      CSwarmMeshException(const std::string& str_what, Args... t_args) throw() :
         m_strWhat(MakeString(str_what, t_args...)) {
      }
      
      const char* what() const throw() override {
         return m_strWhat.c_str();
      }
      
   private:

      /**
       * The error message that explains what happened and why the exception was thrown.
       */
      std::string m_strWhat;
   };

   /****************************************/
   /****************************************/

   void PackUInt8(std::vector<uint8_t>& vec_buffer, uint8_t un_value);

   void PackUInt16(std::vector<uint8_t>& vec_buffer, uint16_t un_value);

   void PackUInt32(std::vector<uint8_t>& vec_buffer, uint32_t un_value);

   void PackFloat(std::vector<uint8_t>& vec_buffer, float f_value);

   void PackString(std::vector<uint8_t>& vec_buffer, std::string str_value);

   bool UnpackValid(const std::vector<uint8_t>& vec_buffer, size_t un_offset, size_t un_length);

   template<class T> T UnpackSerializable(T& c_obj,
                    const std::vector<uint8_t>& vec_buffer,
                    size_t& un_offset);

   uint8_t UnpackUInt8(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

   uint16_t UnpackUInt16(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

   uint32_t UnpackUInt32(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

   float UnpackFloat(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

   std::string UnpackString(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

   /****************************************/
   /****************************************/

   template< typename T, typename Pred >
   typename std::vector<T>::iterator insert_sorted( 
               std::vector<T> & vec, T const& item, Pred pred )
   {
   return vec.insert
      ( 
         std::lower_bound( vec.begin(), vec.end(), item, pred ),
         item 
      );
   }

   /****************************************/
   /****************************************/

   /**
   * The Serializable trait.
   *
   * This trait defines two abstract methods to serialize and deserialize an
   * object.
   */
   class CSerializable {
      
   public:

      /**
       * Serializes the object.
       *
       * Adds the serialized representation of this object to the given buffer.
       */
      virtual void Serialize(std::vector<uint8_t>& vec_buffer) = 0;

      /**
       * Deserializes the object.
       *
       * @param vec_buffer The byte buffer.
       * @param un_offset  Where to start deserializing in the buffer.
       * @return The new offset.
       */
      virtual size_t Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) = 0;

   };

   /****************************************/
   /****************************************/

   /**
    * The SwarmMesh class.
    */
   template <typename T>
   class CSwarmMesh : public CSerializable {

   public:

      void Init(uint16_t un_robot_id, std::function<SKey(T&)> fun_hash) {
         m_unRId = un_robot_id;
         m_funHash = fun_hash;
      }

   public:

      /**
       * The tuple stored by SwarmMesh.
       */
      struct STuple {

         swarmmesh::SKey Key;
         T Value;

         /* Copy operator */
         STuple& operator=(const STuple& s_tuple) {
            Key = s_tuple.Key;
            Value = s_tuple.Value;
            return *this;
         }

         /* Equality operator */
         bool operator==(const STuple& s_tuple) {
            return (Key.Identifier == s_tuple.Key.Identifier);
         }

      };

      /* Predicate for sorting tuples */
      struct TupleLess
      {
         bool operator()(const STuple& s_x, const STuple& s_y) const {
            if (s_x.Key.Hash < s_y.Key.Hash)
                  return true;
            else if (s_x.Key.Hash == s_y.Key.Hash)
                  return s_x.Key.Identifier < s_y.Key.Identifier;
            return false;
         }
      };

      /* Predicate for sorting tuples */
      struct TupleGreater
      {
         bool operator()(const STuple& s_x, const STuple& s_y) const {
            if (s_x.Key.Hash > s_y.Key.Hash)
               return true;
            else if (s_x.Key.Hash == s_y.Key.Hash)
               return s_x.Key.Identifier > s_y.Key.Identifier;
            return false;
         }
      };


   private:

      struct SNeighbor {

         float Distance;
         float Azimuth;
         uint32_t NodeId = 0;

         SNeighbor() {}

         SNeighbor(float f_distance, float f_azimuth):
         Distance(f_distance),
         Azimuth(f_azimuth){}

      };

   public:

      /****************************************/
      /****************************************/

      /**
       * A generic serializable operation.
       */
      class COperation : public CSerializable {

      public:

         COperation(CSwarmMesh* pc_sm) : m_pcSM(pc_sm) {}

         inline CSwarmMesh& SwarmMesh() { return *m_pcSM; }

         inline const CSwarmMesh& SwarmMesh() const { return *m_pcSM; }

      private:

         CSwarmMesh* m_pcSM;
      };

      /****************************************/
      /****************************************/

      /**
       * A functor that filters the tuples according to a predicate.
       */
      class CFilterOperation : public COperation {
      
      public:

         CFilterOperation(CSwarmMesh* pc_sm) : COperation(pc_sm) {}
         virtual ~CFilterOperation() {}

         /**
          * The application operator.
          * @param s_tuple The tuple to test.
          * @return true if the tuple passes the test, false otherwise.
          */
         virtual bool operator()(const STuple& s_tuple) = 0;

         /**
          * Initialize filter parameters.
          * @param map_filter_params Filter Parameters
          * @return void
          */
         virtual void Init(std::unordered_map<std::string, std::any>& map_filter_params) = 0;
      };

      /****************************************/
      /****************************************/

      /**
       * A functor that transforms a tuple into another tuple.
       */
      class CTupleOperation : public COperation {
      
      public:

         CTupleOperation(CSwarmMesh* pc_sm) : COperation(pc_sm) {}
         virtual ~CTupleOperation() {}

         /**
          * The application operator.
          * @param s_tuple The tuple to transform.
          * @return The transformed tuple.
          */
         virtual STuple operator()(const STuple& s_tuple) = 0;
      };

      /****************************************/
      /****************************************/

      /**
       * A functor that aggregates two tuples.
       */
      class CAggregateOperation : public COperation {
      
      public:

         CAggregateOperation(CSwarmMesh* pc_sm) : COperation(pc_sm) {}
         virtual ~CAggregateOperation() {}

         /**
          * The application operator.
          * @param t_accumulated The aggregated value so far.
          * @param s_current     The current tuple to aggregate.
          * @return The new aggregated tuple.
          */
         virtual T operator()(const T& t_aggregated,
                              const STuple& s_current) = 0;
      };

   /****************************************/
   /****************************************/

   public:

      CSwarmMesh(std::function < T(const std::vector<uint8_t>&, size_t&) > fun_unpack,
                 std::function < void(std::vector<uint8_t>&, const T&) > fun_pack) :
         m_funUnpack(fun_unpack),
         m_funPack(fun_pack) {
            /* initialize random seed: */
            srand(0);
         }
   
      virtual ~CSwarmMesh() {}

      inline std::vector<STuple>& StoredTuples() {
         return m_vecStoredTuples;
      }

      inline const std::vector<STuple>& StoredTuples() const {
         return m_vecStoredTuples;
      }

      inline std::vector<STuple>& RoutingTuples() {
         return m_vecRoutingTuples;
      }

      inline const std::vector<STuple>& RoutingTuples() const {
         return m_vecRoutingTuples;
      }

      /* Clear map of current neighbors*/
      void ResetNeighbors() {
         m_mapNeighbors.clear();
      }

      /* Add new neighbor */
      void AddNeighbor(uint16_t un_robot_id, float f_distance, float f_azimuth) {
         m_mapNeighbors[un_robot_id] = SNeighbor(f_distance, f_azimuth);
      }

      /* Compute Node ID 
      *  i.e partition key space
      */
      uint32_t Partition() {
         /* Get degree in communication graph */
         size_t unNumNeighbors = m_mapNeighbors.size();
         /* Get available storage memory */
         size_t unFreeMemory = MEMORY_CAPACITY - m_vecStoredTuples.size();
         /* Return node id given current neighbor list and storage memory */
         uint32_t unRes = (unNumNeighbors == 0) ? unFreeMemory : unFreeMemory * unNumNeighbors;
         return unRes;
      }

      /* Check if this tuple can be stored */
      bool IsStorableTuple(STuple s_tuple, uint32_t un_node_id) {
         /* Get degree in communication graph */
         size_t unNumNeighbors = m_mapNeighbors.size();
         /* Storable if node id higher than hash 
            if storing it */
         return s_tuple.Key.Hash < (un_node_id - unNumNeighbors);
      }

      void Serialize(std::vector<uint8_t>& vec_buffer) override {

         /* TODO: handle limited message size */

         /* Go through message queue */
         for (auto const& item : m_queueOutMsgs) {

            EMsgType eMsgType = item.first;
            auto rangeMsg = m_queueOutMsgs.equal_range(eMsgType);
            switch (eMsgType) {
            case MSG_NGHBRS:
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               /* Set robot Id */
               PackUInt16(vec_buffer, m_unRId);
               /* Set Node Id*/
               PackUInt32(vec_buffer, Partition());
               break;
            case MSG_OP_PUT:
               for (auto i = rangeMsg.first; i != rangeMsg.second; ++i)
               {
                  /* Set message type */
                  PackUInt8(vec_buffer, (uint8_t) eMsgType);
                  /* Get messages from map */
                  STuple sTuple = i->second.Msg;
                  /* Serialize */
                  m_funPack(vec_buffer, sTuple.Value);
               }
               break;
            case MSG_OP_ERASE:
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               break;
            case MSG_OP_FILTER:
               for (auto i = rangeMsg.first; i != rangeMsg.second; ++i) {
                  /* Set message type */
                  PackUInt8(vec_buffer, (uint8_t) eMsgType);
                  /* Set filter type */
                  uint16_t unFilterType = i->second.FilterType;
                  PackUInt16(vec_buffer, unFilterType);
                  /* Create filter with filter parameters */
                  CFilterOperation* pcFilterOp = m_cFilters.New(unFilterType);
                  try {
                     pcFilterOp->Init(i->second.FilterParameters);
                  } catch (const std::out_of_range& e) {
                     throw CSwarmMeshException("Filter initialization failed ", e.what());
                  }
                  /* Serialize filter */
                  pcFilterOp->Serialize(vec_buffer);
               }
               break;
            case MSG_OP_TUPLE:
               /* TODO */
               break;
            case MSG_OP_AGGREGATE:
               /* TODO */
               break;
            default:
               throw CSwarmMeshException("Unknown message type ", (int) eMsgType);
            }
         }
         /* TODO handle limited message size */
         m_queueOutMsgs.clear();
      }

      size_t Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) override {

         while(un_offset < vec_buffer.size()) {
            /* Get message type */
            uint8_t unMsgType = UnpackUInt8(vec_buffer, un_offset);
            switch(unMsgType) {
               case MSG_NGHBRS: {
                  /* Deserialize neighbor information */
                  uint16_t unRobotId = UnpackUInt16(vec_buffer, un_offset);
                  uint32_t unNodeId = UnpackUInt32(vec_buffer, un_offset);
                  /* Add received information about neighbor */
                  if(m_mapNeighbors.count(unRobotId)) {
                     m_mapNeighbors[unRobotId].NodeId = unNodeId;
                  }
                  break;
               }
               case MSG_OP_PUT: {
                  // return un_offset;
                  /* Create the tuple */
                  STuple sTuple;
                  sTuple.Value = m_funUnpack(vec_buffer, un_offset);
                  sTuple.Key = m_funHash(sTuple.Value);
                  /* Perform put operation */
                  DoPut(sTuple);
                  break;
               }
               case MSG_OP_ERASE: {
                  /* TODO */
                  break;
               }
               case MSG_OP_FILTER: {
                  /* Create the tuple filter */
                  uint16_t unOpType = UnpackUInt16(vec_buffer, un_offset);
                  CFilterOperation* pcFilterOp = m_cFilters.New(unOpType);
                  un_offset = pcFilterOp->Deserialize(vec_buffer, un_offset);
                  /* Go! */
                  std::vector<STuple> vecResult;
                  DoFilter(vecResult, *pcFilterOp, m_vecStoredTuples);
                  DoFilter(vecResult, *pcFilterOp, m_vecRoutingTuples);
                  // TODO do something with vecResult
                  delete pcFilterOp;
                  break;
               }
               case MSG_OP_TUPLE: {
                  /* Create the tuple operation */
                  uint16_t unOpType = UnpackUInt16(vec_buffer, un_offset);
                  CTupleOperation* pcTupleOp = m_cTupleOps.New(unOpType);
                  un_offset = pcTupleOp->Deserialize(vec_buffer, un_offset);
                  /* Create the tuple filter */
                  unOpType = UnpackUInt16(vec_buffer, un_offset);
                  CFilterOperation* pcFilterOp = m_cFilters.New(unOpType);
                  un_offset = pcFilterOp->Deserialize(vec_buffer, un_offset);
                  /* Go! */
                  std::vector<STuple> vecFiltered;
                  DoFilter(vecFiltered, *pcFilterOp, m_vecStoredTuples);
                  DoFilter(vecFiltered, *pcFilterOp, m_vecRoutingTuples);
                  std::vector<STuple> vecResult;
                  Exec(vecResult, *pcTupleOp, vecFiltered);
                  // TODO do something with vecResult
                  /* Cleanup */
                  delete pcTupleOp;
                  delete pcFilterOp;
                  break;
               }
               case MSG_OP_AGGREGATE: {
                  /* Create the tuple operation */
                  uint16_t unOpType = UnpackUInt16(vec_buffer, un_offset);
                  CAggregateOperation* pcAggregateOp = m_cAggregateOps.New(unOpType);
                  un_offset = pcAggregateOp->Deserialize(vec_buffer, un_offset);
                  /* Create the tuple filter */
                  unOpType = UnpackUInt16(vec_buffer, un_offset);
                  CFilterOperation* pcFilterOp = m_cFilters.New(unOpType);
                  un_offset = pcFilterOp->Deserialize(vec_buffer, un_offset);
                  /* Go! */
                  std::vector<STuple> vecFiltered;
                  DoFilter(vecFiltered, *pcFilterOp, m_vecStoredTuples);
                  DoFilter(vecFiltered, *pcFilterOp, m_vecRoutingTuples);
                  T tResult;
                  Exec(tResult, *pcAggregateOp, vecFiltered);
                  /* Cleanup */
                  delete pcAggregateOp;
                  delete pcFilterOp;
                  break;
               }
               default: {
                  return un_offset;
                  // throw CSwarmMeshException("Unknown message type ", unMsgType, " at offset ", un_offset);
               }
            } // switch(unMsgType)
         } // while(un_offset < vec_buffer.size())
         return un_offset;
      }

      /**
       * User-facing Filter function
       */
      void Filter(uint16_t un_type, std::unordered_map<std::string, std::any>& map_filter_params) {
         SMsg msg;
         msg.SetFilterParameters(un_type, map_filter_params);
         m_queueOutMsgs.insert(std::make_pair(MSG_OP_FILTER, msg));
      }

      /**
       * User-facing put function 
       */
      void Put(const T& t_data) {
         /* Convert user datatype to tuple 
            using user-defined pack/unpack 
            functions  */
         std::vector<uint8_t> vec_buffer;
         m_funPack(vec_buffer, t_data);
         /* Create tuple */
         STuple sTuple;
         size_t unOffset = 0;
         sTuple.Value = m_funUnpack(vec_buffer, unOffset);
         sTuple.Key = m_funHash(sTuple.Value);
         /* Call put operation on tuple */
         DoPut(sTuple);
      }
   


      /**
       * Queue messages to send to neighbors.
       */
      void Route() {
         /* Add base message to all neighbors */
         m_queueOutMsgs.insert(std::make_pair(MSG_NGHBRS, SMsg()));

         /* Route messages for put operation */
         uint16_t unCount = 0;
         while (!m_mapNeighbors.empty() &&
                !m_vecRoutingTuples.empty() && 
                 unCount < TUPLE_THROUGHPUT)
         {
            STuple sTuple = m_vecRoutingTuples.back();
            m_vecRoutingTuples.pop_back();
            /* Find candidates for receiving tuple */
            std::vector<uint16_t> vecCandidates;
            uint32_t unMax = 0;
            uint16_t unKeyMax = 0;
            for (auto const& item : m_mapNeighbors)
            {
               /* Record all neighbors that can hold tuple */
               if(item.second.NodeId > sTuple.Key.Hash)
                  vecCandidates.push_back(item.first);
               /* Record neighbor of maximum NodeId */
               if(item.second.NodeId > unMax)
               {
                  unMax = item.second.NodeId;
                  unKeyMax = item.first;
               }
            }
            if(!vecCandidates.empty())
            {
               /* Pick from candidates at random */
               size_t unIndex = rand() % vecCandidates.size();
               /* Put message in queue */
               m_queueOutMsgs.insert(std::make_pair(MSG_OP_PUT, SMsg(sTuple, vecCandidates[unIndex])));
            }
            else /* Pick neighbor with highest NodeId */
            {
               /* Put message in queue */
               m_queueOutMsgs.insert(std::make_pair(MSG_OP_PUT, SMsg(sTuple, unKeyMax)));
            }
            ++unCount;
         }
         /*TODO: Route other messages */
         // for (auto const& item : m_queueOutMsgs) {
         //    std::cout << " MESSAGE IN QUEUE " << (uint8_t) item.first << std::endl;
         // }

      }

      /****************************************/
      /****************************************/

   protected:

      /**
       * Registers a new tuple filter.
       */
      template<class C>
      uint16_t RegisterFilter(CSwarmMesh<T>* pc_sm) {
         return m_cFilters.Register([pc_sm]() -> CFilterOperation* { return new C(pc_sm); });
      }

      /**
       * Registers a new tuple operation.
       */
      template<class C>
      uint16_t RegisterTupleOperation(CSwarmMesh<T>* pc_sm) {
         return m_cTupleOps.Register([pc_sm]() -> CTupleOperation* { return new C(pc_sm); });
      }

      /**
       * Registers a new aggregate operation.
       */
      template<class C>
      uint16_t RegisterAggregateOperation(CSwarmMesh<T>* pc_sm) {
         return m_cAggregateOps.Register([pc_sm]() -> CAggregateOperation* { return new C(pc_sm); });
      }

   private:

      /**
       * Executes the given tuple operation on a list of tuples.
       * @param c_op       The operation.
       * @param vec_tuples The tuples on which the operation is applied.
       * @return A list of transformed tuple.
       */
      std::vector<STuple>& Exec(std::vector<STuple>& vec_result,
                                CTupleOperation& c_op,
                                const std::vector<STuple>& vec_tuples) {
         for(auto t_tuple : vec_tuples)
            vec_result.push_back(c_op(t_tuple));
         return vec_result;
      }

      /**
       * Executes the given aggregate operation on a list of tuples.
       * @param t_result   The aggregated result so far.
       * @param c_op       The operation.
       * @param vec_tuples The tuples on which the operation is applied.
       * @return The aggregated tuple.
       */
      T Exec(T& t_result,
             CAggregateOperation& c_op,
             const std::vector<STuple>& vec_tuples) {
         for(auto t_tuple : vec_tuples)
            t_result = c_op(t_result, t_tuple);
         return t_result;
      }

      /**
       * Filters the given tuples.
       *
       * @param vec_result The list of tuples that passed the filter so far.
       * @param vec_tuples The tuples to filter.
       * @param c_filter   The filter to apply.
       * @return The final list of tuples that pass the filter.
       */
      std::vector<STuple>& DoFilter(std::vector<STuple>& vec_result,
                                  CFilterOperation& c_filter,
                                  const std::vector<STuple>& vec_tuples) {
         for(auto t_tuple : vec_tuples) {
            if(c_filter(t_tuple))
               vec_result.push_back(t_tuple);
         }
         return vec_result;
      }

      /**
       * Put operation
       */
      void DoPut(const STuple& s_tuple) {
         
         /* Compute current Node Id */
         uint32_t unNodeId = Partition();
         /* Decide whether to store tuple or route it */
         bool bKeep = IsStorableTuple(s_tuple, unNodeId);
         if(bKeep) {
            /* Put tuple in storage queue, hash descending order */
            insert_sorted(m_vecStoredTuples, s_tuple, TupleGreater());
            uint32_t unHighestHash = (m_vecStoredTuples.front()).Key.Hash;
            /* Move unstorable tuples to routing queue */
            while((!m_vecStoredTuples.empty()) &&
                  (Partition() < unHighestHash || 
                  m_vecStoredTuples.size() > MEMORY_CAPACITY))
            {
               /* Put lowest hash tuple in routing queue, hash ascending order */
               insert_sorted(m_vecRoutingTuples, m_vecStoredTuples.back(), TupleLess());
               /* Remove tuple from stored tuples */
               m_vecStoredTuples.pop_back();
            }
         }
         else {
            /* Put tuple in routing queue, hash ascending order */
            insert_sorted(m_vecRoutingTuples, s_tuple, TupleLess());
         }

         /* Discard tuples if total size exceeded */
         while(m_vecStoredTuples.size() + m_vecRoutingTuples.size() > MEMORY_CAPACITY + ROUTING_CAPACITY)
         {
            /* Discard highest hash tuple */
            m_vecRoutingTuples.pop_back();
            /* TODO: add log message or buffer of discarded tuples */
         }
      }

   private:

      /**
       * A factory to register and create operations on tuples.
       */
      template<class C>
      class CFactory {
      
      private:

         /**
          * A function pointer to store the lambda that creates the operation.
          */
         typedef std::function<C*()> TCreator;

      public:

         /**
          * Registers a new operation.
          *
          * It does not check for duplicates internally.
          *
          * @param t_creator A lambda function that creates a new operation functor.
          * @return The id of the operation.
          */
         uint16_t Register(TCreator t_creator) {
            m_vecOps.push_back(t_creator);
            return m_vecOps.size() - 1;
         }

         /**
          * Creates a new operation functor.
          *
          * Internally checks that the given id is valid. If it's invalid, this
          * method returns nullptr.
          *
          * @param un_op The id of the operation.
          * @return A new operation functor.
          */
         C* New(uint16_t un_op) {
            if(un_op < m_vecOps.size()) return m_vecOps[un_op]();
            throw CSwarmMeshException("Invalid operation id ", un_op);
         }

      private:

         std::vector<TCreator> m_vecOps;
      
      };

   private:

      /**
       * Unique identifier
       */ 

      uint16_t m_unRId;

      /**
       * Map of current neighbors indexed by 
       */
      std::unordered_map<uint16_t, SNeighbor> m_mapNeighbors;

      /**
       * The queue of locally stored tuples
       * sorted by hash (low hash first out).
       */
      std::vector<STuple> m_vecStoredTuples;

      /**
       * The queue of tuples to route elsewhere
       * sorted by hash (high hash first out).
       */
      std::vector<STuple> m_vecRoutingTuples;


   private:

      enum EMsgType {
         MSG_NGHBRS = 1,
         MSG_OP_PUT,
         MSG_OP_ERASE,
         MSG_OP_FILTER,
         MSG_OP_TUPLE,
         MSG_OP_AGGREGATE,
         MSG_NUM
      };

      struct SMsg {
         
         uint16_t Destination;
         STuple Msg;

         /**
          * Map of filter parameters for the FILTER operation
          */
         std::unordered_map<std::string, std::any> FilterParameters;

         /**
          * Type of filter
          */
         uint16_t FilterType;
         /* Default Constructor */
         SMsg() {}
         /* Constructor */
         SMsg(STuple s_tuple, uint16_t un_id = 0):
            Msg(s_tuple),
            Destination(un_id) {}
         
         void SetFilterParameters(uint16_t un_filter_type, std::unordered_map<std::string, std::any>& un_filter_params) {
            FilterType = un_filter_type;
            FilterParameters = un_filter_params;
         }
      };

      /**
       * Message queue
       * Map ordered by message type
       */
      std::multimap<EMsgType, SMsg> m_queueOutMsgs;

      /**
       * The hash function.
       */
      std::function<SKey(T&)> m_funHash;

      /**
       * The value unpacking function.
       */
      std::function<T(const std::vector<uint8_t>&, size_t&)> m_funUnpack;

      /**
       * The value packing function.
       */
      std::function<void(std::vector<uint8_t>&, const T&)> m_funPack;

      /**
       * The factory of tuple operations.
       */
      CFactory<CFilterOperation> m_cFilters;

      /**
       * The factory of tuple operations.
       */
      CFactory<CTupleOperation> m_cTupleOps;

      /**
       * The factory of aggregate operations.
       */
      CFactory<CAggregateOperation> m_cAggregateOps;

   };

} 