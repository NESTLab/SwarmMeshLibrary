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
#include <deque>

const static uint16_t STORAGE_CAPACITY_DEFAULT = 10;
const static uint16_t ROUTING_CAPACITY_DEFAULT = 10;
const static uint16_t MESSAGE_LIMIT_DEFAULT = 1000;
const static uint16_t QUERY_MEMORY_DEFAULT = 100;

/* Make this configurable? */
const static uint16_t TUPLE_THROUGHPUT = 1;

namespace swarmmesh {

   /****************************************/
   /****************************************/

   struct SKey {

      /* Data hash 
      *  (prefix in paper) 
      */
      uint16_t Hash; 
      /* Tuple unique identifier 
      *  (suffix in paper) 
      */
      uint32_t Identifier; 

      /* Default constructor */
      SKey() = default;
      
      /* Constructor */
      SKey(uint16_t un_hash, uint32_t un_identifier): 
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

   /**
    * Converts the given object to string and appends it to the given string
    * @param std_sofar The given string
    * @param t_arg The object to be converted to string
    * @return The new string
    */
   template<typename T>
   std::string MakeString(const std::string& str_sofar, T& t_arg) {
      std::ostringstream cOSS;
      cOSS << str_sofar << t_arg;
      return cOSS.str();
   }

   /**
    * Converts the given objects to string and appends it to the given string
    * @param std_sofar The given string
    * @param t_arg
    * @param t_rest The objects to be converted to string
    * @return The new string
    */
   template<typename T, typename... Args>
   std::string MakeString(const std::string& str_sofar, T& t_arg, Args... t_rest) {
      std::ostringstream cOSS;
      cOSS << str_sofar << t_arg;
      return MakeString(cOSS.str(), t_rest...);
   }

   /****************************************/
   /****************************************/

   /**
    * The SwarmMesh Exception class
    * 
    */
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

   void PackUInt8(std::vector<uint8_t>& vec_buffer,
                  uint8_t un_value);

   void PackUInt16(std::vector<uint8_t>& vec_buffer,
                   uint16_t un_value);

   void PackUInt32(std::vector<uint8_t>& vec_buffer,
                   uint32_t un_value);

   void PackFloat(std::vector<uint8_t>& vec_buffer,
                  float f_value);

   void PackString(std::vector<uint8_t>& vec_buffer,
                   std::string str_value);

   bool UnpackValid(const std::vector<uint8_t>& vec_buffer,
                    size_t un_offset,
                    size_t un_length);

   template<class T> T UnpackSerializable(T& c_obj,
                                          const std::vector<uint8_t>& vec_buffer,
                                          size_t& un_offset);

   uint8_t UnpackUInt8(const std::vector<uint8_t>& vec_buffer,
                       size_t& un_offset);

   uint16_t UnpackUInt16(const std::vector<uint8_t>& vec_buffer,
                         size_t& un_offset);

   uint32_t UnpackUInt32(const std::vector<uint8_t>& vec_buffer,
                         size_t& un_offset);

   float UnpackFloat(const std::vector<uint8_t>& vec_buffer,
                     size_t& un_offset);

   std::string UnpackString(const std::vector<uint8_t>& vec_buffer,
                            size_t& un_offset);

   /****************************************/
   /****************************************/

   /**
    * Insert object into its sorted position in the vector based on the given sorting predicate
    * 
    * @tparam T 
    * @tparam Pred 
    * @param vec_buf The given vector of objects
    * @param t_item The object to be inserted
    * @param t_pred The sorting predicate
    * @return std::vector<T>::iterator 
    */
   template< typename T, typename Pred >
   typename std::vector<T>::iterator insert_sorted( 
      std::vector<T> & vec_buf, T const& t_item, Pred t_pred ) {
      return vec_buf.insert(std::lower_bound(
                           vec_buf.begin(),
                           vec_buf.end(),
                           t_item,
                           t_pred),
                        t_item);
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
       * @param vec_buffer The byte buffer
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

      /**
       * Initialize members of SwarmMesh
       * 
       * @param un_robotId The robot ID
       * @param fun_hash The Hashing function for the tuple key
       */
      void Init(uint16_t un_robotId, std::function<SKey(T&)> fun_hash, 
                uint16_t un_messageLimit = MESSAGE_LIMIT_DEFAULT, 
                uint16_t un_storageSize = STORAGE_CAPACITY_DEFAULT,
                uint16_t un_routingSize = ROUTING_CAPACITY_DEFAULT) {
         m_unRId = un_robotId;
         m_funHash = fun_hash;
         m_unMessageLimit = un_messageLimit;
         m_unStorageSize = un_storageSize;
         m_unRoutingSize = un_routingSize;
      }

   public:

      /**
       * The tuple stored by SwarmMesh.
       */
      struct STuple {

         SKey Key;
         T Value;

         STuple() = default;

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

      struct SQueryRequest {
         /**
          * Unique query identifier
          */
         uint32_t Identifier;
         uint16_t HopCount;
         
         uint16_t Source;
         
         /**
          * Map of filter parameters for the FILTER operation
          */
         std::unordered_map<std::string, std::any> FilterParameters;

         /**
          * Type of filter
          */
         uint8_t FilterType;

         SQueryRequest() = default;
         SQueryRequest(const SQueryRequest&) = default;

         /**
          * Construct a new SQueryRequest object
          * 
          * @param un_identifier 
          * @param map_filter_params 
          * @param un_filter_type 
          * @param un_source 
          * @param un_hop_count 
          */
         SQueryRequest(uint32_t un_identifier,
                       std::unordered_map<std::string, std::any>& map_filter_params, 
                       uint8_t un_filter_type,
                       uint16_t un_source,
                       uint16_t un_hop_count) : 
                     Identifier(un_identifier),
                     FilterParameters(map_filter_params),
                     FilterType(un_filter_type),
                     Source(un_source),
                     HopCount(un_hop_count) {}

         /**
          * @brief Override the assignment operator
          * 
          * @param s_request 
          * @return SQueryRequest& 
          */
         SQueryRequest& operator=(const SQueryRequest& s_request) {
            HopCount = s_request.HopCount;
            Source = s_request.Source;
            FilterType = s_request.FilterType;
            FilterParameters = s_request.FilterParameters;
            Identifier = s_request.Identifier;
            return *this;
         }
      };

      struct SQueryResponse {
         std::vector<STuple> Tuples;
         uint16_t Destination;
         uint16_t HopCount;
         uint32_t Identifier;

         SQueryResponse() = default;
         SQueryResponse(const SQueryResponse&) = default;

         /**
          * Construct a new SQueryResponse object
          * 
          * @param s_tuples 
          * @param un_dest 
          * @param un_hop_count 
          * @param un_identifier 
          */
         SQueryResponse(const std::vector<STuple>& s_tuples,
                        uint16_t un_dest,
                        uint16_t un_hop_count, 
                        uint32_t un_identifier) :
            Tuples(s_tuples),
            Destination(un_dest),
            HopCount(un_hop_count),
            Identifier(un_identifier) {}
      
         /**
          * Override the assignment operator
          * 
          * @param s_response 
          * @return SQueryResponse& 
          */
         SQueryResponse& operator=(const SQueryResponse& s_response) {
            Tuples = s_response.Tuples;
            Destination = s_response.Destination;
            return *this;
         }
      };  

      /* Predicate for sorting tuples */
      struct TupleLess {
         bool operator()(const STuple& s_x,
                         const STuple& s_y) const {
            if (s_x.Key.Hash < s_y.Key.Hash)
               return true;
            else if (s_x.Key.Hash == s_y.Key.Hash)
               return s_x.Key.Identifier < s_y.Key.Identifier;
            return false;
         }
      };

      /* Predicate for sorting tuples */
      struct TupleGreater {
         bool operator()(const STuple& s_x,
                         const STuple& s_y) const {
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
         uint16_t NodeId = 0;

         SNeighbor() {}

         /**
          * Construct a new SNeighbor object
          * 
          * @param f_distance 
          * @param f_azimuth 
          */
         SNeighbor(float f_distance,
                   float f_azimuth):
         Distance(f_distance),
         Azimuth(f_azimuth) {}

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
          * @param map_filterParams Filter Parameters
          */
         virtual void Init(const std::unordered_map<std::string,
                           std::any>& map_filterParams) = 0;
         
         /**
          * Get the Paramaters of the filter
          * 
          * @return std::unordered_map<std::string, std::any> 
          */
         virtual std::unordered_map<std::string, std::any> GetParams() = 0;
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

   /** User-facing functions **/

   public:

      CSwarmMesh(std::function <T(const std::vector<uint8_t>&, size_t&)> fun_unpack,
                 std::function <void(std::vector<uint8_t>&, const T&)> fun_pack) :
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

      inline std::deque<uint32_t>& QueryIds() {
         return m_bufQueryIds;
      } 

      inline const std::deque<uint32_t>& QueryIds() const {
         return m_bufQueryIds;
      } 

      inline std::unordered_map<uint32_t, std::vector<STuple>>& QueryResults() {
         return m_mapQueryResults;
      }

      inline const std::unordered_map<uint32_t, std::vector<STuple>>& QueryResults() const {
         return m_mapQueryResults;
      }

      /* Clear map of current neighbors*/
      void ResetNeighbors() {
         m_mapNeighbors.clear();
      }

      /**
       * Add new neighbor 
       * 
       * @param un_robotId The robot ID
       * @param f_distance Distance to robot
       * @param f_azimuth Azimuth angle
       * @return 
       */
      void AddNeighbor(uint16_t un_robotId, float f_distance, float f_azimuth) {
         m_mapNeighbors[un_robotId] = SNeighbor(f_distance, f_azimuth);
      }

      /**
       * Serialize all messages and store them in a byte buffer
       * 
       * @param vec_buffer The byte buffer
       */
      void Serialize(std::vector<uint8_t>& vec_buffer) override {

         /* Current message size */
         uint16_t unSize = 0;

         /* Previous message size */
         uint16_t unPrevSize;

         auto itMsgEnd = m_queueOutMsgs.begin();

         /* Go through message queue */
         for (; itMsgEnd != m_queueOutMsgs.end(); ++itMsgEnd) {
            
            /* Get item in message queue */
            auto const& item = *itMsgEnd;
            /* Update previous and current message size */
            unPrevSize = unSize;
            unSize = vec_buffer.size() * 8;
            /* Last message filled buffer */
            if(unSize == m_unMessageLimit){
               printf(" %d: Exceeded message size \n", m_unRId);
               break;
            } 
            /* Last message exceeded buffer */
            if (unSize > m_unMessageLimit)
            {
               printf(" %d: Exceeded message size \n", m_unRId);
               if((unSize - unPrevSize) >= m_unMessageLimit) printf("Message can never be sent \n");
               /* Remove previous message from buffer */
               int count = 0;
               while(count <  (unSize - unPrevSize) / 8)
               {
                  vec_buffer.pop_back();
                  ++count;
               }
               /* Update iterator to one after last sent message */
               --itMsgEnd;
               break;
            }
            EMsgType eMsgType = item.first;
            switch (eMsgType) {
            case MSG_NGHBRS:
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               /* Set robot Id */
               PackUInt16(vec_buffer, m_unRId);
               /* Set Node Id*/
               PackUInt16(vec_buffer, Partition());
               break;
            case MSG_OP_PUT: {
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               /* Pack destination */
               PackUInt16(vec_buffer, item.first);
               /* Get messages from map */
               SMsg sMsg = std::any_cast<SMsg>(item.second);
               STuple sTuple = sMsg.Msg;
               /* Serialize */
               m_funPack(vec_buffer, sTuple.Value);
               break;
            }
            case MSG_OP_ERASE: {
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               SQueryRequest sEraseRequest = std::any_cast<SQueryRequest>(item.second);
               /* Pack source */
               PackUInt16(vec_buffer, sEraseRequest.Source);
               /* Pack Query Identifier */
               PackUInt32(vec_buffer, sEraseRequest.Identifier);
               /* Set filter type */
               uint16_t unEraseFilterType = sEraseRequest.FilterType;
               PackUInt8(vec_buffer, unEraseFilterType);
               CFilterOperation* pcEraseOp = m_cFilters.New(unEraseFilterType);
               try {
                  pcEraseOp->Init(sEraseRequest.FilterParameters);
               } catch (const std::out_of_range& e) {
                  throw CSwarmMeshException("Filter initialization failed ", e.what());
               }
               /* Serialize filter */
               pcEraseOp->Serialize(vec_buffer);
               break;
            }
            case MSG_OP_FILTER_REQUEST: {
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               SQueryRequest sRequest = std::any_cast<SQueryRequest>(item.second);
               /* Pack source */
               PackUInt16(vec_buffer, sRequest.Source);
               /* Pack Query Identifier */
               PackUInt32(vec_buffer, sRequest.Identifier);
               /* Pack Hop Count */
               PackUInt16(vec_buffer, sRequest.HopCount);
               /* Set filter type */
               uint8_t unFilterType = sRequest.FilterType;
               PackUInt8(vec_buffer, unFilterType);
               /* Create filter with filter parameters */
               CFilterOperation* pcFilterOp = m_cFilters.New(unFilterType);
               try {
                  pcFilterOp->Init(sRequest.FilterParameters);
               } catch (const std::out_of_range& e) {
                  throw CSwarmMeshException("Filter initialization failed ", e.what());
               }
               /* Serialize filter */
               pcFilterOp->Serialize(vec_buffer);
               break;
            }
            case MSG_OP_FILTER_RESPONSE: {
               /* Set message type */
               PackUInt8(vec_buffer, (uint8_t) eMsgType);
               SQueryResponse sResponse = std::any_cast<SQueryResponse>(item.second);
               /* Set Query Identifier */
               PackUInt32(vec_buffer, sResponse.Identifier);
               /* Set HopCount */
               PackUInt16(vec_buffer, sResponse.HopCount);
               uint32_t unNumTuples = sResponse.Tuples.size();
               /* Set number of tuples */
               PackUInt8(vec_buffer, unNumTuples);
               /* Pack each tuple */
               for (STuple &sTuple: sResponse.Tuples) {
                  PackUInt16(vec_buffer, sTuple.Key.Hash);
                  PackUInt32(vec_buffer, sTuple.Key.Identifier);
                  m_funPack(vec_buffer, sTuple.Value);
               }
               break;
            }
            case MSG_OP_TUPLE:
               /* TODO m_queueOutMsgs.insert(std::make_pair(MSG_OP_FILTER, sMsg)); */
               break;
            case MSG_OP_AGGREGATE:
               /* TODO */
               break;
            default:
               throw CSwarmMeshException("Unknown message type ", (int) eMsgType);
            }
         }

         /* Clear the sent messages in the message queue */
         auto it = m_queueOutMsgs.begin();
         while (it != itMsgEnd){
            m_queueOutMsgs.erase(it++);
         }
         /* Print number of undelivered messages */
         if(m_queueOutMsgs.size() > 0)
         {
            printf("Queue size %lu \n", m_queueOutMsgs.size());
         }
      }

      /**
       * Deserialize the given byte buffer and retrieve all messages
       * 
       * @param vec_buffer The byte buffer
       * @param un_offset Initial offset
       * @return size_t Final offset after deserialization
       */
      size_t Deserialize(const std::vector<uint8_t>& vec_buffer,
                         size_t un_offset) override {
         while(un_offset < vec_buffer.size()) {
            /* Get message type */
            uint8_t unMsgType = UnpackUInt8(vec_buffer, un_offset);
            switch(unMsgType) {
               case MSG_NGHBRS: {
                  /* Deserialize neighbor information */
                  uint16_t unRobotId = UnpackUInt16(vec_buffer, un_offset);
                  uint16_t unNodeId = UnpackUInt16(vec_buffer, un_offset);
                  /* Add received information about neighbor */
                  if(m_mapNeighbors.count(unRobotId)) {
                     m_mapNeighbors[unRobotId].NodeId = unNodeId;
                  }
                  break;
               }
               case MSG_OP_PUT: {
                  // return un_offset;
                  /* Create the tuple */
                  /* Unpack destination */
                  uint16_t unDestination = UnpackUInt16(vec_buffer, un_offset);
                  STuple sTuple;
                  sTuple.Value = m_funUnpack(vec_buffer, un_offset);
                  sTuple.Key = m_funHash(sTuple.Value);
                  /* Perform put operation */
                  if (unDestination == m_unRId) {
                     DoPut(sTuple);
                  }
                  break;
               }
               case MSG_OP_ERASE: {
                  /* Unpack Source, Query Identifier */
                  uint16_t unSource = UnpackUInt16(vec_buffer, un_offset);
                  uint32_t unQueryId = UnpackUInt32(vec_buffer, un_offset);
                  /* Create the tuple filter */
                  uint8_t unOpType = UnpackUInt8(vec_buffer, un_offset);
                  CFilterOperation* pcFilterOp = m_cFilters.New(unOpType);
                  un_offset = pcFilterOp->Deserialize(vec_buffer, un_offset);

                  /* Erase request already received */
                  if (std::find(m_bufEraseRequests.begin(), 
                      m_bufEraseRequests.end(), unQueryId) != 
                      m_bufEraseRequests.end()) {
                     delete pcFilterOp;
                     break;
                  }

                  std::unordered_map<std::string, std::any> mapFilterParams = pcFilterOp->GetParams();
                  SQueryRequest sRequest(unQueryId, mapFilterParams, unOpType, unSource, 0);
                  /* Forward request on to neighbors */
                  m_queueOutMsgs.insert(std::make_pair(MSG_OP_ERASE, sRequest));
                  m_bufEraseRequests.push_back(unQueryId);

                  DoErase(*pcFilterOp, m_vecStoredTuples);
                  DoErase(*pcFilterOp, m_vecRoutingTuples);
                  delete pcFilterOp;
                  break;
               }
               case MSG_OP_FILTER_REQUEST: {
                  /* Unpack Source, Query Identifier, Hop Count */
                  uint16_t unSource = UnpackUInt16(vec_buffer, un_offset);
                  uint32_t unQueryId = UnpackUInt32(vec_buffer, un_offset);
                  uint16_t unHopCount = UnpackUInt16(vec_buffer, un_offset);
                  /* Create the tuple filter */
                  uint8_t unOpType = UnpackUInt8(vec_buffer, un_offset);
                  CFilterOperation* pcFilterOp = m_cFilters.New(unOpType);
                  un_offset = pcFilterOp->Deserialize(vec_buffer, un_offset);
                  /* Request already received */
                  if (m_mapQueryRequests.find(unQueryId) != m_mapQueryRequests.end()) {
                     delete pcFilterOp;
                     break;
                  }
                  m_mapQueryRequests[unQueryId] = std::make_pair(unHopCount, unSource);
                  std::unordered_map<std::string, std::any> mapFilterParams = pcFilterOp->GetParams();
                  SQueryRequest sRequest(unQueryId, mapFilterParams, unOpType, unSource, unHopCount + 1);
                  /* Forward request on to neighbors */
                  m_queueOutMsgs.insert(std::make_pair(MSG_OP_FILTER_REQUEST, sRequest));
                  /* Go! */
                  std::vector<STuple> vecResult;
                  DoFilter(vecResult, *pcFilterOp, m_vecStoredTuples);
                  DoFilter(vecResult, *pcFilterOp, m_vecRoutingTuples);
                  /* Send response only if there are tuples to be sent */
                  if (vecResult.size() > 0) {
                     SQueryResponse sResponse(vecResult, unSource, unHopCount, unQueryId);
                     m_queueOutMsgs.insert(std::make_pair(MSG_OP_FILTER_RESPONSE, sResponse));
                  }
                  delete pcFilterOp;
                  break;
               }
               case MSG_OP_FILTER_RESPONSE: {
                  uint32_t unQueryId = UnpackUInt32(vec_buffer, un_offset);
                  uint16_t unHopCount = UnpackUInt16(vec_buffer, un_offset);
                  uint8_t unNumTuples = UnpackUInt8(vec_buffer, un_offset);
                  for (uint8_t i = 0; i < unNumTuples; i++) {
                     STuple sTuple;
                     sTuple.Key = SKey(UnpackUInt16(vec_buffer, un_offset), UnpackUInt32(vec_buffer, un_offset));
                     sTuple.Value = m_funUnpack(vec_buffer, un_offset);
                     /* Do not route if response is received from node with lower hop count or if corresponding
                     request was not received */
                     if (m_mapQueryRequests.find(unQueryId) == m_mapQueryRequests.end() || 
                        m_mapQueryRequests[unQueryId].first >= unHopCount) {
                        continue;
                     }
                     /* Check if received tuple not received previously */
                     if (m_mapQueryResponses.count(unQueryId) <= 0 ||
                        m_mapQueryResponses[unQueryId].count(sTuple.Key.Identifier) <= 0) {
                        m_queueQueryResponses[unQueryId].push_back(sTuple);
                        m_mapQueryResponses[unQueryId][sTuple.Key.Identifier] = true;
                     }
                  }
                  break;
               }
               case MSG_OP_TUPLE: {
                  /* Create the tuple operation */
                  uint8_t unOpType = UnpackUInt8(vec_buffer, un_offset);
                  CTupleOperation* pcTupleOp = m_cTupleOps.New(unOpType);
                  un_offset = pcTupleOp->Deserialize(vec_buffer, un_offset);
                  /* Create the tuple filter */
                  unOpType = UnpackUInt8(vec_buffer, un_offset);
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
                  uint8_t unOpType = UnpackUInt8(vec_buffer, un_offset);
                  CAggregateOperation* pcAggregateOp = m_cAggregateOps.New(unOpType);
                  un_offset = pcAggregateOp->Deserialize(vec_buffer, un_offset);
                  /* Create the tuple filter */
                  unOpType = UnpackUInt8(vec_buffer, un_offset);
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
       * 
       * @param un_type Filter type
       * @param map_filter_params Filter Parameters
       */
      uint32_t Filter(uint8_t un_type,
                  std::unordered_map<std::string, std::any>& map_filter_params) {
         m_unQueryCount++;
         uint32_t unQueryId = ((uint32_t) m_unRId << 16) + m_unQueryCount;
         
         /* Add query id to circular buffer */
         m_bufQueryIds.push_back(unQueryId);
         if(m_bufQueryIds.size() > QUERY_MEMORY_DEFAULT) m_bufQueryIds.pop_front();

         uint16_t unHopCount = 0;
         m_mapQueryRequests[unQueryId] = std::make_pair(unHopCount, m_unRId);
         SQueryRequest sRequest(unQueryId, map_filter_params, un_type, m_unRId, unHopCount + 1);
         m_queueOutMsgs.insert(std::make_pair(MSG_OP_FILTER_REQUEST, sRequest));

         CFilterOperation* pcFilterOp = m_cFilters.New(un_type);
         try {
            pcFilterOp->Init(map_filter_params);
         } catch (const std::out_of_range& e) {
            throw CSwarmMeshException("Filter initialization failed ", e.what());
         }
         std::vector<STuple> vecResult;
         DoFilter(vecResult, *pcFilterOp, m_vecStoredTuples);
         DoFilter(vecResult, *pcFilterOp, m_vecRoutingTuples);
         m_mapQueryResults[unQueryId] = vecResult;
         return unQueryId;
      }

      /**
       * User-facing put function
       * 
       * @param t_data Data to be put into swarmmesh data structure
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
       * User-facing erase function
       * @param un_type Filter type
       * @param map_filter_params Filter Parameters
       */
      void Erase(uint8_t un_type,
                 std::unordered_map<std::string, std::any>& map_filter_params) {
         m_unQueryCount++;
         uint32_t unQueryId = ((uint32_t) m_unRId << 16) + m_unQueryCount;         

         m_bufEraseRequests.push_back(unQueryId);
         if(m_bufEraseRequests.size() > QUERY_MEMORY_DEFAULT) m_bufEraseRequests.pop_front();

         SQueryRequest sRequest(unQueryId, map_filter_params, un_type, m_unRId, 0);
         m_queueOutMsgs.insert(std::make_pair(MSG_OP_ERASE, sRequest));
         CFilterOperation* pcFilterOp = m_cFilters.New(un_type);
         try {
            pcFilterOp->Init(map_filter_params);
         } catch (const std::out_of_range& e) {
            throw CSwarmMeshException("Erase filter initialization failed ", e.what());
         }
         
         DoErase(*pcFilterOp, m_vecStoredTuples);
         DoErase(*pcFilterOp, m_vecRoutingTuples);
      }

      /**
       * Route messages to neighbors.
       */
      void Route() {
         /* Add base message to all neighbors */
         m_queueOutMsgs.insert(std::make_pair(MSG_NGHBRS, SMsg()));
         /* Route messages for put operation */
         uint16_t unCount = 0;
         while (!m_mapNeighbors.empty() &&
                !m_vecRoutingTuples.empty() && 
                 unCount < TUPLE_THROUGHPUT) {
            STuple sTuple = m_vecRoutingTuples.back();
            m_vecRoutingTuples.pop_back();
            /* Find candidates for receiving tuple */
            std::vector<uint16_t> vecCandidates;
            uint32_t unMax = 0;
            uint16_t unKeyMax = 0;
            for (auto const& item : m_mapNeighbors) {
               /* Record all neighbors that can hold tuple */
               if(item.second.NodeId > sTuple.Key.Hash)
                  vecCandidates.push_back(item.first);
               /* Record neighbor of maximum NodeId */
               if(item.second.NodeId > unMax) {
                  unMax = item.second.NodeId;
                  unKeyMax = item.first;
               }
            }
            if(!vecCandidates.empty()) {
               /* Pick from candidates at random */
               size_t unIndex = rand() % vecCandidates.size();
               /* Put message in queue */
               m_queueOutMsgs.insert(std::make_pair(MSG_OP_PUT, SMsg(sTuple, vecCandidates[unIndex])));
            }
            else { /* Pick neighbor with highest NodeId */
               /* Put message in queue */
               m_queueOutMsgs.insert(std::make_pair(MSG_OP_PUT, SMsg(sTuple, unKeyMax)));
            }
            ++unCount;
         }
         // Route MSG_OP_FILTER_RESPONSE messages
         for (auto const &item : m_queueQueryResponses) {
            uint32_t unQueryId = item.first;
            /* Destination robot reached */
            if (m_mapQueryRequests[unQueryId].second == m_unRId) {
               for (STuple sTuple : item.second) {
                  m_mapQueryResults[unQueryId].push_back(sTuple);
               }
               continue;
            }
            /* Forward response to all neighbors if tuples exist after removing duplicates */
            if (item.second.size() > 0) {
               SQueryResponse sResponse(item.second, m_mapQueryRequests[unQueryId].second, 
                        m_mapQueryRequests[unQueryId].first, unQueryId);
               m_queueOutMsgs.insert(std::make_pair(MSG_OP_FILTER_RESPONSE, sResponse));
            }
         }
         m_queueQueryResponses.clear();
      }

      /**
       * Compute node ID i.e partition key space
       * @return NodeID
       */
      uint16_t Partition() {
         /* Get degree in communication graph */
         size_t unNumNeighbors = m_mapNeighbors.size();
         /* Get available storage memory */
         size_t unFreeMemory = m_unStorageSize - m_vecStoredTuples.size();
         /* Return node id given current neighbor list and storage memory */
         return (unNumNeighbors == 0) ? unFreeMemory : unFreeMemory * unNumNeighbors;
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
       * Erases the given tuples that meet filter condition.
       *
       * @param vec_tuples The tuples to filter.
       * @param c_filter   The filter to apply.
       * @return The final list of tuples that pass the filter.
       */
      void DoErase(CFilterOperation& c_filter,
                   std::vector<STuple>& vec_tuples) {
         for(auto it = vec_tuples.begin(); 
                  it != vec_tuples.end(); ) {
            if(c_filter(*it))
               it = vec_tuples.erase(it);
            else it++; 
         }
      }

      /**
       * Put operation
       * 
       * @param s_tuple Tuple to be put into swarmmesh
       */
      void DoPut(const STuple& s_tuple) {
         /* Compute current Node Id */
         uint16_t unNodeId = Partition();
         /* Decide whether to store tuple or route it */
         bool bKeep = IsStorableTuple(s_tuple, unNodeId);
         if(bKeep) {
            /* Put tuple in storage queue, hash descending order */
            insert_sorted(m_vecStoredTuples, s_tuple, TupleGreater());
            uint16_t unHighestHash = (m_vecStoredTuples.front()).Key.Hash;
            /* Move unstorable tuples to routing queue */
            while((!m_vecStoredTuples.empty()) &&
                  (Partition() < unHighestHash || 
                  m_vecStoredTuples.size() > m_unStorageSize)) {
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
         while(m_vecStoredTuples.size() + m_vecRoutingTuples.size() >
               m_unStorageSize + m_unRoutingSize) {
            /* Discard highest hash tuple */
            m_vecRoutingTuples.pop_back();
            /* Log message or buffer of discarded tuples */
            printf("%d:Discarding tuple \n", m_unRId);
            // printf("Storage: %lu Routing: %lu \n", m_vecStoredTuples.size(), m_vecRoutingTuples.size());
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
         uint8_t Register(TCreator t_creator) {
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
         C* New(uint8_t un_op) {
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
       * Message size limit
       */
      uint16_t m_unMessageLimit;

      /**
       * Storage queue size (in tuple units)
       */ 
      uint16_t m_unStorageSize;

      /**
       * Routing queue size (in tuple units)
       */ 
      uint16_t m_unRoutingSize;

      /**
       * Query count
       */
      uint16_t m_unQueryCount = 0;

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

      /**
       * Circular buffer of queries emitted by the 
       * robot.
       */
      std::deque<uint32_t> m_bufQueryIds;

      /**
       * Circular buffer of erase queries received.
       */ 
      std::deque<uint32_t> m_bufEraseRequests;

      /**
       * The results for all queries sent by the robot
       */
      std::unordered_map<uint32_t, std::vector<STuple>> m_mapQueryResults;

      /**
       * Check if this tuple can be stored
       * 
       * @param s_tuple Tuple to be stored
       * @param un_nodeId Node ID of the robot
       * @return true if tuple can be stored
       * @return false if tuple cannot be stored
       */
      bool IsStorableTuple(STuple s_tuple, uint16_t un_nodeId) {
         /* Get degree in communication graph */
         size_t unNumNeighbors = m_mapNeighbors.size();
         // printf("Node id %lu , neighbors %lu \n", un_nodeId, unNumNeighbors);
         // printf("Trying to store tuple with hash %lu \n", s_tuple.Key.Hash);
         /* Storable if node id higher than hash 
            if storing it */
         return s_tuple.Key.Hash < (un_nodeId - unNumNeighbors);
      }

   private:

      enum EMsgType {
         MSG_NGHBRS = 1,
         MSG_OP_PUT,
         MSG_OP_ERASE,
         MSG_OP_FILTER_REQUEST,
         MSG_OP_FILTER_RESPONSE,
         MSG_OP_TUPLE,
         MSG_OP_AGGREGATE,
         MSG_NUM
      };

      struct SMsg {
         uint16_t Destination;
         STuple Msg;

         /* Default Constructor */
         SMsg() {}

         /**
          * Construct a new SMsg object
          * 
          * @param s_tuple 
          * @param un_id 
          */
         SMsg(const STuple& s_tuple, uint16_t un_id = 0):
            Msg(s_tuple),
            Destination(un_id) {}

         SMsg(const SMsg&) = default;
      };

      /**
       * Queue of response tuples to be forwarded ordered by query identifier
       */
      std::unordered_map<uint32_t, std::vector<STuple>> m_queueQueryResponses;

      /**
       * Map indicating query response tuples that have been routed already
       */
      std::unordered_map<uint32_t, std::unordered_map<uint32_t, bool>> m_mapQueryResponses;

      /** 
       * Map of queries received so far (Query Identifier -> (HopCount, Destination))
       */
      std::unordered_map<uint32_t, std::pair<uint16_t, uint16_t>> m_mapQueryRequests;

      /**
       * Message queue
       * Map ordered by message type
       */
      std::multimap<EMsgType, std::any> m_queueOutMsgs;

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
