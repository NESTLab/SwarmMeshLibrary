#pragma once

#include <cinttypes>
#include <vector>
#include <string>
#include <stdexcept>
#include <sstream>
#include <functional>

namespace swarmmesh {

   /****************************************/
   /****************************************/

   struct SKey {
      uint32_t Hash; // data hash, prefix in paper
      uint32_t Identifier; // unique tuple identifier, suffix in paper
      SKey() {}
      SKey(uint32_t h, uint32_t i): 
      Hash(h),
      Identifier(i) {}
   };

   /****************************************/
   /****************************************/

   // template<typename T> 
   // std::string MakeString(const std::string& str_sofar, T& t_arg);

   // template<typename T, typename... Args>
   // std::string MakeString(const std::string& str_sofar, T& t_arg, Args... t_rest);

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

      /**
       * The tuple stored by SwarmMesh.
       */
      struct STuple {
         swarmmesh::SKey Key;
         T Value;
      };

   public:

      /****************************************/
      /****************************************/

      // /**
      //  * Serializable neighbor class.
      //  */
      // class CNeighbors : public CSerializable {
         
      //    CNeighbors(CSwarmMesh* pc_sm) : m_pcSM(pc_sm) {}

      // private:

      //    CSwarmMesh* m_pcSM;
      // };


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

      class CPutOperation : public COperation {

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
          * @param t_tuple The tuple to test.
          * @return true if the tuple passes the test, false otherwise.
          */
         virtual bool operator()(const STuple& t_tuple) = 0;
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
          * @param t_tuple The tuple to transform.
          * @return The transformed tuple.
          */
         virtual STuple operator()(const STuple& t_tuple) = 0;
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
          * @param t_current     The current tuple to aggregate.
          * @return The new aggregated tuple.
          */
         virtual T operator()(const T& t_aggregated,
                              const STuple& t_current) = 0;
      };

      /****************************************/
      /****************************************/

   public:

      CSwarmMesh(std::function < T(const std::vector<uint8_t>&, size_t&) > fun_unpack,
                 std::function < SKey(T&) > fun_hash) :
         m_funHash(fun_hash),
         m_funUnpack(fun_unpack) {}
   
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

      void Serialize(std::vector<uint8_t>& vec_buffer) override {
         // TODO
      }

      size_t Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) override {
         
         while(un_offset < vec_buffer.size()) {
            /* Get message type */
            uint8_t unMsgType = UnpackUInt8(vec_buffer, un_offset);
            switch(unMsgType) {
               case MSG_NGHBRS: {
                  /* Create the neighbors information */
                  uint16_t unRobotId = UnpackUInt16(vec_buffer, un_offset);
                  uint32_t unNodeId = UnpackUInt16(vec_buffer, un_offset);
                  // SNeighbor sNeighbor(unRobotId, unNodeId);

                  break;
               }
               case MSG_OP_PUT: {
                  /* Create the tuple */
                  STuple sTuple;
                  sTuple.Value = m_funUnpack(vec_buffer, un_offset);
                  sTuple.Key = m_funHash(sTuple.Value);
                  /* Decide whether to store it or route it */
                  // TODO
                  break;
               }
               case MSG_OP_FILTER: {
                  /* Create the tuple filter */
                  uint16_t unOpType = UnpackUInt16(vec_buffer, un_offset);
                  CFilterOperation* pcFilterOp = m_cFilters.New(unOpType);
                  un_offset = pcFilterOp->Deserialize(vec_buffer, un_offset);
                  /* Go! */
                  std::vector<STuple> vecResult;
                  Filter(vecResult, *pcFilterOp, m_vecStoredTuples);
                  Filter(vecResult, *pcFilterOp, m_vecRoutingTuples);
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
                  Filter(vecFiltered, *pcFilterOp, m_vecStoredTuples);
                  Filter(vecFiltered, *pcFilterOp, m_vecRoutingTuples);
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
                  Filter(vecFiltered, *pcFilterOp, m_vecStoredTuples);
                  Filter(vecFiltered, *pcFilterOp, m_vecRoutingTuples);
                  T tResult;
                  Exec(tResult, *pcAggregateOp, vecFiltered);
                  /* Cleanup */
                  delete pcAggregateOp;
                  delete pcFilterOp;
                  break;
               }
               default: {
                  throw CSwarmMeshException("Unknown message type ", unMsgType, " at offset ", un_offset);
               }
            } // switch(unMsgType)
         } // while(un_offset < vec_buffer.size())
         return un_offset;
      }

      /**
       * Filters the given tuples.
       *
       * @param vec_result The list of tuples that passed the filter so far.
       * @param vec_tuples The tuples to filter.
       * @param c_filter   The filter to apply.
       * @return The final list of tuples that pass the filter.
       */
      std::vector<STuple>& Filter(std::vector<STuple>& vec_result,
                                  CFilterOperation& c_filter,
                                  const std::vector<STuple>& vec_tuples) {
         for(auto t_tuple : vec_tuples) {
            if(c_filter(t_tuple))
               vec_result.push_back(t_tuple);
         }
         return vec_result;
      }

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
       * 
       */
      // void Put(const T& t_data) {
      //    std::vector<uint8_t> vec_buffer;

      // }

      /**
       * Decides which tuples to store and which to route elsewhere.
       */
      virtual void Route() {
         // TODO
      }

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
       * The hash function.
       */
      std::function<SKey(T&)> m_funHash;

      /**
       * The value unpacking function.
       */
      std::function<T(const std::vector<uint8_t>&, size_t&)> m_funUnpack;

      /**
       * The list of locally stored tuples.
       */
      std::vector<STuple> m_vecStoredTuples;

      /**
       * The list of tuples to route elsewhere.
       */
      std::vector<STuple> m_vecRoutingTuples;

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

   private:

      enum EMsgType {
         MSG_NGHBRS = 0,
         MSG_OP_PUT,
         MSG_OP_FILTER,
         MSG_OP_TUPLE,
         MSG_OP_AGGREGATE,
         MSG_NUM
      };

   };

} 