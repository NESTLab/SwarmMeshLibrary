#include "swarmmesh.h"

/****************************************/
/****************************************/

/**
 * Serialize uint8 value and append it to the given byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_value The given value
 */
void swarmmesh::PackUInt8(std::vector<uint8_t>& vec_buffer, uint8_t un_value) {
    vec_buffer.push_back(un_value);
}

 /**
 * Serialize uint16 value and append it to the given byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_value The given value
 */
void swarmmesh::PackUInt16(std::vector<uint8_t>& vec_buffer, uint16_t un_value) {
    vec_buffer.push_back(un_value >> 8);
    vec_buffer.push_back(un_value & 0xFF);
}
    
/**
 * Serialize uint32 value and append it to the given byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_value The given value
 */
void swarmmesh::PackUInt32(std::vector<uint8_t>& vec_buffer, uint32_t un_value) {
    vec_buffer.push_back(un_value >> 24);
    vec_buffer.push_back(un_value >> 16);
    vec_buffer.push_back(un_value >> 8);
    vec_buffer.push_back(un_value & 0x000000FF);
}

/**
 * Serialize float value and append it to the given byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_value The given value
 */  
void swarmmesh::PackFloat(std::vector<uint8_t>& vec_buffer, float f_value) {
    uint32_t unValue = *reinterpret_cast<uint32_t*>(&f_value); 
    swarmmesh::PackUInt32(vec_buffer, unValue);
}

/**
 * Serialize string and append it to the given byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_value The given string
 */
void swarmmesh::PackString(std::vector<uint8_t>& vec_buffer, std::string str_value) {
    vec_buffer.push_back(uint8_t(str_value.size()));
    for (auto it = str_value.begin(); it != str_value.end(); ++it)
    {
        vec_buffer.push_back((uint8_t)(*it));
    }
}

/****************************************/
/****************************************/

/**
 * Check if data can be unpacked from the byte buffer
 * 
 * @param vec_buffer The Byte buffer
 * @param un_offset Offset into the buffer
 * @param un_length Length of the data to be unpacked
 * @return true if data to be unpacked is valid
 * @return false if data to be unpacked is not valid
 */
bool swarmmesh::UnpackValid(const std::vector<uint8_t>& vec_buffer, size_t un_offset, size_t un_length) {
    return vec_buffer.size() >= un_offset + un_length;
}

/**
 * Unpack serializable object from the byte buffer
 * 
 * @tparam T 
 * @param c_obj Object data is unpacked into
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return T Unpacked object
 */
template<class T>
T swarmmesh::UnpackSerializable(T& c_obj,
                    const std::vector<uint8_t>& vec_buffer,
                    size_t& un_offset) {
    un_offset = c_obj.Deserialize(vec_buffer, un_offset);
    return c_obj;
}

/**
 * Unpack uint8 value from buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return uint8_t Unpacked value
 */
uint8_t swarmmesh::UnpackUInt8(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
    if(swarmmesh::UnpackValid(vec_buffer, un_offset, sizeof(uint8_t)))
        return vec_buffer[un_offset++];
    throw swarmmesh::CSwarmMeshException("Can't parse uint8_t at offset ", un_offset);
}

/**
 * Unpack uint16 value from buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return uint16_t Unpacked value
 */   
uint16_t swarmmesh::UnpackUInt16(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
    if(swarmmesh::UnpackValid(vec_buffer, un_offset, sizeof(uint16_t))) {
        uint16_t unReturn =
        vec_buffer[un_offset] << 8 |
        vec_buffer[un_offset+1];
        un_offset += sizeof(uint16_t);
        return unReturn;
    }
    throw swarmmesh::CSwarmMeshException("Can't parse uint16_t at offset ", un_offset);
}
    
/**
 * Unpack uint32 value from buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return uint32_t Unpacked value
 */
uint32_t swarmmesh::UnpackUInt32(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
    if(swarmmesh::UnpackValid(vec_buffer, un_offset, sizeof(uint32_t))) {
        uint32_t unReturn =
        vec_buffer[un_offset]   << 24 |
        vec_buffer[un_offset+1] << 16 |
        vec_buffer[un_offset+2] <<  8 |
        vec_buffer[un_offset+3];
        un_offset += sizeof(uint32_t);
        return unReturn;
    }
    throw CSwarmMeshException("Can't parse uint32_t at offset ", un_offset);
}

/**
 * Unpack float value from buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return float Unpacked value
 */
float swarmmesh::UnpackFloat(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {

    if(swarmmesh::UnpackValid(vec_buffer, un_offset, sizeof(float))) {
        uint32_t unTemp = 0;
        unTemp =
        vec_buffer[un_offset]   << 24 |
        vec_buffer[un_offset+1] << 16 |
        vec_buffer[un_offset+2] <<  8 |
        vec_buffer[un_offset+3];
        un_offset += sizeof(float);
        return *((float *) &unTemp);
    }
    throw swarmmesh::CSwarmMeshException("Can't parse float at offset ", un_offset);
}

/**
 * Unpack string from buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return std::string Unpacked string
 */
std::string swarmmesh::UnpackString(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
    uint8_t unLength;
    try {
        unLength = swarmmesh::UnpackUInt8(vec_buffer, un_offset);
    }
    catch(swarmmesh::CSwarmMeshException) {
        throw swarmmesh::CSwarmMeshException("Can't parse string length at offset ", un_offset);
    }
    if(swarmmesh::UnpackValid(vec_buffer, un_offset, unLength)) {
        std::string strReturn;
        for(size_t i = 0; i < unLength; ++i) {
          strReturn += vec_buffer[un_offset++];
        }
        return strReturn;
    }         
    throw swarmmesh::CSwarmMeshException("Can't parse string at offset ", un_offset);
}

/****************************************/
/****************************************/
