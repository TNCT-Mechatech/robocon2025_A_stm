#ifndef ENCODER_STRUCT_HPP_
#define ENCODER_STRUCT_HPP_

#include <Message.hpp>

typedef struct EncoderStructType
{
    float rpm_axis1;
    float rpm_axis2;
    int32_t pos_axis1;
    int32_t pos_axis2;

} encoder_struct_t;

//  create message
typedef sb::Message<encoder_struct_t> EncoderStruct;

#endif
