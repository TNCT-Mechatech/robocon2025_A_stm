#ifndef ARM_STRUCT_HPP_
#define ARM_STRUCT_HPP_

#include <Message.hpp>

typedef struct ArmStructType
{
    float axis1;
    float axit2;
} arm_struct_t;

//  create message
typedef sb::Message<arm_struct_t> ArmStruct;

#endif