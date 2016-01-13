#ifndef depthimage_to_laserscan_utils_h
#define depthimage_to_laserscan_utils_h

#include <ros/types.h>

namespace depthimage_to_laserscan
{
inline uint16_t rgb565ToUint16(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

} // depthimage_to_laserscan

#endif // depthimage_to_laserscan_utils_h
