#ifndef NE_F32_H
#define NE_F32_H

#include "ne_type.h"

typedef float f32;

#define NE_PI	(3.141592653589793238462643f)
#define NE_RAD_TO_DEG(A) ((f32)(((A) * (180.0f / NE_PI))))
#define NE_DEG_TO_RAD(A) ((f32)(((A) * (NE_PI / 180.0f))))
#define NE_RI			  NE_DEG_TO_RAD(1)
#define NE_ZERO (1.0e-6f)

#include <cmath>

NEINLINE f32 neAbs(f32 f)
{
    return std::abs(f);
}

NEINLINE f32 neSqrt(f32 f)
{
    return std::sqrt(f);
}

NEINLINE f32 neAtan2(f32 a, f32 b)
{
    return std::atan2(a, b);
}

NEINLINE f32 neAcos(f32 f)
{
    return std::acos(f);
}

NEINLINE f32 neSin(f32 f)
{
    return std::sin(f);
}

NEINLINE f32 neCos(f32 f)
{
    return std::cos(f);
}

NEINLINE neBool neFinite(f32 f)
{
    return std::isfinite(f);
}

NEINLINE neBool neRealsEqual(f32 s1, f32 s2)
{
    if ( (2.0f * neAbs( s1 - s2 ) / ( s1 + s2 )) < NE_ZERO )
    {
        return true;
    }

    return false;
}

//=========================================================================

NEINLINE neBool neIsConsiderZero(f32 f)
{
    return (neAbs(f) < NE_ZERO);
}

#endif // NE_F32_H
