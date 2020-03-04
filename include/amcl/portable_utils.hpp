#ifndef PORTABLE_UTILS_H
#define PORTABLE_UTILS_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_Drand48
// Some system (e.g., Windows) doesn't come with Drand48(), Srand48().
// Use rand, and srand for such system.
static double Drand48(void)
{
    return ((double)rand())/RAND_MAX;
}

static void Srand48(long int seedval)
{
    srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif