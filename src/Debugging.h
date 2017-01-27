#ifndef DEBUGGING_H
#define DEBUGGING_H

#include "WPILib.h"

#define TRACE_ENTRY 1

#if TRACE_ENTRY
#define ENTRY(s) { fprintf( stdout, "%s: %s, %d\n", s, __FUNCTION__, __LINE__ ); }
#else
#define ENTRY(s)
#endif

#define CHECK_NULL(p) if (!(p)) { fprintf( stderr, "NULL pointer: %s, %d\n", __FUNCTION__, __LINE__ ); }

extern int enableDoPeriodic;
// completes a certain stmt every period_count
#define DO_PERIODIC( period_count, stmt ) \
		if (enableDoPeriodic == 1) { \
			do { \
				static int __count__ = 0; \
				if ((period_count) > 0 && ( __count__++ % ( period_count )) == 0 ) { \
					stmt; \
				} \
			} while(0); \
		}

#define MATCH_PERIODIC( period_count, stmt ) do { static int __count__ = 0; if (( __count__++ % ( period_count )) == 0 ) { stmt; }} while(0);
// MATCH_PERIODIC used for periodic functions that always need to occur

#define USE_NAVX true
#define USE_CAMERA false
#define USE_USB_CAMERA false
#endif
