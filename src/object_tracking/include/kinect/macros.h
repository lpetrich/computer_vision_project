/* lpetrich 20/04/18 /**/

#ifndef MACROS_H_
#define MACROS_H_

#include <libgen.h>
#include <iostream>

// #define ASSERT_DEBUG
#define TRACE_DEBUG
// #define MARKER_DEBUG

#ifndef ASSERT_DEBUG
	#define ASSERT(x)
#else
	#define ASSERT(x) \
		if(!(x)) \
		{ \
			std::cout << "\nERROR!! Assert failed on line " << __LINE__  << " in file " << __FILE__ << "\n"; \
		}
#endif /* ASSERT_DEBUG */

#ifndef TRACE_DEBUG
	#define TRACE()
#else
	#define TRACE() \
		std::cout << __LINE__ << " << " << __FUNCTION__ << " (" << __FILE__ << ")\n"; \

#endif /* TRACE_DEBUG */

#endif /* MACROS_H_ */
