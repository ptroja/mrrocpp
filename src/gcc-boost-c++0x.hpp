/**
 * Modify Boost configuration to fix C++0x support.
 */

#include <boost/config/select_compiler_config.hpp>

#ifdef BOOST_HAS_RVALUE_REFS
#  undef BOOST_HAS_RVALUE_REFS
#endif
#ifndef BOOST_NO_RVALUE_REFERENCES
#  define BOOST_NO_RVALUE_REFERENCES
#endif
