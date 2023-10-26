#ifndef GAZEBOSENSORPLUGINS__VISIBILITY_CONTROL_H_
#define GAZEBOSENSORPLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GAZEBOSENSORPLUGINS_EXPORT __attribute__ ((dllexport))
    #define GAZEBOSENSORPLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define GAZEBOSENSORPLUGINS_EXPORT __declspec(dllexport)
    #define GAZEBOSENSORPLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GAZEBOSENSORPLUGINS_BUILDING_LIBRARY
    #define GAZEBOSENSORPLUGINS_PUBLIC GAZEBOSENSORPLUGINS_EXPORT
  #else
    #define GAZEBOSENSORPLUGINS_PUBLIC GAZEBOSENSORPLUGINS_IMPORT
  #endif
  #define GAZEBOSENSORPLUGINS_PUBLIC_TYPE GAZEBOSENSORPLUGINS_PUBLIC
  #define GAZEBOSENSORPLUGINS_LOCAL
#else
  #define GAZEBOSENSORPLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define GAZEBOSENSORPLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define GAZEBOSENSORPLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define GAZEBOSENSORPLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GAZEBOSENSORPLUGINS_PUBLIC
    #define GAZEBOSENSORPLUGINS_LOCAL
  #endif
  #define GAZEBOSENSORPLUGINS_PUBLIC_TYPE
#endif

#endif  // GAZEBOSENSORPLUGINS__VISIBILITY_CONTROL_H_
