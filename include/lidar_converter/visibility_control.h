#ifndef LIDAR_CONVERTER__VISIBILITY_CONTROL_H_
#define LIDAR_CONVERTER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIDAR_CONVERTER_EXPORT __attribute__ ((dllexport))
    #define LIDAR_CONVERTER_IMPORT __attribute__ ((dllimport))
  #else
    #define LIDAR_CONVERTER_EXPORT __declspec(dllexport)
    #define LIDAR_CONVERTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIDAR_CONVERTER_BUILDING_DLL
    #define LIDAR_CONVERTER_PUBLIC LIDAR_CONVERTER_EXPORT
  #else
    #define LIDAR_CONVERTER_PUBLIC LIDAR_CONVERTER_IMPORT
  #endif
  #define LIDAR_CONVERTER_PUBLIC_TYPE LIDAR_CONVERTER_PUBLIC
  #define LIDAR_CONVERTER_LOCAL
#else
  #define LIDAR_CONVERTER_EXPORT __attribute__ ((visibility("default")))
  #define LIDAR_CONVERTER_IMPORT
  #if __GNUC__ >= 4
    #define LIDAR_CONVERTER_PUBLIC __attribute__ ((visibility("default")))
    #define LIDAR_CONVERTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIDAR_CONVERTER_PUBLIC
    #define LIDAR_CONVERTER_LOCAL
  #endif
  #define LIDAR_CONVERTER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_CONVERTER__VISIBILITY_CONTROL_H_