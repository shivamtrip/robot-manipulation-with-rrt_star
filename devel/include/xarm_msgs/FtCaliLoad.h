// Generated by gencpp from file xarm_msgs/FtCaliLoad.msg
// DO NOT EDIT!


#ifndef XARM_MSGS_MESSAGE_FTCALILOAD_H
#define XARM_MSGS_MESSAGE_FTCALILOAD_H

#include <ros/service_traits.h>


#include <xarm_msgs/FtCaliLoadRequest.h>
#include <xarm_msgs/FtCaliLoadResponse.h>


namespace xarm_msgs
{

struct FtCaliLoad
{

typedef FtCaliLoadRequest Request;
typedef FtCaliLoadResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FtCaliLoad
} // namespace xarm_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::xarm_msgs::FtCaliLoad > {
  static const char* value()
  {
    return "002aec91323cfbf6a70a5f91f59ae0cf";
  }

  static const char* value(const ::xarm_msgs::FtCaliLoad&) { return value(); }
};

template<>
struct DataType< ::xarm_msgs::FtCaliLoad > {
  static const char* value()
  {
    return "xarm_msgs/FtCaliLoad";
  }

  static const char* value(const ::xarm_msgs::FtCaliLoad&) { return value(); }
};


// service_traits::MD5Sum< ::xarm_msgs::FtCaliLoadRequest> should match
// service_traits::MD5Sum< ::xarm_msgs::FtCaliLoad >
template<>
struct MD5Sum< ::xarm_msgs::FtCaliLoadRequest>
{
  static const char* value()
  {
    return MD5Sum< ::xarm_msgs::FtCaliLoad >::value();
  }
  static const char* value(const ::xarm_msgs::FtCaliLoadRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::xarm_msgs::FtCaliLoadRequest> should match
// service_traits::DataType< ::xarm_msgs::FtCaliLoad >
template<>
struct DataType< ::xarm_msgs::FtCaliLoadRequest>
{
  static const char* value()
  {
    return DataType< ::xarm_msgs::FtCaliLoad >::value();
  }
  static const char* value(const ::xarm_msgs::FtCaliLoadRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::xarm_msgs::FtCaliLoadResponse> should match
// service_traits::MD5Sum< ::xarm_msgs::FtCaliLoad >
template<>
struct MD5Sum< ::xarm_msgs::FtCaliLoadResponse>
{
  static const char* value()
  {
    return MD5Sum< ::xarm_msgs::FtCaliLoad >::value();
  }
  static const char* value(const ::xarm_msgs::FtCaliLoadResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::xarm_msgs::FtCaliLoadResponse> should match
// service_traits::DataType< ::xarm_msgs::FtCaliLoad >
template<>
struct DataType< ::xarm_msgs::FtCaliLoadResponse>
{
  static const char* value()
  {
    return DataType< ::xarm_msgs::FtCaliLoad >::value();
  }
  static const char* value(const ::xarm_msgs::FtCaliLoadResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // XARM_MSGS_MESSAGE_FTCALILOAD_H
