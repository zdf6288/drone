// Generated by gencpp from file ardrone_autonomy/RecordEnableResponse.msg
// DO NOT EDIT!


#ifndef ARDRONE_AUTONOMY_MESSAGE_RECORDENABLERESPONSE_H
#define ARDRONE_AUTONOMY_MESSAGE_RECORDENABLERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct RecordEnableResponse_
{
  typedef RecordEnableResponse_<ContainerAllocator> Type;

  RecordEnableResponse_()
    : result(false)  {
    }
  RecordEnableResponse_(const ContainerAllocator& _alloc)
    : result(false)  {
  (void)_alloc;
    }



   typedef uint8_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RecordEnableResponse_

typedef ::ardrone_autonomy::RecordEnableResponse_<std::allocator<void> > RecordEnableResponse;

typedef boost::shared_ptr< ::ardrone_autonomy::RecordEnableResponse > RecordEnableResponsePtr;
typedef boost::shared_ptr< ::ardrone_autonomy::RecordEnableResponse const> RecordEnableResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator1> & lhs, const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator1> & lhs, const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb13ac1f1354ccecb7941ee8fa2192e8";
  }

  static const char* value(const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb13ac1f1354ccecULL;
  static const uint64_t static_value2 = 0xb7941ee8fa2192e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_autonomy/RecordEnableResponse";
  }

  static const char* value(const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool result\n"
;
  }

  static const char* value(const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RecordEnableResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_autonomy::RecordEnableResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AUTONOMY_MESSAGE_RECORDENABLERESPONSE_H
