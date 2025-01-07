// Generated by gencpp from file ardrone_as/ArdroneGoal.msg
// DO NOT EDIT!


#ifndef ARDRONE_AS_MESSAGE_ARDRONEGOAL_H
#define ARDRONE_AS_MESSAGE_ARDRONEGOAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ardrone_as
{
template <class ContainerAllocator>
struct ArdroneGoal_
{
  typedef ArdroneGoal_<ContainerAllocator> Type;

  ArdroneGoal_()
    : nseconds(0)  {
    }
  ArdroneGoal_(const ContainerAllocator& _alloc)
    : nseconds(0)  {
  (void)_alloc;
    }



   typedef int32_t _nseconds_type;
  _nseconds_type nseconds;





  typedef boost::shared_ptr< ::ardrone_as::ArdroneGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_as::ArdroneGoal_<ContainerAllocator> const> ConstPtr;

}; // struct ArdroneGoal_

typedef ::ardrone_as::ArdroneGoal_<std::allocator<void> > ArdroneGoal;

typedef boost::shared_ptr< ::ardrone_as::ArdroneGoal > ArdroneGoalPtr;
typedef boost::shared_ptr< ::ardrone_as::ArdroneGoal const> ArdroneGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_as::ArdroneGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ardrone_as::ArdroneGoal_<ContainerAllocator1> & lhs, const ::ardrone_as::ArdroneGoal_<ContainerAllocator2> & rhs)
{
  return lhs.nseconds == rhs.nseconds;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ardrone_as::ArdroneGoal_<ContainerAllocator1> & lhs, const ::ardrone_as::ArdroneGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ardrone_as

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_as::ArdroneGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_as::ArdroneGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_as::ArdroneGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d1469fb31a40224ea7d41afb312d107";
  }

  static const char* value(const ::ardrone_as::ArdroneGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d1469fb31a40224ULL;
  static const uint64_t static_value2 = 0xea7d41afb312d107ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_as/ArdroneGoal";
  }

  static const char* value(const ::ardrone_as::ArdroneGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#goal for the drone\n"
"int32 nseconds  # the number of seconds the drone will be taking pictures\n"
"\n"
;
  }

  static const char* value(const ::ardrone_as::ArdroneGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.nseconds);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArdroneGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_as::ArdroneGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_as::ArdroneGoal_<ContainerAllocator>& v)
  {
    s << indent << "nseconds: ";
    Printer<int32_t>::stream(s, indent + "  ", v.nseconds);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AS_MESSAGE_ARDRONEGOAL_H
