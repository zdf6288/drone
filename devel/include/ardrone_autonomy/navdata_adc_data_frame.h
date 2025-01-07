// Generated by gencpp from file ardrone_autonomy/navdata_adc_data_frame.msg
// DO NOT EDIT!


#ifndef ARDRONE_AUTONOMY_MESSAGE_NAVDATA_ADC_DATA_FRAME_H
#define ARDRONE_AUTONOMY_MESSAGE_NAVDATA_ADC_DATA_FRAME_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct navdata_adc_data_frame_
{
  typedef navdata_adc_data_frame_<ContainerAllocator> Type;

  navdata_adc_data_frame_()
    : header()
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , version(0)
    , data_frame()  {
    }
  navdata_adc_data_frame_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , version(0)
    , data_frame(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _drone_time_type;
  _drone_time_type drone_time;

   typedef uint16_t _tag_type;
  _tag_type tag;

   typedef uint16_t _size_type;
  _size_type size;

   typedef uint32_t _version_type;
  _version_type version;

   typedef std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> _data_frame_type;
  _data_frame_type data_frame;





  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> const> ConstPtr;

}; // struct navdata_adc_data_frame_

typedef ::ardrone_autonomy::navdata_adc_data_frame_<std::allocator<void> > navdata_adc_data_frame;

typedef boost::shared_ptr< ::ardrone_autonomy::navdata_adc_data_frame > navdata_adc_data_framePtr;
typedef boost::shared_ptr< ::ardrone_autonomy::navdata_adc_data_frame const> navdata_adc_data_frameConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator1> & lhs, const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.drone_time == rhs.drone_time &&
    lhs.tag == rhs.tag &&
    lhs.size == rhs.size &&
    lhs.version == rhs.version &&
    lhs.data_frame == rhs.data_frame;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator1> & lhs, const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "403dbf67522d768c3f509d9614c23941";
  }

  static const char* value(const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x403dbf67522d768cULL;
  static const uint64_t static_value2 = 0x3f509d9614c23941ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_autonomy/navdata_adc_data_frame";
  }

  static const char* value(const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 drone_time\n"
"uint16 tag\n"
"uint16 size\n"
"uint32 version\n"
"uint8[] data_frame\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.drone_time);
      stream.next(m.tag);
      stream.next(m.size);
      stream.next(m.version);
      stream.next(m.data_frame);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct navdata_adc_data_frame_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_autonomy::navdata_adc_data_frame_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "drone_time: ";
    Printer<double>::stream(s, indent + "  ", v.drone_time);
    s << indent << "tag: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tag);
    s << indent << "size: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.size);
    s << indent << "version: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.version);
    s << indent << "data_frame[]" << std::endl;
    for (size_t i = 0; i < v.data_frame.size(); ++i)
    {
      s << indent << "  data_frame[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data_frame[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AUTONOMY_MESSAGE_NAVDATA_ADC_DATA_FRAME_H
