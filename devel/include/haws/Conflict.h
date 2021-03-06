// Generated by gencpp from file haws/Conflict.msg
// DO NOT EDIT!


#ifndef HAWS_MESSAGE_CONFLICT_H
#define HAWS_MESSAGE_CONFLICT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace haws
{
template <class ContainerAllocator>
struct Conflict_
{
  typedef Conflict_<ContainerAllocator> Type;

  Conflict_()
    : avoid_activated(false)
    , tc(0.0)
    , gamma(0.0)  {
    }
  Conflict_(const ContainerAllocator& _alloc)
    : avoid_activated(false)
    , tc(0.0)
    , gamma(0.0)  {
    }



   typedef uint8_t _avoid_activated_type;
  _avoid_activated_type avoid_activated;

   typedef double _tc_type;
  _tc_type tc;

   typedef double _gamma_type;
  _gamma_type gamma;




  typedef boost::shared_ptr< ::haws::Conflict_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::haws::Conflict_<ContainerAllocator> const> ConstPtr;

}; // struct Conflict_

typedef ::haws::Conflict_<std::allocator<void> > Conflict;

typedef boost::shared_ptr< ::haws::Conflict > ConflictPtr;
typedef boost::shared_ptr< ::haws::Conflict const> ConflictConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::haws::Conflict_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::haws::Conflict_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace haws

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'haws': ['/home/rafa/wsdift/src/haws/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::haws::Conflict_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::haws::Conflict_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Conflict_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Conflict_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Conflict_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Conflict_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::haws::Conflict_<ContainerAllocator> >
{
  static const char* value()
  {
    return "01359ce202f254731c57b778944aa8d5";
  }

  static const char* value(const ::haws::Conflict_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x01359ce202f25473ULL;
  static const uint64_t static_value2 = 0x1c57b778944aa8d5ULL;
};

template<class ContainerAllocator>
struct DataType< ::haws::Conflict_<ContainerAllocator> >
{
  static const char* value()
  {
    return "haws/Conflict";
  }

  static const char* value(const ::haws::Conflict_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::haws::Conflict_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool avoid_activated\n\
float64 tc\n\
float64 gamma\n\
";
  }

  static const char* value(const ::haws::Conflict_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::haws::Conflict_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.avoid_activated);
      stream.next(m.tc);
      stream.next(m.gamma);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Conflict_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::haws::Conflict_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::haws::Conflict_<ContainerAllocator>& v)
  {
    s << indent << "avoid_activated: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.avoid_activated);
    s << indent << "tc: ";
    Printer<double>::stream(s, indent + "  ", v.tc);
    s << indent << "gamma: ";
    Printer<double>::stream(s, indent + "  ", v.gamma);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAWS_MESSAGE_CONFLICT_H
