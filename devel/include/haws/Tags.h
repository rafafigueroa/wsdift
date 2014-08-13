/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/rafa/wsdift/src/haws/msg/Tags.msg
 *
 */


#ifndef HAWS_MESSAGE_TAGS_H
#define HAWS_MESSAGE_TAGS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <haws/Pose.h>

namespace haws
{
template <class ContainerAllocator>
struct Tags_
{
  typedef Tags_<ContainerAllocator> Type;

  Tags_()
    : do_nothing(0)
    , do_inputs(0)
    , do_current(0)
    , path_current()  {
    }
  Tags_(const ContainerAllocator& _alloc)
    : do_nothing(0)
    , do_inputs(0)
    , do_current(0)
    , path_current(_alloc)  {
    }



   typedef uint8_t _do_nothing_type;
  _do_nothing_type do_nothing;

   typedef uint8_t _do_inputs_type;
  _do_inputs_type do_inputs;

   typedef uint8_t _do_current_type;
  _do_current_type do_current;

   typedef std::vector< ::haws::Pose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::haws::Pose_<ContainerAllocator> >::other >  _path_current_type;
  _path_current_type path_current;




  typedef boost::shared_ptr< ::haws::Tags_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::haws::Tags_<ContainerAllocator> const> ConstPtr;

}; // struct Tags_

typedef ::haws::Tags_<std::allocator<void> > Tags;

typedef boost::shared_ptr< ::haws::Tags > TagsPtr;
typedef boost::shared_ptr< ::haws::Tags const> TagsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::haws::Tags_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::haws::Tags_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace haws

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'haws': ['/home/rafa/wsdift/src/haws/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::haws::Tags_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::haws::Tags_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Tags_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Tags_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Tags_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Tags_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::haws::Tags_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6d844e42d2aea2ef03da0c40d877b30f";
  }

  static const char* value(const ::haws::Tags_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6d844e42d2aea2efULL;
  static const uint64_t static_value2 = 0x03da0c40d877b30fULL;
};

template<class ContainerAllocator>
struct DataType< ::haws::Tags_<ContainerAllocator> >
{
  static const char* value()
  {
    return "haws/Tags";
  }

  static const char* value(const ::haws::Tags_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::haws::Tags_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 do_nothing\n\
uint8 do_inputs\n\
uint8 do_current\n\
haws/Pose[] path_current\n\
	\n\
\n\
================================================================================\n\
MSG: haws/Pose\n\
float32 x\n\
float32 y\n\
float32 theta\n\
\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
";
  }

  static const char* value(const ::haws::Tags_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::haws::Tags_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.do_nothing);
      stream.next(m.do_inputs);
      stream.next(m.do_current);
      stream.next(m.path_current);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Tags_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::haws::Tags_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::haws::Tags_<ContainerAllocator>& v)
  {
    s << indent << "do_nothing: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.do_nothing);
    s << indent << "do_inputs: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.do_inputs);
    s << indent << "do_current: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.do_current);
    s << indent << "path_current[]" << std::endl;
    for (size_t i = 0; i < v.path_current.size(); ++i)
    {
      s << indent << "  path_current[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::haws::Pose_<ContainerAllocator> >::stream(s, indent + "    ", v.path_current[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAWS_MESSAGE_TAGS_H