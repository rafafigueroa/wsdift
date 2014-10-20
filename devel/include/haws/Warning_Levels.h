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
 * Auto-generated by genmsg_cpp from file /home/rafa/wsdift/src/haws/msg/Warning_Levels.msg
 *
 */


#ifndef HAWS_MESSAGE_WARNING_LEVELS_H
#define HAWS_MESSAGE_WARNING_LEVELS_H


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
struct Warning_Levels_
{
  typedef Warning_Levels_<ContainerAllocator> Type;

  Warning_Levels_()
    : warning_levels()  {
    }
  Warning_Levels_(const ContainerAllocator& _alloc)
    : warning_levels(_alloc)  {
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _warning_levels_type;
  _warning_levels_type warning_levels;




  typedef boost::shared_ptr< ::haws::Warning_Levels_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::haws::Warning_Levels_<ContainerAllocator> const> ConstPtr;

}; // struct Warning_Levels_

typedef ::haws::Warning_Levels_<std::allocator<void> > Warning_Levels;

typedef boost::shared_ptr< ::haws::Warning_Levels > Warning_LevelsPtr;
typedef boost::shared_ptr< ::haws::Warning_Levels const> Warning_LevelsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::haws::Warning_Levels_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::haws::Warning_Levels_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::haws::Warning_Levels_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::haws::Warning_Levels_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Warning_Levels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Warning_Levels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Warning_Levels_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Warning_Levels_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::haws::Warning_Levels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d4c5c847950ed195bbc9a4965e3b91f3";
  }

  static const char* value(const ::haws::Warning_Levels_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd4c5c847950ed195ULL;
  static const uint64_t static_value2 = 0xbbc9a4965e3b91f3ULL;
};

template<class ContainerAllocator>
struct DataType< ::haws::Warning_Levels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "haws/Warning_Levels";
  }

  static const char* value(const ::haws::Warning_Levels_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::haws::Warning_Levels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] warning_levels\n\
";
  }

  static const char* value(const ::haws::Warning_Levels_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::haws::Warning_Levels_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.warning_levels);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Warning_Levels_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::haws::Warning_Levels_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::haws::Warning_Levels_<ContainerAllocator>& v)
  {
    s << indent << "warning_levels[]" << std::endl;
    for (size_t i = 0; i < v.warning_levels.size(); ++i)
    {
      s << indent << "  warning_levels[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.warning_levels[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAWS_MESSAGE_WARNING_LEVELS_H
