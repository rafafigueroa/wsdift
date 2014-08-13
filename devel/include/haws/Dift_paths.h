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
 * Auto-generated by genmsg_cpp from file /home/rafa/wsdift/src/haws/msg/Dift_paths.msg
 *
 */


#ifndef HAWS_MESSAGE_DIFT_PATHS_H
#define HAWS_MESSAGE_DIFT_PATHS_H


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
struct Dift_paths_
{
  typedef Dift_paths_<ContainerAllocator> Type;

  Dift_paths_()
    : path_v()
    , path_w()
    , path_c()  {
    }
  Dift_paths_(const ContainerAllocator& _alloc)
    : path_v(_alloc)
    , path_w(_alloc)
    , path_c(_alloc)  {
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _path_v_type;
  _path_v_type path_v;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _path_w_type;
  _path_w_type path_w;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _path_c_type;
  _path_c_type path_c;




  typedef boost::shared_ptr< ::haws::Dift_paths_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::haws::Dift_paths_<ContainerAllocator> const> ConstPtr;

}; // struct Dift_paths_

typedef ::haws::Dift_paths_<std::allocator<void> > Dift_paths;

typedef boost::shared_ptr< ::haws::Dift_paths > Dift_pathsPtr;
typedef boost::shared_ptr< ::haws::Dift_paths const> Dift_pathsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::haws::Dift_paths_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::haws::Dift_paths_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace haws

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'haws': ['/home/rafa/wsdift/src/haws/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::haws::Dift_paths_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::haws::Dift_paths_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Dift_paths_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::haws::Dift_paths_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Dift_paths_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::haws::Dift_paths_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::haws::Dift_paths_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39a8534fd21d56d92f54ca2308fa2db8";
  }

  static const char* value(const ::haws::Dift_paths_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39a8534fd21d56d9ULL;
  static const uint64_t static_value2 = 0x2f54ca2308fa2db8ULL;
};

template<class ContainerAllocator>
struct DataType< ::haws::Dift_paths_<ContainerAllocator> >
{
  static const char* value()
  {
    return "haws/Dift_paths";
  }

  static const char* value(const ::haws::Dift_paths_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::haws::Dift_paths_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] path_v\n\
float32[] path_w\n\
float32[] path_c\n\
";
  }

  static const char* value(const ::haws::Dift_paths_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::haws::Dift_paths_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.path_v);
      stream.next(m.path_w);
      stream.next(m.path_c);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Dift_paths_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::haws::Dift_paths_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::haws::Dift_paths_<ContainerAllocator>& v)
  {
    s << indent << "path_v[]" << std::endl;
    for (size_t i = 0; i < v.path_v.size(); ++i)
    {
      s << indent << "  path_v[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.path_v[i]);
    }
    s << indent << "path_w[]" << std::endl;
    for (size_t i = 0; i < v.path_w.size(); ++i)
    {
      s << indent << "  path_w[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.path_w[i]);
    }
    s << indent << "path_c[]" << std::endl;
    for (size_t i = 0; i < v.path_c.size(); ++i)
    {
      s << indent << "  path_c[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.path_c[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HAWS_MESSAGE_DIFT_PATHS_H