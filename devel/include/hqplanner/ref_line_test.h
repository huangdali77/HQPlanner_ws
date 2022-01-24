// Generated by gencpp from file hqplanner/ref_line_test.msg
// DO NOT EDIT!


#ifndef HQPLANNER_MESSAGE_REF_LINE_TEST_H
#define HQPLANNER_MESSAGE_REF_LINE_TEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace hqplanner
{
template <class ContainerAllocator>
struct ref_line_test_
{
  typedef ref_line_test_<ContainerAllocator> Type;

  ref_line_test_()
    : path()  {
    }
  ref_line_test_(const ContainerAllocator& _alloc)
    : path(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _path_type;
  _path_type path;





  typedef boost::shared_ptr< ::hqplanner::ref_line_test_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hqplanner::ref_line_test_<ContainerAllocator> const> ConstPtr;

}; // struct ref_line_test_

typedef ::hqplanner::ref_line_test_<std::allocator<void> > ref_line_test;

typedef boost::shared_ptr< ::hqplanner::ref_line_test > ref_line_testPtr;
typedef boost::shared_ptr< ::hqplanner::ref_line_test const> ref_line_testConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hqplanner::ref_line_test_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hqplanner::ref_line_test_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hqplanner::ref_line_test_<ContainerAllocator1> & lhs, const ::hqplanner::ref_line_test_<ContainerAllocator2> & rhs)
{
  return lhs.path == rhs.path;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hqplanner::ref_line_test_<ContainerAllocator1> & lhs, const ::hqplanner::ref_line_test_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hqplanner

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hqplanner::ref_line_test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hqplanner::ref_line_test_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hqplanner::ref_line_test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hqplanner::ref_line_test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hqplanner::ref_line_test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hqplanner::ref_line_test_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hqplanner::ref_line_test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b58b29f4d3d5430fc9d5efc2f5262786";
  }

  static const char* value(const ::hqplanner::ref_line_test_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb58b29f4d3d5430fULL;
  static const uint64_t static_value2 = 0xc9d5efc2f5262786ULL;
};

template<class ContainerAllocator>
struct DataType< ::hqplanner::ref_line_test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hqplanner/ref_line_test";
  }

  static const char* value(const ::hqplanner::ref_line_test_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hqplanner::ref_line_test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point[] path\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::hqplanner::ref_line_test_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hqplanner::ref_line_test_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ref_line_test_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hqplanner::ref_line_test_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hqplanner::ref_line_test_<ContainerAllocator>& v)
  {
    s << indent << "path[]" << std::endl;
    for (size_t i = 0; i < v.path.size(); ++i)
    {
      s << indent << "  path[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.path[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HQPLANNER_MESSAGE_REF_LINE_TEST_H
