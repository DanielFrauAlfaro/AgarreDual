// Generated by gencpp from file ur_msgs/MasterboardDataMsg.msg
// DO NOT EDIT!


#ifndef UR_MSGS_MESSAGE_MASTERBOARDDATAMSG_H
#define UR_MSGS_MESSAGE_MASTERBOARDDATAMSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ur_msgs
{
template <class ContainerAllocator>
struct MasterboardDataMsg_
{
  typedef MasterboardDataMsg_<ContainerAllocator> Type;

  MasterboardDataMsg_()
    : digital_input_bits(0)
    , digital_output_bits(0)
    , analog_input_range0(0)
    , analog_input_range1(0)
    , analog_input0(0.0)
    , analog_input1(0.0)
    , analog_output_domain0(0)
    , analog_output_domain1(0)
    , analog_output0(0.0)
    , analog_output1(0.0)
    , masterboard_temperature(0.0)
    , robot_voltage_48V(0.0)
    , robot_current(0.0)
    , master_io_current(0.0)
    , master_safety_state(0)
    , master_onoff_state(0)  {
    }
  MasterboardDataMsg_(const ContainerAllocator& _alloc)
    : digital_input_bits(0)
    , digital_output_bits(0)
    , analog_input_range0(0)
    , analog_input_range1(0)
    , analog_input0(0.0)
    , analog_input1(0.0)
    , analog_output_domain0(0)
    , analog_output_domain1(0)
    , analog_output0(0.0)
    , analog_output1(0.0)
    , masterboard_temperature(0.0)
    , robot_voltage_48V(0.0)
    , robot_current(0.0)
    , master_io_current(0.0)
    , master_safety_state(0)
    , master_onoff_state(0)  {
  (void)_alloc;
    }



   typedef uint32_t _digital_input_bits_type;
  _digital_input_bits_type digital_input_bits;

   typedef uint32_t _digital_output_bits_type;
  _digital_output_bits_type digital_output_bits;

   typedef int8_t _analog_input_range0_type;
  _analog_input_range0_type analog_input_range0;

   typedef int8_t _analog_input_range1_type;
  _analog_input_range1_type analog_input_range1;

   typedef double _analog_input0_type;
  _analog_input0_type analog_input0;

   typedef double _analog_input1_type;
  _analog_input1_type analog_input1;

   typedef int8_t _analog_output_domain0_type;
  _analog_output_domain0_type analog_output_domain0;

   typedef int8_t _analog_output_domain1_type;
  _analog_output_domain1_type analog_output_domain1;

   typedef double _analog_output0_type;
  _analog_output0_type analog_output0;

   typedef double _analog_output1_type;
  _analog_output1_type analog_output1;

   typedef float _masterboard_temperature_type;
  _masterboard_temperature_type masterboard_temperature;

   typedef float _robot_voltage_48V_type;
  _robot_voltage_48V_type robot_voltage_48V;

   typedef float _robot_current_type;
  _robot_current_type robot_current;

   typedef float _master_io_current_type;
  _master_io_current_type master_io_current;

   typedef uint8_t _master_safety_state_type;
  _master_safety_state_type master_safety_state;

   typedef uint8_t _master_onoff_state_type;
  _master_onoff_state_type master_onoff_state;





  typedef boost::shared_ptr< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> const> ConstPtr;

}; // struct MasterboardDataMsg_

typedef ::ur_msgs::MasterboardDataMsg_<std::allocator<void> > MasterboardDataMsg;

typedef boost::shared_ptr< ::ur_msgs::MasterboardDataMsg > MasterboardDataMsgPtr;
typedef boost::shared_ptr< ::ur_msgs::MasterboardDataMsg const> MasterboardDataMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator1> & lhs, const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator2> & rhs)
{
  return lhs.digital_input_bits == rhs.digital_input_bits &&
    lhs.digital_output_bits == rhs.digital_output_bits &&
    lhs.analog_input_range0 == rhs.analog_input_range0 &&
    lhs.analog_input_range1 == rhs.analog_input_range1 &&
    lhs.analog_input0 == rhs.analog_input0 &&
    lhs.analog_input1 == rhs.analog_input1 &&
    lhs.analog_output_domain0 == rhs.analog_output_domain0 &&
    lhs.analog_output_domain1 == rhs.analog_output_domain1 &&
    lhs.analog_output0 == rhs.analog_output0 &&
    lhs.analog_output1 == rhs.analog_output1 &&
    lhs.masterboard_temperature == rhs.masterboard_temperature &&
    lhs.robot_voltage_48V == rhs.robot_voltage_48V &&
    lhs.robot_current == rhs.robot_current &&
    lhs.master_io_current == rhs.master_io_current &&
    lhs.master_safety_state == rhs.master_safety_state &&
    lhs.master_onoff_state == rhs.master_onoff_state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator1> & lhs, const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "807af5dc427082b111fa23d1fd2cd585";
  }

  static const char* value(const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x807af5dc427082b1ULL;
  static const uint64_t static_value2 = 0x11fa23d1fd2cd585ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_msgs/MasterboardDataMsg";
  }

  static const char* value(const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This data structure contains the MasterboardData structure\n"
"# used by the Universal Robots controller\n"
"#\n"
"# MasterboardData is part of the data structure being send on the \n"
"# secondary client communications interface\n"
"# \n"
"# This data structure is send at 10 Hz on TCP port 30002\n"
"# \n"
"# Documentation can be found on the Universal Robots Support site, article\n"
"# number 16496.\n"
"\n"
"uint32 digital_input_bits\n"
"uint32 digital_output_bits\n"
"int8 analog_input_range0\n"
"int8 analog_input_range1\n"
"float64 analog_input0\n"
"float64 analog_input1\n"
"int8 analog_output_domain0\n"
"int8 analog_output_domain1\n"
"float64 analog_output0\n"
"float64 analog_output1\n"
"float32 masterboard_temperature\n"
"float32 robot_voltage_48V\n"
"float32 robot_current\n"
"float32 master_io_current\n"
"uint8 master_safety_state\n"
"uint8 master_onoff_state\n"
;
  }

  static const char* value(const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.digital_input_bits);
      stream.next(m.digital_output_bits);
      stream.next(m.analog_input_range0);
      stream.next(m.analog_input_range1);
      stream.next(m.analog_input0);
      stream.next(m.analog_input1);
      stream.next(m.analog_output_domain0);
      stream.next(m.analog_output_domain1);
      stream.next(m.analog_output0);
      stream.next(m.analog_output1);
      stream.next(m.masterboard_temperature);
      stream.next(m.robot_voltage_48V);
      stream.next(m.robot_current);
      stream.next(m.master_io_current);
      stream.next(m.master_safety_state);
      stream.next(m.master_onoff_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MasterboardDataMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_msgs::MasterboardDataMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_msgs::MasterboardDataMsg_<ContainerAllocator>& v)
  {
    s << indent << "digital_input_bits: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.digital_input_bits);
    s << indent << "digital_output_bits: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.digital_output_bits);
    s << indent << "analog_input_range0: ";
    Printer<int8_t>::stream(s, indent + "  ", v.analog_input_range0);
    s << indent << "analog_input_range1: ";
    Printer<int8_t>::stream(s, indent + "  ", v.analog_input_range1);
    s << indent << "analog_input0: ";
    Printer<double>::stream(s, indent + "  ", v.analog_input0);
    s << indent << "analog_input1: ";
    Printer<double>::stream(s, indent + "  ", v.analog_input1);
    s << indent << "analog_output_domain0: ";
    Printer<int8_t>::stream(s, indent + "  ", v.analog_output_domain0);
    s << indent << "analog_output_domain1: ";
    Printer<int8_t>::stream(s, indent + "  ", v.analog_output_domain1);
    s << indent << "analog_output0: ";
    Printer<double>::stream(s, indent + "  ", v.analog_output0);
    s << indent << "analog_output1: ";
    Printer<double>::stream(s, indent + "  ", v.analog_output1);
    s << indent << "masterboard_temperature: ";
    Printer<float>::stream(s, indent + "  ", v.masterboard_temperature);
    s << indent << "robot_voltage_48V: ";
    Printer<float>::stream(s, indent + "  ", v.robot_voltage_48V);
    s << indent << "robot_current: ";
    Printer<float>::stream(s, indent + "  ", v.robot_current);
    s << indent << "master_io_current: ";
    Printer<float>::stream(s, indent + "  ", v.master_io_current);
    s << indent << "master_safety_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.master_safety_state);
    s << indent << "master_onoff_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.master_onoff_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_MSGS_MESSAGE_MASTERBOARDDATAMSG_H
