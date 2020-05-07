#include <XmlRpcValue.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <pylon/PylonIncludes.h>

#include "basler_parameters.h"

namespace
{
template <typename T, typename NodePtrT>
void handle_basler_parameter(Pylon::CInstantCamera& camera, const std::string& name, const T& value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        NodePtrT this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' not implemented ");
            ROS_ERROR_STREAM(name << " set to " << this_node->GetValue());
            return;
        }
        this_node->SetValue(value);
        ROS_DEBUG_STREAM(name << " set to " << this_node->GetValue());
    }
    catch (const Pylon::GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void handle_basler_enum_parameter(Pylon::CInstantCamera& camera, const std::string& name, const std::string& value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        GenApi::CEnumerationPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            ROS_ERROR_STREAM(name << " set to " << this_node->ToString());
            return;
        }
        if (!IsAvailable(this_node->GetEntryByName(value.c_str())))
        {
            ROS_ERROR_STREAM("Value '" << value << "' isn't available for basler param '" << name << "'.");
            return;
        }
        this_node->FromString(value.c_str());
        ROS_DEBUG_STREAM(name << " set to " << this_node->ToString());
    }
    catch (const Pylon::GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

} // anonymous namespace

void handle_basler_parameters(Pylon::CInstantCamera& camera)
{
    ros::NodeHandle private_handle("~");
    XmlRpc::XmlRpcValue params;
    private_handle.getParam("basler_params", params);
    ROS_ASSERT_MSG(params.getType() == XmlRpc::XmlRpcValue::TypeArray, "Badly formed basler param yaml");
    for (size_t index = 0; index < params.size(); ++index)
    {
        const auto& param = params[index];
        ROS_ASSERT_MSG(param.getType() == XmlRpc::XmlRpcValue::TypeStruct, "Badly formed basler param yaml");
        ROS_ASSERT_MSG(param.hasMember("name"), "Param needs name");
        ROS_ASSERT_MSG(param.hasMember("type"), "Param needs type");
        ROS_ASSERT_MSG(param.hasMember("value"), "Param needs value");
        std::string name = param["name"];
        std::string type = param["type"];
        auto& value = param["value"];
        if (type == "bool")
        {
            ROS_ASSERT_MSG(value.getType() == XmlRpc::XmlRpcValue::TypeBoolean,
                           "Type of value for %s must be bool", name.c_str());
            handle_basler_parameter<bool, GenApi::CBooleanPtr>(camera, name, value);
        }
        else if (type == "int")
        {
            ROS_ASSERT_MSG(value.getType() == XmlRpc::XmlRpcValue::TypeInt,
                           "Type of value for %s must be int", name.c_str());
            handle_basler_parameter<int, GenApi::CIntegerPtr>(camera, name, value);
        }
        else if (type == "float")
        {
            // xmlrpc uses doubles
            ROS_ASSERT_MSG(value.getType() == XmlRpc::XmlRpcValue::TypeDouble,
                           "Type of value for %s must be float", name.c_str());
            handle_basler_parameter<double, GenApi::CFloatPtr>(camera, name, value);
        }
        else if (type == "enum")
        {
            ROS_ASSERT_MSG(value.getType() == XmlRpc::XmlRpcValue::TypeString,
                           "Type of value for enum %s must be string", name.c_str());
            handle_basler_enum_parameter(camera, name, value);
        }
        else
        {
            ROS_FATAL_STREAM("Unknown param type for parameter " << name << ": " << type);
        }
    }
}
