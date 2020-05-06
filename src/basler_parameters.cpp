#include <XmlRpcValue.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <pylon/PylonIncludes.h>

#include "basler_parameters.h"

void handle_basler_bool_parameter(Pylon::CInstantCamera& camera, std::string name, bool value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting int param " << name << " to " << value << ".");
        GenApi::CBooleanPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            ROS_ERROR_STREAM(name << " set to " << this_node->GetValue());
            return;
        }
        this_node->SetValue(value);
        ROS_INFO_STREAM(name << " set to " << this_node->GetValue());
    }
    catch (const Pylon::GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void handle_basler_int_parameter(Pylon::CInstantCamera& camera, std::string name, int value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting int param " << name << " to " << value << ".");
        GenApi::CIntegerPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
            ROS_ERROR_STREAM(name << " set to " << this_node->GetValue());
            return;
        }
        this_node->SetValue(value);
        ROS_INFO_STREAM(name << " set to " << this_node->GetValue());
    }
    catch (const Pylon::GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void handle_basler_float_parameter(Pylon::CInstantCamera& camera, std::string name, double value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting float param " << name << " to " << value << ".");
        GenApi::CFloatPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsImplemented(this_node))
        {
            ROS_ERROR_STREAM("Basler float parameter '" << name << "' not implemented ");
            ROS_ERROR_STREAM(name << " set to " << this_node->GetValue());
            return;
        }
        this_node->SetValue(value);
        ROS_INFO_STREAM(name << " set to " << this_node->GetValue());
    }
    catch (const Pylon::GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void handle_basler_enum_parameter(Pylon::CInstantCamera& camera, std::string name, std::string value)
{
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    try
    {
        ROS_INFO_STREAM("Setting enum param " << name << " to " << value << ".");
        GenApi::CEnumerationPtr this_node(nodemap.GetNode(name.c_str()));
        if (!IsWritable(this_node))
        {
            ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");

            return;
        }
        if (!IsAvailable(this_node->GetEntryByName(value.c_str())))
        {
            ROS_ERROR_STREAM("Value '" << value << "' isn't available for basler param '" << name << "'.");
            return;
        }
        this_node->FromString(value.c_str());
        //ROS_INFO_STREAM(name << " set to " << this_node->GetValue());
    }
    catch (const Pylon::GenericException& e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
    }
}

void handle_basler_parameter(Pylon::CInstantCamera& camera, XmlRpc::XmlRpcValue& param)
{
    std::string type = param["type"];
    if ("int" == type)
    {
        ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeInt,
                       "Type of value for %s must be int", std::string(param["name"]).c_str());
        handle_basler_int_parameter(camera, param["name"], param["value"]);
    }
    else if ("float" == type)
    {
        ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble,
                       "Type of value for %s must be float", std::string(param["name"]).c_str());
        handle_basler_float_parameter(camera, param["name"], param["value"]);
    }
    else if ("enum" == type)
    {
        ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeString,
                       "Type of value for %s must be string", std::string(param["name"]).c_str());
        handle_basler_enum_parameter(camera, param["name"], param["value"]);
    }
    else if ("bool" == type)
    {
        ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeBoolean,
                       "Type of value for %s must be string", std::string(param["name"]).c_str());
        handle_basler_bool_parameter(camera, param["name"], param["value"]);
    }
    else
    {
        ROS_FATAL_STREAM("Unknown param type for parameter " << param["name"] << ": " << type);
    }
}

void handle_basler_parameters(Pylon::CInstantCamera& camera)
{
    ros::NodeHandle private_handle("~");
    XmlRpc::XmlRpcValue params;
    private_handle.getParam("basler_params", params);
    ROS_ASSERT_MSG(params.getType() == XmlRpc::XmlRpcValue::TypeArray, "Badly formed basler param yaml");
    for (size_t index = 0; index < params.size(); ++index)
    {
        ROS_ASSERT_MSG(params[index].getType() == XmlRpc::XmlRpcValue::TypeStruct, "Badly formed basler param yaml");
        ROS_ASSERT_MSG(params[index].hasMember("name"), "Param needs name");
        ROS_ASSERT_MSG(params[index].hasMember("type"), "Param needs type");
        ROS_ASSERT_MSG(params[index].hasMember("value"), "Param needs value");

        handle_basler_parameter(camera, params[index]);
    }
}
