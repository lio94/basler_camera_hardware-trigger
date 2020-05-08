#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include "basler_parameters.h"
#include "image_publisher.h"
#include <XmlRpcValue.h>

using namespace Pylon;
using namespace GenApi;
using std::string;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "basler_camera");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    int frame_rate;
    if(!pnh.getParam("frame_rate", frame_rate))
        frame_rate = 20;

    string camera_info_url;
    if(!pnh.getParam("camera_info_url", camera_info_url))
        camera_info_url = "";

    string frame_id;
    if(!pnh.getParam("frame_id", frame_id))
        frame_id = "";

    std::string serial_number;
    if(!pnh.getParam("serial_number", serial_number))
        serial_number = "";

    std::string camera_name;
    if(!pnh.getParam("camera_name", camera_name))
        camera_name = "camera";

    int exitCode = 0;

    camera_info_manager::CameraInfoManager cinfo_manager_(nh, camera_name);

    // Automatically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;
    CGrabResultPtr ptrGrabResult;

    CTlFactory& tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    CInstantCamera camera;

    
    bool camera_found = false;

    if (!camera_found)
    {
        ros::Time end = ros::Time::now() + ros::Duration(15.0);
        ros::Rate r(0.5);
        
        while ( ros::ok() && !camera_found )
        {
            ROS_INFO_STREAM("Looking for Camera");
            if (tlFactory.EnumerateDevices(devices) == 0)
            {
                ROS_INFO_STREAM("No devices found. Still searching...");
            }
            if (serial_number == "") 
            {
                // Create an instant camera object for the camera device found first.
                camera.Attach(CTlFactory::GetInstance().CreateFirstDevice());
                
            } 
            else 
            {
                // Look up the camera by its serial number
                ROS_INFO_STREAM("Camera serials found: ");
                for (size_t i=0; i<devices.size(); i++) 
                {
                    ROS_INFO_STREAM(devices[i].GetSerialNumber());
                    if (devices[i].GetSerialNumber().c_str() == serial_number) 
                    {
                        ROS_INFO_STREAM("Found camera with matching serial " << serial_number);
                        camera.Attach(tlFactory.CreateDevice(devices[i]));
                        camera_found = true;
                    }
                }
            }
            r.sleep();
            ros::spinOnce();
        }
    }   
    ROS_INFO_STREAM("Using Camera: " << camera.GetDeviceInfo().GetModelName());

    camera.GrabCameraEvents = true;
    if (camera_info_url != "")
    {
        cinfo_manager_.loadCameraInfo(camera_info_url);
    }

    sensor_msgs::CameraInfo::Ptr cinfo(
        new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

    camera.RegisterImageEventHandler(new ImagePublisher(nh, cinfo, frame_id), RegistrationMode_Append, Cleanup_Delete);
    camera.RegisterConfiguration(new CAcquireContinuousConfiguration , RegistrationMode_ReplaceAll, Cleanup_Delete);

    camera.Open();

    //handle_basler_float_parameter(camera, "AcquisitionFrameRate", frame_rate); // This would be overwriten if you specifed
    // different frame rate via a yaml file, here to honour previously documented although apparently unimplemented feature.

    handle_basler_parameters(camera);

    // Set the pixel format to RGB8 if available.
    INodeMap& nodemap = camera.GetNodeMap();
    CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
    String_t oldPixelFormat = pixelFormat->ToString();
    if (IsAvailable(pixelFormat->GetEntryByName("RGB8")))
    {
        pixelFormat->FromString("RGB8");
    }

    camera.StartGrabbing();
    ROS_INFO_STREAM("Image capture start");
    while (camera.IsGrabbing() && ros::ok())
    {
        
        camera.RetrieveResult(1, ptrGrabResult, TimeoutHandling_Return);
        ros::spinOnce();
    }


    return exitCode;
}
