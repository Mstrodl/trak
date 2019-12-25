//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr/openvr_driver.h>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <cstring>
#include <chrono>
#include <iostream>

#if defined( _WINDOWS )
#include <windows.h>
#include <enumser.h>
#endif
#include <serial/serial.h>

using namespace vr;


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

inline HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
  HmdQuaternion_t quat;
  quat.w = w;
  quat.x = x;
  quat.y = y;
  quat.z = z;
  return quat;
}

inline void HmdMatrix_SetIdentity( HmdMatrix34_t *pMatrix )
{
  pMatrix->m[0][0] = 1.f;
  pMatrix->m[0][1] = 0.f;
  pMatrix->m[0][2] = 0.f;
  pMatrix->m[0][3] = 0.f;
  pMatrix->m[1][0] = 0.f;
  pMatrix->m[1][1] = 1.f;
  pMatrix->m[1][2] = 0.f;
  pMatrix->m[1][3] = 0.f;
  pMatrix->m[2][0] = 0.f;
  pMatrix->m[2][1] = 0.f;
  pMatrix->m[2][2] = 1.f;
  pMatrix->m[2][3] = 0.f;
}


// keys for use with the settings API
static const char * const k_pch_Trak_Section = "driver_taka";
static const char * const k_pch_Trak_SerialNumber_String = "serialNumber";
static const char * const k_pch_Trak_ModelNumber_String = "modelNumber";
static const char * const k_pch_Trak_WindowX_Int32 = "windowX";
static const char * const k_pch_Trak_WindowY_Int32 = "windowY";
static const char * const k_pch_Trak_WindowWidth_Int32 = "windowWidth";
static const char * const k_pch_Trak_WindowHeight_Int32 = "windowHeight";
static const char * const k_pch_Trak_RenderWidth_Int32 = "renderWidth";
static const char * const k_pch_Trak_RenderHeight_Int32 = "renderHeight";
static const char * const k_pch_Trak_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_Trak_DisplayFrequency_Float = "displayFrequency";

bool g_bExiting = false;

typedef struct {
  double x;
  double y;
  double z;
} Vec3;

typedef struct {
  double w;
  double x;
  double y;
  double z;
} Quat;

typedef struct {
  Vec3 position;
  Quat rotation;
} DeviceMessage;

#define PACKET_SIZE sizeof(DeviceMessage)

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CTrakControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
  CTrakControllerDriver(std::string portId, std::string portName) { 
    m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

    // Serial numbers triple hetero
    m_sSerialNumber = portName;
    m_sPortId = portId;

    m_sModelNumber = "taka_tracker_0";
  }

  virtual ~CTrakControllerDriver() {
  }


  void MessagingThread() {
    // (1000 / 90) ~= 1 packet every 11.1ms, so a 1s timeout should be completely safe
    serial::Serial *device = new serial::Serial(m_sPortId, 115200, serial::Timeout::simpleTimeout(1000));
    DeviceMessage msg;
    while(m_bMessagingThreadState) {
      memcpy(&msg, (char *) device->read(PACKET_SIZE).c_str(), sizeof(DeviceMessage));
      m_pose.vecPosition[0] = msg.position.x;
      m_pose.vecPosition[1] = msg.position.y;
      m_pose.vecPosition[2] = msg.position.z;
      
      m_pose.qRotation.w = msg.rotation.w;
      m_pose.qRotation.x = msg.rotation.x;
      m_pose.qRotation.y = msg.rotation.y;
      m_pose.qRotation.z = msg.rotation.z;

      m_pose.poseIsValid = true;
      m_pose.deviceIsConnected = true;
    }
    delete device;
  }

  
  virtual EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
  {
    m_unObjectId = unObjectId;
    m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );
    // This would be m_sModelNumber.c_str() but we don't have a model :p
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "generic_tracker" );

    // "OptOut" says we aren't a hand. See ETrackedControllerRole
    vr::VRProperties()->SetInt32Property( m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut );

    // this file tells the UI what to show the user for binding this controller as well as what default bindings should
    // be for legacy or other apps
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{taka}/input/mycontroller_profile.json" );

    // Tell the messaging thread we want it running still, and spin it up!
    m_bMessagingThreadState = true;
    m_tMessagingThread = std::thread(&CTrakControllerDriver::MessagingThread, this);
    
    return VRInitError_None;
  }

  virtual void Deactivate()
  {
    m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    m_bMessagingThreadState = false;
    m_tMessagingThread.join();
  }

  virtual void EnterStandby()
  {
  }

  void *GetComponent( const char *pchComponentNameAndVersion )
  {
    // override this to add a component to a driver
    return NULL;
  }

  virtual void PowerOff()
  {
  }

  /** debug request from a client */
  virtual void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
  {
    if ( unResponseBufferSize >= 1 )
      pchResponseBuffer[0] = 0;
  }

  // Initial pose. Not sure why this exists tbh...
  // I guess we'll leave it here and hope for the best?
  virtual DriverPose_t GetPose()
  {
    DriverPose_t pose = { 0 };
    pose.poseIsValid = false;
    pose.result = TrackingResult_Calibrating_OutOfRange;
    pose.deviceIsConnected = true;

    pose.qWorldFromDriverRotation = HmdQuaternion_Init( 1, 0, 0, 0 );
    pose.qDriverFromHeadRotation = HmdQuaternion_Init( 1, 0, 0, 0 );

    return pose;
  }


  void RunFrame()
  {
  }

  void ProcessEvent( const vr::VREvent_t & vrEvent )
  {
  }


  std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
  vr::TrackedDeviceIndex_t m_unObjectId;
  vr::PropertyContainerHandle_t m_ulPropertyContainer;

  vr::VRInputComponentHandle_t m_compHaptic;

  std::string m_sSerialNumber;
  std::string m_sPortId;
  std::string m_sModelNumber;


  DriverPose_t m_pose;

  std::thread m_tMessagingThread;
  bool m_bMessagingThreadState;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_Trak: public IServerTrackedDeviceProvider
{
public:
  virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
  virtual void Cleanup() ;
  virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
  virtual void RunFrame() ;
  virtual bool ShouldBlockStandbyMode()  { return false; }
  virtual void EnterStandby()  {}
  virtual void LeaveStandby()  {}

private:
  std::vector<CTrakControllerDriver*> m_controllers;
};

CServerDriver_Trak g_serverDriverNull;


EVRInitError CServerDriver_Trak::Init( vr::IVRDriverContext *pDriverContext )
{
  std::cout << "Initializing Driver Provider...\n";
  VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
  InitDriverLog( vr::VRDriverLog() );

#ifdef _WIN32
  //Initialize COM (Required by CEnumerateSerial::UsingWMI)
  HRESULT hr = CoInitialize(nullptr);
  if (FAILED(hr)) {
    _tprintf(_T("Failed to initialize COM, Error:%x\n"), hr);
  }

  //Initialize COM security (Required by CEnumerateSerial::UsingWMI)
  hr = CoInitializeSecurity(nullptr, -1, nullptr, nullptr, RPC_C_AUTHN_LEVEL_DEFAULT, RPC_C_IMP_LEVEL_IMPERSONATE, nullptr, EOAC_NONE, nullptr);
  if (FAILED(hr)) {
    _tprintf(_T("Failed to initialize COM security, Error:%08X\n"), hr);
    CoUninitialize();
  }

  CEnumerateSerial::CPortAndNamesArray portAndNames;
  hr = CEnumerateSerial::UsingWMI(portAndNames);
  if (SUCCEEDED(hr)) {
    for (const auto& port : portAndNames) {
      // port.second == std::string of the name
      // port.first == unsigned int (%u) of the port
      // Create the controller, push it into the vector
      CTrakControllerDriver *controller = new CTrakControllerDriver(port.first, port.second);
      m_controllers.push_back(controller);
      // ...And add it to the host
      vr::VRServerDriverHost()->TrackedDeviceAdded( controller->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, controller );
    }
  } else {
    _tprintf(_T("CEnumerateSerial::UsingWMI failed, Error:%08X\n"), hr);
  }
#else
#define HARD_COMM_COUNT 1
  for(int i = 0; i < HARD_COMM_COUNT; ++i) {
    std::ostringstream portIdS;
    portIdS << "/dev/rfcomm" << i;
    std::string portId = portIdS.str();
    CTrakControllerDriver *controller = new CTrakControllerDriver(portId, "Device on " + portId);
    m_controllers.push_back(controller);
    vr::VRServerDriverHost()->TrackedDeviceAdded(controller->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, controller);
  }
#endif

  return VRInitError_None;
}

void CServerDriver_Trak::Cleanup() 
{
  CleanupDriverLog();
  m_controllers.clear();
}


void CServerDriver_Trak::RunFrame()
{
}

// Returns an instance of our driver! ("Driver factory function")
HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
  std::cout << "Factory..\n";
  if( 0 == strcmp( IServerTrackedDeviceProvider_Version, pInterfaceName ) )
  {
    return &g_serverDriverNull;
  }

  if( pReturnCode )
    *pReturnCode = VRInitError_Init_InterfaceNotFound;

  return NULL;
}
