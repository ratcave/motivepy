//======================================================================================================
// Copyright 2010 NaturalPoint, Inc.
//======================================================================================================
#pragma once

#include <memory>
#include <vector>
#include <variant>

#include "Core/UID.h"
#include "Core/Label.h"
#include "Core/Vector2.h"
#include "Core/Quaternion.h"
#include "Core/Marker.h"

#ifdef MOTIVE_API_EXPORTS
#define MOTIVE_API __declspec(dllexport)
#elif defined MOTIVE_API_IMPORTS
#define MOTIVE_API __declspec(dllimport)
#else
#define MOTIVE_API
#endif

namespace CameraLibrary
{
    class Camera;
    class CameraManager;
    class cCameraModule;
}

/// <summary>
/// The Motive API is a C++ API that is intended for use with live OptiTrack cameras. It supports a wide range
/// of runtime configuration, data retrieval, property editing, and workflows.
/// </summary>
namespace MotiveAPI
{
    /// <summary>
    /// Used to return status information from many of the API methods.
    /// </summary>
    enum eResult
    {
        kApiResult_Success = 0,
        kApiResult_Failed,
        kApiResult_FileNotFound,
        kApiResult_LoadFailed,
        kApiResult_SaveFailed,
        kApiResult_InvalidFile,
        kApiResult_InvalidLicense,
        kApiResult_NoFrameAvailable,
        kApiResult_TooFewMarkers,
        kApiResult_ToManyMarkers,
        kApiResult_UnableToFindGroundPlane,
        kApiResult_UnableGetGroundPlane,
        kApiResult_RemoveCalibrationSquare
    };

    /// <summary>
    /// Used in sPropertyValue to indicate which data type the property value is.
    /// </summary>
    enum ePropertyDataType
    {
        eInvalid = 0,   // Returned when an invalid property is requested.
        eDouble,
        eInt,
        eBool,
        eUID,
        eVector2f,
        eVector3f,
        eRotationf,
        eString
    };

    /// <summary>
    /// Represents a property value, from a fixed set of property value types.
    /// </summary>
    struct sPropertyValue
    {
        /// <summary>
        /// Shorthand definition for a property value variant type.
        /// </summary>
        using ValueType = std::variant<double, int, bool, Core::cUID, Core::cVector2f, Core::cVector3f, Core::cRotationf, std::wstring>;

        /// <summary>
        /// The data type stored in mData.
        /// </summary>
        ePropertyDataType mDataType;

        /// <summary>
        /// Data storage for the property value.
        /// </summary>
        ValueType mData;
    };

    // Licensing ===========================================================================================

    /// <summary>Load a license file from a memory buffer.</summary>
    /// <remarks>
    /// Licenses are automatically loaded from the OptiTrack license directory. This method is not needed except to
    /// accommodate some very rare user scenarios.  Call this and provide the contents of a license
    /// file located outside the license folder.  Call this function before Initialize()
    /// </remarks>
    /// <param name="buffer">The memory buffer to read from.</param>
    /// <param name="bufferSize">The size of the memory buffer.</param>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult LoadLicenseFromMemory( const unsigned char* buffer, int bufferSize );

    // Startup / Shutdown ==================================================================================

    /// <summary>Call this method before calling any other API methods.</summary>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult Initialize();

    /// <summary>Determine if the API has been initialized.</summary>
    /// <returns>True if Initialize has been called, and Shutdown has not been called.</returns>
    MOTIVE_API bool IsInitialized();

    /// <summary>Clean up and shut down the API.</summary>
    MOTIVE_API void Shutdown();

    /// <summary>Determine if this instance of the API can connect to OptiTrack devices.</summary>
    /// <returns>True if no other OptiTrack software is running on this host and consuming device connections.</returns>
    MOTIVE_API bool CanConnectToDevices();

    /// <summary>
    /// Retrieve the specific build number of the API. This is correlated with the software
    /// release version, but the software release version is not encoded in this value.
    /// </summary>
    /// <returns>The build number.</returns>
    MOTIVE_API int BuildNumber();

    // User Profile Interface ==============================================================================

    /// <summary>Load a user profile file.</summary>
    /// <param name="filename">The file to load from.</param>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult LoadProfile( const wchar_t* filename );

    /// <summary>Save a user profile file.</summary>
    /// <param name="filename">The file to save to.</param>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult SaveProfile( const wchar_t* filename );

    // Frame Processing ====================================================================================

    /// <summary>Process the latest frame of camera data and make it available to the API.</summary>
    /// <returns>kApiResult_NoFrameAvailable if no new frame is available.</returns>
    MOTIVE_API eResult Update();

    /// <summary>Process one frame of camera data and make it available to the API, leaving any remaining frames
    /// on the queue. The oldest frame on the queue is processed.</summary>
    /// <returns>kApiResult_NoFrameAvailable if no new frame is available.</returns>
    MOTIVE_API eResult UpdateSingleFrame();

    /// <summary>Flushes all queued camera frames.</summary>
    /// <remarks>In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers), 
    /// and you find that the data you're getting out has sufficient latency you can call FlushCameraQueues()
    /// to catch up before calling Update(). Ideally, after calling FlushCameraQueues() you'll want to
    /// not call it again until after Update() returns kApiResult_Success</remarks>
    MOTIVE_API void FlushCameraQueues();

    // Camera Calibration Interface ========================================================================

    /// <summary>
    /// Load a calibration from file.
    /// </summary>
    /// <param name="filename">Filename to read the calibration from.</param>
    /// <param name="cameraCount">(Output) The number of cameras read from the calibration file.</param>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult LoadCalibration( const wchar_t* filename, int* cameraCount = nullptr );

    /// <summary>
    /// Load a calibration file that was previously loaded into a memory block.
    /// </summary>
    /// <param name="buffer">The memory buffer to read from.</param>
    /// <param name="bufferSize">The size of the memory buffer.</param>
    /// <param name="cameraCount">(Output) The number of cameras read from the calibration file.</param>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult LoadCalibrationFromMemory( unsigned char* buffer, int bufferSize, int* cameraCount = nullptr );

    /// <summary>
    /// Save the current calibration to file.
    /// </summary>
    /// <param name="filename">File to save to.</param>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult SaveCalibration( const wchar_t* filename );

    /// <summary>Structure for passing camera extrinsics through the API.</summary>
    struct sCameraInfo
    {
        /// <summary>
        /// Camera serial number.
        /// </summary>
        int CameraSerial;

        /// <summary>
        /// 3-vector of the world space position of the camera, in meters.
        /// </summary>
        float Position[3];

        /// <summary>
        /// 3x3 matrix representing the world-space camera rotation.
        /// </summary>
        float Orientation[9];
    };

    /// <summary>
    /// Get camera extrinsics from a calibration file in memory. This allows for acquiring camera
    /// extrinsics for cameras not connected to system.  It simply returns the list of details for all
    /// cameras contained in the current calibration.
    /// </summary>
    /// <param name="buffer">Memory buffer to read from.</param>
    /// <param name="bufferSize">Size of the memory buffer.</param>
    /// <param name="result">kApiResult_Success on success, or an error code otherwise</param>
    /// <returns>Camera calibration information for each camera that was read from the buffer.</returns>
    MOTIVE_API std::vector<sCameraInfo> CameraExtrinsicsCalibrationFromMemory( unsigned char* buffer, int bufferSize,
        eResult& result );

    /// <summary>Start a new calibration wanding for all cameras. This will cancel any existing calibration process.</summary>
    MOTIVE_API void StartCalibrationWanding();

    /// <summary>The possible calibration states.</summary>
    enum eCalibrationState
    {
        Initialized = 0,
        Wanding,
        WandingComplete,
        PreparingSolver,
        EstimatingFocals,
        CalculatingInitial,
        Phase1,
        Phase2,
        Phase3,
        Phase4,
        Complete,
        CalibrationError
    };

    /// <summary>Retrieve the current calibration state.</summary>
    /// <returns>Current calibration state.</returns>
    MOTIVE_API eCalibrationState CalibrationState();

    /// <summary>During calibration wanding, this will return a vector of camera indices that are lacking the minimum
    /// number of calibration samples to begin calculation.</summary>
    /// <remarks>When the returned vector for this method goes to
    /// zero size, you can call StartCalibrationCalculation() to begin calibration calculations. Wanding
    /// samples will continue to be collected until StartCalibrationCalculation() is called.</remarks>
    /// <returns>Vector of camera indices who lack sufficient samples for clibration.</returns>
    MOTIVE_API std::vector<int> CalibrationCamerasLackingSamples();

    /// <summary>
    /// During calibration wanding, this will return the number of wand samples collected for the given
    /// camera. Returns zero otherwise.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera to query.</param>
    /// <returns>Number of samples for the requested camera, or zero if the camera was not found.</returns>
    MOTIVE_API int CameraCalibrationSamples( int cameraIndex );

    /// <summary>Cancels either wanding or calculation and resets the calibration engine.</summary>
    MOTIVE_API void CancelCalibration();

    /// <summary>Once wanding is complete, call this to begin the calibration calculations.</summary>
    /// <returns>True if calibration calculation was started.</returns>
    MOTIVE_API bool StartCalibrationCalculation();

    /// <summary>
    /// Retrieve the current calibration quality.
    /// </summary>
    /// <returns>The current calibration quality in the range [0-5], with 5 being best. Returns zero otherwise.</returns>
    MOTIVE_API int CurrentCalibrationQuality();

    /// <summary>Once CalibrationState() returns "Complete", call this method to apply the calibration results
    /// to all cameras.</summary>
    /// <returns>True if the calibration was complete and could be applied.</returns>
    MOTIVE_API bool ApplyCalibrationCalculation();

    /// <summary>Set the ground plane using a standard or custom ground plane template.</summary>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult SetGroundPlane( bool useCustomGroundPlane );

    /// <summary>
    /// <summary>Translate the existing ground plane.</summary>
    /// </summary>
    /// <param name="x">X amount to translate by, in mm.</param>
    /// <param name="y">Y amount to translate by, in mm.</param>
    /// <param name="z">Z amount to translate by, in mm.</param>
    MOTIVE_API void TranslateGroundPlane( float x, float y, float z );

    /// <summary>
    /// The pre-defined calibration square types.
    /// </summary>
    enum eCalibrationSquareType
    {
        kNone,
        kCS400,
        kClassicLFrame,
        kCS200,
        kCS100
    };

    /// <summary>Returns the Calibration Square in the volume if one is detected.</summary>
    /// <returns>The detected calibration square type, or kNone if none found.</returns>
    MOTIVE_API eCalibrationSquareType AutoDetectCalibrationSquare();

    /// <summary>Get Ground Plane Markers. Returns marker count and list of markers found on the ground plane.</summary>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult GetGroundPlaneMarkers( std::vector<Core::cMarker>& markers );

    // Data Streaming ======================================================================================

    /// <summary>Enable or disable NatNet streaming.</summary>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult StreamNP( bool enable );

    /// <summary>Enable or disable VRPN streaming.</summary>
    /// <returns>kApiResult_Success on success, or an error code otherwise.</returns>
    MOTIVE_API eResult StreamVRPN( bool enable, int port );

    // Frame Info ==========================================================================================

    /// <summary>Retrieve the frame ID of the current frame.</summary>
    /// <returns>The current frame ID.</returns>
    MOTIVE_API int FrameID();

    /// <summary>Frame timestamp of the current frame.</summary>
    /// <returns>Frame timestamp (in seconds from startup).</returns>
    MOTIVE_API double FrameTimeStamp();

    /// <summary>A structure for retrieving timecode information for the current frame.</summary>
    struct MOTIVE_API sTimecode
    {
        /// <summary>Hours</summary>
        int hh;

        /// <summary>Minutes</summary>
        int mm;

        /// <summary>Seconds</summary>
        int ss;

        /// <summary>Frames</summary>
        int ff;

        /// <summary>Sub-frames</summary>
        int subFrame;

        /// <summary>
        /// Human-readable string representation of the timecode value.
        /// </summary>
        char tcString[32];

        /// <summary>
        /// True if this is drop-frame timecode.
        /// </summary>
        bool isDropFrame;
    };

    /// <summary>Retrieve timecode information for the current frame.</summary>
    /// <param name="tc">(Output) Timecode structure to fill.</param>
    /// <returns>True if timecode is available and the timecode structure was filled.</returns>
    MOTIVE_API bool FrameTimeCode( sTimecode& tc );

    // Marker Interface ====================================================================================

    /// <summary>Retrieve the number of reconstructed markers in the current frame.</summary>
    /// <returns>Total number of reconstructed markers.</returns>
    MOTIVE_API int MarkerCount();

    /// <summary>
    /// Calculate the average marker diameter in meters
    /// </summary>
    /// <returns>Average diameter of markers in meters</returns>
    MOTIVE_API float MarkerAverageSize();

    /// <summary>Retrieve a specific marker in the current frame.</summary>
    /// <param name="markerIndex">Index of the marker to retrieve.</param>
    /// <param name="marker">Reference to the marker structure to load with marker info.</param>
    /// <returns>True if the marker index was valid. False otherwise.</returns>
    MOTIVE_API bool Marker( int markerIndex, Core::cMarker& marker );

    /// <summary>Retrieve the 3D reconstructed position of a specific marker in the current frame.</summary>
    /// <param name="markerIndex">Index of the marker to retrieve.</param>
    /// <param name="x">X coordinate of the marker, in meters.</param>
    /// <param name="y">Y coordinate of the marker, in meters.</param>
    /// <param name="z">Z coordinate of the marker, in meters.</param>
    /// <returns>True if the marker index was valid, false otherwise.</returns>
    MOTIVE_API bool MarkerXYZ( int markerIndex, float& x, float& y, float& z );

    /// <summary>Retrieve the ID of a specific marker in the current frame.</summary>
    /// <param name="markerIndex">Index of the marker to retrieve.</param>
    /// <returns>The ID of the marker, or Core::cUID::kInvalid if the marker index was invalid.</returns>
    MOTIVE_API Core::cUID MarkerID( int markerIndex );

    /// <summary>Retrieve the reconstruction residual of a specific marker in the current frame.</summary>
    /// <param name="markerIndex">Index of the marker to retrieve.</param>
    /// <returns>The residual of the marker, or zero if the marker index was invalid.</returns>
    MOTIVE_API float MarkerResidual( int markerIndex );

    /// <summary>
    /// Retrieve the number of rays that contributed to the reconstruction of a specific marker in the current frame.
    /// </summary>
    /// <param name="markerIndex">Index of the marker to retrieve.</param>
    /// <returns>Number of contributing rays, or zero if the index was invalid.</returns>
    MOTIVE_API int MarkerContributingRaysCount( int markerIndex );

    /// <summary>
    /// Retrieve the average ray length for all rays contributing to a specific marker in the current frame.
    /// </summary>
    /// <param name="markerIndex">Index of the marker to retrieve.</param>
    /// <returns>Average ray length, or zero if the index was invalid or no rays contributed to the marker.</returns>
    MOTIVE_API float MarkerAverageRayLength( int markerIndex );

    /// <summary>
    /// Determine if a given camera is contributing a ray to a specific marker on the current frame, and returns
    /// the 2D coordinates of that ray in the view of the camera.
    /// </summary>
    /// <param name="markerIndex">Index of the marker.</param>
    /// <param name="cameraIndex">Index of the camera.</param>
    /// <param name="x">X coordinate in the camera view.</param>
    /// <param name="y">X coordinate in the camera view.</param>
    /// <returns>True if the requested camera contributes a ray to the specified marker.</returns>
    MOTIVE_API bool MarkerCameraCentroid( int markerIndex, int cameraIndex, float& x, float& y );

    // Rigid Body Interface ================================================================================

    /// <summary>
    /// Retrieve the total number of rigid bodies defined, including all tracked and untracked assets.
    /// </summary>
    /// <returns>Total number of defined rigid bodies.</returns>
    MOTIVE_API int RigidBodyCount();

    /// <summary>
    /// Create a rigid body based on the marker count and marker list provided.  The marker list is
    /// expected to contain of list of marker coordinates in the order: x1,y1,z1,x2,y2,z2,...xN,yN,zN.
    /// </summary>
    /// <param name="name">Name to assign to the rigid body.</param>
    /// <param name="id">User ID to assign to the rigid body.</param>
    /// <param name="markerCount">Number of markers in the markerList</param>
    /// <param name="markerList">Array of 3 * markerCount values, given in X, Y, Z order.</param>
    /// <returns></returns>
    MOTIVE_API eResult CreateRigidBody( const wchar_t* name, int id, int markerCount, float* markerList );

    /// <summary>
    /// Returns properties names for properties found of a rigidbody
    /// </summary>
    /// <param name="rbIndex">Index in the rigidbody list.</param>
    /// <param name="propertyNames">list names of properties on a rigidbody .</param>/// 
    /// <returns> is successful or not</returns>
    MOTIVE_API bool RigidBodyPropertyNames(int rbIndex, std::vector<std::wstring>& propertyNames);

    /// <summary>
    /// Retrieves the property value of a given property
    /// </summary>
    /// <remarks>
    /// Valid camera property names:
    /// <list type="bullet">
    ///<item><description> NodeName                 (String) </description></item>
    ///<item><description> AssetName                (String) </description></item>
    ///<item><description> GeometryYawPitchRoll     (eVector3f) </description></item>
    ///<item><description> BoneMajorAxis            (Int) </description></item>
    ///<item><description> DefaultBoneLength        (double) </description></item>
    ///<item><description> DefaultBoneDiameter      (double) </description></item>
    ///<item><description> JointName                (String) </description></item>
    ///<item><description> ParentInfo               (String) </description></item>
    ///<item><description> ChildInfo                (String) </description></item>
    ///<item><description> JointVisible             (Bool) </description></item>
    ///<item><description> JointType                (String) </description></item>
    ///<item><description> DegreesOfFreedom         (Int) </description></item>
    ///<item><description> RotationOrder            (Int) </description></item>
    ///<item><description> RotationOffset           (eRotationf) </description></item>
    ///<item><description> TranslationOffset        (eVector3f) </description></item>
    ///<item><description> TipOffset                (eVector3f) </description></item>
    ///<item><description> AssetVisible             (Bool) </description></item>
    ///<item><description> Comment                  (String) </description></item>
    ///<item><description> MinimumBootingLabels     (Int) </description></item>
    ///<item><description> MinimumMarkerCount       (Int) </description></item>
    ///<item><description> MinimumBootingActive     (Int) </description></item>
    ///<item><description> Scale                    (double) </description></item>
    ///<item><description> SyntheticLabelGraphScale (double) </description></item>
    ///<item><description> ShowLabel                (Bool) </description></item>
    ///<item><description> ShowIMUState             (Int) </description></item>
    ///<item><description> DisplayTracked           (Bool) </description></item>
    ///<item><description> Color                    (Int) </description></item>
    ///<item><description> ShowBones                (Bool) </description></item>
    ///<item><description> BoneColor                (Int) </description></item>
    ///<item><description> ShowAxis                 (Bool) </description></item>
    ///<item><description> DisplayPositionHistory   (Bool) </description></item>
    ///<item><description> DisplayHistoryLength     (Int) </description></item>
    ///<item><description> ShowDOF                  (Bool) </description></item>
    ///<item><description> ShowMarkerSet            (Bool) </description></item>
    ///<item><description> ShowTargetMarkerLines    (Bool) </description></item>
    ///<item><description> ShowMarkerLines          (Bool) </description></item>
    ///<item><description> Smoothing                (double) </description></item>
    ///<item><description> PredictionTime           (double) </description></item>
    ///<item><description> PositionDamping          (eVector3f) </description></item>
    ///<item><description> RotationDamping          (double) </description></item>
    ///<item><description> RotationDampingAxis      (Int) </description></item>
    ///<item><description> ModelAlpha               (double) </description></item>
    ///<item><description> GeometryType             (Int) </description></item>
    ///<item><description> GeometryFile             (String) </description></item>
    ///<item><description> GeometryScale            (eVector3f) </description></item>
    ///<item><description> GeometryOffset           (eVector3f) </description></item>
    ///<item><description> GeometryPitchYawRoll     (eVector3f) </description></item>
    ///<item><description> Name                     (String) </description></item>
    ///<item><description> UserData                 (Int) </description></item>
    ///<item><description> ActiveTagID              (Int) </description></item>
    ///<item><description> ActiveTagRfChannel       (Int) </description></item>
    ///<item><description> TrackingAlgorithmLevel   (Int) </description></item>
    ///<item><description> ShareMarkers             (Bool) </description></item>
    ///<item><description> MarkerID                 (Int) </description></item>
    ///<item><description> MarkerLocation           (eVector3f) </description></item>
    /// </list
    /// </remarks>
    /// <param name="rbIndex">Index in the rigidbody list.</param>
    /// <param name="propertyName">Name of the property to set.</param>
    /// <returns>The value of the property. If the rigidbody or property could not be found, the mDataType member
    /// of the returned structure will be set to eInvalid.</returns>
    MOTIVE_API sPropertyValue RigidBodyProperty(int rbIndex, const std::wstring& propertyName);

    /// <summary>
    /// Returns property type for property found of from a rigidbody
    /// </summary>
    /// <param name="rbIndex">Index in the rigidbody list.</param>
    /// <param name="propertyName">Name of the property to query.</param>
    /// <returns>If an invalid property name is specified, eInvalid is returned. Otherwise, the data type
    /// of the requested property is returned.</returns>
    MOTIVE_API ePropertyDataType RigidBodyPropertyType(int rbIndex, const std::wstring& propertyName);

    /// <summary>
    /// Set the value of a asset property.
    /// </summary>
    /// <remarks>
    /// Please note that if you change a value to be outside of its constraints, it will not change
    /// </remarks>
    /// <param name="rbIndex">Index in the rigidbody list.</param>
    /// <param name="propertyName">Name of the property to set.</param>
    /// <param name="value">Value to set the property to.</param>
    /// <returns>True if the property was found and the value was set.</returns>
    MOTIVE_API bool SetRigidBodyProperty(int rbIndex, const std::wstring& propertyName, const sPropertyValue& value);


    /// <summary>
    /// Clear all rigid bodies
    /// </summary>
    MOTIVE_API void ClearRigidBodies();

    /// <summary>
    /// Load rigid body definitions from file and replace any that exist in the scene.
    /// </summary>
    /// <param name="filename">Full file name to load from.</param>
    /// <returns>kApiResult_Success if successful, or an error code if not.</returns>
    MOTIVE_API eResult LoadRigidBodies( const wchar_t* filename );

    /// <summary>
    /// Load rigid bodies and add them to the scene, leaving any that already exist in place.
    /// </summary>
    /// <param name="filename">Full file name to load from.</param>
    /// <returns>kApiResult_Success if successful, or an error code if not.</returns>
    MOTIVE_API eResult AddRigidBodies( const wchar_t* filename );

    /// <summary>
    /// Save rigid body definitions to file.
    /// </summary>
    /// <param name="filename">Full file name to save to.</param>
    /// <returns>kApiResult_Success if successful, or an error code if not.</returns>
    MOTIVE_API eResult SaveRigidBodies( const wchar_t* filename );

    /// <summary>
    /// Retrieve the unique ID of the rigid body at the given index.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <returns>Unique ID of the rigid body, or Core::cUID::kInvalid if not found.</returns>
    MOTIVE_API Core::cUID RigidBodyID( int rbIndex );

    /// <summary>
    /// Retrieve the name of the rigid body at the given index.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <param name="buffer">String buffer to place the name in.</param>
    /// <param name="bufferSize">Size of the passed-in string buffer.</param>
    /// <returns>True on success.</returns>
    MOTIVE_API bool RigidBodyName( int rbIndex, wchar_t* buffer, int bufferSize );

    /// <summary>
    /// Determine if the rigid body at the given index is tracked in the current frame.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <returns>True if the rigid body is found and tracked. False otherwise.</returns>
    MOTIVE_API bool IsRigidBodyTracked( int rbIndex );

    /// <summary>
    /// Returns the world-space transform of the requested rigid body in the current frame.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <param name="x">(Output) X position of the rigid body, in meters.</param>
    /// <param name="y">(Output) Y position of the rigid body, in meters.</param>
    /// <param name="z">(Output) Z position of the rigid body, in meters.</param>
    /// <param name="qx">(Output) X component of the unit quaternion rotation of the rigid body.</param>
    /// <param name="qy">(Output) Y component of the unit quaternion rotation of the rigid body.</param>
    /// <param name="qz">(Output) Z component of the unit quaternion rotation of the rigid body.</param>
    /// <param name="qw">(Output) W component of the unit quaternion rotation of the rigid body.</param>
    /// <param name="yaw">(Output) Yaw rotation of the rigid body, in degrees.</param>
    /// <param name="pitch">(Output) Pitch rotation of the rigid body, in degrees.</param>
    /// <param name="roll">(Output) Roll rotation of the rigid body, in degrees.</param>
    /// <returns>True if the rigid body is found and its transform information was filled in. False otherwise.</returns>
    MOTIVE_API bool RigidBodyTransform( int rbIndex,
        float* x, float* y, float* z,
        float* qx, float* qy, float* qz, float* qw,
        float* yaw, float* pitch, float* roll );

    /// <summary>
    /// Remove the rigid body at the given index in the rigid body list.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <returns>kApiResult_Success if successful, or an error code if not.</returns>
    MOTIVE_API eResult RemoveRigidBody( int rbIndex );

    /// <summary>
    /// Enable or disable the rigid body at the given index in the rigid body list.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <param name="enabled">Pass true to enable the rigid body, or false to disable it.</param>
    MOTIVE_API void SetRigidBodyEnabled( int rbIndex, bool enabled );

    /// <summary>
    /// Determine if a rigid body is enabled in the current frame.
    /// </summary>
    /// <param name="rbIndex">Index into the rigid body list.</param>
    /// <returns>True if the rigid is enabled. False otherwise.</returns>
    MOTIVE_API bool RigidBodyEnabled( int rbIndex );

    /// <summary>
    /// Translate the pivot point (origin) of a defined rigid body.
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <param name="x">X amount to translate the pivot by, in meters.</param>
    /// <param name="y">Y amount to translate the pivot by, in meters.</param>
    /// <param name="z">Z amount to translate the pivot by, in meters.</param>
    /// <returns>kApiResult_Success if successful, or an error code if not.</returns>
    MOTIVE_API eResult RigidBodyTranslatePivot( int rbIndex, float x, float y, float z );

    /// <summary>
    /// Orient the pivot point of a defined rigid body with the world-space coordinate frame.
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <returns>True if the rigid body was found and reoriented. False otherwise.</returns>
    MOTIVE_API bool RigidBodyResetOrientation( int rbIndex );

    /// <summary>
    /// Retrieve the number of marker constraints associated with a defined rigid body.
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <returns>Total number of marker constraints defined for the rigid body.</returns>
    MOTIVE_API int RigidBodyMarkerCount( int rbIndex );

    /// <summary>
    /// Retrieve the positional offset of a marker constraint from a defined rigid body.
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <param name="markerIndex">Index of the marker constriant in the rigid body's constraint list.</param>
    /// <param name="x">X offset of the marker constraint, in meters.</param>
    /// <param name="y">Y offset of the marker constraint, in meters.</param>
    /// <param name="z">Z offset of the marker constraint, in meters.</param>
    /// <returns>True if the rigid body and constraint were found and the marker's information was returned.</returns>
    MOTIVE_API bool RigidBodyMarker( int rbIndex, int markerIndex, float* x, float* y, float* z );

    /// <summary>
    /// Set the positional offset of a marker constraint for a defined rigid body.
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <param name="markerIndex">Index of the marker constriant in the rigid body's constraint list.</param>
    /// <param name="x">X offset of the marker constraint, in meters.</param>
    /// <param name="y">Y offset of the marker constraint, in meters.</param>
    /// <param name="z">Z offset of the marker constraint, in meters.</param>
    /// <returns>True if the rigid body and constraint were found and the marker's information was set.</returns>
    MOTIVE_API bool RigidBodyUpdateMarker( int rbIndex, int markerIndex, float x, float y, float z );

    /// <summary>
    /// Retrieve the reconstructed marker location for a marker constraint on a defined rigid body in the current frame.
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <param name="markerIndex">Index of the marker constriant in the rigid body's constraint list.</param>
    /// <param name="tracked">True if the marker constraint has a corresponding reconstruction in the current frame.</param>
    /// <param name="x">X position of the reconstructed marker, in meters.</param>
    /// <param name="y">Y position of the reconstructed marker, in meters.</param>
    /// <param name="z">Z position of the reconstructed marker, in meters.</param>
    /// <returns>True if the rigid body and corresponding reconstruction were found and the marker's information was returned.</returns>
    MOTIVE_API bool RigidBodyReconstructedMarker( int rbIndex, int markerIndex, bool& tracked,
        float& x, float& y, float& z );

    /// <summary>
    /// Get mean error of tracked rigid body. (in meters)
    /// </summary>
    /// <param name="rbIndex">Index of the rigid body in the rigid body list.</param>
    /// <returns>Average distance between the constraint location and the corresponding reconstructed marker, for all constraints.</returns>
    MOTIVE_API float RigidBodyMeanError( int rbIndex );

    // Rigid Body Refinement ===============================================================================

    /// <summary>
    /// Start a rigid body refinement on the given rigid body.
    /// </summary>
    /// <param name="rigidBodyID">Unique ID of the rigid body, as returned from RigidBodyID().</param>
    /// <param name="sampleCount">Number of target samples to collect for refinement.</param>
    /// <returns>True if the rigid body was found and refinement was started on it.</returns>
    MOTIVE_API bool RigidBodyRefineStart( Core::cUID rigidBodyID, int sampleCount );

    /// <summary>
    /// Call for every frame after calling RigidBodyRefineStart.
    /// </summary>
    /// <remarks>This will allow the refinement process to collect samples.  You can check the progress
    /// of sampling by calling RigidBodyRefineProgress() and it will report a percentage (0-1) of the
    /// total samples collected.  The refinement process will not collect samples when the rigid body is untracked.
    /// </remarks>
    /// <returns>True if a refinement is in progress.</returns>
    MOTIVE_API bool RigidBodyRefineSample();

    /// <summary>
    /// Valid states for the rigid body refine solver.
    /// </summary>
    enum eRigidBodyRefineState
    {
        RigidBodyRefine_Initialized = 0,
        RigidBodyRefine_Sampling,
        RigidBodyRefine_Solving,
        RigidBodyRefine_Complete,
        RigidBodyRefine_Uninitialized
    };

    /// <summary>
    /// Current state of the rigid body refinement solver.
    /// </summary>
    /// <returns>Current state of the solver.</returns>
    MOTIVE_API eRigidBodyRefineState RigidBodyRefineState();

    /// <summary>
    /// Retrieve the overall sampling progress of the rigid body refinement solver.
    /// Progress is reported as a percentage of the total samples.
    /// </summary>
    /// <returns>Percentage of progress, expressed in the range [0,1].</returns>
    MOTIVE_API float RigidBodyRefineProgress();

    /// <summary>
    /// Retrieve the rigid body refinement solver error value prior to any sampling or solving.
    /// </summary>
    /// <remarks>
    /// Once RigidBodyRefine_Complete state is reached, you can use RigidBodyRefineInitialError() and
    /// RigidBodyRefineResultError() to determine if the result has improved prior to calling 
    /// RigidBodyRefineApplyResult().
    /// </remarks>
    /// <returns>The initial solver error value.</returns>
    MOTIVE_API float RigidBodyRefineInitialError();

    /// <summary>
    /// Retrieve the final solver error value after rigid body sampling and refining.
    /// </summary>
    /// <returns>The resultant solver error value.</returns>
    MOTIVE_API float RigidBodyRefineResultError();

    /// <summary>
    /// Apply the resulting rigid body refinement result.
    /// </summary>
    /// <returns>True if a valid rigid body refinement result existed and was applied.</returns>
    MOTIVE_API bool RigidBodyRefineApplyResult();

    /// <summary>
    /// Discard the resulting rigid body refinement result.
    /// </summary>
    /// <returns>True if a valid rigid body refinement result existed and was discarded.</returns>
    MOTIVE_API bool RigidBodyRefineReset();

    // Rigid Body Pivot Location Solver ====================================================================

    /// <summary>
    /// Start a rigid body pivot solve on the given rigid body.
    /// </summary>
    /// <param name="rigidBodyID">Unique ID of the rigid body, as returned from RigidBodyID().</param>
    /// <param name="sampleCount">Number of target samples to collect for pivot solving.</param>
    /// <returns></returns>
    MOTIVE_API bool RigidBodyPivotSolverStart( Core::cUID rigidBodyID, int sampleCount );

    /// <summary>
    /// Call for every frame after calling RigidBodyPivotSolverStart.
    /// </summary>
    /// <remarks>This will allow the pivot solving process to collect samples. You can check the progress
    /// of sampling by calling RigidBodyPivotSolverProgress() and it will report a percentage (0-1) of the
    /// total samples collected. The solving process will not collect samples when the rigid body is untracked.
    /// </remarks>
    /// <returns>True if a refinement is in progress.</returns>
    MOTIVE_API bool RigidBodyPivotSolverSample();

    /// <summary>
    /// Valid states for the rigid body pivot solver.
    /// </summary>
    enum eRigidBodyPivotSolverState
    {
        RigidBodyPivotSolver_Initialized = 0,
        RigidBodyPivotSolver_Sampling,
        RigidBodyPivotSolver_Solving,
        RigidBodyPivotSolver_Complete,
        RigidBodyPivotSolver_Uninitialized
    };

    /// <summary>
    /// Current state of the rigid body pivot solver.
    /// </summary>
    /// <returns>Current state of the solver.</returns>
    MOTIVE_API eRigidBodyPivotSolverState RigidBodyPivotSolverState();

    /// <summary>
    /// Retrieve the overall sampling progress of the rigid body pivot solver.
    /// Progress is reported as a percentage of the total samples.
    /// </summary>
    /// <returns>Percentage of progress, expressed in the range [0,1].</returns>
    MOTIVE_API float RigidBodyPivotSolverProgress();

    /// <summary>
    /// Retrieve the rigid body refinement solver error value prior to any sampling or solving.
    /// </summary>
    /// <remarks>
    /// Once RigidBodyPivotSolver_Complete state is reached, you can use RigidBodyPivotSolverInitialError() and
    /// RigidBodyPivotSolverResultError() to determine if the result has improved prior to calling 
    /// RigidBodyPivotSolverApplyResult().
    /// </remarks>
    /// <returns>The initial solver error value.</returns>
    MOTIVE_API float RigidBodyPivotSolverInitialError();

    /// <summary>
    /// Retrieve the final solver error value after rigid body sampling and pivot solving.
    /// </summary>
    /// <returns>The resultant solver error value.</returns>
    MOTIVE_API float RigidBodyPivotSolverResultError();

    /// <summary>
    /// Apply the resulting rigid body pivot solving result.
    /// </summary>
    /// <returns>True if a valid rigid body pivot solve result existed and was applied.</returns>
    MOTIVE_API bool RigidBodyPivotSolverApplyResult();

    /// <summary>
    /// Discard the resulting rigid body pivot solve result.
    /// </summary>
    /// <returns>True if a valid rigid body pivot solver result existed and was discarded.</returns>
    MOTIVE_API bool RigidBodyPivotSolverReset();

    // Application Settings ================================================================================

    /// <summary>
    /// Retrieve a property's data type, which is useful to deduce a property type before calling SetProperty.
    /// </summary>
    /// <param name="propertyName">Name of the property to query.</param>
    /// <returns>If an invalid property name is specified, eInvalid is returned.</returns>
    MOTIVE_API ePropertyDataType ApplicationPropertyType( const std::wstring& propertyName );

    /// <summary>
    /// Retrieve the property value embedded in a sPropertyValue structure.
    /// </summary>
    /// <param name="propertyName">Name of the property to query.</param>
    /// <returns>If an invalid property name is specified, the return value's mDataType member will be set to eInvalid.</returns>
    MOTIVE_API sPropertyValue ApplicationProperty( const std::wstring& propertyName );

    /// <summary>
    /// Set the value of an application property.
    /// </summary>
    /// <remarks>
    /// Application property names:
    /// <list type="bullet">
    /// <item><description>PCResidual (double)</description></item>
    /// <item><description>PCMinRayLength (double)</description></item>
    /// <item><description>PCMaxRayLength (double)</description></item>
    /// <item><description>PCObjectFilterLevel (int)</description></item>
    /// <item><description>PCObjectFilterMinSize (int)</description></item>
    /// <item><description>PCObjectFilterMaxSize (int)</description></item>
    /// <item><description>PCObjectFilterGrayscaleFloor (int)</description></item>
    /// <item><description>PCObjectFilterAspectTolerance (int)</description></item>
    /// <item><description>PCObjectFilterObjectMargin (int)</description></item>
    /// <item><description>PCObjectFilterCircularity (int)</description></item>
    /// <item><description>BoundReconstruction (bool)</description></item>
    /// <item><description>ReconstructionBoundsShape (int)</description></item>
    /// <item><description>ReconstructionBoundsXCenter (double)</description></item>
    /// <item><description>ReconstructionBoundsYCenter (double)</description></item>
    /// <item><description>ReconstructionBoundsZCenter (double)</description></item>
    /// <item><description>ReconstructionBoundsXWidth (double)</description></item>
    /// <item><description>ReconstructionBoundsYWidth (double)</description></item>
    /// <item><description>ReconstructionBoundsZWidth (double)</description></item>
    /// <item><description>ReconstructionBoundsRadius (double)</description></item>
    /// <item><description>ReconstructionBoundsHeight (double)</description></item>
    /// <item><description>ContinuousCameraCalibration (bool)</description></item>
    /// <item><description>ShutterDelay (int)</description></item>
    /// <item><description>SynchronizerControl (int)</description></item>
    /// <item><description>PrecisionPacketCap (int)</description></item>
    /// <item><description>MaskPadding (double)</description></item>
    /// <item><description>MaskingAdditive (bool)</description></item>
    /// <item><description>MarkerDiameterType (int)</description></item>
    /// <item><description>MarkerDiameterForceSize (double)</description></item>
    /// </list>
    /// </remarks>
    /// <param name="propertyName">Name of the property to set.</param>
    /// <param name="value">Value to set the property to.</param>
    /// <returns>True if the property was found and the requested value was set.</returns>
    MOTIVE_API bool SetApplicationProperty( const std::wstring& propertyName, const sPropertyValue& value );

    // Filter switches are enabled by default 

    /// <summary>
    /// Enable or disable filter switches on all cameras that support them.
    /// </summary>
    /// <param name="enabled">True to enable filter switching.</param>
    MOTIVE_API void SetEnabledFilterSwitch( bool enabled );

    /// <summary>
    /// Determine if camera filter switches are enabled for use.
    /// </summary>
    /// <returns>True if camera filter switching is enabled.</returns>
    MOTIVE_API bool IsFilterSwitchEnabled();

    // Camera Interface ====================================================================================

    /// <summary>
    /// Retrieve the current camera count.
    /// </summary>
    /// <returns>Total camera count.</returns>
    MOTIVE_API int CameraCount();

    /// <summary>
    /// Retrieve the total number of camera groups.
    /// </summary>
    /// <remarks>
    /// This will always be two. One for tracking cameras and one for reference cameras.
    /// </remarks>
    /// <returns></returns>
    MOTIVE_API int CameraGroupCount();

    /// <summary>
    /// Determine which camera group a given camera is in.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Zero if camera is in the Tracking group, one if it is in the reference group, and -1 if not found.</returns>
    MOTIVE_API int CameraGroup( int cameraIndex );

    /// <summary>
    /// Retrieve the serial number of a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Camera serial number.</returns>
    MOTIVE_API int CameraSerial( int cameraIndex );

    /// <summary>
    /// Maps a camera serial number to an index in the camera list.
    /// </summary>
    /// <param name="serial">Camera serial number to map.</param>
    /// <returns>The index of the camera in the camera list, or -1 if not found.</returns>
    MOTIVE_API int CameraIndexFromSerial( int serial );

    /// <summary>
    /// Retrieve a camera's world-space position.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="x">(Output) X position, in meters</param>
    /// <param name="y">(Output) Y position, in meters</param>
    /// <param name="z">(Output) Z position, in meters</param>
    /// <returns>True if the camera was found and the position values were filled in.</returns>
    MOTIVE_API bool CameraPosition( int cameraIndex, float& x, float& y, float& z );

    /// <summary>
    /// Retrieve a camera's position and orientation.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="position">(Output) A 3-vector of the X, Y, and Z position of the camera, in meters.</param>
    /// <param name="orientation">(Output) A 3x3 rotation matrix for the camera</param>
    /// <returns>True if the camera was found and the position and orientation values were filled in.</returns>
    MOTIVE_API bool CameraPositionOrientation( int cameraIndex, float position[3], float orientation[9] );

    /// <summary>
    /// Retrieve the number of objects detected by a camera in the current frame.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Total object count.</returns>
    MOTIVE_API int CameraObjectCount( int cameraIndex );

    /// <summary>
    /// Retrieve the 2D coordinates of a detected object (centroid) in a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="objectIndex">Index of the object. Use CameraObjectCount() to determine object list size.</param>
    /// <param name="x">(Output) X coordinate of the object in the view of the camera.</param>
    /// <param name="y">(Output) Y coordinate of the object in the view of the camera.</param>
    /// <returns>True if the camera and object were found and the coordinates were filled in.</returns>
    MOTIVE_API bool CameraObject( int cameraIndex, int objectIndex, float& x, float& y );

    /// <summary>
    /// Retrieve the pre-distorted object location in the view of the camera.
    /// </summary>
    /// <remarks>
    /// This is basically where the camera would see the object if there was no lens distortion.
    /// For most NaturalPoint cameras/lenses, this location is only a few pixels from the distorted position.
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="objectIndex">Index of the object. Use CameraObjectCount() to determine object list size.</param>
    /// <param name="x">(Output) X coordinate of the object in the view of the camera.</param>
    /// <param name="y">(Output) Y coordinate of the object in the view of the camera.</param>
    /// <returns>True if the camera and object were found and the coordinates were filled in.</returns>
    MOTIVE_API bool CameraObjectPredistorted( int cameraIndex, int objectIndex, float& x, float& y );

    /// <summary>
    /// Possible values of the "CameraNodeCameraVideoMode" property.
    /// </summary>
    enum eVideoType
    {
        kVideoType_Segment = 0,
        kVideoType_Grayscale = 1,
        kVideoType_Object = 2,
        kVideoType_Precision = 4,
        kVideoType_MJPEG = 6,
        kVideoType_ColorH264 = 9
    };

    /// <summary>
    /// MJPEG Quality Settings
    /// <remarks>
    /// Flex13 and Ethernet cameras support all four quality settings. All other cameras are limited
    /// to LowQuality and HighQuality.
    /// <remarks>
    enum eMJPEGQuality
    {
        kMJPEG_MinQuality = 0,
        kMJPEG_LowQuality = 1,
        kMJPEG_StandardQuality = 2,
        kMJPEG_HighQuality = 3
    };

	/// <summary>
    /// Returns properties names for properties found on the camera
	/// Retrieve a list of cameras properties
	/// </summary>
	/// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="properties">list names of properties on the camera .</param>/// 
	/// <returns> is successfull </returns>
    MOTIVE_API bool CameraPropertyNames( int cameraIndex, std::vector<std::wstring>& propertyNames );

    /// <summary>
    /// Retrieve a property's data type, which is useful to deduce a property type before calling SetProperty.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="propertyName">Name of the property to query.</param>
    /// <returns>If an invalid property name is specified, eInvalid is returned. Otherwise, the data type
    /// of the requested property is returned.</returns>
    MOTIVE_API ePropertyDataType CameraPropertyType( int cameraIndex, const std::wstring& propertyName );

    /// <summary>
    /// Set the value of a camera property.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="propertyName">Name of the property to set.</param>
    /// <param name="value">Value to set the property to.</param>
    /// <returns>True if the property was found and the value was set.</returns>
    MOTIVE_API bool SetCameraProperty( int cameraIndex, const std::wstring& propertyName, const sPropertyValue& value );

    /// <summary>
    /// Retrieve the value of a camera property.
    /// </summary>
    /// <remarks>
    /// Valid camera property names:
    /// <list type="bullet">
    /// <item><description>NodeName (string)</description></item>
    /// <item><description>CameraNodeCameraEnabled (bool)</description></item>
    /// <item><description>CameraNodeReconstructionEnabled (bool)</description></item>
    /// <item><description>CameraNodeImagerPixelSize (Vector2)</description></item>
    /// <item><description>CameraNodeCameraVideoMode (int)</description></item>
    /// <item><description>CameraNodeCameraExposure (int)</description></item>
    /// <item><description>CameraNodeCameraThreshold (int)</description></item>
    /// <item><description>CameraNodeCameraLED (bool)</description></item>
    /// <item><description>CameraNodeCameraIRFilterEnabled (bool)</description></item>
    /// <item><description>CameraNodeCameraGain (int)</description></item>
    /// <item><description>CameraNodeCameraFrameRate (double)</description></item>
    /// <item><description>CameraNodeCameraMJPEGQuality (int)</description></item>
    /// <item><description>CameraNodeCameraMaximizePower (bool)</description></item>
    /// <item><description>CameraNodeBitrate (int)</description></item>
    /// <item><description>CameraNodePartition (int)</description></item>
    /// </list>
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="propertyName">Name of the property to set.</param>
    /// <returns>The value of the property. If the camera or property could not be found, the mDataType member
    /// of the returned structure will be set to eInvalid.</returns>
    MOTIVE_API sPropertyValue CameraProperty( int cameraIndex, const std::wstring& propertyName );

    // Camera's Full Frame Grayscale Decimation

    /// <summary>
    /// Set the grayscale decimation level for cameras that support it.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="value">Grayscale decimation level.</param>
    /// <returns>True if the camera was found and the value was set.</returns>
    MOTIVE_API bool SetCameraGrayscaleDecimation( int cameraIndex, int value );

    /// <summary>
    /// Retrieve a camera's current grayscale decimation level.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Current grayscale decimation level.</returns>
    MOTIVE_API int CameraGrayscaleDecimation( int cameraIndex );

	/// <summary>
	/// Set the cameras videoType
	/// </summary>
	/// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="videoType"> EVideo Type</param>
    /// <returns>True if value is setthe camera was found and does support continuous illumination.</returns>/// 
    MOTIVE_API bool SetCameraVideoMode( int cameraIndex, eVideoType videoType );

    /// <summary>
    /// Determine if a camera support continuous (un-strobed) illumination.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>True if the camera was found and does support continuous illumination.</returns>
    MOTIVE_API bool CameraIsContinuousIRAvailable( int cameraIndex );

    /// <summary>
    /// Turn on continuous illumination for a camera, if the camera supports it.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="enable">True to turn on continuous illumination, false otherwise.</param>
    /// <returns>True if the camera was found and the value was set.</returns>
    MOTIVE_API bool CameraSetContinuousIR( int cameraIndex, bool enable );

    /// <summary>
    /// Determine if a camera's continuous illumination is currently on.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>True if the camera was found and continuous illumination is on.</returns>
    MOTIVE_API bool CameraContinuousIR( int cameraIndex );

    /// <summary>
    /// Set the master frame rate of the camera system.
    /// </summary>
    /// <remarks>
    /// All tracking cameras must run at the same frame rate.
    /// </remarks>
    /// <param name="frameRate">Requested camera system master frame rate, in frames/sec.</param>
    /// <returns>True if the frame rate was set.</returns>
    MOTIVE_API bool SetCameraSystemFrameRate( int frameRate );

    /// <summary>
    /// Retrieve the current master system frame rate.
    /// </summary>
    /// <returns>Nominal master system frame rate, in frames/sec.</returns>
    MOTIVE_API int CameraSystemFrameRate();

    /// <summary>
    /// Set the frame rate divisor for a reference camera.
    /// </summary>
    /// <remarks>
    /// Reference cameras can be made to run at a reduced rate by setting their frame rate divisor.
    /// The system_rate/divisor must result in a whole number for the divisor to be valid.
    /// A camera must first be made a reference camera, by switching its video type, prior to applying a divisor
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="divisor">The requested frame rate divisor.</param>
    /// <returns>True if the camera was found, was in a reference video mode, and the divisor was set.</returns>
    MOTIVE_API bool SetCameraFrameRateDivisor( int cameraIndex, int divisor );

    // Returns the measured camera system frame rate

    /// <summary>
    /// Retrieve the current measured master system frame rate.
    /// </summary>
    /// <returns>Actual system frame rate, in frames/sec.</returns>
    MOTIVE_API double MeasuredIncomingFrameRate();

    /// <summary>
    /// Retrieves a measure of the total current incoming data rate.
    /// </summary>
    /// <returns>Total incoming data rate, in bytes/sec.</returns>
    MOTIVE_API double MeasuredIncomingDataRate();

    /// <summary>
    /// Retrieve the measured internal temperature of a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Internal camera temperature, in degrees Celsius, or zero if the camera was not found
    /// or temperature measurements are not supported.</returns>
    MOTIVE_API float CameraTemperature( int cameraIndex );

    /// <summary>
    /// Retrieve the internal measured temperature of a camera's ring light.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Internal ring light temperature, in degrees Celsius, or zero if the camera was not found
    /// or temperature measurement is not supported.</returns>
    MOTIVE_API float CameraRinglightTemperature( int cameraIndex );

	/// <summary>
	/// Sets the cameras Ring Light to RGB color
	/// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
	/// <param name="R"> 0-225 red.</param>
    /// <param name="G"> 0-225 green.</param>
    /// <param name="B"> 0-225 blue.</param>
    /// <returns>True if the camera was found and the values were set.</returns>
    MOTIVE_API bool  SetCameraRingStatusRingRGB( int cameraIndex, unsigned char R, unsigned char G, unsigned char B );

    /// <summary>
    /// Enable or disable Automatic Gain Control for a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="enable">True to enable auto gain control. False to disable.</param>
    /// <returns>True if the camera was found and the property value was set.</returns>
    MOTIVE_API bool SetCameraAGC( int cameraIndex, bool enable );

    /// <summary>
    /// Enable or disable Automatic Exposure Control for a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="enable">True to enable auto exposure control. False to disable.</param>
    /// <returns>True if the camera was found and the property value was set.</returns>
    MOTIVE_API bool SetCameraAEC( int cameraIndex, bool enable );

    /// <summary>
    /// Retrieve the number of available imager gain levels for a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>Number of available gain levels, or zero if the camera was not found.</returns>
    MOTIVE_API int CameraImagerGainLevels( int cameraIndex );

    // Camera Masking

    /// <summary>
    /// Clear all blocking masks for a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>True if the camera was found and its masks were cleared.</returns>
    MOTIVE_API bool ClearCameraMask( int cameraIndex );

    /// <summary>
    /// Set the full blocking mask for a camera from a memory buffer.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="buffer">Memory buffer to read from.</param>
    /// <param name="bufferSize">Size of the memory buffer.</param>
    /// <returns>True if the camera was found and the blocking mask was set from the given data.</returns>
    MOTIVE_API bool SetCameraMask( int cameraIndex, unsigned char* buffer, int bufferSize );

    /// <summary>
    /// Retrieve the current blocking mask for a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="buffer">(Output) Memory buffer to write to.</param>
    /// <param name="bufferSize">Size of the memory buffer.</param>
    /// <returns>True if the camera was found and the blocking mask was written into the buffer.</returns>
    MOTIVE_API bool CameraMask( int cameraIndex, unsigned char* buffer, int bufferSize );

    /// <summary>
    /// Retrieve the dimensions and pixel block size for a camera's blocking mask.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="blockingMaskWidth">(Output) Width of blocking mask, in grid units.</param>
    /// <param name="blockingMaskHeight">(Output) Height of blocking mask, in grid units.</param>
    /// <param name="blockingMaskGrid">(Output) Number of pixels per grid unit.</param>
    /// <returns>True if the camera was found and the output values were filled.</returns>
    MOTIVE_API bool CameraMaskInfo( int cameraIndex, int& blockingMaskWidth, int& blockingMaskHeight,
        int& blockingMaskGrid );

    /// <summary>
    /// Determine if a camera is detecting any objects with the current blocking mask.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>True if the camera was found and detects more than zero objects in the current frame.</returns>
    MOTIVE_API bool CameraHasVisibleObjects( int cameraIndex );

    /// <summary>
    /// Performs auto-masking for all tracking cameras.
    /// </summary>
    /// <remarks>
    /// This is additive to any existing masking. To clear masks on a camera, call ClearCameraMask prior to auto-masking.
    /// </remarks>
    MOTIVE_API void AutoMaskAllCameras();

    /// <summary>
    /// Valid camera enabled/disabled and reconstruction states.
    /// </summary>
    enum eCameraState
    {
        Camera_Enabled = 0,
        Camera_Disabled_For_Reconstruction = 1,
        Camera_Disabled = 2
    };

    /// <summary>
    /// Set a camera's participation state.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="state">The requested state.</param>
    /// <returns>True if the camera was found and the state was set.</returns>
    MOTIVE_API bool SetCameraState( int cameraIndex, eCameraState state );

    /// <summary>
    /// Retrieve the current participation state of a camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="currentState">(Output) The current state.</param>
    /// <returns>True if the camera was found and the output valus was set.</returns>
    MOTIVE_API bool CameraState( int cameraIndex, eCameraState& currentState );

    /// <summary>
    /// Retrieve an ordering camera ID for a camera.
    /// </summary>
    /// <remarks>
    /// Camera ID's may change and are usually set in an ordered way based on the arrangement of cameras
    /// in a volume, as calculated from a camera calibration.
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>The ID of the camera, or zero if it was not found.</returns>
    MOTIVE_API int CameraID( int cameraIndex );

    /// <summary>
    /// Retrieve a camera's current frame buffer.
    /// </summary>
    /// <remarks>
    /// This function fills the provided buffer with an image of what is in the camera view.
    /// The resulting image depends on what video mode the camera is in.  If the camera is in grayscale mode,
    /// for example, a grayscale image is returned from this call.
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="bufferPixelWidth">Pixel width of the buffer.</param>
    /// <param name="bufferPixelHeight">Pixel height of the buffer.</param>
    /// <param name="bufferByteSpan">Number of bytes per pixel row of the buffer. Must be greater than or equal
    /// to bufferPixelWidth * bufferPixelBitDepth</param>
    /// <param name="bufferPixelBitDepth">Number of bits per pixel.</param>
    /// <param name="buffer">(Output) The memory buffer to write to, which should be
    /// bufferPixelWidth * bufferByteSpan * bufferPixelBitDepth in size.</param>
    /// <returns></returns>
    MOTIVE_API bool CameraFrameBuffer( int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,
        int bufferByteSpan, int bufferPixelBitDepth, unsigned char* buffer );

    /// <summary>
    /// Write a camera's current frame buffer to a BMP image file.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="filename">File to write to.</param>
    /// <returns>True if the camera was found and the frame buffer was written to the requested file.</returns>
    MOTIVE_API bool CameraFrameBufferSaveAsBMP( int cameraIndex, const wchar_t* filename );

    /// <summary>
    /// Back-project a 3D coordinate to 2D coordinates in the view of a camera.
    /// </summary>
    /// <remarks>
    /// If you give this function a 3D location, it will return where the point would land on the camera imager.
    /// This is a relatively expensive function call, so use sparingly.
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="x">X coordinate of the 3D point, in meters.</param>
    /// <param name="y">Y coordinate of the 3D point, in meters.</param>
    /// <param name="z">Z coordinate of the 3D point, in meters.</param>
    /// <param name="cameraX">(Output) X coordinate of the 2D projection of the 3D point in the camera's view, in pixels.</param>
    /// <param name="cameraY">(Output) Y coordinate of the 2D projection of the 3D point in the camera's view, in pixels.</param>
    MOTIVE_API void CameraBackproject( int cameraIndex, float x, float y, float z, float& cameraX,
        float& cameraY );

    /// <summary>
    /// Map a distorted 2D coordinate to an undistorted one for a given camera.
    /// </summary>
    /// <remarks>
    /// The raw 2D centroids a camera reports are distorted by the lens.  To remove the distortion call
    /// this method.  Also if you have a 2D undistorted point that you'd like to convert back
    /// to a distorted point call CameraDistort2DPoint.
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="x">(Input/Output) The X coordinate of the point, in pixels.</param>
    /// <param name="y">(Input/Output) The Y coordinate of the point, in pixels.</param>
    /// <returns>True if the camera was found and the coordinate values were replaced with undistorted values.</returns>
    MOTIVE_API bool CameraUndistort2DPoint( int cameraIndex, float& x, float& y );

    /// <summary>
    /// Map an undistorted 2D coordinate to a distorted one for a given camera.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="x">(Input/Output) The X coordinate of the point, in pixels.</param>
    /// <param name="y">(Input/Output) The Y coordinate of the point, in pixels.</param>
    /// <returns>True if the camera was found and the coordinate values were replaced with distorted values.</returns>
    MOTIVE_API bool CameraDistort2DPoint( int cameraIndex, float& x, float& y );

    /// <summary>
    /// Map undistorted coordinates from a camera to a 3D ray.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="x">X coordinate of the undistorted point, in pixels.</param>
    /// <param name="y">Y coordinate of the undistorted point, in pixels.</param>
    /// <param name="rayStartX">(Output) The X coordinate of the ray origin, in meters.</param>
    /// <param name="rayStartY">(Output) The Y coordinate of the ray origin, in meters.</param>
    /// <param name="rayStartZ">(Output) The Z coordinate of the ray origin, in meters.</param>
    /// <param name="rayEndX">(Output) The X coordinate of a point along the ray, in meters.</param>
    /// <param name="rayEndY">(Output) The Y coordinate of a point along the ray, in meters.</param>
    /// <param name="rayEndZ">(Output) The Z coordinate of a point along the ray, in meters.</param>
    /// <returns>True if the camera was found and the coordinate values were filled on the outputs.</returns>
    MOTIVE_API bool CameraRay( int cameraIndex, float x, float y, float& rayStartX, float& rayStartY,
        float& rayStartZ, float& rayEndX, float& rayEndY, float& rayEndZ );

    /// <summary>
    /// Set a camera's extrinsics (position & orientation).
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="x">X coordinate of the camera position, in meters.</param>
    /// <param name="y">Y coordinate of the camera position, in meters.</param>
    /// <param name="z">Z coordinate of the camera position, in meters.</param>
    /// <param name="orientation">3x3 orientation matrix.</param>
    /// <returns>True if the camera was found and the extrinsics were set.</returns>
    MOTIVE_API bool SetCameraPose( int cameraIndex, float x, float y, float z,
        const float* orientation );

    /// <summary>
    /// Retrieve a CameraLibrary Camera object.
    /// </summary>
    /// <remarks>This is for use when also linking to the CameraLibrary, which will allow more fine-grained
    /// use of camera instances.
    /// While the API takes over the data path which prohibits fetching the frames directly from the camera,
    /// it may still be useful to be able to communicate with the camera directly.
    /// </remarks>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <returns>A shared pointer to the CameraLibrary Camera instance, or null if not found.</returns>
    MOTIVE_API std::shared_ptr<CameraLibrary::Camera> GetCamera( int cameraIndex );

    // Additional Functionality ============================================================================

    /// <summary>
    /// Set the system to use either frame ID-based timing or system clock timing.
    /// </summary>
    /// <param name="enable">True to enable frame ID-based timing (the default).</param>
    MOTIVE_API void SetFrameIDBasedTiming( bool enable );

    /// <summary>
    /// Allow frame ID's to roll over without reporting a frame discontinuity.
    /// </summary>
    /// <param name="enable"> True to allow frame ID rollover to happen smoothly, the default.</param>
    MOTIVE_API void SetSuppressOutOfOrder( bool enable );

    /// <summary>
    /// Attach a camera module to a Camera instance.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="module">The module to attach to the camera.</param>
    /// <returns>True if the camera was found and the module was attached.</returns>
    MOTIVE_API bool AttachCameraModule( int cameraIndex, CameraLibrary::cCameraModule* module );

    /// <summary>
    /// Remove a camera module from a Camera instance.
    /// </summary>
    /// <param name="cameraIndex">Index of the camera in the camera list.</param>
    /// <param name="module">The module to searchfor and remove.</param>
    /// <returns>True if the camera and module were found and the module was removed.</returns>
    MOTIVE_API bool DetachCameraModule( int cameraIndex, CameraLibrary::cCameraModule* module );

    /// <summary>
    /// Reorient a Duo or Trio device in world space.
    /// </summary>
    /// <param name="positionX">Requested X position, in meters.</param>
    /// <param name="positionY">Requested Y position, in meters.</param>
    /// <param name="positionZ">Requested Z position, in meters.</param>
    /// <param name="orientationX">Requested unit quaternion rotation X component.</param>
    /// <param name="orientationY">Requested unit quaternion rotation Y component.</param>
    /// <param name="orientationZ">Requested unit quaternion rotation Z component.</param>
    /// <param name="orientationW">Requested unit quaternion rotation W component.</param>
    /// <returns>kApiResult_Success, or an error code.</returns>
    MOTIVE_API eResult OrientTrackingBar( float positionX, float positionY, float positionZ,
        float orientationX, float orientationY, float orientationZ, float orientationW );

    // Camera Manager Access ===============================================================================

    /// <summary>
    /// Retrieve a pointer to the CameraManager singleton.
    /// </summary>
    /// <remarks>
    /// When using the Motive API in conjunction with the Camera SDK, this method will provide access to
    /// the manager class that owns all Camera instances. From here, many system state properties can be
    /// set or queried, cameras can be queried or edited, etc.
    /// <returns>The singleton CameraManager.</returns>
    MOTIVE_API CameraLibrary::CameraManager* CameraManager();

    // API Callbacks =======================================================================================

    /// <summary>
    /// Inherit cAPIListener and override it's methods to receive callbacks from the Motive API.
    /// </summary>
    class MOTIVE_API cAPIListener
    {
    public:
        virtual ~cAPIListener() = default;

        /// <summary>
        /// Called when a new synchronized group of camera frames has been delivered and is ready for processing.
        /// </summary>
        /// <remarks>
        /// You should do minimal processing in this callback, like setting flags in your program to indicate that
        /// a frame is available. You can then use that flag state to determine when to call Update() without
        /// having to poll blindly.
        /// </remarks>
        virtual void FrameAvailable() = 0;

        /// <summary>
        /// Called when a camera becomes connected to the system.
        /// </summary>
        virtual void CameraConnected( int serialNumber ) = 0;

        /// <summary>
        /// Called when a camera becomes disconnected from the system.
        /// </summary>
        virtual void CameraDisconnected( int serialNumber ) = 0;
    };

    /// <summary>
    /// Attaches a cAPIListener-derived listener to the API.
    /// </summary>
    /// <remarks>
    /// Only one listener can be attached to the API at a time.
    /// </remarks>
    /// <param name="listener">The listener instance to attach. This is a borrowed reference and will not be deleted.</param>
    MOTIVE_API void AttachListener( cAPIListener* listener );
    
    /// <summary>
    /// Detach any attached listener from the API.
    /// </summary>
    MOTIVE_API void DetachListener();

    // Result Processing ===================================================================================

	/// <summary>
	/// Maps a eCameraState value to a human-readable string.
	/// </summary>
	/// <param name="result">The value to map.</param>
	/// <returns>A string representation of the eResult value.</returns>
    /// 
	MOTIVE_API const std::wstring MapToCameraStateString( eCameraState result );

	/// <summary>
	/// Maps a ePropertyDataType to a string.
	/// </summary>
	/// <param name="result">The value to map.</param>
	/// <returns>A string representation of the property type.</returns>
	MOTIVE_API const std::wstring MapToPropertyTypeString( ePropertyDataType result );

		/// <summary>
	/// Maps a ePropertyValueType to a string.
	/// </summary>
	/// <param name="result">The value to map.</param>
	/// <returns>A string representation of the property value.</returns>
	MOTIVE_API const std::wstring MapToPropertyValueString( const sPropertyValue& value );

    /// <summary>
    /// Maps a eResult value to a human-readable string.
    /// </summary>
    /// <param name="result">The value to map.</param>
    /// <returns>A string representation of the eResult value.</returns>
    MOTIVE_API const std::wstring MapToResultString( eResult result );

    /// <summary>
    /// Map a Calibration Square type enum value to a human-readable string.
    /// </summary>
    /// <param name="tpye">Calibration square type</param>
    /// <returns>A string representation of the calibration square type.</returns>
    MOTIVE_API const std::wstring MapToCalibrationSquareString( eCalibrationSquareType type );

	/// <summary>
	/// Map a eVideoMode type enum value to a human-readable string.
	/// </summary>
	/// <param name="tpye">Video Mode type</param>
	/// <returns>A string representation of the Video Mode type.</returns>
	MOTIVE_API const std::wstring MapToVideoModeString( eVideoType type );
}
