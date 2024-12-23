//======================================================================================================
// Motive API
// Copyright 2010 NaturalPoint, Inc.
//
// The Motive API is designed to be a simple yet full featured interface to Motive
//======================================================================================================
#pragma once
#include <string>
#include <memory>
#include <vector>

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

#ifndef _CORE_UID_CLASS
#define _CORE_UID_CLASS

namespace Core
{
    /// <summary>
    /// A platform-neutral 128-bit universal identifier. It is essentially guaranteed to never
    /// generate the same ID twice.
    /// </summary>
    class cUID
    {
    public:
        typedef unsigned long long int uint64;

        /// <summary>
        /// Create a default UID. In order to create a UID that has a valid unique identifier you
        /// must call Generate().
        /// </summary>
        cUID() : mHighBits( 0 ), mLowBits( 0 ) { }

        cUID( uint64 high, uint64 low ) : mHighBits( high ), mLowBits( low ) { }

        /// <summary>
        /// Set the value of the UID from two long integer values. It is up to the caller to ensure that
        /// the resulting UID is unique.
        /// </summary>
        void SetValue( uint64 highBits, uint64 lowBits )
        {
            mHighBits = highBits;
            mLowBits = lowBits;
        }

        /// <summary>Get the low 64 bits of the UID.</summary>
        uint64 LowBits() const
        {
            return mLowBits;
        }

        /// <summary>Get the high 64 bits of the UID.</summary>
        uint64 HighBits() const
        {
            return mHighBits;
        }

        /// <summary>Returns true if the ID is valid (i.e. not equal to kInvalid).</summary>
        bool Valid() const
        {
            return !(mHighBits == 0 && mLowBits == 0);
        }

        /// <summary>Generate a new UID value.</summary>
        static cUID Generate();

        //==============================================================================================
        // Comparison operators
        //==============================================================================================

        bool operator<( const cUID&  rhs ) const
        {
            return ((mHighBits < rhs.mHighBits) ? true : (mHighBits == rhs.mHighBits ? (mLowBits < rhs.mLowBits) : false));
        }
        bool operator<=( const cUID&  rhs ) const
        {
            return ((mHighBits < rhs.mHighBits) ? true : (mHighBits == rhs.mHighBits ? (mLowBits <= rhs.mLowBits) : false));
        }

        bool operator>( const cUID&  rhs ) const
        {
            return !(*this <= rhs);
        }
        bool operator>=( const cUID&  rhs ) const
        {
            return !(*this < rhs);
        }

        // Inline these for performance.
        bool operator==( const cUID&  rhs ) const
        {
            return ((mHighBits == rhs.mHighBits) && (mLowBits == rhs.mLowBits));
        }

        bool operator!=( const cUID&  rhs ) const
        {
            return ((mHighBits != rhs.mHighBits) || (mLowBits != rhs.mLowBits));
        }

        //==============================================================================================
        // Constants
        //==============================================================================================

        static const cUID kInvalid;

    private:
        uint64 mHighBits;
        uint64 mLowBits;
    };
}
#endif // _CORE_UID_CLASS

#ifndef _CORE_LABEL_CLASS
#define _CORE_LABEL_CLASS
namespace Core
{
    /// <summary>A class that represents a marker label. Marker labels consist of two parts: The entity that the marker
    /// is associated with (e.g. skeleton, rigid body, etc.), and the (one-based) index into the label list for that entity.
    class cLabel
    {
    public:
        cLabel();
        cLabel( const cUID& entityID, unsigned int memberLabelIndex );

        /// <summary>The node ID for the entity that this label belongs to.</summary>
        const cUID& EntityID() const { return mEntityID; }

        /// <summary>The label ID within the entity.</summary>
        unsigned int MemberID() const { return mMemberLabelID; }

        /// <summary>True if the label has a non-null entity ID. Does not attempt to ensure that the entity ID
        /// is valid or that it belongs to an asset that has associated markers for labeling.</summary>
        bool Valid() const { return ( mEntityID != cUID::kInvalid ); }

        /// <summary>Parse fully qualified name into entity and member names.</summary>
        static void ParseName( const std::wstring& name, std::wstring& entityName, std::wstring& memberName );

        /// <summary>Comparison operators.</summary>
        bool operator==( const cLabel& other ) const;
        bool operator!=( const cLabel& other ) const;
        bool operator<( const cLabel&  rhs ) const
        {
            return ( ( mEntityID < rhs.mEntityID ) ? true : ( mEntityID == rhs.mEntityID ? ( mMemberLabelID < rhs.mMemberLabelID ) : false ) );
        }

        // Convenience constants
        static const cLabel kInvalid;

    private:
        cUID mEntityID;
        unsigned int mMemberLabelID;

        static const long long klabelIdentifier;
        static const long long kTypeMask;
        static bool LegacyIsLabel( const Core::cUID& uid, bool checkForValidType = false );
    };
}
#endif // _CORE_LABEL_CLASS

#ifndef _CORE_MARKER_CLASS
#define _CORE_MARKER_CLASS
namespace Core
{
    template <typename T>
    class cTMarker
    {
    public:
        cTMarker() : X( 0 ), Y( 0 ), Z( 0 ), ID( cUID::kInvalid ), Size( 0 ), Label( cLabel::kInvalid ), Residual( 0 ), Selected( false ),
            Synthetic( false ), Flags( 0 )
        {
        }
        cTMarker( T x, T y, T z ) : X( x ), Y( y ), Z( z ), Size( 0 ), Label( cLabel::kInvalid ), Residual( 0 ), Selected( false ),
            ID( cUID::kInvalid ), Synthetic( false ), Flags( 0 )
        {
        }

        bool operator==( const cTMarker& other ) { return ( Label == other.Label ); }
        bool operator!=( const cTMarker& other ) { return ( Label != other.Label ); }

        /// <summary>Set the position.</summary>
        void SetPosition( T x, T y, T z ) { X = x; Y = y; Z = z; }

        /// <summary>Returns true if this was recorded from an active marker.</summary>
        bool IsActiveMarker() const { return ( ID.LowBits() == kActiveMarkerIDTag ); }

        /// <summary>Returns true if this was recorded from a measurement (probe) point.</summary>
        bool IsMeasurement() const { return ( ( Flags&  Measurement ) != 0 ); }

        /// <summary>Return the active ID for this marker, or zero if it is not an active marker.</summary>
        unsigned int ActiveID() const { return ( IsActiveMarker() ? (int) ID.HighBits() : 0 ); }

        /// <summary>Special method for generating a UID that is a little easier on the eyes when viewing the
        /// ID of unlabeled markers.
        static cUID GenerateUnlabeledMarkerUID()
        {
            return cUID( sNextMarkerID++, sMarkerIDLowBits );
        }

        cUID ID;                // Marker ID (which may be assigned during reconstruction)
        T X;                    // Position in meters
        T Y;                    // Position in meters
        T Z;                    // Position in meters
        T Size;                 // Diameter in meters
        T Residual;             // Residual in mm/ray
        cLabel Label;           // Marker Label
        bool Selected;          // Selection state
        bool Synthetic;         // Synthetic markers created in pipeline such as virtual finger tip markers
        unsigned short  Flags;  // bit-encoded marker flags (occluded, model-solved, active, unlabeled, etc)

                                        // When a marker is actively labeled, it will have an ID that includes this constant.
        static const unsigned int kActiveMarkerIDTag = 0x8d403b2a; // Do not change this value.

        static unsigned long long sNextMarkerID;
        static unsigned long long sMarkerIDLowBits;
    };

    template <typename T>
    unsigned long long cTMarker<T>::sNextMarkerID = 1;
    template <typename T>
    unsigned long long cTMarker<T>::sMarkerIDLowBits = cUID::Generate().HighBits(); // HighBits is more random than LowBits.

    typedef cTMarker<float> cMarker;
    typedef cTMarker<float> cMarkerf;
    typedef cTMarker<double> cMarkerd;
}
#endif // _CORE_MARKER_CLASS

enum eMotiveAPIResult
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
    kApiResult_CouldNotFindGroundPlane,
    kApiResult_UnableToAccessCameras
};

enum eMotiveAPICalibrationState
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

// Startup / Shutdown ==================================================================================

MOTIVE_API eMotiveAPIResult TT_Initialize();
MOTIVE_API eMotiveAPIResult TT_Shutdown();

// Determine if any other OptiTrack software is currently using devices.
MOTIVE_API eMotiveAPIResult TT_TestSoftwareMutex(); 

// Software Release Build #
MOTIVE_API int TT_BuildNumber();

// Frame Processing ====================================================================================

// Process incoming camera data
// TT_Update() clears the frame queue, so only the most recent frame is available, old frames are dropped
// TT_UpdateSingleFrame() removes only one frame from the queue
// Both will return kApiResult_NoFrameAvailable if no new frame is available
MOTIVE_API eMotiveAPIResult TT_Update();
MOTIVE_API eMotiveAPIResult TT_UpdateSingleFrame();

// User Profile Interface ==============================================================================

// Load User Profile File
MOTIVE_API eMotiveAPIResult TT_LoadProfile( const wchar_t* filename );

// Save User Profile File
MOTIVE_API eMotiveAPIResult TT_SaveProfile( const wchar_t* filename ); 

// Camera Calibration Interface ========================================================================

MOTIVE_API eMotiveAPIResult TT_LoadCalibration( const wchar_t* filename, int* cameraCount = nullptr );
MOTIVE_API eMotiveAPIResult TT_LoadCalibrationFromMemory( unsigned char* buffer, int bufferSize, int* cameraCount = nullptr );
MOTIVE_API eMotiveAPIResult TT_SaveCalibration( const wchar_t* filename );

struct MotiveAPICameraInfo
{
    int CameraSerial;
    float Position[3];
    float Orientation[9];
};

class MOTIVE_API MotiveAPICameraList
{
public:
    MotiveAPICameraList();

    // CameraCount() returns the number of cameras present in MotiveAPICameraList

    int CameraCount() const;

    // Camera() returns a MotiveAPICameraInfo structure that contains camera
    // serial number, position, and orientation information.

    MotiveAPICameraInfo Camera( int index ) const;

    void AddCamera( const MotiveAPICameraInfo& cameraInfo );

private:
    std::vector<MotiveAPICameraInfo> mCameraList;
};

// Get camera extrinsics from a calibration file in memory. This allows for acquiring camera
// extrinsics for cameras not connected to system.  It simply returns the list of details for all
// cameras contained in the calibration file.
MOTIVE_API MotiveAPICameraList TT_CameraExtrinsicsCalibrationFromMemory( unsigned char* buffer, int bufferSize,
    eMotiveAPIResult& result );

// Start a new calibration wanding for all cameras. This will cancel any existing calibration process.
MOTIVE_API void TT_StartCalibrationWanding();

// Returns the current calibration state.
MOTIVE_API eMotiveAPICalibrationState TT_CalibrationState();

// During calibration wanding, this will return a vector of camera indices that are lacking the minimum
// number of calibration samples to begin calculation. When the returned vector for this method goes to
// zero size, you can call TT_StartCalibrationCalculation() to begin calibration calculations. Wanding
// samples will continue to be collected until TT_StartCalibrationCalculation() is called.
MOTIVE_API std::vector<int> TT_CalibrationCamerasLackingSamples();

// During calibration wanding, this will return the number of wand samples collected for the given
// camera. Returns zero otherwise.
MOTIVE_API int TT_CameraCalibrationSamples( int cameraIndex );

// Cancels either wanding or calculation and resets the calibration engine.
MOTIVE_API void TT_CancelCalibration();

// Once wanding is complete, call this to begin the calibration calculations.
MOTIVE_API bool TT_StartCalibrationCalculation();

// During calibration calculation, this method will return the current calibration quality in the range
// [0-5], with 5 being best. Returns zero otherwise.
MOTIVE_API int TT_CurrentCalibrationQuality();

// Once TT_CalibrationState() returns "Complete", call this method to apply the calibration results
// to all cameras.
MOTIVE_API bool TT_ApplyCalibrationCalculation();

// Set the ground plane using a standard or custom ground plane template.
MOTIVE_API eMotiveAPIResult TT_SetGroundPlane( bool useCustomGroundPlane );

// Translate the existing ground plane (in mm).
MOTIVE_API void TT_TranslateGroundPlane( float x, float y, float z );

// Licensing ===========================================================================================

// Licenses are automatically loaded from the OptiTrack license directory. This is not needed except to
// accommodate some very rare user scenarios.  Call this and provide the contents of a license
// file located outside the license folder.  Call this function before TT_Initialize();
MOTIVE_API eMotiveAPIResult TT_LoadLicenseFromMemory( const unsigned char* buffer, int bufferSize );

// Data Streaming ======================================================================================

// VRPN Streaming
MOTIVE_API eMotiveAPIResult TT_StreamVRPN( bool enable, int port );

// NatNet Streaming
MOTIVE_API eMotiveAPIResult TT_StreamNP( bool enable );

// Frame Info ==========================================================================================

struct MOTIVE_API MotiveAPITimecode
{
    int hh;
    int mm;
    int ss;
    int ff;
    int subFrame;
    char tcString[32];
    bool isDropFrame;
};

MOTIVE_API double TT_FrameTimeStamp();                  // Time Stamp of Frame (seconds)
MOTIVE_API int TT_FrameID();                            // FrameID of Frame
MOTIVE_API bool TT_FrameTimeCode( MotiveAPITimecode& tc );   // Frame time code

// In the event that you are tracking a very high number of 2D and/or 3D markers (hundreds of 3D markers), 
// and you find that the data you're getting out has sufficient latency you can call TT_FlushCameraQueues()
// to catch up before calling TT_Update(). Ideally, after calling TT_FlushCameraQueues() you'll want to
// not call it again until after TT_Update() returns NPRESULT_SUCCESS
MOTIVE_API void TT_FlushCameraQueues();

// Marker Info =========================================================================================

MOTIVE_API int TT_FrameMarkerCount();                // Returns Frame Markers Count
MOTIVE_API bool TT_FrameMarkerXYZ( int markerIndex, float& x, float& y, float& z );  // Get marker coordinates
MOTIVE_API float TT_FrameMarkerX( int markerIndex ); // Returns X Coord of Marker
MOTIVE_API float TT_FrameMarkerY( int markerIndex ); // Returns Y Coord of Marker
MOTIVE_API float TT_FrameMarkerZ( int markerIndex ); // Returns Z Coord of Marker
MOTIVE_API Core::cUID TT_FrameMarkerLabel( int markerIndex ); // Returns Label of Marker
MOTIVE_API float TT_FrameMarkerResidual( int markerIndex );   // Returns Residual of Marker

// Returns the number of rays that contributed to the reconstruction of the given marker.
MOTIVE_API int TT_FrameMarkerContributingRaysCount( int markerIndex );

// Calculate the average ray length of all rays contributing to the marker. Returns zero if the
// marker index is invalid or no rays contribute to a given marker in the current frame.
MOTIVE_API float TT_FrameMarkerAverageRayLength( int markerIndex );

// TT_FrameCameraCentroid returns true if the camera is contributing to this 3D marker.
// It also returns the location of the 2D centroid that is reconstructing to this 3D marker.
MOTIVE_API bool TT_FrameCameraCentroid( int markerIndex, int cameraIndex, float& x, float& y );

// Rigid Body Interface ================================================================================

MOTIVE_API int TT_RigidBodyCount();

// Create a rigid body based on the marker count and marker list provided.  The marker list is
// expected to contain of list of marker coordinates in the order: x1,y1,z1,x2,y2,z2,...xN,yN,zN.
MOTIVE_API eMotiveAPIResult TT_CreateRigidBody( const wchar_t* name, int id, int markerCount, float *markerList );

// Clear all rigid bodies
MOTIVE_API void TT_ClearRigidBodies();

// Load rigid body definitions from file and replace any that exist in the scene.
MOTIVE_API eMotiveAPIResult TT_LoadRigidBodies( const wchar_t* filename );

// Load rigid bodies and add them to the scene, leaving any that already exist in place.
MOTIVE_API eMotiveAPIResult TT_AddRigidBodies( const wchar_t* filename );

// Save rigid body definitions to file.
MOTIVE_API eMotiveAPIResult TT_SaveRigidBodies( const wchar_t* filename );

MOTIVE_API bool TT_IsRigidBodyTracked( int rbIndex );

MOTIVE_API bool TT_RigidBodyLocation( int rbIndex,
    float* x, float* y, float* z,                           // Position
    float* qx, float* qy, float* qz, float* qw,             // Quaternion orientation
    float* yaw, float* pitch, float* roll );                // Euler orientation

MOTIVE_API eMotiveAPIResult TT_RemoveRigidBody( int rbIndex );

MOTIVE_API int TT_RigidBodyUserData( int rbIndex );
MOTIVE_API void TT_SetRigidBodyUserData( int rbIndex, int ID );

MOTIVE_API bool TT_RigidBodyName( int rbIndex, wchar_t* buffer, int bufferSize );

MOTIVE_API void TT_SetRigidBodyEnabled( int rbIndex, bool enabled );
MOTIVE_API bool TT_RigidBodyEnabled( int rbIndex );

MOTIVE_API eMotiveAPIResult TT_RigidBodyTranslatePivot( int rbIndex, float x, float y, float z );
MOTIVE_API bool TT_RigidBodyResetOrientation( int rbIndex );

MOTIVE_API int TT_RigidBodyMarkerCount( int rbIndex );
MOTIVE_API bool TT_RigidBodyMarker( int rbIndex, int markerIndex, float* x, float* y, float* z );
MOTIVE_API bool TT_RigidBodyUpdateMarker( int rbIndex, int markerIndex, float x, float y, float z );

// Get corresponding point cloud marker. If tracked is false, there is no corresponding point cloud marker.
MOTIVE_API bool TT_RigidBodyPointCloudMarker( int rbIndex, int markerIndex, bool& tracked,
    float& x, float& y, float& z );

// Get world location of the rigid body marker if tracked.
MOTIVE_API bool TT_RigidBodyPlacedMarker( int rbIndex, int markerIndex, bool& tracked,
    float& x, float& y, float& z );                         

// Get mean error of tracked rigid body. (in meters)
MOTIVE_API float TT_RigidBodyMeanError( int rbIndex );

MOTIVE_API Core::cUID TT_RigidBodyID( int rbIndex );

// Camera Manager Access ===============================================================================

MOTIVE_API CameraLibrary::CameraManager* TT_GetCameraManager(); 

// Universal Cameras Settings ==========================================================================

class MOTIVE_API MotiveAPIFilterSettings
{
public:
    MotiveAPIFilterSettings();
    ~MotiveAPIFilterSettings();

    enum eFilterType
    {
        FilterNone,
        FilterSizeRoundness,
        FilterCount
    };

    eFilterType FilterType;
    int MinMarkerSize;
    int MaxMarkerSize;
    float MinRoundness;
};

MOTIVE_API eMotiveAPIResult TT_CameraFilterSettings( MotiveAPIFilterSettings& settings );
MOTIVE_API eMotiveAPIResult TT_SetCameraFilterSettings( MotiveAPIFilterSettings& settings );

// Shutter delay for all cameras.
MOTIVE_API void TT_SetShutterDelay( int microseconds );

// To set or retrieve application settings, just construct an instance of this class, then call the respective Set*Parameter()
// or *Parameter() method to set or get individual application settings.
class MOTIVE_API MotiveAPIProcessingSettings
{
public:
    enum Setting : unsigned long long
    {
        eShowCameras = 1LL << 1,                // bool
        ePCResidual = 1LL << 3,                 // double
        ePCMinAngle = 1LL << 4,                 // double
        ePCMinRays = 1LL << 5,                  // int
        eShutterDelay = 1LL << 6,               // int
        ePrecisionPacketCap = 1LL << 7,         // int
        ePCMinRayLength = 1LL << 8,             // double
        ePCMaxRayLength = 1LL << 9,             // double
        ePCReconstructXCenter = 1LL << 10,      // double
        ePCReconstructYCenter = 1LL << 11,      // double
        ePCReconstructZCenter = 1LL << 12,      // double
        ePCReconstructXWidth = 1LL << 13,       // double
        ePCReconstructYWidth = 1LL << 14,       // double
        ePCReconstructZWidth = 1LL << 15,       // double
        ePCReconstructRadius = 1LL << 16,       // double
        ePCReconstructHeight = 1LL << 17,       // double
        ePCReconstructShape = 1LL << 18,        // shape 0=Cuboid,1=Spherical,2=Cylindrical,3=Ellipsoid
        ePCObjectFilterLevel = 1LL << 19,       // int
        ePCObjectFilterMinSize = 1LL << 20,     // int
        ePCObjectFilterMaxSize = 1LL << 21,     // int
        ePCObjectFilterCircularity = 1LL << 22, // double
        ePCObjectFilterGrayscaleFloor = 1LL << 23, // int
        ePCObjectFilterAspectTolerance = 1LL << 24, // int
        ePCObjectFilterObjectMargin = 1LL << 25, // int
        eShowReconstructionBounds = 1LL << 26,  // bool
        eBoundReconstruction = 1LL << 27,       // bool
        eShow3DMarkers = 1LL << 29,             // bool
        eMaskPadding = 1LL << 37,               // double
        eSynchronizerEngine = 1LL << 39,        // int 1=v1.0  2=v2.0
        eMarkerDiameterType = 1LL << 40,        // int
        eMarkerDiameterForceSize = 1LL << 41,   // double
        eSynchronizerControl = 1LL << 42,       // int
        eContinuousCalibration = 1LL << 43,     // bool
        eBumpedCameraCalibration = 1LL << 44,   // bool
        eApplyContinuousCalibration = 1LL << 45, // bool
        eSettingsCount
    };

    MotiveAPIProcessingSettings();
    ~MotiveAPIProcessingSettings();

    // Set individual parameter values. Only values that are set will be changed when submitting
    // the settings block to TT_SetCameraGroupPointCloudSettings. These methods will return false 
    // if there is a mismatch between the requested parameter and its expected type.
    bool SetBoolParameter( Setting which, bool val );
    bool SetDoubleParameter( Setting which, double val );
    bool SetIntParameter( Setting which, int val );

    // Retrieve individual parameter settings from the parameter block. These methods will return false 
    // if there is a mismatch between the requested parameter and its expected type.
    bool BoolParameter( Setting which, bool& val ) const;
    bool DoubleParameter( Setting which, double& val ) const;
    bool IntParameter( Setting which, int& val ) const;

private:
    void* mSettings;
};

class MOTIVE_API MotiveAPIMarkerSizeSettings
{
public:
    MotiveAPIMarkerSizeSettings();
    ~MotiveAPIMarkerSizeSettings();

    enum eMarkerSizeType
    {
        MarkerSizeCalculated,
        MarkerSizeFixed,
        MarkerSizeCount
    };

    eMarkerSizeType MarkerSizeType;
    float MarkerSize;
};

MOTIVE_API eMotiveAPIResult TT_CameraMarkerSize( MotiveAPIMarkerSizeSettings& settings );
MOTIVE_API eMotiveAPIResult TT_SetCameraMarkerSize( MotiveAPIMarkerSizeSettings& settings );

MOTIVE_API eMotiveAPIResult TT_SetEnabledFilterSwitch( bool enabled ); // Enabled by default 
MOTIVE_API bool TT_IsFilterSwitchEnabled();

// Camera Interface ====================================================================================

MOTIVE_API int TT_CameraCount();                            // Returns Camera Count 
MOTIVE_API int TT_CameraGroupCount();                       // Returns number of camera groups

MOTIVE_API int TT_CameraGroup( int cameraIndex );           // Returns Camera's camera group index

MOTIVE_API bool TT_CameraPositionOrientation( int cameraIndex, float pos[3], float matrix[9] );

MOTIVE_API float TT_CameraXLocation( int cameraIndex );     // Returns Camera's X Coord 
MOTIVE_API float TT_CameraYLocation( int cameraIndex );     // Returns Camera's Y Coord 
MOTIVE_API float TT_CameraZLocation( int cameraIndex );     // Returns Camera's Z Coord 
MOTIVE_API float TT_CameraOrientationMatrix( int cameraIndex, int matrixIndex ); // Orientation

MOTIVE_API bool TT_CameraName( int cameraIndex, wchar_t* buffer, int bufferSize ); // Returns Camera Name 
MOTIVE_API int TT_CameraSerial( int cameraIndex );          // Returns Camera Serial Number
MOTIVE_API int TT_CameraIndexFromSerial( int serial );      // Returns the camera index from a serial number

MOTIVE_API int TT_CameraMarkerCount( int cameraIndex );     // Camera's 2D Marker Count

// CameraMarker fetches the 2D centroid location of the marker as seen by the camera.
MOTIVE_API bool TT_CameraMarker( int cameraIndex, int markerIndex, float& x, float& y );

// Camera Pixel Resolution
MOTIVE_API bool TT_CameraPixelResolution( int cameraIndex, int& width, int& height );

// Fetch pre-distorted marker location.  This is basically where the camera would see the marker if
// there was no lens distortion. For most of our cameras/lenses, this location is only a few pixels
// from the distorted (TT_CameraMarker) position.
MOTIVE_API bool TT_CameraMarkerPredistorted( int cameraIndex, int markerIndex, float& x, float& y );

// Camera Video Type Definitions
enum eMotiveAPIVideoType
{
    kVideoType_Segment   = 0,
    kVideoType_Grayscale = 1,
    kVideoType_Object    = 2,
    kVideoType_Precision = 4,
    kVideoType_MJPEG     = 6,
    kVideoType_ColorH264 = 9
};

// MJPEG Quality Settings
// Note: Flex13 and Ethernet cameras support all four quality settings
//       All other cameras are limited to LowQuality and HighQuality
enum eMotiveAPIMJPEGQuality
{
    kMJPEG_MinQuality      = 0,
    kMJPEG_LowQuality      = 1,
    kMJPEG_StandardQuality = 2,
    kMJPEG_HighQuality     = 3
};

// Use these functions to update settings on individual cameras
// Returns true if the operation was successful, false otherwise
// VideoType    : One of eMotiveAPIVideoType above
// Exposure     : Exposure time in micro-seconds
// Threshold    : Threshold value 0-255
// IRLedsOn     : True to enable IR illumination, false to disable
// FilterSwitch : True for the IR spectrum, false for the visible spectrum
MOTIVE_API bool TT_SetCameraVideoType( int cameraIndex, int videoType );
MOTIVE_API bool TT_SetCameraExposure( int cameraIndex, int exposureMicroseconds );
MOTIVE_API bool TT_SetCameraThreshold( int cameraIndex, int threshold );
MOTIVE_API bool TT_SetCameraIRLedsOn( int cameraIndex, bool irLedsOn );
MOTIVE_API bool TT_SetCameraFilterSwitch( int cameraIndex, bool enableIRFilter );

MOTIVE_API bool TT_SetCameraSettings( int cameraIndex, int videoType, int exposureMicroseconds,
                                      int threshold, bool irLedsOn );

// Set the frame rate of the camera system
// All tracking cameras must run at the same frame rate
// Reference cameras can be made to run at a reduced rate by setting their frame rate divisor
// The system_rate / divisor must result in a whole number for the divisor to be valid
// A camera must first be made a reference camera, by switching its video type, prior to applying a divisor
MOTIVE_API bool TT_SetCameraSystemFrameRate( int frameRate );
MOTIVE_API bool TT_SetCameraFrameRateDivisor( int cameraIndex, int divisor );

// Returns the camera system frame rate
MOTIVE_API int TT_CameraSystemFrameRate();          // frames/sec (nominal)
MOTIVE_API double TT_MeasuredIncomingFrameRate();   // frames/sec (measured live)
MOTIVE_API double TT_MeasuredIncomingDataRate();    // bytes/sec

// Get camera settings for a given camera index. A negative return value indicates the value was not
// available. This usually means that either the camera index is not valid or devices have not been
// initialized with TT_Initialize()
//
// See VideoType above.
MOTIVE_API int TT_CameraVideoType( int cameraIndex );

MOTIVE_API int TT_CameraFrameRate( int cameraIndex ); // frames/sec
MOTIVE_API int TT_CameraExposure( int cameraIndex );
MOTIVE_API int TT_CameraThreshold( int cameraIndex );
MOTIVE_API bool TT_CameraIRLedsOn( int cameraIndex );
MOTIVE_API bool TT_CameraFilterSwitch( int cameraIndex );
MOTIVE_API float TT_CameraTemperature( int cameraIndex );
MOTIVE_API float TT_CameraRinglightTemperature( int cameraIndex );

// Camera's Full Frame Grayscale Decimation
MOTIVE_API int TT_CameraGrayscaleDecimation( int cameraIndex );
MOTIVE_API bool TT_SetCameraGrayscaleDecimation( int cameraIndex, int value );

// Toggle camera extended options
MOTIVE_API bool TT_SetCameraAGC( int cameraIndex, bool enableAutomaticGainControl );
MOTIVE_API bool TT_SetCameraAEC( int cameraIndex, bool enableAutomaticExposureControl );
MOTIVE_API bool TT_SetCameraHighPower( int cameraIndex, bool enableHighPowerMode );

// See eMotiveAPIMJPEGQuality above
MOTIVE_API bool TT_SetCameraMJPEGQuality( int cameraIndex, int mjpegQuality );

// Camera Imager Gain
MOTIVE_API int TT_CameraImagerGain( int cameraIndex );
MOTIVE_API int TT_CameraImagerGainLevels( int cameraIndex );
MOTIVE_API bool TT_SetCameraImagerGain( int cameraIndex, int value );

// Camera Illumination
MOTIVE_API bool TT_CameraIsContinuousIRAvailable( int cameraIndex );
MOTIVE_API bool TT_CameraContinuousIR( int cameraIndex );
MOTIVE_API bool TT_CameraSetContinuousIR( int cameraIndex, bool enable );

// Camera Masking
MOTIVE_API bool TT_ClearCameraMask( int cameraIndex );
MOTIVE_API bool TT_SetCameraMask( int cameraIndex, unsigned char* buffer, int bufferSize );
MOTIVE_API bool TT_CameraMask( int cameraIndex, unsigned char* buffer, int bufferSize );
MOTIVE_API bool TT_CameraMaskInfo( int cameraIndex, int& blockingMaskWidth, int& blockingMaskHeight,
    int& blockingMaskGrid );

// Returns true if the camera has visible objects in view that are not currently masked.
MOTIVE_API bool TT_CameraHasVisibleObjects( int cameraIndex );

// Auto-mask all cameras. This is additive to any existing masking. To clear masks on a camera,
// call TT_ClearCameraMask prior to auto-masking.
MOTIVE_API void TT_AutoMaskAllCameras();

// Camera State
enum eMotiveAPICameraStates
{
    Camera_Enabled = 0,
    Camera_Disabled_For_Reconstruction = 1,
    Camera_Disabled = 2,
    CameraStatesCount = 3
};

MOTIVE_API bool TT_SetCameraState( int cameraIndex, eMotiveAPICameraStates state );
MOTIVE_API bool TT_CameraState( int cameraIndex, eMotiveAPICameraStates& currentState );

// Camera ID
MOTIVE_API int TT_CameraID( int cameraIndex );

// Fetch the camera's frame buffer.  This function fills the provided buffer with an image of what
// is in the camera view.  The resulting image depends on what video mode the camera is in.  If
// the camera is in grayscale mode, for example, a grayscale image is returned from this call.
MOTIVE_API bool TT_CameraFrameBuffer( int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,
    int bufferByteSpan, int bufferPixelBitDepth, unsigned char *buffer );

// Save camera's frame buffer as a BMP image file
MOTIVE_API bool TT_CameraFrameBufferSaveAsBMP( int cameraIndex, const wchar_t *filename );

// Back-project from 3D space to 2D space.  If you give this function a 3D location and select a
// camera, it will return where the point would land on the imager of that camera in to 2D space.
// This basically locates where in the camera's FOV a 3D point would be located.
MOTIVE_API void TT_CameraBackproject( int cameraIndex, float x, float y, float z, float& cameraX, 
    float& cameraY );

// The 2D centroids the camera reports are distorted by the lens.  To remove the distortion call
// CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
// to a distorted point call CameraDistort2DPoint.
MOTIVE_API bool TT_CameraUndistort2DPoint( int cameraIndex, float& x, float& y );
MOTIVE_API bool TT_CameraDistort2DPoint( int cameraIndex, float& x, float& y );

// Takes an undistorted 2D centroid and return a camera ray in the world coordinate system.
MOTIVE_API bool TT_CameraRay( int cameraIndex, float x, float y, float& rayStartX, float& rayStartY, 
    float& rayStartZ, float& rayEndX, float& rayEndY, float& rayEndZ );

// Set a camera's extrinsic (position&  orientation) and intrinsic (lens distortion) parameters with
// parameters compatible with the OpenCV intrinsic model.
MOTIVE_API bool TT_CameraModel( int cameraIndex, float x, float y, float z, // Camera Position
    float *orientation,                                     // Orientation (3x3 matrix)
    float principleX, float principleY,                     // Lens center (in pixels)
    float focalLengthX, float focalLengthY,                 // Lens focal  (in pixels)
    float kc1, float kc2, float kc3,                        // Barrel distortion coefficients
    float tangential0, float tangential1 );                 // Tangential distortion

// Set a camera's extrinsic (position & orientation).
MOTIVE_API bool TT_CameraPose( int cameraIndex, float x, float y, float z,  // Camera Position
    float *orientation );                                   // Orientation (3x3 matrix)

// This function will return the Camera SDK's camera pointer.  While the API takes over the
// data path which prohibits fetching the frames directly from the camera, it is still very usefuL to be
// able to communicate with the camera directly for setting camera settings or attaching modules.
MOTIVE_API CameraLibrary::Camera* TT_GetCamera( int cameraIndex );

// Rigid Body Refinement ===============================================================================

// You will want to get a rigid body's ID from TT_RigidBodyID for use with the following functions.
// To start the refine process, call TT_RigidBodyRefineStart with the rigid body's ID along with the
// number of samples you'd like to take before the refinement is performed.
MOTIVE_API bool TT_RigidBodyRefineStart( Core::cUID rigidBodyID, int sampleCount );

// Call TT_RigidBodyRefineSample() every frame after calling TT_RigiDBodyRefineStart. This will allow 
// the refinement process to collect samples.  You can check the progress of samples by calling
// TT_RigidBodyRefineProgress() and it will report a percentage of the total samples collected.  The
// refinement process will not collect samples when the rigid body is untracked.
MOTIVE_API bool TT_RigidBodyRefineSample();

enum MotiveAPIRigidBodyRefineState
{
    RigidBodyRefine_Initialized = 0,
    RigidBodyRefine_Sampling,
    RigidBodyRefine_Solving,
    RigidBodyRefine_Complete,
    RigidBodyRefine_Uninitialized
};

MOTIVE_API MotiveAPIRigidBodyRefineState TT_RigidBodyRefineState();

// To ensure the refinement solver is collecting samples, call TT_RigidBodyRefineProgress() during 
// RigidBodyRefine_Sampling state.  Progress is reported as a percentage of the total samples during
// sampling.
MOTIVE_API float TT_RigidBodyRefineProgress();

// Once RigidBodyRefine_Complete state is reached, you can use TT_RigidBodyRefineInitialError() and
// TT_RigidBodyRefineResultError() to determine if the result has improved prior to calling 
// TT_RigidBodyRefineApplyResult().
MOTIVE_API float TT_RigidBodyRefineInitialError();
MOTIVE_API float TT_RigidBodyRefineResultError();

// Apply the resulting rigid body refinement result by calling TT_RigidBodyRefineApplyResult().
MOTIVE_API bool TT_RigidBodyRefineApplyResult();

// To discard the rigid body refinement result, call TT_RigidBodyRefineReset().
MOTIVE_API bool TT_RigidBodyRefineReset();

// Rigid Body Pivot Location Solver ====================================================================

// You will want to get a rigid body's ID from TT_RigidBodyID for use with the following functions.
// To start the pivot solving process, call TT_RigidBodyPivotSolverStart with the rigid body's ID along 
// with the number of samples you'd like to take before the solving is performed.
MOTIVE_API bool TT_RigidBodyPivotSolverStart( Core::cUID rigidBodyID, int sampleCount );

// Call TT_RigidBodyPivotSolverSample() every frame after calling TT_RigidBodyPivotSolverStart.
// This will allow the solving process to collect samples.  You can check the progress of samples by 
// calling TT_RigidBodyPivotSolverProgress() and it will report a percentage of the total
// samples collected.  The solving process will not collect samples when the rigid body is untracked.
MOTIVE_API bool TT_RigidBodyPivotSolverSample();

enum eMotiveAPIRigidBodyPivotSolverState
{
    RigidBodyPivotSolver_Initialized = 0,
    RigidBodyPivotSolver_Sampling,
    RigidBodyPivotSolver_Solving,
    RigidBodyPivotSolver_Complete,
    RigidBodyPivotSolver_Uninitialized
};

MOTIVE_API eMotiveAPIRigidBodyPivotSolverState TT_RigidBodyPivotSolverState();

// To ensure the refinement solver is collecting samples, call TT_RigidBodyPivotSolverProgress() 
// during RigidBodyPivotSolver_Sampling state.  Progress is reported as a percentage of the total 
// samples during sampling.
MOTIVE_API float TT_RigidBodyPivotSolverProgress();

// Once RigidBodyPivotSolver_Complete state is reached, you can use TT_RigidBodyPivotSolverInitialError()
// and TT_RigidBodyPivotSolverResultError() to determine if the result has improved prior to calling 
// TT_RigidBodyPivotSolverApplyResult().
MOTIVE_API float TT_RigidBodyPivotSolverInitialError();
MOTIVE_API float TT_RigidBodyPivotSolverResultError();

// Apply the resulting rigid body refinement result by calling TT_RigidBodyPivotSolverApplyResult().
MOTIVE_API bool TT_RigidBodyPivotSolverApplyResult();

// To discard the rigid body refinement result, call TT_RigidBodyPivotSolverReset().
MOTIVE_API bool TT_RigidBodyPivotSolverReset();

// Additional Functionality ============================================================================

MOTIVE_API bool TT_SetFrameIDBasedTiming( bool enable );
MOTIVE_API bool TT_SetSuppressOutOfOrder( bool enable );

MOTIVE_API bool TT_AttachCameraModule( int cameraIndex, CameraLibrary::cCameraModule* module );
MOTIVE_API bool TT_DetachCameraModule( int cameraIndex, CameraLibrary::cCameraModule* module );

MOTIVE_API eMotiveAPIResult TT_OrientTrackingBar( float positionX, float positionY, float positionZ,
    float orientationX, float orientationY, float orientationZ, float orientationW );

// API Callbacks =======================================================================================

// Inherit MotiveAPIListener and override it's methods to receive callbacks from the Motive API.
// You must attach your listening class via TT_AttachListener.
class MOTIVE_API MotiveAPIListener
{
public:
    virtual ~MotiveAPIListener() = default;

    // TTAPIFrameAvailable callback is called when a new synchronized group of camera frames has been
    // delivered to the MOTIVE_API and is ready for processing.  You can use this notification to then
    // call TT_Update() without having to poll blindly for new data.
    virtual void FrameAvailable() = 0;

    // TTAPICameraConnected callback is called when a camera is connected.
    virtual void CameraConnected( int serialNumber ) = 0;

    // TTAPICameraDisconnected callback is called when a camera is disconnected.
    virtual void CameraDisconnected( int serialNumber ) = 0;
};

MOTIVE_API void TT_AttachListener( MotiveAPIListener* listener );
MOTIVE_API void TT_DetachListener();

// Result Processing ===================================================================================

MOTIVE_API const wchar_t* TT_GetResultString( eMotiveAPIResult result ); // Return Plain Text Message
