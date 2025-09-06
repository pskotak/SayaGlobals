#ifndef GLOBALS_H
#define GLOBALS_H

#include <atomic>
#include <vector>
#include <cstdint>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp> // For glm::quat
//#include <glm/gtx/euler_angles.hpp> // For glm::yawPitchRoll

// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define ProgVersion "Saya V1.0.1"

// Union TWB - pristup k wordu po bytech/
typedef union {
    uint16_t W; ///< unsigned long
    unsigned char B[2]; ///< jednotlive byty LSB first
} TWB;

// Union TLB - pristup k longu po bytech/
typedef union {
    uint32_t L; ///< unsigned long
    unsigned char B[4]; ///< jednotlive byty LSB first
} TLB;

// Union TLB - pristup k uint64_t po bytech/
typedef union {
    uint64_t N; ///< unsigned long
    uint32_t L[2];
    uint16_t W[4];
    unsigned char B[6]; ///< jednotlive byty LSB first
} TL64B;

// Union TSLB - pristup ke znamenkovemu longu po bytech
typedef union {
    int32_t L; ///< long
    unsigned char B[4]; ///< jednotlive byty LSB first
} TSLB;

// Union TFB - pristup k floatu po bytech
typedef union {
    float F; ///< float
    unsigned char B[4]; ///< jednotlive byty LSB first
} TFB;

typedef struct {
    float x,y,z;
    float r,g,b;
} TVertex;

typedef struct {
    float x;
    float y;
    float z;
    float CosNorm;
    uint8_t Color;
} TColoredVertex;

typedef struct {
    float x,y;
} TPoint2D;

typedef struct {
    int x,y;
} TPoint2DInt;

typedef struct {
    float x,y,z;
} TPoint3D;

typedef struct { // 2D bod se vzdalenosti od nejakeho pocatku
    int x, y;
    float dist;
} TPoint2D_D;

typedef struct {
    float x,y,z;
} T3Dpoint;

typedef struct {
    float x,y,z,dist;
} TScanPoint;

typedef struct {
    uint8_t r,g,b;
} TRawColor;

typedef struct {
    double Mag; // Polar Magnitude
    double Theta; // Polar Angle - RADIANS
    double X; // Cartesian X
    double Y; // Cartesian Y
} TPolarVector;

typedef struct {
    uint32_t lat;
    uint32_t lon;
} TGPSWaypoint;

// ----------------------------------------------------------------------------

#define PrnBufLen 255
extern char PrnBuf[PrnBufLen];

#define ODRV_MFGR_ID 0x1209
#define ODRV_DEV_ID 0x0D32

#define RadDeg 57.295779513082 //57.2957795
#define DegRad 0.017453292519943 //0.0174532925
#define PI 3.1415926535897932384626433832795 //3.141592654

#define EarthR_meters 6371000.0 // Earth radius [m]
#define EarthR 6371.0 // Earth radius [km]

// ----------------------------------------------------------------------------

// Intel Cameras

// --- Color Stream Intrinsics ---
// Width: 848
// Height: 480
// Principal Point (px, py): (422.692, 256.959)
// Focal Length (fx, fy): (420.806, 420.484)
// Distortion Model: Inverse Brown Conrady
// Distortion Coefficients (k1, k2, p1, p2, k3): [-0.0550274, 0.0655312, -0.000867809, 4.69469e-05, -0.0211155]
//
// --- Color Stream FOV ---
// Horizontal FOV: 90.43 degrees
// Vertical FOV:   59.37 degrees
//
// --- Depth Stream Intrinsics ---
// Width: 848
// Height: 480
// Principal Point (px, py): (422.76, 238.46)
// Focal Length (fx, fy): (424.30, 424.30)
// Distortion Model: Brown Conrady
// Distortion Coefficients (k1, k2, p1, p2, k3): [0.00, 0.00, 0.00, 0.00, 0.00]
//
// --- Depth Stream FOV ---
// Horizontal FOV: 89.96 degrees
// Vertical FOV:   58.99 degrees

extern std::mutex D455_mutex;
extern std::mutex T265_mutex;
extern std::mutex LocMap_mutex;

#define DepthCamScale 0.001 // hloubka je v milimetrech unsigned short (Z16)
#define D455depth_max 4000.0f // millimeters
#define DepthCamMaxDistM 4.000 // Maximalni vzdalenost depth v metrech

#define SegW 513
#define SegH 513
#define SegChan 3
extern cv::Mat DispRGB_image;
extern cv::Mat depth_image;
extern cv::Mat lmap_depth_image;
extern cv::Mat DispSegmented_image;
extern cv::Mat SegBin;

extern uint16_t YawPixel;
extern bool NewYawPixel;
extern float VisYaw;
extern bool SteerMode;

#define D455W 848
#define D455H 480

#define D455_color_Ppx 422.692
#define D455_color_Ppy 256.959
#define D455_color_Fx 420.692
#define D455_color_Fy 420.37
#define D455_color_HFOV 90.43
#define D455_color_VFOV 59.37

#define D455_depth_Ppx 422.759
#define D455_depth_Ppy 238.457
#define D455_depth_Fx 424.295
#define D455_depth_Fy 424.295
#define D455_depth_HFOV 89.96
#define D455_depth_VFOV 58.99

#define D455DispScale 0.5f //1.0f //
#define D455depth_color_max 3000.0f

#define SegmentedMultiDispScale 0.82f
#define SegmentedDispScale 0.95f

#define Reduced_HFOV (D455_color_HFOV / 848.0 * 513) // Redukce HFOV z 848 na 513 px
#define PixelAngle (Reduced_HFOV / D455W)
#define PixelAngleRad ((Reduced_HFOV * DegRad) / D455W)

extern glm::vec3 BotPos;
extern glm::quat BotOrientation;

// Logging
#define DepthBufSize (D455W*D455H*2)
#define RGBBufSize (D455W*D455H*3)
#define LogRGBBufSize ((D455W)*(D455H)*3)
#define LogDepthBufSize ((D455W)*(D455H)*2)
typedef struct {
    uint32_t RecNo; // Record number
    float Roll;
    float Pitch;
    float Yaw;
    float Velocity;
    float AngularVelocity;
    float PosX;
    float PosY;
    float PosZ;
    float setVelocity;
    float setAngularVelocity;
    float omegaL;
    float omegaR;
    unsigned char DepthData[DepthBufSize]; // Raw depth image buffer
    unsigned char RGBData[RGBBufSize]; // Raw RGB image buffer
} TRecord;
//extern TRecord LogRec;

// ============================================================================
extern rs2_vector T265toSys(const rs2_vector inpos);
extern float GetExponential(const float In, const float Ex, const float Lim);

extern float CalcGPSDistance(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2);
extern float CalcGPSBearingRad(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2);
extern float CalcGPSBearingDeg(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2);
extern float CalcYawDiff(const float CurrentAngle, const float TargetAngle);
extern uint32_t GPStoUL(const char *Coord);

extern float ToAngularRangeDeg(const float a);
extern float ToAngularRange2Pi(const float a);
extern int ToZeroMax(const int N, const int Max);
extern float ToPlusMinusJedna(const float R);
extern long LAbs(long L);
extern float Distance2D(const float X1,const float Y1,const float X2,const float Y2); // Euclidean distance
extern void PolarToCart(TPolarVector* InV);
extern void CartToPolar(TPolarVector* InV);
extern void Bresenham(int X1, int Y1, int X2, int Y2, std::vector<TPoint2D> &pts);
extern void BresenhamLim(int X1, int Y1, int X2, int Y2, int Min, int Max, std::vector<TPoint2D> &pts);
extern void BresenhamLim2(int X1, int Y1, int X2, int Y2, int Min, int Max, std::vector<TPoint2D> &pts);
extern void BresenhamLim3(int X1, int Y1, int X2, int Y2, int MinX, int MaxX, int MinY, int MaxY, std::vector<TPoint2DInt> &pts);
//extern TPoint2D BresenhamLimObstacle(int X1, int Y1, int X2, int Y2, int Min, int Max);
extern void Circle(int xm, int ym, int r, std::vector<TPoint2D> &pts);
extern void CircleLim(int xm, int ym, int r, int MinX, int MaxX, int MinY, int MaxY, std::vector<TPoint2DInt> &pts);

extern double GPSGoalAngle(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2);

#endif
