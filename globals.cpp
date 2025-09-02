#include <math.h>
#include "globals.h"

char PrnBuf[PrnBufLen];

// unsigned int SCR_WIDTH = 1600;
// unsigned int SCR_HEIGHT = 900; // 780;

// ----------------------------------------------------------------------------
std::mutex LocMap_mutex;
std::mutex D455_mutex;
std::mutex T265_mutex;

glm::vec3 BotPos;
glm::quat BotOrientation;

// ============================================================================
// output = ( (1 - factor) x input^3 ) + ( factor x input )
// factor = 1 is linear.
// 0 < factor < 1 is less sensitive in the center, more sensitive at the ends of throw.
float GetExponential(const float In, const float Ex, const float Lim) {
    float Q = 0.0;
    float Cub = In*In*In;

    Q = ((1.0-Ex) * Cub) + (Ex*In);
    Q = Q * Lim;
    return Q;
}

float CalcGPSDistance(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2) {
 double Lat,Lon,Lat1,Lon1,DeltaLat,DeltaLon,A,C,D;

 Lat = InLat1; Lat = (Lat / 1E7) * DegRad;
 Lon = InLon1; Lon = (Lon / 1E7) * DegRad;
 Lat1 = InLat2; Lat1 = (Lat1 / 1E7) * DegRad;
 Lon1 = InLon2; Lon1 = (Lon1 / 1E7) * DegRad;
 DeltaLat = Lat1 - Lat; DeltaLon = Lon1 - Lon;
 A = sin(DeltaLat/2.0) * sin(DeltaLat/2.0) + cos(Lat) * cos(Lat1) * sin(DeltaLon/2.0) * sin(DeltaLon/2.0);
 C = 2 * atan2(sqrt(A), sqrt(1.0-A));
 D = (EarthR * C) * 1000.0; // Vzdalenost v metrech
 return D;
}

float CalcGPSBearingRad(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2) {
 double Lat,Lon,Lat1,Lon1,D;

 Lat = InLat1; Lat = (Lat / 1E7) * DegRad;
 Lon = InLon1; Lon = (Lon / 1E7) * DegRad;
 Lat1 = InLat2; Lat1 = (Lat1 / 1E7) * DegRad;
 Lon1 = InLon2; Lon1 = (Lon1 / 1E7) * DegRad;
 D = atan2(sin(Lon1 - Lon) * cos(Lat1), cos(Lat) * sin(Lat1) - sin(Lat) * cos(Lat1) * cos(Lon1 - Lon));
 return D;
}

float CalcGPSBearingDeg(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2) {
 double Brg;
 Brg = CalcGPSBearingRad(InLat1,InLon1,InLat2,InLon2);
 Brg = Brg * RadDeg;
 Brg = fmod(Brg + 360.0, 360.0);
 return Brg;
}

float CalcYawDiff(const float CurrentAngle, const float TargetAngle) {
 float Diff, AbsDiff;

 Diff = TargetAngle - CurrentAngle;
 AbsDiff = fabs(Diff);
 if (AbsDiff <= 180.0) {
  return AbsDiff == 180 ? AbsDiff : Diff;
 }
 else if (TargetAngle > CurrentAngle) {
  return AbsDiff - 360.0;
 }
 else {
  return 360.0 - AbsDiff;
 }
}

/**
 @brief Konvertuje Lat a Lon ve formatu desetinneho cisla v retezci na format (long * 1E7)
 @param Coord Souradnice ve formatu desetinneho cisla v retezci
 @return Souradnice ve formatu (long * 1E7)

 Motivaci je nepouzivat plovouci cisla, aby fungovalo vyhledavani uzlu podle souradnic.
 Celociselna reprezentace netrpi ztratou presnosti.
*/
uint32_t GPStoUL(const char *Coord) {
 uint32_t L = 0;
 char C,S[80];
 int I = 0;

 S[0] = 0; C = *Coord++;
 while (C == ' ') C = *Coord++; // Leading whitespaces
 while (C != 0) {
  if (C != '.') S[I++] = C; S[I] = 0;
  C = *Coord++;
  if (I > 8)
  break;
 }
 while (I < 9) {
  S[I++] = '0';
 }
 S[I++] = 0;
 L = atol(S);
 return L;
}

void PolarToCart(TPolarVector* InV) {
    TPolarVector V;

    V = *InV;
    V.X = V.Mag * cos(V.Theta);
    V.Y = V.Mag * sin(V.Theta);
    *InV = V;
}

void CartToPolar(TPolarVector* InV) {
    TPolarVector V;

    V = *InV;
    V.Mag = sqrt((V.X*V.X)+(V.Y*V.Y));
    V.Theta = atan2(V.Y,V.X);
    *InV = V;
}

float ToAngularRangeDeg(const float a) {
    float A = a;
    while (A < 0.0) A = A + 360.0;
    while (A >= 360.0) A = A - 360.0;
    return A;
}

float ToAngularRange2Pi(const float a) {
    float A = a;
    while (A < 0.0) A = A + (2*PI);
    while (A >= (2*PI)) A = A - (2*PI);
    return A;
}

int ToZeroMax(const int N, const int Max) {
    int n = N;
    if (n < 0) n = 0;
    if (n > Max) n = Max;
    return n;
}

float ToPlusMinusJedna(const float R) {
    float r;
    r = R;
    if (r > 1.0) r = 1.0;
    if (r < -1.0) r = -1.0;
    return r;
}

long LAbs(long L) {
    if (L >= 0)
        return L;
    else
        return (~L + 1);
}

void Bresenham(int X1, int Y1, int X2, int Y2, std::vector<TPoint2D> &pts) {
    TPoint2D point;

    pts.clear();
    long dx =  abs(X2-X1), sx = X1<X2 ? 1 : -1;
    long dy = -abs(Y2-Y1), sy = Y1<Y2 ? 1 : -1;
    long err = dx+dy, e2; // error value e_xy
    for(;;) {
        point.x = X1; point.y = Y1;
        pts.push_back(point);
        if (X1==X2 && Y1==Y2) break;
        e2 = 2*err;
        if (e2 >= dy) { err += dy; X1 += sx; } // e_xy+e_x > 0
        if (e2 <= dx) { err += dx; Y1 += sy; } // e_xy+e_y < 0
    }
}

void BresenhamLim(int X1, int Y1, int X2, int Y2, int Min, int Max, std::vector<TPoint2D> &pts) {
    TPoint2D point;

    pts.clear();
    long dx =  abs(X2-X1), sx = X1<X2 ? 1 : -1;
    long dy = -abs(Y2-Y1), sy = Y1<Y2 ? 1 : -1;
    long err = dx+dy, e2; // error value e_xy
    for(;;) {
        if ((X1 < Min) || (X1 > Max) || (Y1 < Min) || (Y1 > Max)) {
            break;
        }
        else {
            point.x = X1; point.y = Y1;
            pts.push_back(point);
            if (X1==X2 && Y1==Y2) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; X1 += sx; } // e_xy+e_x > 0
            if (e2 <= dx) { err += dx; Y1 += sy; } // e_xy+e_y < 0
        }
    }
}

void BresenhamLim2(int X1, int Y1, int X2, int Y2, int Min, int Max, std::vector<TPoint2D> &pts) {
    TPoint2D point;

    pts.clear();
    long dx =  abs(X2-X1), sx = X1<X2 ? 1 : -1;
    long dy = -abs(Y2-Y1), sy = Y1<Y2 ? 1 : -1;
    long err = dx+dy, e2; // error value e_xy
    for(;;) {
        if ((X1 < Min) || (X1 > Max) || (Y1 < Min) || (Y1 > Max)) {
            break;
        }
        else {
            point.x = X1; point.y = Y1;
            pts.push_back(point);
            //if (X1==X2 && Y1==Y2) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; X1 += sx; } // e_xy+e_x > 0
            if (e2 <= dx) { err += dx; Y1 += sy; } // e_xy+e_y < 0
        }
    }
}

void BresenhamLim3(int X1, int Y1, int X2, int Y2, int MinX, int MaxX, int MinY, int MaxY, std::vector<TPoint2DInt> &pts) {
    TPoint2DInt point;

    pts.clear();
    long dx =  abs(X2-X1), sx = X1<X2 ? 1 : -1;
    long dy = -abs(Y2-Y1), sy = Y1<Y2 ? 1 : -1;
    long err = dx+dy, e2; // error value e_xy
    for(;;) {
        if ((X1 < MinX) || (X1 > MaxX) || (Y1 < MinY) || (Y1 > MaxY)) {
            break;
        }
        else {
            point.x = X1; point.y = Y1;
            pts.push_back(point);
            if (X1==X2 && Y1==Y2) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; X1 += sx; } // e_xy+e_x > 0
            if (e2 <= dx) { err += dx; Y1 += sy; } // e_xy+e_y < 0
        }
    }
}

void Circle(int xm, int ym, int r, std::vector<TPoint2D> &pts) {
    TPoint2D point;

    int x = -r, y = 0, err = 2-2*r; // II. Quadrant
    do {
        point.x = xm-x; point.y = ym+y; pts.push_back(point); // Pixel(xm-x,ym+y,Color); //   I. Quadrant
        point.x = xm-y; point.y = ym-x; pts.push_back(point); // Pixel(xm-y,ym-x,Color); //  II. Quadrant
        point.x = xm+x; point.y = ym-y; pts.push_back(point); // Pixel(xm+x,ym-y,Color); // III. Quadrant
        point.x = xm+y; point.y = ym+x; pts.push_back(point); // Pixel(xm+y,ym+x,Color); //  IV. Quadrant
        r = err;
        if (r <= y) err += ++y*2+1;           // e_xy+e_y < 0
        if (r > x || err > y) err += ++x*2+1; // e_xy+e_x > 0 or no 2nd y-step
    } while (x < 0);
}

void CircleLim(int xm, int ym, int r, int MinX, int MaxX, int MinY, int MaxY, std::vector<TPoint2DInt> &pts) {
    TPoint2DInt pt;

    int x = -r, y = 0, err = 2-2*r; // II. Quadrant
    do {
        pt.x = xm-x; pt.y = ym+y; if ((pt.x >= MinX) && (pt.x <= MaxX) && (pt.y >= MinY) && (pt.y <= MaxY)) pts.push_back(pt); // Pixel(xm-x,ym+y,Color); //   I. Quadrant
        pt.x = xm-y; pt.y = ym-x; if ((pt.x >= MinX) && (pt.x <= MaxX) && (pt.y >= MinY) && (pt.y <= MaxY)) pts.push_back(pt); // Pixel(xm-y,ym-x,Color); //  II. Quadrant
        pt.x = xm+x; pt.y = ym-y; if ((pt.x >= MinX) && (pt.x <= MaxX) && (pt.y >= MinY) && (pt.y <= MaxY)) pts.push_back(pt); // Pixel(xm+x,ym-y,Color); // III. Quadrant
        pt.x = xm+y; pt.y = ym+x; if ((pt.x >= MinX) && (pt.x <= MaxX) && (pt.y >= MinY) && (pt.y <= MaxY)) pts.push_back(pt); // Pixel(xm+y,ym+x,Color); //  IV. Quadrant
        r = err;
        if (r <= y) err += ++y*2+1;           // e_xy+e_y < 0
        if (r > x || err > y) err += ++x*2+1; // e_xy+e_x > 0 or no 2nd y-step
    } while (x < 0);
}

float Distance2D(const float X1,const float Y1,const float X2,const float Y2) {
 float d = sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) );
 return d;
}

rs2_vector T265toSys(const rs2_vector inpos) {
    rs2_vector out;

    out.x = -inpos.z;
    out.y = -inpos.x;
    out.z = inpos.y;
    return out;
}

double GPSGoalAngle(const uint32_t InLat1, const uint32_t InLon1, const uint32_t InLat2, const uint32_t InLon2) {
 double Lat,Lon,Lat1,Lon1;

 Lat = InLat1; Lat = (Lat / 1E7) * DegRad;
 Lon = InLon1; Lon = (Lon / 1E7) * DegRad;
 Lat1 = InLat2; Lat1 = (Lat1 / 1E7) * DegRad;
 Lon1 = InLon2; Lon1 = (Lon1 / 1E7) * DegRad;
 double deltaLon = Lon1-Lon;
 double x = std::cos(Lat1) * std::sin(deltaLon);
 double y = std::cos(Lat) * std::sin(Lat1) - std::sin(Lat) * std::cos(Lat1) * std::cos(deltaLon);
 double bearingRad = std::atan2(x,y);
 return bearingRad;
}
