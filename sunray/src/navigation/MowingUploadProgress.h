#pragma once
#include <cmath>
#include <cstdint>

// AT map uploads carry coordinates, not stable waypoint IDs. Retain an index
// only for the same ordered mowing route at the protocol's centimetre precision.
class MowingUploadProgress {
  struct Identity {
    int count = 0;
    uint64_t hash = UINT64_C(14695981039346656037);
    bool valid = true;
  } previous;
  int index = 0;
  bool pending = false;

  template<class Polygon> static Identity identify(Polygon &route) {
    Identity id;
    id.count = route.numPoints;
    for (int i=0; i<route.numPoints; ++i) {
      const double coordinates[] = {double(route.points[i].x())*100.0,
                                    double(route.points[i].y())*100.0};
      for (double coordinate : coordinates) {
        if (!std::isfinite(coordinate) || std::fabs(coordinate)>2147483647.0) {
          id.valid=false; return id;
        }
        uint32_t cm=static_cast<uint32_t>(static_cast<int32_t>(std::llround(coordinate)));
        for (int byte=0; byte<4; ++byte) {
          id.hash ^= (cm >> (byte*8)) & 0xff;
          id.hash *= UINT64_C(1099511628211);
        }
      }
    }
    return id;
  }
public:
  template<class Polygon> void capture(Polygon &route, int currentIndex) {
    previous=identify(route);
    index=currentIndex;
    pending=previous.valid && index>=0 && index<previous.count;
  }
  template<class Polygon> int restoredIndex(Polygon &route) const {
    if (!pending) return 0;
    const Identity next=identify(route);
    return next.valid && next.count==previous.count && next.hash==previous.hash ? index : 0;
  }
  void reset() { pending=false; }
};
