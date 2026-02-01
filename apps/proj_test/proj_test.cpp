#include <proj.h>
#include <iostream>
#include <cmath>
#include <string>

static int utm_zone_from_lon(double lon_deg)
{
    // UTM zone: 1..60
    return int(std::floor((lon_deg + 180.0) / 6.0)) + 1;
}

int main()
{
    const char* paths[] = {
        "../3rdparty/Proj/PROJ/data",
        "3rdparty/Proj/PROJ/data"

    };
    proj_context_set_search_paths(nullptr, 1, paths);
    // Input (WGS84)
    const double lat_deg = 52.2297;   // Warsaw
    const double lon_deg = 21.0122;

    const int zone = utm_zone_from_lon(lon_deg);
    const bool north = (lat_deg >= 0.0);

    // UTM EPSG:
    // north: EPSG:326##   south: EPSG:327##
    const int epsg = (north ? 32600 : 32700) + zone;

    PJ_CONTEXT* C = proj_context_create();
    if (!C) {
        std::cerr << "Failed to create PROJ context\n";
        return 1;
    }

    // Create transformation: WGS84 geographic -> UTM projected
    std::string dst = "EPSG:" + std::to_string(epsg);

    PJ* P = proj_create_crs_to_crs(C, "EPSG:4326", dst.c_str(), nullptr);
    if (!P) {
        std::cerr << "Failed to create transformation\n";
        proj_context_destroy(C);
        return 1;
    }

    // Normalize so we can use lon/lat in degrees as input
    PJ* Pnorm = proj_normalize_for_visualization(C, P);
    proj_destroy(P);
    P = Pnorm;

    PJ_COORD in;
    in.lpzt.lam = lon_deg;  // longitude
    in.lpzt.phi = lat_deg;  // latitude
    in.lpzt.z   = 0.0;
    in.lpzt.t   = 0.0;

    PJ_COORD out = proj_trans(P, PJ_FWD, in);

    const double easting  = out.xy.x;
    const double northing = out.xy.y;

    std::cout << "WGS84:  lon=" << lon_deg << " lat=" << lat_deg << "\n";
    std::cout << "UTM zone " << zone << (north ? "N" : "S")
              << " (EPSG:" << epsg << ")\n";
    std::cout << "Easting:  " << easting << " m\n";
    std::cout << "Northing: " << northing << " m\n";

    proj_destroy(P);
    proj_context_destroy(C);
    return 0;
}
