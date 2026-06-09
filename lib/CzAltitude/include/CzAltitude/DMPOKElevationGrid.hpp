#ifndef DMPOK_ELEVATION_GRID_H
#define DMPOK_ELEVATION_GRID_H

#include <string>
#include <vector>
#include <memory>
#include <proj.h>

/**
 * OVERVIEW OF COORDINATE SYSTEMS USED IN THIS PROGRAM:
 * --------------------------------------------------------
 * 1. WGS84 (EPSG:4326 / EPSG:4979) - "GPS Coordinates"
 * - The global system used for GPS.
 * - Coordinates: Latitude and Longitude.
 * - Height: Ellipsoidal height (height above a mathematically smooth model of the Earth).
 *  
 * 2. S-JTSK (EPSG:5514) - "Křovák"
 * - The Czech national grid system used for land surveying and official mapping.
 * - The raw DMPOK data (LAZ point clouds) are stored in this system.
 *
 * 3. EGM96 / AMSL / Bpv - "Mean Sea Level Height"
 * - AMSL (Above Mean Sea Level): Altitude relative to the sea level.
 * - EGM96: The geoid model used to determine where "sea level" is globally.
 * - Bpv (Baltic After Adjustment): The specific vertical datum used in the Czech Republic.
 * - Raw data from DMPOK files return heights in this AMSL/Bpv system.
 *
 * NOTE: "SHIFT" BETWEEN WGS84 AND MAPS (Difference ~45 meters)
 * - Standard web maps (like Mapy.cz or Google Maps) display AMSL (orthometric) height.
 * - This program's 'getEllipsoidElevation' method returns ELLIPSOIDAL (WGS84) height.
 * - In the Czech Republic, the geoid (sea level) is approximately 43 to 46 meters 
 * "below" the WGS84 ellipsoid.
 */

 /**
 * EXTERNAL LIBRARIES OVERVIEW:
 * --------------------------------------------------------
 * 1. PROJ (proj.h) version: 6.3.1-1
 * - Handles coordinate reference system (CRS) transformations.
 *
 * 2. libcurl (curl/curl.h) version: 7.68.0-1ubuntu2.25
 * - Responsible for downloading the compressed elevation data (.zip files) 
 * from the official remote server via HTTP/HTTPS.
 *
 * 3. libzip (zip.h) version: 3.0-11build1
 * - A library for reading, creating, and modifying zip archives.
 * - Used to extract the raw LiDAR data (.laz files) from the downloaded 
 * zip packages into the local directory.
 *
 * 4. LASzip (laszip_api.h)
 * - A specialized C/C++ library for reading and writing compressed LiDAR data.
 * - Used to parse and read .laz (compressed LiDAR) files efficiently and extract 
 * the precise X, Y, and Z spatial coordinates.
 */


struct Point2D {
    double x, y;
};

struct Point3D {
    double x, y, z;
};

// Map tile
struct Tile {
    std::string filename;
    std::string zip_url;
    std::vector<Point2D> polygon_coords;
    double min_x, min_y, max_x, max_y;

    bool contains(double x, double y) const;
};

class DMPOKElevationGrid {
public:
    DMPOKElevationGrid(std::string csv_index = "dmpok_index.csv", std::string laz_dir = "dmpok_unzipped");

    ~DMPOKElevationGrid();

    std::pair<double, std::string> getEllipsoidElevation(double lat, double lon);

private:
    const size_t MAX_RASTER_CACHE_TILES = 50;

    std::string csv_index_path;
    std::string laz_directory;      // Directory where downloaded LAZ/ZIP files are stored.

    // PROJ transformation object for converting WGS84 (GPS) to S-JTSK.
    PJ *pj_wgs84_to_s_jtsk;
    // PROJ transformation object for vertical datum conversion (EGM96 to WGS84 ellipsoid).
    PJ *pj_egm96_to_wgs84;
    // PROJ context used for threading and state management.
    PJ_CONTEXT *ctx;

    std::vector<Tile> tiles;
    
    struct LoadedRaster {
        std::string path;
        double min_x, min_y, max_x, max_y;
        double res;
        int cols, rows;
        std::vector<float> data;
    };

    double raster_resolution = 1;   // [m]
    std::vector<std::unique_ptr<LoadedRaster>> raster_cache;

    double getElevationFromRaster(const std::string& laz_path, double x, double y, bool laz_already_existed, const Tile& tile);


    void loadIndex();
    std::vector<const Tile*> findTilesForGps(double x, double y);
    std::string downloadAndUnzipTile(const std::string& tile_name, const std::string& url);
    
    // 2D spatial grid for fast point-in-polygon tile lookups.
    std::vector<std::vector<std::vector<size_t>>> spatial_grid;
    double grid_minx, grid_miny, step_x, step_y;
    void buildSpatialGrid();
};

#endif