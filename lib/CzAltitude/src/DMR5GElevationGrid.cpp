#include "../include/CzAltitude/DMR5GElevationGrid.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <curl/curl.h>
#include <zip.h>
#include <filesystem>
#include <memory>
#include <laszip/laszip_api.h>


// Helper function to split a string (replacement for Python's .split())
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

DMR5GElevationGrid::DMR5GElevationGrid(std::string csv_index, std::string laz_dir) 
    : csv_index_path(csv_index), laz_directory(laz_dir) {
    
    // 1. Initialize PROJ context
    ctx = proj_context_create();

    // 2. Define transformations
    // Transformation from WGS84 (GPS) to S-JTSK (Křovák) - EPSG:5514
    // We use a "pipeline" or direct code. PJ_DEFAULT_CTX for global context.
    PJ* raw_sjtsk = proj_create_crs_to_crs(ctx, "EPSG:4326", "EPSG:5514", NULL);    // Auxiliary transformation for S-JTSK (forces East, North order)
    pj_wgs84_to_s_jtsk = proj_normalize_for_visualization(ctx, raw_sjtsk);
    proj_destroy(raw_sjtsk);

    // Auxiliary transformation for height (forces Lon, Lat, Alt order)
    // Height transformation: EGM96 -> WGS84 Ellipsoid
    // In PROJ, geoids are handled using Compound CRS
    PJ* raw_egm = proj_create_crs_to_crs(ctx, 
        "+proj=latlong +datum=WGS84 +geoidgrids=egm96_15.gtx", 
        "EPSG:4979", 
        NULL);
    pj_egm96_to_wgs84 = proj_normalize_for_visualization(ctx, raw_egm);
    proj_destroy(raw_egm);

    if (!pj_wgs84_to_s_jtsk || !pj_egm96_to_wgs84) {
        throw std::runtime_error("Error initializing PROJ transformations. Check for the presence of egm96_15.gtx.");
    }

    // 3. Load the index (tile polygons)
    std::cout << "Loading tile index..." << std::endl;
    loadIndex();

    // 4. Build a spatial grid for fast spatial queries
    std::cout << "Building spatial grid..." << std::endl;
    buildSpatialGrid();

    std::cout << "Ready" << std::endl;
}

DMR5GElevationGrid::~DMR5GElevationGrid() {
    if (pj_wgs84_to_s_jtsk) proj_destroy(pj_wgs84_to_s_jtsk);
    if (pj_egm96_to_wgs84) proj_destroy(pj_egm96_to_wgs84);
    if (ctx) proj_context_destroy(ctx);
}

void DMR5GElevationGrid::loadIndex() {
    // Opens the CSV index file containing tile metadata (filename, polygon coordinates, zip URL).
    std::ifstream file(csv_index_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open index file: " + csv_index_path);
    }

    std::string line;
    std::getline(file, line); // Skip the CSV header (filename,polygon,zip_url)

    // Read the file line by line to parse each tile's properties.
    while (std::getline(file, line)) {
        // Simple CSV parser (assuming format: filename,polygon,url)
        std::stringstream ss(line);
        std::string filename, polygon_str, url;
        
        std::getline(ss, filename, ',');
        
        // The polygon string is enclosed in quotes, so we need to extract it properly
        if (ss.peek() == '"') {
            ss.get(); // Remove opening quote
            std::getline(ss, polygon_str, '"');
            if (ss.peek() == ',') ss.get(); // Remove comma after the closing quote
        } else {
            std::getline(ss, polygon_str, ',');
        }
        // Read the rest of the line as the URL.
        std::getline(ss, url);
        // Remove any trailing whitespace (like \r) that might have been left at the end of the line,
        // which would otherwise cause a cURL error.
        url.erase(url.find_last_not_of(" \t\n\r\f\v") + 1);

        Tile tile;
        tile.filename = filename;
        tile.zip_url = url;

        // Parse polygon coordinates (numbers separated by spaces)
        std::stringstream ss_poly(polygon_str);
        std::vector<double> coords;
        double val;
        while (ss_poly >> val) {
            coords.push_back(val);
        }

        // Initialize minimum and maximum boundaries for the bounding box
        tile.min_x = 1e18; tile.min_y = 1e18;
        tile.max_x = -1e18; tile.max_y = -1e18;

        // Convert Lat/Lon to S-JTSK and compute the Bounding Box for the tile
        for (size_t i = 0; i < coords.size(); i += 2) {
            double lat = coords[i];
            double lon = coords[i+1];

            // PROJ transformation (note the lon, lat order for EPSG:4326 in modern PROJ)
            PJ_COORD c_in = proj_coord(lon, lat, 0, 0);
            PJ_COORD c_out = proj_trans(pj_wgs84_to_s_jtsk, PJ_FWD, c_in);

            Point2D p = {c_out.xy.x, c_out.xy.y};
            tile.polygon_coords.push_back(p);

            // Update bounds (bounding box) for the spatial grid
            tile.min_x = std::min(tile.min_x, p.x);
            tile.min_y = std::min(tile.min_y, p.y);
            tile.max_x = std::max(tile.max_x, p.x);
            tile.max_y = std::max(tile.max_y, p.y);
        }
        tiles.push_back(tile);
    }
}

void DMR5GElevationGrid::buildSpatialGrid() {
    // This function creates a 2D spatial grid (a simple spatial index) to accelerate
    // point-in-polygon queries. Instead of checking a GPS coordinate against every tile,
    // we first look up the grid cell and only check the tiles intersecting that cell.

    if (tiles.empty()) return;

    // 1. Find the overall global bounding box for all tiles combined
    grid_minx = 1e18; grid_miny = 1e18;
    double maxx = -1e18, maxy = -1e18;
    double sum_w = 0, sum_h = 0;

    for (const auto& tile : tiles) {
        grid_minx = std::min(grid_minx, tile.min_x);
        grid_miny = std::min(grid_miny, tile.min_y);
        maxx = std::max(maxx, tile.max_x);
        maxy = std::max(maxy, tile.max_y);
        sum_w += (tile.max_x - tile.min_x);
        sum_h += (tile.max_y - tile.min_y);
    }

    // 2. Calculate the grid step size (step_x, step_y) similarly to the Python implementation
    // The step is based on half the average tile width and height.
    double avg_w = sum_w / tiles.size();
    double avg_h = sum_h / tiles.size();
    step_x = avg_w * 0.5; // step_factor = 0.5
    step_y = avg_h * 0.5;

    // Determine the number of rows and columns in the grid
    int n_cols = static_cast<int>((maxx - grid_minx) / step_x) + 1;
    int n_rows = static_cast<int>((maxy - grid_miny) / step_y) + 1;

    // Initialize a 3D vector: [row][column][list of tile indices]
    spatial_grid.resize(n_rows, std::vector<std::vector<size_t>>(n_cols));

    // 3. Distribute the tiles into the grid cells based on their bounding boxes.
    // A tile is added to every grid cell that intersects its bounding box.
    for (size_t i = 0; i < tiles.size(); ++i) {
        const auto& tile = tiles[i];
        int min_col = std::max(0, static_cast<int>((tile.min_x - grid_minx) / step_x));
        int max_col = std::min(n_cols - 1, static_cast<int>((tile.max_x - grid_minx) / step_x));
        int min_row = std::max(0, static_cast<int>((tile.min_y - grid_miny) / step_y));
        int max_row = std::min(n_rows - 1, static_cast<int>((tile.max_y - grid_miny) / step_y));

        for (int r = min_row; r <= max_row; ++r) {
            for (int c = min_col; c <= max_col; ++c) {
                spatial_grid[r][c].push_back(i);
            }
        }
    }
}



namespace fs = std::filesystem;

// Helper function for cURL: Write downloaded data to a file
static size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    size_t written = fwrite(ptr, size, nmemb, stream);
    return written;
}

// Implementation of the 'contains' method for the Tile struct
bool Tile::contains(double x, double y) const {
    // Fast check using the Bounding Box
    if (x < min_x || x > max_x || y < min_y || y > max_y) return false;

    // Ray Casting algorithm to test if a point is inside the polygon
    bool inside = false;
    size_t n = polygon_coords.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon_coords[i].y > y) != (polygon_coords[j].y > y)) &&
            (x < (polygon_coords[j].x - polygon_coords[i].x) * (y - polygon_coords[i].y) / 
                 (polygon_coords[j].y - polygon_coords[i].y) + polygon_coords[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

std::string DMR5GElevationGrid::findTileForGps(double lat, double lon) {
    // 1. Transform GPS to S-JTSK
    PJ_COORD c_in = proj_coord(lon, lat, 0, 0);
    PJ_COORD c_out = proj_trans(pj_wgs84_to_s_jtsk, PJ_FWD, c_in);
    double x = c_out.xy.x;
    double y = c_out.xy.y;

    // 2. Calculate the index in the spatial grid
    int col = static_cast<int>((x - grid_minx) / step_x);
    int row = static_cast<int>((y - grid_miny) / step_y);

    // Check grid boundaries
    if (row >= 0 && row < (int)spatial_grid.size() && 
        col >= 0 && col < (int)spatial_grid[0].size()) {
        
        // 3. Search through tiles in the given cell
        for (size_t tile_idx : spatial_grid[row][col]) {
            if (tiles[tile_idx].contains(x, y)) {
                return tiles[tile_idx].filename;
            }
        }
    }
    return "";
}

std::string DMR5GElevationGrid::downloadAndUnzipTile(const std::string& tile_name, const std::string& url) {
    fs::path laz_path = fs::path(laz_directory) / tile_name;
    
    // If the LAZ file already exists, do not download it again
    if (fs::exists(laz_path)) return laz_path.string();

    // Create the directory if it does not exist
    fs::create_directories(laz_directory);

    std::string zip_name = tile_name;
    // Change the .laz extension to .zip (simple replacement)
    size_t last_dot = zip_name.find_last_of(".");
    if (last_dot != std::string::npos) zip_name.replace(last_dot, 4, ".zip");
    
    fs::path zip_path = fs::path(laz_directory) / zip_name;

    // --- 1. Download using libcurl ---
    std::cout << "Downloading " << zip_name << " from " << url << "..." << std::endl;
    CURL *curl = curl_easy_init();
    if (curl) {
        FILE *fp = fopen(zip_path.string().c_str(), "wb");
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Follow redirects

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        fclose(fp);

        if (res != CURLE_OK) {
            throw std::runtime_error("CURL download failed: " + std::string(curl_easy_strerror(res)));
        }
    }

    // --- 2. Extract using libzip ---
    std::cout << "Extracting " << zip_name << "..." << std::endl;
    int err = 0;
    zip *z = zip_open(zip_path.string().c_str(), 0, &err);
    if (!z) throw std::runtime_error("Cannot open ZIP file.");

    zip_int64_t num_entries = zip_get_num_entries(z, 0);
    for (zip_int64_t i = 0; i < num_entries; i++) {
        struct zip_stat st;
        zip_stat_index(z, i, 0, &st);
        
        zip_file *f = zip_fopen_index(z, i, 0);
        if (!f) continue;

        fs::path out_file = fs::path(laz_directory) / st.name;
        FILE *out_fp = fopen(out_file.string().c_str(), "wb");
        
        char buffer[8192];
        zip_int64_t n;
        while ((n = zip_fread(f, buffer, sizeof(buffer))) > 0) {
            fwrite(buffer, 1, n, out_fp);
        }

        fclose(out_fp);
        zip_fclose(f);
    }
    zip_close(z);

    // --- 3. Cleanup ---
    fs::remove(zip_path);

    return laz_path.string();
}



double DMR5GElevationGrid::getElevationFromLaz(const std::string& laz_path, double x, double y) {
    // 1. Kontrola cache (zůstává stejné)
    auto it = std::find_if(loaded_cache.begin(), loaded_cache.end(),
        [&laz_path](const std::unique_ptr<LoadedLaz>& item) {
            return item->path == laz_path;
        });

    size_t cache_idx;

    if (it == loaded_cache.end()) {
        std::cout << "LASzip loading points from: " << laz_path << std::endl;
        
        auto new_entry = std::make_unique<LoadedLaz>();
        new_entry->path = laz_path;

        // --- 2. Načítání pomocí LASzip API ---
        laszip_POINTER laszip_reader;
        if (laszip_create(&laszip_reader)) {
            throw std::runtime_error("LASzip: Could not create reader.");
        }

        laszip_BOOL is_compressed = 0;
        if (laszip_open_reader(laszip_reader, laz_path.c_str(), &is_compressed)) {
            laszip_clean(laszip_reader);
            throw std::runtime_error("LASzip: Could not open file: " + laz_path);
        }

        laszip_header* header;
        if (laszip_get_header_pointer(laszip_reader, &header)) {
            laszip_close_reader(laszip_reader);
            laszip_clean(laszip_reader);
            throw std::runtime_error("LASzip: Could not read header.");
        }

        laszip_point* point;
        if (laszip_get_point_pointer(laszip_reader, &point)) {
            laszip_close_reader(laszip_reader);
            laszip_clean(laszip_reader);
            throw std::runtime_error("LASzip: Could not get point pointer.");
        }

        // LAS 1.4 formát (DMR5G) typicky používá extended_number_of_point_records, starší verze number_of_point_records
        laszip_I64 num_points = (header->number_of_point_records > 0) ? 
                                header->number_of_point_records : 
                                header->extended_number_of_point_records;

        // Rezervace paměti pro body
        new_entry->cloud.pts.reserve(num_points);

        // 3. Iterace přes všechny body a převod na double souřadnice
        for (laszip_I64 i = 0; i < num_points; i++) {
            if (laszip_read_point(laszip_reader)) break;

            Point3D p;
            // Výpočet reálné souřadnice: (raw_integer * scale) + offset
            p.x = point->X * header->x_scale_factor + header->x_offset;
            p.y = point->Y * header->y_scale_factor + header->y_offset;
            p.z = point->Z * header->z_scale_factor + header->z_offset;
            
            new_entry->cloud.pts.push_back(p);
        }
        // Úklid po LASzip
        laszip_close_reader(laszip_reader);
        laszip_clean(laszip_reader);

        if (new_entry->cloud.pts.empty()) {
            throw std::runtime_error("LASzip: No points were loaded from the file.");
        }

        // --- 4. Sestavení KD-Tree (nanoflann) ---
        new_entry->tree = std::make_unique<KDTree>(2, new_entry->cloud, 
                          nanoflann::KDTreeSingleIndexAdaptorParams(10));
        new_entry->tree->buildIndex();

        loaded_cache.push_back(std::move(new_entry));
        cache_idx = loaded_cache.size() - 1;
    } else {
        cache_idx = std::distance(loaded_cache.begin(), it);
    }

    // --- 5. Vyhledání nejbližšího bodu (zůstává stejné) ---
    const auto& entry = loaded_cache[cache_idx];
    double query_pt[2] = {x, y};
    
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);

    if (entry->tree->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters())) {
        return entry->cloud.pts[ret_index].z;
    }

    throw std::runtime_error("No point was found in the tile.");
}



std::pair<double, std::string> DMR5GElevationGrid::getEllipsoidElevation(double lat, double lon) {
    // 1. Find the tile name for the given coordinates
    // findTileForGps internally performs the transformation to S-JTSK and searches the grid
    std::string tile_name = findTileForGps(lat, lon);
    if (tile_name.empty()) {
        throw std::runtime_error("No tile found for coordinates (" + std::to_string(lat) + ", " + 
                                 std::to_string(lon) + ").");
    }

    // 2. Find the URL for the given tile in our tiles list
    std::string url = "";
    for (const auto& t : tiles) {
        if (t.filename == tile_name) {
            url = t.zip_url;
            break;
        }
    }

    if (url.empty()) {
        throw std::runtime_error("No URL found for tile: " + tile_name);
    }

    // 3. Download and extract the tile (if not already in dmr5g_unzipped)
    std::string laz_path = downloadAndUnzipTile(tile_name, url);

    // 4. To get the height from LAZ, we need the coordinates in S-JTSK
    PJ_COORD c_in = proj_coord(lon, lat, 0, 0);
    PJ_COORD c_sjtsk = proj_trans(pj_wgs84_to_s_jtsk, PJ_FWD, c_in);

    // 5. Get the altitude (in the Bpv / AMSL system)
    double h_amsl = getElevationFromLaz(laz_path, c_sjtsk.xy.x, c_sjtsk.xy.y);

    // 6. Vertical transformation: EGM96 (AMSL) -> WGS84 (Ellipsoid)
    // Due to the normalization in step 1, we send (Lon, Lat, h_amsl)
    PJ_COORD c_egm = proj_coord(lon, lat, h_amsl, 0);
    PJ_COORD c_wgs84 = proj_trans(pj_egm96_to_wgs84, PJ_FWD, c_egm);

    // The result is in .v[2] (the third coordinate component)
    double h_ellipsoid = c_wgs84.v[2];

    return {h_ellipsoid, tile_name};
}