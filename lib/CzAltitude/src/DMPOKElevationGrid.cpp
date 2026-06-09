#include "../include/CzAltitude/DMPOKElevationGrid.hpp"
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
#include <limits>


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

DMPOKElevationGrid::DMPOKElevationGrid(std::string csv_index, std::string laz_dir) 
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

DMPOKElevationGrid::~DMPOKElevationGrid() {
    if (pj_wgs84_to_s_jtsk) proj_destroy(pj_wgs84_to_s_jtsk);
    if (pj_egm96_to_wgs84) proj_destroy(pj_egm96_to_wgs84);
    if (ctx) proj_context_destroy(ctx);
}

void DMPOKElevationGrid::loadIndex() {
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

void DMPOKElevationGrid::buildSpatialGrid() {
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

std::vector<const Tile*> DMPOKElevationGrid::findTilesForGps(double x, double y) {
    std::vector<const Tile*> found_tiles;
    
    // Calculate the index in the spatial grid
    int col = static_cast<int>((x - grid_minx) / step_x);
    int row = static_cast<int>((y - grid_miny) / step_y);

    // Check grid boundaries
    if (row >= 0 && row < (int)spatial_grid.size() && 
        col >= 0 && col < (int)spatial_grid[0].size()) {
        
        // Search through tiles in the given cell
        for (size_t tile_idx : spatial_grid[row][col]) {
            if (tiles[tile_idx].contains(x, y)) {
                found_tiles.push_back(&tiles[tile_idx]);
            }
        }
    }
    return found_tiles;
}

std::string DMPOKElevationGrid::downloadAndUnzipTile(const std::string& tile_name, const std::string& url) {
    // Remove the original extension (e.g., .zip or .laz from CSV) and ensure .laz extension
    std::string base_name = tile_name;
    size_t last_dot = base_name.find_last_of(".");
    if (last_dot != std::string::npos) {
        base_name = base_name.substr(0, last_dot);
    }
    
    std::string laz_name = base_name + ".laz";
    fs::path laz_path = fs::path(laz_directory) / laz_name;

    // If the LAZ file already exists, do not download it again
    if (fs::exists(laz_path)) return laz_path.string();

    // Create the directory if it does not exist
    fs::create_directories(laz_directory);

    bool is_zip = (url.length() >= 4 && url.substr(url.length() - 4) == ".zip");
    std::string download_name = base_name + (is_zip ? ".zip" : ".laz");
    fs::path download_path = fs::path(laz_directory) / download_name;

    // --- 1. Download using libcurl ---
    std::cout << "Downloading " << (is_zip ? "ZIP" : "LAZ") << ": " << download_name << " from " << url << "..." << std::endl;
    CURL *curl = curl_easy_init();
    if (curl) {
        FILE *fp = fopen(download_path.string().c_str(), "wb");
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Follow redirects

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        fclose(fp);

        if (res != CURLE_OK) {
            fs::remove(download_path); // Cleanup on failure
            throw std::runtime_error("CURL download failed: " + std::string(curl_easy_strerror(res)));
        }
    }

    // --- 2. Extract using libzip (if it's a ZIP archive) ---
    if (is_zip) {
        std::cout << "Extracting " << download_name << "..." << std::endl;
        int err = 0;
        zip *z = zip_open(download_path.string().c_str(), 0, &err);
        if (!z) throw std::runtime_error("Cannot open ZIP file.");

        zip_int64_t num_entries = zip_get_num_entries(z, 0);
        for (zip_int64_t i = 0; i < num_entries; i++) {
            struct zip_stat st;
            zip_stat_index(z, i, 0, &st);
            
            if (!st.name) continue;
            std::string name_inside = st.name;
            if (name_inside.length() < 4 || name_inside.substr(name_inside.length() - 4) != ".laz") continue;

            zip_file *f = zip_fopen_index(z, i, 0);
            if (!f) continue;

            FILE *out_fp = fopen(laz_path.string().c_str(), "wb");
            if (out_fp) {
                char buffer[8192];
                zip_int64_t n;
                while ((n = zip_fread(f, buffer, sizeof(buffer))) > 0) fwrite(buffer, 1, n, out_fp);
                fclose(out_fp);
            }
            zip_fclose(f);
            break; 
        }
        zip_close(z);

        // --- 3. Cleanup ---
        fs::remove(download_path);
    }

    return laz_path.string();
}




double DMPOKElevationGrid::getElevationFromRaster(const std::string& laz_path, double x, double y, bool laz_already_existed, const Tile& tile) {
    // 1. Check LRU cache (similar to KD-Tree, but we can hold more rasters in memory since they are small)
    auto it = std::find_if(raster_cache.begin(), raster_cache.end(),
        [&laz_path](const std::unique_ptr<LoadedRaster>& item) {
            return item->path == laz_path;
        });

    if (it != raster_cache.end()) {
        std::unique_ptr<LoadedRaster> temp = std::move(*it);
        raster_cache.erase(it);
        raster_cache.push_back(std::move(temp));
    } else {
        if (raster_cache.size() >= MAX_RASTER_CACHE_TILES) {
            std::cout << "Memory cache limit reached. Evicting LRU raster: " << raster_cache.front()->path << std::endl;
            raster_cache.erase(raster_cache.begin());
        }

        fs::path raster_file = laz_path;
        raster_file.replace_extension(".raster");

        auto new_entry = std::make_unique<LoadedRaster>();
        new_entry->path = laz_path;
        new_entry->res = raster_resolution;

        new_entry->min_x = tile.min_x;
        new_entry->min_y = tile.min_y;
        new_entry->max_x = tile.max_x;
        new_entry->max_y = tile.max_y;

        new_entry->cols = static_cast<int>(std::ceil((new_entry->max_x - new_entry->min_x) / new_entry->res)) + 1;
        new_entry->rows = static_cast<int>(std::ceil((new_entry->max_y - new_entry->min_y) / new_entry->res)) + 1;

        // Check if the precomputed raster file already exists on the disk
        if (fs::exists(raster_file)) {
            std::cout << "Loading raster from: " << raster_file.string() << std::endl;
            std::ifstream ifs(raster_file.string(), std::ios::binary);
            if (!ifs) throw std::runtime_error("Cannot open raster file for reading.");
            
            // Read the header (5 doubles: min_x, min_y, max_x, max_y, resolution)
            double header[5];
            ifs.read(reinterpret_cast<char*>(header), sizeof(header));
            
            size_t num_cells = new_entry->cols * new_entry->rows;
            new_entry->data.resize(num_cells);
            ifs.read(reinterpret_cast<char*>(new_entry->data.data()), num_cells * sizeof(float));
        } else {
            std::cout << "Generating raster from LAZ: " << laz_path << std::endl;
            
            // Initialize the raster grid with NaNs (representing empty cells with no data)
            size_t num_cells = new_entry->cols * new_entry->rows;
            new_entry->data.assign(num_cells, std::numeric_limits<float>::quiet_NaN());

            laszip_POINTER laszip_reader;
            if (laszip_create(&laszip_reader)) throw std::runtime_error("LASzip: Could not create reader.");
            
            laszip_BOOL is_compressed = 0;
            if (laszip_open_reader(laszip_reader, laz_path.c_str(), &is_compressed)) {
                laszip_clean(laszip_reader);
                throw std::runtime_error("LASzip: Could not open file: " + laz_path);
            }

            laszip_header* header;
            laszip_get_header_pointer(laszip_reader, &header);
            laszip_point* point;
            laszip_get_point_pointer(laszip_reader, &point);

            laszip_I64 num_points = (header->number_of_point_records > 0) ? 
                                    header->number_of_point_records : header->extended_number_of_point_records;

            int points_inside = 0;
            int points_outside = 0;
            double laz_min_x = 1e18, laz_max_x = -1e18, laz_min_y = 1e18, laz_max_y = -1e18;

            // 2. Insert points into the grid (we always keep the highest Z value for a given cell, creating a Digital Surface Model)
            for (laszip_I64 i = 0; i < num_points; i++) {
                if (laszip_read_point(laszip_reader)) break;
                double px = point->X * header->x_scale_factor + header->x_offset;
                double py = point->Y * header->y_scale_factor + header->y_offset;
                double pz = point->Z * header->z_scale_factor + header->z_offset;

                // Auto-normalize S-JTSK to EPSG:5514 (Negative Easting/Northing).
                // In the Czech Republic, Easting is always < 900k and Northing is always > 900k, 
                // so they can be safely swapped if the axes are flipped.
                if (std::abs(px) > std::abs(py)) std::swap(px, py);
                if (px > 0) px = -px;
                if (py > 0) py = -py;

                laz_min_x = std::min(laz_min_x, px);
                laz_max_x = std::max(laz_max_x, px);
                laz_min_y = std::min(laz_min_y, py);
                laz_max_y = std::max(laz_max_y, py);

                int c = static_cast<int>((px - new_entry->min_x) / new_entry->res);
                int r = static_cast<int>((py - new_entry->min_y) / new_entry->res);
                if (c >= 0 && c < new_entry->cols && r >= 0 && r < new_entry->rows) {
                    points_inside++;
                    int idx = r * new_entry->cols + c;
                    if (std::isnan(new_entry->data[idx])) new_entry->data[idx] = static_cast<float>(pz);
                    else new_entry->data[idx] = std::max(new_entry->data[idx], static_cast<float>(pz));
                } else {
                    points_outside++;
                }
            }
            laszip_close_reader(laszip_reader);
            laszip_clean(laszip_reader);

            // Save the generated raster to disk for fast future access
            std::ofstream ofs(raster_file.string(), std::ios::binary);
            if (ofs) {
                double h_data[5] = {new_entry->min_x, new_entry->min_y, new_entry->max_x, new_entry->max_y, new_entry->res};
                ofs.write(reinterpret_cast<const char*>(h_data), sizeof(h_data));
                ofs.write(reinterpret_cast<const char*>(new_entry->data.data()), new_entry->data.size() * sizeof(float));
            }

            if (!laz_already_existed) {
                std::cout << "Cleaning up LAZ (raster created successfully): " << laz_path << std::endl;
                fs::remove(laz_path);
            }
        }
        raster_cache.push_back(std::move(new_entry));
    }

    // 3. Fast direct mapping of X/Y coordinates to the O(1) raster matrix
    const auto& entry = raster_cache.back();
    int c = static_cast<int>((x - entry->min_x) / entry->res);
    int r = static_cast<int>((y - entry->min_y) / entry->res);

    if (c >= 0 && c < entry->cols && r >= 0 && r < entry->rows) {
        float val = entry->data[r * entry->cols + c];
        if (!std::isnan(val)) {
            return static_cast<double>(val);
        }
        
        // If there is a hole (NaN) in the matrix at the exact location, 
        // intelligently try to expand the search radius up to 2 pixels in all directions.
        for (int radius = 1; radius <= 2; ++radius) {
            for (int dr = -radius; dr <= radius; ++dr) {
                for (int dc = -radius; dc <= radius; ++dc) {
                    if (std::abs(dr) != radius && std::abs(dc) != radius) continue;
                    int nr = r + dr, nc = c + dc;
                    if (nr >= 0 && nr < entry->rows && nc >= 0 && nc < entry->cols) {
                        float nval = entry->data[nr * entry->cols + nc];
                        if (!std::isnan(nval)) {
                            return static_cast<double>(nval);
                        }
                    }
                }
            }
        }
    }
    
    throw std::runtime_error("No point was found in the raster near the requested coordinates.");
}


std::pair<double, std::string> DMPOKElevationGrid::getEllipsoidElevation(double lat, double lon) {
    // 1. Transform GPS to S-JTSK
    PJ_COORD c_in = proj_coord(lon, lat, 0, 0);
    PJ_COORD c_sjtsk = proj_trans(pj_wgs84_to_s_jtsk, PJ_FWD, c_in);
    double x = c_sjtsk.xy.x;
    double y = c_sjtsk.xy.y;

    // 2. Find ALL candidate tiles for the given coordinates
    std::vector<const Tile*> candidate_tiles = findTilesForGps(x, y);
    if (candidate_tiles.empty()) {
        throw std::runtime_error("No tile found for coordinates (" + std::to_string(lat) + ", " + std::to_string(lon) + ").");
    }

    // 3. Try candidates one by one. Due to WGS84 polygon distortion in the CSV,
    // a point can fall into the bloated polygon boundary but completely miss the actual LAZ data.
    for (const Tile* found_tile : candidate_tiles) {
        try {
            std::string tile_name = found_tile->filename;
            std::string url = found_tile->zip_url;

            std::string base_name = tile_name;
            size_t last_dot = base_name.find_last_of(".");
            if (last_dot != std::string::npos) base_name = base_name.substr(0, last_dot);
            fs::path expected_laz = fs::path(laz_directory) / (base_name + ".laz");
            fs::path expected_raster = fs::path(laz_directory) / (base_name + ".raster");
            bool laz_already_existed = fs::exists(expected_laz);

            std::string laz_path = expected_laz.string();

            // 4. Download and extract the tile (skip if the raster already exists)
            bool has_cached_raster = fs::exists(expected_raster);
            if (!has_cached_raster) {
                laz_path = downloadAndUnzipTile(tile_name, url);
            }

            // 5. Get the altitude
            double h_amsl = getElevationFromRaster(laz_path, x, y, laz_already_existed, *found_tile);

            // 6. Vertical transformation: EGM96 (AMSL) -> WGS84 (Ellipsoid)
            PJ_COORD c_egm = proj_coord(lon, lat, h_amsl, 0);
            PJ_COORD c_wgs84 = proj_trans(pj_egm96_to_wgs84, PJ_FWD, c_egm);

            return {c_wgs84.v[2], tile_name};
        } catch (const std::exception& e) {
            std::cout << "[INFO] Point not valid in tile " << found_tile->filename << " (" << e.what() << "), trying next if available..." << std::endl;
        }
    }

    throw std::runtime_error("No valid point was found in any candidate tile.");
}
