
#include "CzAltitude/DMPOKElevationGrid.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

/**
 * WHAT IS THIS FILE FOR:
 * This file is just for testing if the DMPOKElevationGrid class and getEllipsoidElevation(...) function work properly
 */

int main() {
    try {
        std::cout << "🚀 Starting altitude test..." << std::endl;

        DMPOKElevationGrid elev_grid("../data/dmp_ok_index.csv", "../data/dmp_ok_unzipped");

        // Testing coordinates
        std::vector<std::pair<double, double>> points = {
            {50.557976, 13.514551},
            {50.557940, 13.523207},
            {50.557613, 13.530760},
            {50.557507, 13.537455},
            {50.552322, 13.537111},
            {50.552868, 13.530159},
            {50.553413, 13.522606},
            {50.553470, 13.515139},
            {50.547306, 13.515310},
            {50.547634, 13.522520},
            {50.547523, 13.525782},
            {50.547523, 13.529129},
            {50.547523, 13.532562},
            {50.547523, 13.535481}
        };

        std::cout << "\n--- RESULTS ---" << std::endl;
        std::cout << std::fixed << std::setprecision(2);

        for (const auto& pt : points) {
            try {
                auto result = elev_grid.getEllipsoidElevation(pt.first, pt.second);
                std::cout << "📍 Point: [" << pt.first << ", " << pt.second << "]" << std::endl;
                std::cout << "   📦 Tile:       " << result.second << std::endl;
                std::cout << "   🗻 Altitude:   " << result.first << " m (WGS84 elipsoid)" << std::endl;
                std::cout << "-----------------------------------" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "❌ Error for point: " << e.what() << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "💥 Critical error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}