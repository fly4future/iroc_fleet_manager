
#include "CzAltitude/DMR5GElevationGrid.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

/**
 * WHAT IS THIS FILE FOR:
 * This file is just for testing if the DMR5GElevationGrid class and getEllipsoidElevation(...) function work properly
 */

int main() {
    try {
        std::cout << "🚀 Starting altitude test..." << std::endl;

        DMR5GElevationGrid elev_grid("../data/dmr5g_index.csv", "../data/dmr5g_unzipped");

        // Testing coordinates
        std::vector<std::pair<double, double>> points = {
            {50.319065, 12.105336},
            {50.278410, 12.375340},
            {50.405933, 12.799686},
            {50.415560, 13.215794},
            {51.029346, 14.325413},
            {50.987870, 15.050510},
            {50.705196, 15.679478},
            {50.611170, 16.305698},
            {50.380547, 16.989597},
            {50.315697, 16.338657},
            {50.240220, 17.670750},
            {50.041310, 17.689976}
            // {49.991894, 18.052525},
            // {49.600075, 18.667759},
            // {49.400290, 18.349155},
            // {49.063110, 18.041538},
            // {48.875600, 17.591099},
            // {48.933370, 17.184605},
            // {48.691006, 16.964878},
            // {48.868370, 16.481480},
            // {48.781574, 16.228794},
            // {49.052315, 15.020298},
            // {48.607533, 14.690708},
            // {48.636580, 14.108433},
            // {48.998283, 13.443760},
            // {49.156605, 13.240513},
            // {49.400290, 12.817539},
            // {49.745823, 12.465977},
            // {49.961864, 12.597813},
            // {49.503853, 14.300693},
            // {50.018370, 13.482212},
            // {50.541405, 14.525913},
            // {50.788616, 13.960117},
            // {50.299908, 15.454258},
            // {49.806130, 15.201572},
            // {50.004250, 16.624302},
            // {49.603634, 16.382603},
            // {49.386803, 15.603131},
            // {49.610752, 17.234043},
            // {49.813217, 17.700962},
            // {49.389570, 17.700962},
            // {49.167380, 13.982090},
            // {49.720970, 13.240513},
            // {51.679410, 14.130405},
            // {48.654728, 11.911167},
            // {48.502090, 18.459019}
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