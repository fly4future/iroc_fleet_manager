# CZ Altitude
For a given latitude and longitude within the Czech Republic, this library provides the corresponding altitude in the WGS84 coordinate system used by GPS.

## Dependencies
While most dependencies are handled by `rosdep`, the `LASzip` library must be installed manually:
```bash
sudo apt install liblaszip-dev
```
## Important Notes
- ❗The underlying terrain map (DMR5G) represents a "bare-earth" model and does not include buildings, trees, or other obstacles. When planning flights, ensure you set a flight altitude in the `CzAltitudeCoveragePlanner` that is sufficiently high to clear any potential obstructions.
- ❗This library works exclusively with coordinates within the Czech Republic.
- ❗An active internet connection is required for the initial download of map tiles. Once downloaded, they are cached locally in the `data/dmr5g_unzipped` directory.

## How It Works
The altitude data is sourced from the Digital Terrain Model of the Czech Republic (DMR5G) provided by ČÚZK (the Czech Office for Surveying, Mapping and Cadastre).

When an altitude is requested for a specific coordinate, the library performs the following steps:
1.  It identifies and downloads the necessary map tile in LAZ format (a compressed point cloud) if it's not already cached.
2.  To find the altitude, the algorithm identifies the nearest point to the requested coordinates within the point cloud data and returns the point's altitude.
3.  The raw altitude is provided in the S-JTSK coordinate system. The library then transforms it into the WGS84 coordinate system, which is compatible with GPS.

## Testing CZ Altitude library separately
The CZAltitude library can be tested separately by running the code in main.cpp. It can be built and run in CzAltitude repository using these commands:
```bash
mkdir build/
cd build
cmake ..
cmake --build .
./czAltitude
```