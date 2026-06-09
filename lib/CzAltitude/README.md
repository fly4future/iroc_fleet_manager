# CZ Altitude
For a given latitude and longitude within the Czech Republic, this library provides the corresponding altitude in the WGS84 coordinate system used by GPS.

## Dependencies
While most dependencies are handled by `rosdep`, the `LASzip` library must be installed manually:
```bash
sudo apt install liblaszip-dev
```
## Important Notes
- ❗This library works exclusively with coordinates within the Czech Republic.
- ❗An active internet connection is required for the initial download of map tiles. Once downloaded, they are cached locally in the `data/dmp_ok_unzipped` directory.

## How It Works
The altitude data is sourced from the Digital Surface Model of the Czech Republic (DMP OK) provided by ČÚZK (the Czech Office for Surveying, Mapping and Cadastre). You can check the point cloud density, map update dates, and metadata details <a href="https://geoportal.cuzk.gov.cz/(S(eswjykus3f00352vto0nckui))/Default.aspx?lng=CZ&mode=TextMeta&side=vyskopis&metadataID=CZ-CUZK-DMPOK&mapid=8&menu=3032">here</a>. You can view the interactive map <a href="https://ags.cuzk.gov.cz/geoprohlizec/">here</a>.

When an altitude is requested for a specific coordinate, the library performs the following steps:
1. It identifies and downloads the necessary map tile in LAZ format (a compressed point cloud) if it's not already cached.
2. The point cloud is processed into a 2D raster grid with a default resolution of 1 value per 1x1m area. From all the points that fall into a specific 1x1m cell, the highest one is selected. The resulting raster grid is then saved, and the original `.laz` file is deleted to save space.
3. Altitude data is accessed from the grid in constant time O(1).
4. The raw altitude is provided in the S-JTSK coordinate system. The library then transforms it into the WGS84 coordinate system, which is compatible with GPS.

A saved raster file that covers area of aproximately 3x3km takes up approximately 30 MB of disk space at this resolution.

## Testing CZ Altitude library separately
The library can be tested separately by running `main.cpp`. It can be built and run directly from the `CzAltitude` repository using the following commands:
```bash
mkdir build/
cd build
cmake ..
cmake --build .
./czAltitude
```