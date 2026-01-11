# ArduPilot Terrain Tile Visualizer

Python tools for creating both 3D and 2D visualizations of ArduPilot terrain tiles (.DAT files).

## Tools Included

### 1. visualize_terrain.py - 3D Visualization
Creates stunning 3D terrain models with elevation represented by both height and color.

### 2. visualize_terrain_2d.py - 2D Elevation Maps
Creates 2D color-coded elevation maps (topographic view) where color represents height.

## Features

- Reads and parses ArduPilot .DAT terrain tile files
- **3D tool**: Creates beautiful 3D terrain visualizations with rotation
- **2D tool**: Creates topographic-style elevation maps with customizable colormaps
- **2D tool**: Optional hillshade effect for enhanced terrain visualization
- **2D tool**: Optional elevation contour lines
- Supports both combined terrain views and individual block views
- Interactive visualization or save to file
- Displays terrain statistics (min, max, mean elevation)

## Requirements

Install the required Python packages:

```bash
pip install numpy matplotlib
```

## Usage

### 3D Visualization (visualize_terrain.py)

**Basic Usage** - Save 3D terrain to file:
```bash
python3 visualize_terrain.py S45E170.DAT --output terrain_3d.png
```

**Interactive Mode** - Rotate and zoom the 3D terrain:
```bash
python3 visualize_terrain.py S45E170.DAT --interactive
```

**View Individual Blocks** - Show all terrain blocks separately:
```bash
python3 visualize_terrain.py S45E170.DAT --blocks
```

**Command Line Options:**
- `terrain_file` - Path to the .DAT terrain file (required)
- `--interactive` or `-i` - Show interactive 3D plot window
- `--output FILE` or `-o FILE` - Save visualization to image file
- `--blocks` or `-b` - Show individual blocks instead of combined terrain

### 2D Elevation Maps (visualize_terrain_2d.py)

**Basic Usage** - Create a simple 2D elevation map:
```bash
python3 visualize_terrain_2d.py S45E170.DAT --output terrain_2d.png
```

**With Hillshade** - Add shaded relief for better terrain visualization:
```bash
python3 visualize_terrain_2d.py S45E170.DAT --output terrain_2d.png --hillshade
```

**With Contour Lines** - Add elevation contour lines:
```bash
python3 visualize_terrain_2d.py S45E170.DAT --output terrain_2d.png --contours
```

**Custom Colormap** - Use different color schemes:
```bash
python3 visualize_terrain_2d.py S45E170.DAT --output terrain_2d.png --colormap viridis
```

**All Features Combined**:
```bash
python3 visualize_terrain_2d.py S45E170.DAT --output terrain_2d.png --hillshade --contours --colormap gist_earth
```

**Command Line Options:**
- `terrain_file` - Path to the .DAT terrain file (required)
- `--output FILE` or `-o FILE` - Save visualization to image file
- `--colormap NAME` or `-c NAME` - Matplotlib colormap (default: terrain)
  - Popular options: `terrain`, `viridis`, `plasma`, `gist_earth`, `rainbow`, `cividis`
- `--hillshade` - Add hillshade effect for 3D-like appearance
- `--contours` - Add elevation contour lines with labels
- `--blocks` or `-b` - Show individual blocks instead of combined terrain

## Examples

### 3D Visualizations
```bash
# Save 3D visualization to PNG
python3 visualize_terrain.py S45E170.DAT -o south_island_3d.png

# Interactive 3D exploration
python3 visualize_terrain.py S45E171.DAT -i

# View individual terrain blocks in 3D
python3 visualize_terrain.py S45E170.DAT -b
```

### 2D Elevation Maps
```bash
# Simple 2D elevation map with terrain colors
python3 visualize_terrain_2d.py S45E170.DAT -o terrain_2d.png

# Topographic map with hillshade and contours
python3 visualize_terrain_2d.py S45E170.DAT -o topo_map.png --hillshade --contours

# Different color scheme for coastal areas
python3 visualize_terrain_2d.py S45E171.DAT -o coastal.png --colormap viridis

# Individual block heatmaps
python3 visualize_terrain_2d.py S45E170.DAT -o blocks_2d.png --blocks
```

## Terrain File Format

ArduPilot terrain files use the following naming convention:
- `N` or `S` followed by latitude (e.g., S45 = 45° South)
- `E` or `W` followed by longitude (e.g., E170 = 170° East)
- `.DAT` extension

Examples:
- `S45E170.DAT` - South Island, New Zealand region
- `N37W122.DAT` - San Francisco Bay Area
- `S33E151.DAT` - Sydney, Australia region

## Output

The tool displays:
- Number of valid terrain blocks read
- Geographic coverage (latitude/longitude)
- Terrain statistics:
  - Minimum elevation
  - Maximum elevation
  - Mean elevation
  - Grid dimensions

The 3D visualization shows:
- Color-coded elevation (terrain colormap)
- Latitude and longitude axes
- Height in meters
- Interactive rotation (in interactive mode)

## Technical Details

Each terrain tile contains:
- Multiple 2048-byte blocks
- Each block covers a 32×28 grid
- Grid spacing typically 100 meters
- Heights stored as signed 16-bit integers (meters above sea level)
- Blocks are organized by lat/lon coordinates

## Tested With

- S45E170.DAT - Contains 1410 blocks, elevation 22-2299m (South Island mountains)
- S45E171.DAT - Contains 1410 blocks, elevation 0-1373m (coastal regions)
