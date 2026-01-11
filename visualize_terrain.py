#!/usr/bin/env python3
"""
ArduPilot Terrain Tile 3D Visualizer

This tool reads ArduPilot terrain tile files (.DAT) and creates 3D visualizations
of the terrain data.

Usage:
    python visualize_terrain.py S45E170.DAT
    python visualize_terrain.py S45E170.DAT --interactive
    python visualize_terrain.py S45E170.DAT --output terrain.png
"""

import struct
import sys
import os
from argparse import ArgumentParser

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except ImportError:
    print("Error: This tool requires numpy and matplotlib")
    print("Install with: pip install numpy matplotlib")
    sys.exit(1)

# Constants from AP_Terrain
TERRAIN_GRID_BLOCK_SIZE_X = 32
TERRAIN_GRID_BLOCK_SIZE_Y = 28
IO_BLOCK_SIZE = 2048
TERRAIN_GRID_FORMAT_VERSION = 1


class TerrainBlock:
    """Represents a single 2048-byte terrain block"""

    def __init__(self):
        self.bitmap = 0
        self.lat = 0  # degrees * 1e7
        self.lon = 0  # degrees * 1e7
        self.crc = 0
        self.version = 0
        self.spacing = 0  # meters
        self.height = np.zeros((TERRAIN_GRID_BLOCK_SIZE_X, TERRAIN_GRID_BLOCK_SIZE_Y), dtype=np.int16)
        self.grid_idx_x = 0
        self.grid_idx_y = 0
        self.lon_degrees = 0
        self.lat_degrees = 0

    def read_from_buffer(self, buf):
        """Parse a 2048-byte buffer into terrain block data"""
        if len(buf) != IO_BLOCK_SIZE:
            return False

        # Read header (22 bytes)
        (self.bitmap, self.lat, self.lon, self.crc,
         self.version, self.spacing) = struct.unpack("<QiiHHH", buf[:22])

        # Check version
        if self.version != TERRAIN_GRID_FORMAT_VERSION:
            return False

        # Check if block is empty (spacing == 0 means not filled)
        if self.spacing == 0:
            return False

        # Read height data (32x28 grid of int16)
        offset = 22
        for x in range(TERRAIN_GRID_BLOCK_SIZE_X):
            heights = struct.unpack("<{0}h".format(TERRAIN_GRID_BLOCK_SIZE_Y),
                                    buf[offset:offset + TERRAIN_GRID_BLOCK_SIZE_Y * 2])
            self.height[x, :] = heights
            offset += TERRAIN_GRID_BLOCK_SIZE_Y * 2

        # Read grid indices and degree references
        (self.grid_idx_x, self.grid_idx_y,
         self.lon_degrees, self.lat_degrees) = struct.unpack("<HHhb", buf[offset:offset+7])

        return True

    def get_lat_lon_degrees(self):
        """Get the SW corner latitude and longitude in degrees"""
        return self.lat * 1e-7, self.lon * 1e-7


class TerrainFile:
    """Represents a complete terrain .DAT file"""

    def __init__(self, filename):
        self.filename = filename
        self.blocks = []
        self.lat_degrees = None
        self.lon_degrees = None
        self._parse_filename()

    def _parse_filename(self):
        """Extract lat/lon from filename like S45E170.DAT"""
        basename = os.path.basename(self.filename)
        if not basename.endswith('.DAT'):
            raise ValueError("File must have .DAT extension")

        basename = basename[:-4]  # Remove .DAT

        # Parse N/S and E/W
        if basename[0] == 'S':
            lat_sign = -1
            lat_start = 1
        elif basename[0] == 'N':
            lat_sign = 1
            lat_start = 1
        else:
            raise ValueError("Filename must start with N or S")

        # Find E or W
        ew_pos = None
        for i, c in enumerate(basename[lat_start:], lat_start):
            if c in ['E', 'W']:
                ew_pos = i
                break

        if ew_pos is None:
            raise ValueError("Filename must contain E or W")

        self.lat_degrees = lat_sign * int(basename[lat_start:ew_pos])

        if basename[ew_pos] == 'W':
            lon_sign = -1
        else:
            lon_sign = 1

        self.lon_degrees = lon_sign * int(basename[ew_pos+1:])

    def read(self):
        """Read all blocks from the terrain file"""
        if not os.path.exists(self.filename):
            raise FileNotFoundError(f"File not found: {self.filename}")

        with open(self.filename, 'rb') as f:
            block_num = 0
            while True:
                buf = f.read(IO_BLOCK_SIZE)
                if len(buf) != IO_BLOCK_SIZE:
                    break

                block = TerrainBlock()
                if block.read_from_buffer(buf):
                    self.blocks.append(block)
                    block_num += 1

        print(f"Read {len(self.blocks)} valid blocks from {self.filename}")
        return len(self.blocks) > 0

    def get_combined_terrain(self):
        """Combine all blocks into a single height map"""
        if not self.blocks:
            return None, None, None

        # Find the bounding box of all blocks
        min_lat = min(block.lat for block in self.blocks)
        max_lat = max(block.lat for block in self.blocks)
        min_lon = min(block.lon for block in self.blocks)
        max_lon = max(block.lon for block in self.blocks)

        # Get spacing from first block
        spacing = self.blocks[0].spacing

        # Calculate grid dimensions
        lat_range = max_lat - min_lat
        lon_range = max_lon - min_lon

        # Estimate grid size (this is approximate)
        grid_x = int(lat_range * 1e-7 * 111000 / spacing) + TERRAIN_GRID_BLOCK_SIZE_X
        grid_y = int(lon_range * 1e-7 * 111000 / spacing) + TERRAIN_GRID_BLOCK_SIZE_Y

        # Create combined grid
        terrain = np.full((grid_x, grid_y), np.nan, dtype=float)

        # Fill in data from each block
        for block in self.blocks:
            # Calculate offset in the combined grid
            x_offset = int((block.lat - min_lat) * 1e-7 * 111000 / spacing)
            y_offset = int((block.lon - min_lon) * 1e-7 * 111000 / spacing)

            # Copy block data
            for x in range(TERRAIN_GRID_BLOCK_SIZE_X):
                for y in range(TERRAIN_GRID_BLOCK_SIZE_Y):
                    gx = x_offset + x
                    gy = y_offset + y
                    if 0 <= gx < grid_x and 0 <= gy < grid_y:
                        terrain[gx, gy] = block.height[x, y]

        # Create coordinate arrays
        lat_coords = np.linspace(min_lat * 1e-7, max_lat * 1e-7, grid_x)
        lon_coords = np.linspace(min_lon * 1e-7, max_lon * 1e-7, grid_y)

        return terrain, lat_coords, lon_coords


def visualize_3d(terrain_file, interactive=False, output_file=None):
    """Create a 3D visualization of the terrain"""

    # Read the terrain file
    tfile = TerrainFile(terrain_file)
    if not tfile.read():
        print("Error: No valid terrain blocks found")
        return

    print(f"File covers: {tfile.lat_degrees}° lat, {tfile.lon_degrees}° lon")

    # Get combined terrain
    terrain, lat_coords, lon_coords = tfile.get_combined_terrain()

    if terrain is None:
        print("Error: Could not create terrain grid")
        return

    # Remove NaN values for visualization
    valid_mask = ~np.isnan(terrain)
    if not np.any(valid_mask):
        print("Error: No valid terrain data")
        return

    # Get statistics
    valid_terrain = terrain[valid_mask]
    min_height = np.min(valid_terrain)
    max_height = np.max(valid_terrain)
    mean_height = np.mean(valid_terrain)

    print("Terrain stats:")
    print("  Min height: {0:.1f} m".format(min_height))
    print("  Max height: {0:.1f} m".format(max_height))
    print("  Mean height: {0:.1f} m".format(mean_height))
    print("  Grid size: {0}".format(terrain.shape))

    # Create 3D plot
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Create meshgrid for plotting
    LON, LAT = np.meshgrid(lon_coords, lat_coords)

    # Replace NaN with mean for visualization
    Z = terrain.copy()
    Z[np.isnan(Z)] = mean_height

    # Plot surface with colormap
    ax.plot_surface(LON, LAT, Z, cmap=cm.terrain,
                    linewidth=0, antialiased=True, alpha=0.9)

    # Customize the plot
    ax.set_xlabel('Longitude (degrees)', fontsize=10)
    ax.set_ylabel('Latitude (degrees)', fontsize=10)
    ax.set_zlabel('Height (meters)', fontsize=10)
    ax.set_title('ArduPilot Terrain Tile: {0}'.format(os.path.basename(terrain_file)),
                 fontsize=12, fontweight='bold')

    # Add colorbar with proper reference
    mappable = cm.ScalarMappable(cmap=cm.terrain)
    mappable.set_array(Z)
    fig.colorbar(mappable, ax=ax, shrink=0.5, aspect=5, label='Elevation (m)')

    # Set viewing angle
    ax.view_init(elev=30, azim=45)

    # Add grid
    ax.grid(True, alpha=0.3)

    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved visualization to {output_file}")

    if interactive or not output_file:
        print("Displaying interactive 3D plot...")
        plt.show()

    plt.close()


def visualize_blocks(terrain_file):
    """Create a visualization showing individual blocks"""

    tfile = TerrainFile(terrain_file)
    if not tfile.read():
        print("Error: No valid terrain blocks found")
        return

    # Calculate grid layout for subplots
    n_blocks = len(tfile.blocks)
    cols = min(4, n_blocks)
    rows = (n_blocks + cols - 1) // cols

    fig = plt.figure(figsize=(4*cols, 3*rows))

    for i, block in enumerate(tfile.blocks):
        ax = fig.add_subplot(rows, cols, i+1, projection='3d')

        # Create coordinate grids
        x = np.arange(TERRAIN_GRID_BLOCK_SIZE_X) * block.spacing
        y = np.arange(TERRAIN_GRID_BLOCK_SIZE_Y) * block.spacing
        X, Y = np.meshgrid(y, x)

        # Plot the block
        ax.plot_surface(X, Y, block.height, cmap=cm.terrain,
                        linewidth=0, antialiased=False)

        lat, lon = block.get_lat_lon_degrees()
        ax.set_title('Block {0}\n{1:.4f}deg, {2:.4f}deg'.format(i, lat, lon), fontsize=8)
        ax.set_xlabel('E (m)', fontsize=7)
        ax.set_ylabel('N (m)', fontsize=7)
        ax.set_zlabel('H (m)', fontsize=7)
        ax.tick_params(labelsize=6)

    plt.suptitle('Individual Blocks: {0}'.format(os.path.basename(terrain_file)),
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.show()
    plt.close()


def main():
    parser = ArgumentParser(description='Visualize ArduPilot terrain tiles in 3D')
    parser.add_argument('terrain_file', help='Path to .DAT terrain file')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Show interactive 3D plot')
    parser.add_argument('--output', '-o', default=None,
                        help='Save visualization to file (e.g., terrain.png)')
    parser.add_argument('--blocks', '-b', action='store_true',
                        help='Show individual blocks instead of combined terrain')

    args = parser.parse_args()

    if not os.path.exists(args.terrain_file):
        print(f"Error: File not found: {args.terrain_file}")
        sys.exit(1)

    try:
        if args.blocks:
            visualize_blocks(args.terrain_file)
        else:
            visualize_3d(args.terrain_file, args.interactive, args.output)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
