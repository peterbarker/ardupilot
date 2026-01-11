#!/usr/bin/env python3
"""
ArduPilot Terrain Tile 2D Visualizer

This tool reads ArduPilot terrain tile files (.DAT) and creates 2D color-coded
elevation maps where color represents height.

Usage:
    python visualize_terrain_2d.py S45E170.DAT
    python visualize_terrain_2d.py S45E170.DAT --output terrain_2d.png
    python visualize_terrain_2d.py S45E170.DAT --colormap viridis
"""

import struct
import sys
import os
from argparse import ArgumentParser

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib import cm  # noqa: F401
    from matplotlib.colors import LightSource
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
            raise FileNotFoundError("File not found: {0}".format(self.filename))

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

        print("Read {0} valid blocks from {1}".format(len(self.blocks), self.filename))
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


def visualize_2d(terrain_file, output_file=None, colormap='terrain', hillshade=False, contours=False):
    """Create a 2D color-coded elevation map"""

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

    # Create figure
    fig, ax = plt.subplots(figsize=(14, 12))

    # Prepare data for plotting
    plot_terrain = terrain.copy()

    if hillshade:
        # Create hillshade effect
        ls = LightSource(azdeg=315, altdeg=45)
        # Replace NaN with mean for hillshade calculation
        terrain_filled = terrain.copy()
        terrain_filled[np.isnan(terrain_filled)] = mean_height
        # Create the hillshade
        hillshade_array = ls.hillshade(terrain_filled, vert_exag=0.5)
        # Blend hillshade with color
        im = ax.imshow(hillshade_array, extent=[lon_coords[0], lon_coords[-1],
                                                lat_coords[0], lat_coords[-1]],
                       cmap='gray', alpha=0.3, origin='lower')

    # Plot the main terrain with color
    im = ax.imshow(plot_terrain, extent=[lon_coords[0], lon_coords[-1],
                                         lat_coords[0], lat_coords[-1]],
                   cmap=colormap, origin='lower', interpolation='bilinear',
                   vmin=min_height, vmax=max_height)

    # Add contour lines if requested
    if contours:
        # Create contour levels
        num_contours = 15
        contour_levels = np.linspace(min_height, max_height, num_contours)

        # Create meshgrid for contours
        LON, LAT = np.meshgrid(lon_coords, lat_coords)

        # Draw contours
        contour = ax.contour(LON, LAT, plot_terrain, levels=contour_levels,
                             colors='black', alpha=0.3, linewidths=0.5)
        ax.clabel(contour, inline=True, fontsize=6, fmt='%dm')

    # Customize the plot
    ax.set_xlabel('Longitude (degrees)', fontsize=12)
    ax.set_ylabel('Latitude (degrees)', fontsize=12)
    ax.set_title('ArduPilot Terrain Tile (2D): {0}'.format(os.path.basename(terrain_file)),
                 fontsize=14, fontweight='bold', pad=20)

    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, orientation='vertical', pad=0.02, shrink=0.8)
    cbar.set_label('Elevation (meters)', fontsize=11)

    # Add grid
    ax.grid(True, alpha=0.2, linestyle='--', linewidth=0.5)

    # Add statistics text box
    stats_text = 'Min: {0:.0f}m\nMax: {1:.0f}m\nMean: {2:.0f}m'.format(min_height, max_height, mean_height)
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # Set aspect ratio to be geographic (approximate)
    ax.set_aspect('equal', adjustable='box')

    plt.tight_layout()

    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print("Saved visualization to {0}".format(output_file))
    else:
        print("Displaying 2D elevation map...")
        plt.show()

    plt.close()


def visualize_blocks_2d(terrain_file, output_file=None):
    """Create a 2D visualization showing individual blocks"""

    tfile = TerrainFile(terrain_file)
    if not tfile.read():
        print("Error: No valid terrain blocks found")
        return

    # Calculate grid layout for subplots
    n_blocks = min(len(tfile.blocks), 20)  # Limit to first 20 blocks
    cols = min(5, n_blocks)
    rows = (n_blocks + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(3*cols, 2.5*rows))
    if n_blocks == 1:
        axes = np.array([axes])
    axes = axes.flatten()

    for i in range(n_blocks):
        block = tfile.blocks[i]
        ax = axes[i]

        # Plot the block as 2D heatmap
        im = ax.imshow(block.height, cmap='terrain', origin='lower',
                       interpolation='bilinear')

        lat, lon = block.get_lat_lon_degrees()
        ax.set_title('Block {0}\n{1:.4f}deg, {2:.4f}deg'.format(i, lat, lon), fontsize=8)
        ax.set_xlabel('East', fontsize=7)
        ax.set_ylabel('North', fontsize=7)
        ax.tick_params(labelsize=6)

        # Add colorbar to each subplot
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # Hide unused subplots
    for i in range(n_blocks, len(axes)):
        axes[i].set_visible(False)

    plt.suptitle('Individual Blocks (2D): {0}'.format(os.path.basename(terrain_file)),
                 fontsize=12, fontweight='bold')
    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=200, bbox_inches='tight')
        print("Saved blocks visualization to {0}".format(output_file))
    else:
        plt.show()

    plt.close()


def main():
    parser = ArgumentParser(description='Visualize ArduPilot terrain tiles as 2D elevation maps')
    parser.add_argument('terrain_file', help='Path to .DAT terrain file')
    parser.add_argument('--output', '-o', default=None,
                        help='Save visualization to file (e.g., terrain_2d.png)')
    parser.add_argument('--colormap', '-c', default='terrain',
                        help='Matplotlib colormap to use (default: terrain). Try: viridis, plasma, gist_earth, rainbow')
    parser.add_argument('--hillshade', action='store_true',
                        help='Add hillshade effect for better terrain visualization')
    parser.add_argument('--contours', action='store_true',
                        help='Add elevation contour lines')
    parser.add_argument('--blocks', '-b', action='store_true',
                        help='Show individual blocks instead of combined terrain')

    args = parser.parse_args()

    if not os.path.exists(args.terrain_file):
        print("Error: File not found: {0}".format(args.terrain_file))
        sys.exit(1)

    try:
        if args.blocks:
            visualize_blocks_2d(args.terrain_file, args.output)
        else:
            visualize_2d(args.terrain_file, args.output, args.colormap,
                         args.hillshade, args.contours)
    except Exception as e:
        print("Error: {0}".format(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
