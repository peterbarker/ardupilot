#!/usr/bin/env python3
"""
SRTM .hgt File Visualizer

This tool reads SRTM .hgt terrain files and creates 2D/3D visualizations.
.hgt files are raw binary elevation data from the Shuttle Radar Topography Mission.

Usage:
    python visualize_hgt.py S35E149.hgt
    python visualize_hgt.py S35E149.hgt --output terrain.png
    python visualize_hgt.py S35E149.hgt --3d
"""

import struct
import sys
import os
from argparse import ArgumentParser

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from matplotlib.colors import LightSource
except ImportError:
    print("Error: This tool requires numpy and matplotlib")
    print("Install with: pip install numpy matplotlib")
    sys.exit(1)


class HGTFile:
    """Represents an SRTM .hgt file"""

    def __init__(self, filename):
        self.filename = filename
        self.terrain = None
        self.lat_degrees = None
        self.lon_degrees = None
        self.size = None
        self._parse_filename()

    def _parse_filename(self):
        """Extract lat/lon from filename like S35E149.hgt"""
        basename = os.path.basename(self.filename)
        if not basename.upper().endswith('.HGT'):
            raise ValueError("File must have .hgt extension")

        basename = basename[:-4]  # Remove .hgt

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
        """Read the .hgt file"""
        if not os.path.exists(self.filename):
            raise FileNotFoundError("File not found: {0}".format(self.filename))

        # Get file size to determine resolution
        file_size = os.path.getsize(self.filename)

        # SRTM files are square grids
        # SRTM3: 1201x1201 = 1,442,401 samples × 2 bytes = 2,884,802 bytes
        # SRTM1: 3601x3601 = 12,967,201 samples × 2 bytes = 25,934,402 bytes

        if file_size == 2884802:
            self.size = 1201  # SRTM3 (3 arc-second)
            resolution = "SRTM3 (3 arc-second, ~90m)"
        elif file_size == 25934402:
            self.size = 3601  # SRTM1 (1 arc-second)
            resolution = "SRTM1 (1 arc-second, ~30m)"
        else:
            # Try to infer size
            samples = file_size // 2
            size = int(np.sqrt(samples))
            if size * size * 2 == file_size:
                self.size = size
                resolution = "Custom ({0}x{0})".format(size)
            else:
                raise ValueError("Unexpected file size: {0} bytes".format(file_size))

        print("Reading {0} data from {1}".format(resolution, self.filename))

        # Read the binary data (big-endian 16-bit signed integers)
        with open(self.filename, 'rb') as f:
            data = f.read()

        # Unpack as big-endian signed 16-bit integers
        num_samples = len(data) // 2
        elevations = struct.unpack(">{0}h".format(num_samples), data)

        # Reshape into 2D array
        self.terrain = np.array(elevations, dtype=np.int16).reshape((self.size, self.size))

        # SRTM uses -32768 as void/no-data value
        self.terrain = self.terrain.astype(float)
        self.terrain[self.terrain == -32768] = np.nan

        print("Loaded {0}x{0} elevation grid".format(self.size))

        # Get statistics
        valid_terrain = self.terrain[~np.isnan(self.terrain)]
        if len(valid_terrain) > 0:
            print("Elevation range: {0:.0f}m to {1:.0f}m".format(np.min(valid_terrain), np.max(valid_terrain)))
            print("Mean elevation: {0:.0f}m".format(np.mean(valid_terrain)))

        return True

    def get_coordinates(self):
        """Get lat/lon coordinate arrays"""
        # SRTM data is arranged from NW corner, rows going south, columns going east
        # The coordinates are for the SW corner of each degree square
        lat_coords = np.linspace(self.lat_degrees + 1, self.lat_degrees, self.size)
        lon_coords = np.linspace(self.lon_degrees, self.lon_degrees + 1, self.size)
        return lat_coords, lon_coords


def visualize_2d(hgt_file, output_file=None, colormap='terrain', hillshade=False, contours=False):
    """Create a 2D color-coded elevation map"""

    # Read the .hgt file
    hgt = HGTFile(hgt_file)
    if not hgt.read():
        print("Error: Could not read .hgt file")
        return

    terrain = hgt.terrain
    lat_coords, lon_coords = hgt.get_coordinates()

    # Get statistics
    valid_mask = ~np.isnan(terrain)
    if not np.any(valid_mask):
        print("Error: No valid terrain data")
        return

    valid_terrain = terrain[valid_mask]
    min_height = np.min(valid_terrain)
    max_height = np.max(valid_terrain)
    mean_height = np.mean(valid_terrain)

    # Create figure
    fig, ax = plt.subplots(figsize=(14, 12))

    if hillshade:
        # Create hillshade effect
        ls = LightSource(azdeg=315, altdeg=45)
        terrain_filled = terrain.copy()
        terrain_filled[np.isnan(terrain_filled)] = mean_height
        hillshade_array = ls.hillshade(terrain_filled, vert_exag=1.0)
        ax.imshow(hillshade_array, extent=[lon_coords[0], lon_coords[-1],
                                           lat_coords[-1], lat_coords[0]],
                  cmap='gray', alpha=0.3)

    # Plot the main terrain with color
    im = ax.imshow(terrain, extent=[lon_coords[0], lon_coords[-1],
                                    lat_coords[-1], lat_coords[0]],
                   cmap=colormap, interpolation='bilinear',
                   vmin=min_height, vmax=max_height)

    # Add contour lines if requested
    if contours:
        num_contours = 15
        contour_levels = np.linspace(min_height, max_height, num_contours)
        LON, LAT = np.meshgrid(lon_coords, lat_coords)
        contour = ax.contour(LON, LAT, terrain, levels=contour_levels,
                             colors='black', alpha=0.4, linewidths=0.5)
        ax.clabel(contour, inline=True, fontsize=7, fmt='%dm')

    # Customize the plot
    ax.set_xlabel('Longitude (degrees)', fontsize=12)
    ax.set_ylabel('Latitude (degrees)', fontsize=12)
    ax.set_title(f'SRTM Terrain Data: {os.path.basename(hgt_file)}',
                 fontsize=14, fontweight='bold', pad=20)

    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, orientation='vertical', pad=0.02, shrink=0.8)
    cbar.set_label('Elevation (meters)', fontsize=11)

    # Add grid
    ax.grid(True, alpha=0.2, linestyle='--', linewidth=0.5)

    # Add statistics text box
    stats_text = f'Min: {min_height:.0f}m\nMax: {max_height:.0f}m\nMean: {mean_height:.0f}m'
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # Set aspect ratio
    ax.set_aspect('equal', adjustable='box')

    plt.tight_layout()

    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print("Saved visualization to {0}".format(output_file))
    else:
        print("Displaying elevation map...")
        plt.show()

    plt.close()


def visualize_3d(hgt_file, output_file=None, subsample=10):
    """Create a 3D visualization"""

    # Read the .hgt file
    hgt = HGTFile(hgt_file)
    if not hgt.read():
        print("Error: Could not read .hgt file")
        return

    terrain = hgt.terrain
    lat_coords, lon_coords = hgt.get_coordinates()

    # Subsample for performance (full resolution is too slow for 3D)
    terrain_sub = terrain[::subsample, ::subsample]
    lat_sub = lat_coords[::subsample]
    lon_sub = lon_coords[::subsample]

    print("3D visualization using {0} grid (subsampled by {1})".format(terrain_sub.shape, subsample))

    # Get statistics
    valid_mask = ~np.isnan(terrain_sub)
    if not np.any(valid_mask):
        print("Error: No valid terrain data")
        return

    valid_terrain = terrain_sub[valid_mask]
    min_height = np.min(valid_terrain)
    max_height = np.max(valid_terrain)
    mean_height = np.mean(valid_terrain)

    # Replace NaN with mean
    Z = terrain_sub.copy()
    Z[np.isnan(Z)] = mean_height

    # Create 3D plot
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Create meshgrid
    LON, LAT = np.meshgrid(lon_sub, lat_sub)

    # Plot surface
    surf = ax.plot_surface(LON, LAT, Z, cmap=cm.terrain,
                           linewidth=0, antialiased=True, alpha=0.9,
                           vmin=min_height, vmax=max_height)

    # Customize
    ax.set_xlabel('Longitude (degrees)', fontsize=10)
    ax.set_ylabel('Latitude (degrees)', fontsize=10)
    ax.set_zlabel('Height (meters)', fontsize=10)
    ax.set_title(f'SRTM Terrain Data (3D): {os.path.basename(hgt_file)}',
                 fontsize=12, fontweight='bold')

    # Add colorbar
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5, label='Elevation (m)')

    # Set viewing angle
    ax.view_init(elev=30, azim=45)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print("Saved 3D visualization to {0}".format(output_file))
    else:
        print("Displaying 3D terrain...")
        plt.show()

    plt.close()


def main():
    parser = ArgumentParser(description='Visualize SRTM .hgt terrain files')
    parser.add_argument('hgt_file', help='Path to .hgt terrain file')
    parser.add_argument('--output', '-o', default=None,
                        help='Save visualization to file')
    parser.add_argument('--colormap', '-c', default='terrain',
                        help='Matplotlib colormap (default: terrain)')
    parser.add_argument('--hillshade', action='store_true',
                        help='Add hillshade effect')
    parser.add_argument('--contours', action='store_true',
                        help='Add elevation contour lines')
    parser.add_argument('--3d', dest='threed', action='store_true',
                        help='Create 3D visualization instead of 2D')
    parser.add_argument('--subsample', type=int, default=10,
                        help='Subsampling factor for 3D (default: 10)')

    args = parser.parse_args()

    if not os.path.exists(args.hgt_file):
        print("Error: File not found: {0}".format(args.hgt_file))
        sys.exit(1)

    try:
        if args.threed:
            visualize_3d(args.hgt_file, args.output, args.subsample)
        else:
            visualize_2d(args.hgt_file, args.output, args.colormap,
                         args.hillshade, args.contours)
    except Exception as e:
        print("Error: {0}".format(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
