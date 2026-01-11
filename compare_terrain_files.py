#!/usr/bin/env python3
"""
Compare SRTM .hgt and ArduPilot .DAT terrain files

This tool compares the elevation data from both formats and shows differences.

Usage:
    python compare_terrain_files.py S35E149.hgt S35E149.DAT
"""

import struct
import sys
import os
from argparse import ArgumentParser

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("Error: This tool requires numpy and matplotlib")
    print("Install with: pip install numpy matplotlib")
    sys.exit(1)

# ArduPilot constants
TERRAIN_GRID_BLOCK_SIZE_X = 32
TERRAIN_GRID_BLOCK_SIZE_Y = 28
IO_BLOCK_SIZE = 2048
TERRAIN_GRID_FORMAT_VERSION = 1


class HGTFile:
    """Read SRTM .hgt file"""

    def __init__(self, filename):
        self.filename = filename
        self.terrain = None
        self.size = None
        self.lat_degrees = None
        self.lon_degrees = None
        self._parse_filename()

    def _parse_filename(self):
        basename = os.path.basename(self.filename)[:-4]
        lat_sign = -1 if basename[0] == 'S' else 1
        ew_pos = basename.find('E') if 'E' in basename else basename.find('W')
        self.lat_degrees = lat_sign * int(basename[1:ew_pos])
        lon_sign = -1 if basename[ew_pos] == 'W' else 1
        self.lon_degrees = lon_sign * int(basename[ew_pos+1:])

    def read(self):
        file_size = os.path.getsize(self.filename)
        if file_size == 2884802:
            self.size = 1201
        elif file_size == 25934402:
            self.size = 3601
        else:
            samples = file_size // 2
            self.size = int(np.sqrt(samples))

        with open(self.filename, 'rb') as f:
            data = f.read()

        elevations = struct.unpack(f'>{len(data)//2}h', data)
        self.terrain = np.array(elevations, dtype=float).reshape((self.size, self.size))
        self.terrain[self.terrain == -32768] = np.nan

        print("HGT: Loaded {0}x{0} grid".format(self.size))
        valid = self.terrain[~np.isnan(self.terrain)]
        print("HGT: Elevation {0:.0f}m to {1:.0f}m (mean: {2:.0f}m)".format(
            np.min(valid), np.max(valid), np.mean(valid)))
        return True

    def get_elevation_at(self, lat, lon):
        """Get elevation at a specific lat/lon"""
        # SRTM data arranged from NW corner
        # Row 0 = north edge (lat+1), Row N = south edge (lat)
        # Col 0 = west edge (lon), Col N = east edge (lon+1)

        # Calculate fractional position in grid
        lat_frac = (self.lat_degrees + 1 - lat)  # 0 at north edge, 1 at south edge
        lon_frac = (lon - self.lon_degrees)  # 0 at west edge, 1 at east edge

        if lat_frac < 0 or lat_frac > 1 or lon_frac < 0 or lon_frac > 1:
            return np.nan

        # Convert to array indices
        row = lat_frac * (self.size - 1)
        col = lon_frac * (self.size - 1)

        # Bilinear interpolation
        r0, c0 = int(np.floor(row)), int(np.floor(col))
        r1, c1 = min(r0 + 1, self.size - 1), min(c0 + 1, self.size - 1)

        dr, dc = row - r0, col - c0

        # Get corner values
        v00 = self.terrain[r0, c0]
        v01 = self.terrain[r0, c1]
        v10 = self.terrain[r1, c0]
        v11 = self.terrain[r1, c1]

        # Check for NaN
        if np.isnan([v00, v01, v10, v11]).any():
            return np.nan

        # Bilinear interpolation
        v0 = v00 * (1 - dc) + v01 * dc
        v1 = v10 * (1 - dc) + v11 * dc
        return v0 * (1 - dr) + v1 * dr


class DATFile:
    """Read ArduPilot .DAT file"""

    def __init__(self, filename):
        self.filename = filename
        self.blocks = []
        self.lat_degrees = None
        self.lon_degrees = None
        self._parse_filename()

    def _parse_filename(self):
        basename = os.path.basename(self.filename)[:-4]
        lat_sign = -1 if basename[0] == 'S' else 1
        ew_pos = basename.find('E') if 'E' in basename else basename.find('W')
        self.lat_degrees = lat_sign * int(basename[1:ew_pos])
        lon_sign = -1 if basename[ew_pos] == 'W' else 1
        self.lon_degrees = lon_sign * int(basename[ew_pos+1:])

    def read(self):
        with open(self.filename, 'rb') as f:
            while True:
                buf = f.read(IO_BLOCK_SIZE)
                if len(buf) != IO_BLOCK_SIZE:
                    break

                block = {}
                (bitmap, lat, lon, crc, version, spacing) = struct.unpack("<QiiHHH", buf[:22])

                if version != TERRAIN_GRID_FORMAT_VERSION or spacing == 0:
                    continue

                block['lat'] = lat
                block['lon'] = lon
                block['spacing'] = spacing

                # Read heights
                heights = np.zeros((TERRAIN_GRID_BLOCK_SIZE_X, TERRAIN_GRID_BLOCK_SIZE_Y), dtype=np.int16)
                offset = 22
                for x in range(TERRAIN_GRID_BLOCK_SIZE_X):
                    h = struct.unpack("<{0}h".format(TERRAIN_GRID_BLOCK_SIZE_Y),
                                      buf[offset:offset + TERRAIN_GRID_BLOCK_SIZE_Y * 2])
                    heights[x, :] = h
                    offset += TERRAIN_GRID_BLOCK_SIZE_Y * 2

                block['heights'] = heights
                self.blocks.append(block)

        print("DAT: Loaded {len(self.blocks)} blocks")

        # Get statistics
        all_heights = []
        for block in self.blocks:
            all_heights.extend(block['heights'].flatten())
        all_heights = np.array(all_heights)
        print("DAT: Elevation {np.min(all_heights):.0f}m to {np.max(all_heights):.0f}m (mean: {np.mean(all_heights):.0f}m)")

        return len(self.blocks) > 0

    def get_elevation_at(self, lat, lon):
        """Get elevation at a specific lat/lon"""
        lat_e7 = int(lat * 1e7)
        lon_e7 = int(lon * 1e7)

        # Find the block containing this point
        for block in self.blocks:
            # Block SW corner
            blat = block['lat']
            blon = block['lon']
            spacing = block['spacing']

            # Check if point is in this block
            dlat_m = (lat_e7 - blat) * 0.011131884502145034  # approx meters north
            dlon_m = (lon_e7 - blon) * 0.011131884502145034  # approx meters east (simplified)

            # Grid indices
            gx = int(dlat_m / spacing)
            gy = int(dlon_m / spacing)

            if 0 <= gx < TERRAIN_GRID_BLOCK_SIZE_X and 0 <= gy < TERRAIN_GRID_BLOCK_SIZE_Y:
                return float(block['heights'][gx, gy])

        return np.nan


def compare_files(hgt_file, dat_file, output_file=None):
    """Compare the two terrain files"""

    print("\n{0}".format('='*60))
    print("Comparing terrain files:")
    print("  HGT: {0}".format(hgt_file))
    print("  DAT: {0}".format(dat_file))
    print("{'='*60}\n")

    # Read both files
    hgt = HGTFile(hgt_file)
    dat = DATFile(dat_file)

    if not hgt.read():
        print("Error: Could not read HGT file")
        return

    if not dat.read():
        print("Error: Could not read DAT file")
        return

    # Sample points across the degree square
    print("\n" + "="*60)
    print("Sampling elevations at regular intervals...")
    print("="*60)

    samples = 50  # Sample at 50x50 grid
    lat_samples = np.linspace(hgt.lat_degrees, hgt.lat_degrees + 1, samples)
    lon_samples = np.linspace(hgt.lon_degrees, hgt.lon_degrees + 1, samples)

    differences = []
    hgt_values = []
    dat_values = []
    comparison_grid_hgt = np.full((samples, samples), np.nan)
    comparison_grid_dat = np.full((samples, samples), np.nan)
    comparison_grid_diff = np.full((samples, samples), np.nan)

    for i, lat in enumerate(lat_samples):
        for j, lon in enumerate(lon_samples):
            hgt_elev = hgt.get_elevation_at(lat, lon)
            dat_elev = dat.get_elevation_at(lat, lon)

            comparison_grid_hgt[i, j] = hgt_elev
            comparison_grid_dat[i, j] = dat_elev

            if not np.isnan(hgt_elev) and not np.isnan(dat_elev):
                diff = dat_elev - hgt_elev
                differences.append(diff)
                hgt_values.append(hgt_elev)
                dat_values.append(dat_elev)
                comparison_grid_diff[i, j] = diff

    if len(differences) == 0:
        print("Error: No matching data points found!")
        return

    differences = np.array(differences)
    hgt_values = np.array(hgt_values)
    dat_values = np.array(dat_values)

    # Statistics
    print("\nComparison Statistics ({len(differences)} matching points):")
    print("{0}".format('='*60))
    print("  Mean difference (DAT - HGT):  {np.mean(differences):+.2f} m")
    print("  Median difference:            {np.median(differences):+.2f} m")
    print("  Std deviation:                {np.std(differences):.2f} m")
    print("  Min difference:               {np.min(differences):+.2f} m")
    print("  Max difference:               {np.max(differences):+.2f} m")
    print("  RMS difference:               {np.sqrt(np.mean(differences**2)):.2f} m")

    # Correlation
    correlation = np.corrcoef(hgt_values, dat_values)[0, 1]
    print("  Correlation coefficient:      {0:.4f}".format(correlation))

    # Agreement within thresholds
    within_1m = np.sum(np.abs(differences) <= 1) / len(differences) * 100
    within_5m = np.sum(np.abs(differences) <= 5) / len(differences) * 100
    within_10m = np.sum(np.abs(differences) <= 10) / len(differences) * 100

    print("\nAgreement:")
    print("  Within ±1m:   {within_1m:.1f}%")
    print("  Within ±5m:   {within_5m:.1f}%")
    print("  Within ±10m:  {within_10m:.1f}%")

    # Create comparison visualization
    fig = plt.figure(figsize=(18, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

    # HGT data
    ax1 = fig.add_subplot(gs[0, 0])
    im1 = ax1.imshow(comparison_grid_hgt, cmap='terrain', origin='upper',
                     extent=[hgt.lon_degrees, hgt.lon_degrees+1,
                             hgt.lat_degrees, hgt.lat_degrees+1])
    ax1.set_title('HGT (SRTM) Elevations', fontweight='bold')
    ax1.set_xlabel('Longitude')
    ax1.set_ylabel('Latitude')
    plt.colorbar(im1, ax=ax1, label='Elevation (m)')

    # DAT data
    ax2 = fig.add_subplot(gs[0, 1])
    im2 = ax2.imshow(comparison_grid_dat, cmap='terrain', origin='upper',
                     extent=[hgt.lon_degrees, hgt.lon_degrees+1,
                             hgt.lat_degrees, hgt.lat_degrees+1])
    ax2.set_title('DAT (ArduPilot) Elevations', fontweight='bold')
    ax2.set_xlabel('Longitude')
    ax2.set_ylabel('Latitude')
    plt.colorbar(im2, ax=ax2, label='Elevation (m)')

    # Difference map
    ax3 = fig.add_subplot(gs[0, 2])
    vmax = max(abs(np.nanmin(comparison_grid_diff)), abs(np.nanmax(comparison_grid_diff)))
    im3 = ax3.imshow(comparison_grid_diff, cmap='RdBu_r', origin='upper',
                     extent=[hgt.lon_degrees, hgt.lon_degrees+1,
                             hgt.lat_degrees, hgt.lat_degrees+1],
                     vmin=-vmax, vmax=vmax)
    ax3.set_title('Difference (DAT - HGT)', fontweight='bold')
    ax3.set_xlabel('Longitude')
    ax3.set_ylabel('Latitude')
    plt.colorbar(im3, ax=ax3, label='Difference (m)')

    # Scatter plot
    ax4 = fig.add_subplot(gs[1, :2])
    ax4.scatter(hgt_values, dat_values, alpha=0.3, s=10)
    min_val = min(hgt_values.min(), dat_values.min())
    max_val = max(hgt_values.max(), dat_values.max())
    ax4.plot([min_val, max_val], [min_val, max_val], 'r--', label='Perfect agreement')
    ax4.set_xlabel('HGT Elevation (m)')
    ax4.set_ylabel('DAT Elevation (m)')
    ax4.set_title('Elevation Comparison Scatter Plot', fontweight='bold')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_aspect('equal', adjustable='box')

    # Histogram of differences
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.hist(differences, bins=50, edgecolor='black', alpha=0.7)
    ax5.axvline(0, color='red', linestyle='--', linewidth=2, label='Zero difference')
    ax5.axvline(np.mean(differences), color='green', linestyle='-', linewidth=2,
                label=f'Mean: {np.mean(differences):.2f}m')
    ax5.set_xlabel('Difference (DAT - HGT) [m]')
    ax5.set_ylabel('Count')
    ax5.set_title('Difference Distribution', fontweight='bold')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # Statistics text
    ax6 = fig.add_subplot(gs[2, :])
    ax6.axis('off')
    stats_text = f"""
COMPARISON SUMMARY

Files:
  HGT: {os.path.basename(hgt_file)} ({hgt.size}×{hgt.size} samples)
  DAT: {os.path.basename(dat_file)} ({len(dat.blocks)} blocks)

Statistics ({len(differences)} matching points):
  Mean difference (DAT - HGT): {np.mean(differences):+.2f} m
  Median difference:           {np.median(differences):+.2f} m
  Standard deviation:          {np.std(differences):.2f} m
  RMS difference:              {np.sqrt(np.mean(differences**2)):.2f} m
  Correlation coefficient:     {correlation:.4f}

Agreement:
  Within ±1m:  {within_1m:.1f}%    Within ±5m:  {within_5m:.1f}%    Within ±10m: {within_10m:.1f}%
"""
    ax6.text(0.1, 0.5, stats_text, fontfamily='monospace', fontsize=11,
             verticalalignment='center')

    plt.suptitle('Terrain File Comparison: {0} vs {1}'.format(
                 os.path.basename(hgt_file), os.path.basename(dat_file)),
                 fontsize=14, fontweight='bold', y=0.98)

    if output_file:
        plt.savefig(output_file, dpi=200, bbox_inches='tight')
        print("\nSaved comparison to {0}".format(output_file))
    else:
        plt.show()

    plt.close()


def main():
    parser = ArgumentParser(description='Compare SRTM .hgt and ArduPilot .DAT terrain files')
    parser.add_argument('hgt_file', help='Path to .hgt file')
    parser.add_argument('dat_file', help='Path to .DAT file')
    parser.add_argument('--output', '-o', default=None,
                        help='Save comparison plot to file')

    args = parser.parse_args()

    if not os.path.exists(args.hgt_file):
        print("Error: HGT file not found: {0}".format(args.hgt_file))
        sys.exit(1)

    if not os.path.exists(args.dat_file):
        print("Error: DAT file not found: {0}".format(args.dat_file))
        sys.exit(1)

    try:
        compare_files(args.hgt_file, args.dat_file, args.output)
    except Exception as e:
        print("Error: {0}".format(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
