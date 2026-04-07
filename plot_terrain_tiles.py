#!/usr/bin/env python3

'''
Extract latitude/longitude from ArduPilot terrain tile filenames
in an image and plot them on a world map.

Usage:
    python3 plot_terrain_tiles.py image.png output.png
'''

import re
import sys

import matplotlib

matplotlib.use('Agg')
import cartopy.crs as ccrs  # noqa: E402
import cartopy.feature as cfeature  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402


def extract_coordinates_from_filenames(filenames):
    '''parse terrain tile filenames like N31W098.DAT.gz into (lat, lon) tuples'''
    coords = []
    for name in filenames:
        m = re.match(r'([NS])(\d+)([EW])(\d+)', name)
        if m is None:
            continue
        lat = int(m.group(2))
        if m.group(1) == 'S':
            lat = -lat
        lon = int(m.group(4))
        if m.group(3) == 'W':
            lon = -lon
        coords.append((lat, lon))
    return coords


def extract_filenames_from_image(image_path):
    '''use OCR to extract terrain tile filenames from an image'''
    try:
        import pytesseract

        from PIL import Image
    except ImportError:
        print("pytesseract and Pillow are required for OCR: pip install pytesseract Pillow")
        sys.exit(1)

    img = Image.open(image_path)
    text = pytesseract.image_to_string(img)
    return re.findall(r'[NS]\d{2,3}[EW]\d{2,3}', text)


def plot_coordinates(coords, output_path):
    '''plot (lat, lon) coordinates on a world map'''
    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]

    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(1, 1, 1, projection=ccrs.PlateCarree())
    ax.set_global()
    ax.add_feature(cfeature.LAND, facecolor='lightgray')
    ax.add_feature(cfeature.OCEAN, facecolor='lightblue')
    ax.add_feature(cfeature.COASTLINE, linewidth=0.5)
    ax.add_feature(cfeature.BORDERS, linewidth=0.3, linestyle='--')
    ax.gridlines(draw_labels=True, linewidth=0.3, alpha=0.5)

    ax.scatter(
        lons, lats,
        c='red', s=80, zorder=5,
        transform=ccrs.PlateCarree(),
        edgecolors='black', linewidths=0.5,
    )

    for lat, lon in coords:
        ns = 'N' if lat >= 0 else 'S'
        ew = 'E' if lon >= 0 else 'W'
        label = "%s%02d%s%03d" % (ns, abs(lat), ew, abs(lon))
        ax.text(
            lon + 3, lat + 2, label,
            fontsize=7, transform=ccrs.PlateCarree(), zorder=6,
        )

    ax.set_title('Terrain Data Tile Locations', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print("Saved %s" % output_path)


def main():
    if len(sys.argv) != 3:
        print("Usage: %s <input_image> <output_png>" % sys.argv[0])
        sys.exit(1)

    image_path = sys.argv[1]
    output_path = sys.argv[2]

    filenames = extract_filenames_from_image(image_path)
    if not filenames:
        print("No terrain tile filenames found in %s" % image_path)
        sys.exit(1)

    print("Found tiles: %s" % ", ".join(filenames))
    coords = extract_coordinates_from_filenames(filenames)
    plot_coordinates(coords, output_path)


if __name__ == '__main__':
    main()
