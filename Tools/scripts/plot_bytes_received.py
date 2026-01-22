#!/usr/bin/env python3
"""
Plot bytes received over time from a MAVLink log file.

Supports both .bin (DataFlash) and .tlog (telemetry) log formats.

Usage:
    ./plot_bytes_received.py <logfile> [options]

Examples:
    ./plot_bytes_received.py flight.bin
    ./plot_bytes_received.py flight.tlog --window 1.0
    ./plot_bytes_received.py flight.bin --cumulative
    ./plot_bytes_received.py flight.tlog --source-system 1
    ./plot_bytes_received.py flight.tlog --source-system 1 --source-component 1
"""

import argparse
import sys
import os
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink not installed. Install with: pip install pymavlink")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    import numpy as np
except ImportError:
    print("Error: matplotlib/numpy not installed. Install with: pip install matplotlib numpy")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot bytes received over time from a MAVLink log file"
    )
    parser.add_argument("logfile", help="Path to log file (.bin or .tlog)")
    parser.add_argument(
        "--window",
        type=float,
        default=1.0,
        help="Time window in seconds for rate calculation (default: 1.0)",
    )
    parser.add_argument(
        "--cumulative",
        action="store_true",
        help="Plot cumulative bytes instead of rate",
    )
    parser.add_argument(
        "--by-msgtype",
        action="store_true",
        help="Break down by message type (top N types)",
    )
    parser.add_argument(
        "--top-n",
        type=int,
        default=10,
        help="Number of top message types to show (default: 10)",
    )
    parser.add_argument(
        "--output",
        "-o",
        help="Save plot to file instead of displaying",
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=None,
        help="Filter to only messages from this source system ID",
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=None,
        help="Filter to only messages from this source component ID",
    )
    return parser.parse_args()


def get_message_size(msg):
    """Get the size of a MAVLink message in bytes."""
    try:
        # Try to get the packed message length
        if hasattr(msg, "get_msgbuf"):
            return len(msg.get_msgbuf())
        elif hasattr(msg, "_msgbuf"):
            return len(msg._msgbuf)
        else:
            # Estimate based on message type
            # MAVLink v1: 8 byte header + payload + 2 byte checksum
            # MAVLink v2: 10 byte header + payload + 2 byte checksum (+ optional signature)
            if hasattr(msg, "_header"):
                return msg._header.mlen + 12  # Approximate
            return 50  # Default estimate
    except Exception:
        return 50  # Default estimate


def get_source_system(msg):
    """Get the source system ID from a message."""
    if hasattr(msg, "_header") and hasattr(msg._header, "srcSystem"):
        return msg._header.srcSystem
    if hasattr(msg, "get_srcSystem"):
        return msg.get_srcSystem()
    return None


def get_source_component(msg):
    """Get the source component ID from a message."""
    if hasattr(msg, "_header") and hasattr(msg._header, "srcComponent"):
        return msg._header.srcComponent
    if hasattr(msg, "get_srcComponent"):
        return msg.get_srcComponent()
    return None


def process_log(logfile, window_size, by_msgtype=False, source_system=None, source_component=None):
    """
    Process a log file and extract timing and byte information.

    Args:
        logfile: path to log file
        window_size: time window in seconds for binning
        by_msgtype: if True, return bytes broken down by message type
        source_system: if specified, only include messages from this source system
        source_component: if specified, only include messages from this source component

    Returns:
        times: list of relative timestamps (seconds from start)
        bytes_data: list of byte counts per window, or dict of {msgtype: [bytes]} if by_msgtype
        total_bytes: total bytes in log
        total_messages: total message count
        start_timestamp: Unix timestamp of first message (for time-of-day display)
    """
    print(f"Opening {logfile}...")
    mlog = mavutil.mavlink_connection(logfile, robust_parsing=True)

    # Print filter info
    if source_system is not None or source_component is not None:
        filter_parts = []
        if source_system is not None:
            filter_parts.append(f"source_system={source_system}")
        if source_component is not None:
            filter_parts.append(f"source_component={source_component}")
        print(f"Filtering: {', '.join(filter_parts)}")

    # Data collection
    message_data = []  # [(timestamp, msgtype, size), ...]
    total_bytes = 0
    total_messages = 0
    filtered_messages = 0

    print("Processing messages...")
    msg_count = 0
    while True:
        try:
            msg = mlog.recv_match()
            if msg is None:
                break

            msg_count += 1
            if msg_count % 50000 == 0:
                print(f"  Processed {msg_count} messages...")

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

            # Apply source system filter
            if source_system is not None:
                msg_src_sys = get_source_system(msg)
                if msg_src_sys is None or msg_src_sys != source_system:
                    filtered_messages += 1
                    continue

            # Apply source component filter
            if source_component is not None:
                msg_src_comp = get_source_component(msg)
                if msg_src_comp is None or msg_src_comp != source_component:
                    filtered_messages += 1
                    continue

            # Get timestamp
            timestamp = getattr(msg, "_timestamp", 0)
            if timestamp == 0:
                # Try to get timestamp from message fields
                if hasattr(msg, "time_boot_ms"):
                    timestamp = msg.time_boot_ms / 1000.0
                elif hasattr(msg, "time_usec"):
                    timestamp = msg.time_usec / 1e6
                elif hasattr(msg, "TimeUS"):
                    timestamp = msg.TimeUS / 1e6
                else:
                    continue  # Skip messages without timestamps

            msg_size = get_message_size(msg)
            total_bytes += msg_size
            total_messages += 1

            message_data.append((timestamp, msg_type, msg_size))

        except Exception as e:
            print(f"Warning: Error processing message: {e}")
            continue

    if not message_data:
        print("Error: No valid messages found in log")
        if filtered_messages > 0:
            print(f"  ({filtered_messages} messages were filtered out by source system/component)")
        sys.exit(1)

    print(f"Processed {total_messages} messages, {total_bytes} bytes total")
    if filtered_messages > 0:
        print(f"  ({filtered_messages} messages filtered out by source system/component)")

    # Sort by timestamp
    message_data.sort(key=lambda x: x[0])

    # Store original start timestamp for time-of-day display
    start_timestamp = message_data[0][0]

    # Normalize timestamps to start at 0
    message_data = [(t - start_timestamp, mt, sz) for t, mt, sz in message_data]
    end_time = message_data[-1][0]

    # Print time info
    print(f"Log duration: {end_time:.1f} seconds")
    if start_timestamp > 1e9:  # Looks like Unix timestamp
        start_dt = datetime.fromtimestamp(start_timestamp)
        print(f"Start time: {start_dt.strftime('%Y-%m-%d %H:%M:%S')}")

    # Bin the data into windows
    num_windows = max(1, int(end_time / window_size) + 1)
    times = [i * window_size for i in range(num_windows)]

    if by_msgtype:
        # Track bytes per message type per window
        msgtype_bytes = {}  # {msgtype: [bytes_per_window]}
        for timestamp, msg_type, msg_size in message_data:
            window_idx = min(int(timestamp / window_size), num_windows - 1)
            if msg_type not in msgtype_bytes:
                msgtype_bytes[msg_type] = [0] * num_windows
            msgtype_bytes[msg_type][window_idx] += msg_size
        return times, msgtype_bytes, total_bytes, total_messages, start_timestamp
    else:
        # Track total bytes per window
        bytes_per_window = [0] * num_windows
        for timestamp, msg_type, msg_size in message_data:
            window_idx = min(int(timestamp / window_size), num_windows - 1)
            bytes_per_window[window_idx] += msg_size
        return times, bytes_per_window, total_bytes, total_messages, start_timestamp


def make_filter_subtitle(source_system, source_component):
    """Create a subtitle string describing active filters."""
    if source_system is None and source_component is None:
        return ""
    parts = []
    if source_system is not None:
        parts.append(f"sysid={source_system}")
    if source_component is not None:
        parts.append(f"compid={source_component}")
    return f" [filter: {', '.join(parts)}]"


def setup_time_axis(ax, times, start_timestamp):
    """
    Configure x-axis to show time of day instead of relative seconds.

    Args:
        ax: matplotlib axis
        times: list of relative timestamps (seconds from start)
        start_timestamp: Unix timestamp of start time
    """
    # Check if start_timestamp looks like a valid Unix timestamp
    if start_timestamp < 1e9:
        # Not a Unix timestamp, use relative time
        ax.set_xlabel("Time (seconds)")
        return

    # Convert relative times to datetime objects
    datetimes = [datetime.fromtimestamp(start_timestamp + t) for t in times]

    # Determine appropriate format based on duration
    duration = times[-1] - times[0] if len(times) > 1 else 0

    if duration < 60:
        # Less than 1 minute: show seconds
        date_format = '%H:%M:%S'
        locator = mdates.SecondLocator(interval=10)
    elif duration < 3600:
        # Less than 1 hour: show minutes:seconds
        date_format = '%H:%M:%S'
        locator = mdates.MinuteLocator(interval=1)
    elif duration < 86400:
        # Less than 1 day: show hours:minutes
        date_format = '%H:%M'
        locator = mdates.MinuteLocator(interval=10)
    else:
        # More than 1 day: show date and time
        date_format = '%m-%d %H:%M'
        locator = mdates.HourLocator(interval=1)

    ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
    ax.xaxis.set_major_locator(locator)

    # Rotate labels for better readability
    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

    # Add date to xlabel if log spans multiple days
    start_dt = datetime.fromtimestamp(start_timestamp)
    ax.set_xlabel(f"Time of Day ({start_dt.strftime('%Y-%m-%d')})")

    return datetimes


def generate_distinct_colors(n):
    """
    Generate n visually distinct colors for plotting.

    Uses a combination of qualitative colormaps and manual selection
    to maximize visual distinction between adjacent colors.
    """
    if n <= 0:
        return []

    # Start with hand-picked highly distinct colors
    base_colors = [
        '#1f77b4',  # blue
        '#ff7f0e',  # orange
        '#2ca02c',  # green
        '#d62728',  # red
        '#9467bd',  # purple
        '#8c564b',  # brown
        '#e377c2',  # pink
        '#17becf',  # cyan
        '#bcbd22',  # olive
        '#7f7f7f',  # gray
    ]

    if n <= len(base_colors):
        return base_colors[:n]

    # For more colors, combine multiple colormaps
    colors = list(base_colors)

    # Add colors from tab20, skipping similar ones
    tab20 = plt.cm.tab20(np.linspace(0, 1, 20))
    tab20b = plt.cm.tab20b(np.linspace(0, 1, 20))
    tab20c = plt.cm.tab20c(np.linspace(0, 1, 20))

    # Interleave from different colormaps for maximum distinction
    extra_colors = []
    for i in range(20):
        if i % 3 == 0:
            extra_colors.append(tab20[i])
        elif i % 3 == 1:
            extra_colors.append(tab20b[i])
        else:
            extra_colors.append(tab20c[i])

    # Add extra colors until we have enough
    for c in extra_colors:
        if len(colors) >= n:
            break
        colors.append(c)

    # If still not enough, use hsv colormap for remaining
    if len(colors) < n:
        remaining = n - len(colors)
        hsv_colors = plt.cm.hsv(np.linspace(0, 0.9, remaining + 1)[:-1])
        colors.extend(hsv_colors)

    return colors[:n]


def plot_cumulative(times, bytes_per_window, total_bytes, start_timestamp,
                    output=None, source_system=None, source_component=None):
    """Plot cumulative bytes over time."""
    cumulative = np.cumsum(bytes_per_window)

    fig, ax = plt.subplots(figsize=(12, 6))

    # Convert to datetime for x-axis if we have valid timestamps
    if start_timestamp > 1e9:
        x_values = [datetime.fromtimestamp(start_timestamp + t) for t in times]
        setup_time_axis(ax, times, start_timestamp)
    else:
        x_values = times
        ax.set_xlabel("Time (seconds)")

    ax.plot(x_values, cumulative, "b-", linewidth=1.5)
    ax.fill_between(x_values, cumulative, alpha=0.3)

    ax.set_ylabel("Cumulative Bytes")
    filter_str = make_filter_subtitle(source_system, source_component)
    ax.set_title(f"Cumulative Bytes Received Over Time{filter_str}")
    ax.grid(True, alpha=0.3)

    # Add total bytes annotation
    ax.annotate(
        f"Total: {total_bytes:,} bytes",
        xy=(0.98, 0.95),
        xycoords="axes fraction",
        ha="right",
        va="top",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150)
        print(f"Saved plot to {output}")
    else:
        plt.show()


def plot_rate(times, bytes_per_window, window_size, total_bytes, total_messages,
              start_timestamp, output=None, source_system=None, source_component=None):
    """Plot byte rate over time."""
    # Convert to rate (bytes per second)
    rate = [b / window_size for b in bytes_per_window]

    fig, ax = plt.subplots(figsize=(12, 6))

    # Convert to datetime for x-axis if we have valid timestamps
    if start_timestamp > 1e9:
        x_values = [datetime.fromtimestamp(start_timestamp + t) for t in times]
        setup_time_axis(ax, times, start_timestamp)
    else:
        x_values = times
        ax.set_xlabel("Time (seconds)")

    ax.plot(x_values, rate, "b-", linewidth=1)
    ax.fill_between(x_values, rate, alpha=0.3)

    ax.set_ylabel("Bytes per second")
    filter_str = make_filter_subtitle(source_system, source_component)
    ax.set_title(f"Data Rate Over Time (window={window_size}s){filter_str}")
    ax.grid(True, alpha=0.3)

    # Calculate statistics
    rate_array = np.array(rate)
    avg_rate = np.mean(rate_array)
    max_rate = np.max(rate_array)

    # Add statistics annotation
    stats_text = (
        f"Avg: {avg_rate:,.0f} B/s ({avg_rate*8/1000:.1f} kbps)\n"
        f"Max: {max_rate:,.0f} B/s ({max_rate*8/1000:.1f} kbps)\n"
        f"Total: {total_bytes:,} bytes\n"
        f"Messages: {total_messages:,}"
    )
    ax.annotate(
        stats_text,
        xy=(0.98, 0.95),
        xycoords="axes fraction",
        ha="right",
        va="top",
        fontsize=9,
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150)
        print(f"Saved plot to {output}")
    else:
        plt.show()


def plot_by_msgtype(times, msgtype_bytes, window_size, top_n, start_timestamp,
                    output=None, source_system=None, source_component=None):
    """Plot byte rate over time broken down by message type."""
    # Calculate total bytes per message type
    totals = {mt: sum(bytes_list) for mt, bytes_list in msgtype_bytes.items()}

    # Get top N message types by total bytes
    sorted_types = sorted(totals.items(), key=lambda x: x[1], reverse=True)
    top_types = [mt for mt, _ in sorted_types[:top_n]]

    # Calculate "other" category
    other_bytes = [0] * len(times)
    for mt, bytes_list in msgtype_bytes.items():
        if mt not in top_types:
            for i, b in enumerate(bytes_list):
                other_bytes[i] += b

    fig, ax = plt.subplots(figsize=(14, 7))

    # Convert to datetime for x-axis if we have valid timestamps
    if start_timestamp > 1e9:
        x_values = [datetime.fromtimestamp(start_timestamp + t) for t in times]
        setup_time_axis(ax, times, start_timestamp)
    else:
        x_values = times
        ax.set_xlabel("Time (seconds)")

    # Convert to rates and stack
    bottom = np.zeros(len(times))
    colors = generate_distinct_colors(top_n)

    for i, msg_type in enumerate(top_types):
        rate = np.array(msgtype_bytes[msg_type]) / window_size
        ax.fill_between(x_values, bottom, bottom + rate, label=msg_type,
                        color=colors[i], alpha=0.8)
        bottom += rate

    # Add "other" category
    if sum(other_bytes) > 0:
        other_rate = np.array(other_bytes) / window_size
        ax.fill_between(x_values, bottom, bottom + other_rate,
                        label="Other", color="gray", alpha=0.5)

    ax.set_ylabel("Bytes per second")
    filter_str = make_filter_subtitle(source_system, source_component)
    ax.set_title(f"Data Rate by Message Type (window={window_size}s){filter_str}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=8)

    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {output}")
    else:
        plt.show()


def main():
    args = parse_args()

    if not os.path.exists(args.logfile):
        print(f"Error: File not found: {args.logfile}")
        sys.exit(1)

    times, bytes_data, total_bytes, total_messages, start_timestamp = process_log(
        args.logfile,
        args.window,
        args.by_msgtype,
        source_system=args.source_system,
        source_component=args.source_component,
    )

    if args.by_msgtype:
        plot_by_msgtype(times, bytes_data, args.window, args.top_n, start_timestamp,
                        args.output, args.source_system, args.source_component)
    elif args.cumulative:
        plot_cumulative(times, bytes_data, total_bytes, start_timestamp,
                        args.output, args.source_system, args.source_component)
    else:
        plot_rate(times, bytes_data, args.window, total_bytes, total_messages,
                  start_timestamp, args.output, args.source_system, args.source_component)


if __name__ == "__main__":
    main()
