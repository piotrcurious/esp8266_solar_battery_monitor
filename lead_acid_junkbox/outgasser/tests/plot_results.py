import sys
import os

def generate_svg(csv_file, output_svg, title):
    times = []
    v_bats = []
    i_bats = []
    modes = []

    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found")
        return

    with open(csv_file, 'r') as f:
        for line in f:
            if line.startswith("TELE:"):
                try:
                    parts = line.strip().split(" ")[1].split(",")
                    if len(parts) >= 4:
                        times.append(float(parts[0]) / 1000.0) # ms to s
                        modes.append(int(parts[1]))
                        v_bats.append(float(parts[2]))
                        i_bats.append(float(parts[3]) / 1000.0) # mA to A
                except:
                    continue

    if not times:
        print("No telemetry data found.")
        return

    # Downsample for clarity and SVG size
    step = max(1, len(times) // 1000)
    times = times[::step]
    v_bats = v_bats[::step]
    i_bats = i_bats[::step]

    width = 800
    height = 500
    padding = 60

    t_min, t_max = min(times), max(times)
    v_min, v_max = min(v_bats), max(v_bats)
    i_min, i_max = min(i_bats), max(i_bats)

    # Grid lines logic
    def get_grid(mi, ma, n=5):
        if ma == mi: return [mi]
        step = (ma - mi) / n
        return [mi + step * i for i in range(n + 1)]

    def scale(val, src_min, src_max, dst_min, dst_max):
        if src_max == src_min: return dst_min
        return dst_min + (val - src_min) * (dst_max - dst_min) / (src_max - src_min)

    with open(output_svg, 'w') as f:
        f.write(f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">\n')
        f.write(f'<rect width="100%" height="100%" fill="#ffffff" stroke="#eeeeee"/>\n')
        f.write(f'<text x="{width/2}" y="30" text-anchor="middle" font-family="sans-serif" font-size="20" font-weight="bold">{title}</text>\n')

        # Draw background grid
        v_grid = get_grid(v_min, v_max)
        for g in v_grid:
            y = scale(g, v_min, v_max, height-padding, padding)
            f.write(f'<line x1="{padding}" y1="{y}" x2="{width-padding}" y2="{y}" stroke="#dddddd" stroke-width="0.5"/>\n')
            f.write(f'<text x="{padding-5}" y="{y+5}" text-anchor="end" font-family="sans-serif" font-size="10" fill="red">{g:.2f}V</text>\n')

        i_grid = get_grid(i_min, i_max)
        for g in i_grid:
            y = scale(g, i_min, i_max, height-padding, padding)
            f.write(f'<text x="{width-padding+5}" y="{y+5}" text-anchor="start" font-family="sans-serif" font-size="10" fill="blue">{g:.2f}A</text>\n')

        # Axis
        f.write(f'<line x1="{padding}" y1="{height-padding}" x2="{width-padding}" y2="{height-padding}" stroke="black" stroke-width="1.5"/>\n')
        f.write(f'<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{height-padding}" stroke="black" stroke-width="1.5"/>\n')
        f.write(f'<line x1="{width-padding}" y1="{padding}" x2="{width-padding}" y2="{height-padding}" stroke="black" stroke-width="1.5"/>\n')

        # Vbatt path (Red)
        v_points = []
        for t, v in zip(times, v_bats):
            x = scale(t, t_min, t_max, padding, width-padding)
            y = scale(v, v_min, v_max, height-padding, padding)
            v_points.append(f"{x:.1f},{y:.1f}")
        f.write(f'<polyline points="{" ".join(v_points)}" fill="none" stroke="red" stroke-width="2"/>\n')

        # Ibatt path (Blue)
        i_points = []
        for t, i in zip(times, i_bats):
            x = scale(t, t_min, t_max, padding, width-padding)
            y = scale(i, i_min, i_max, height-padding, padding)
            i_points.append(f"{x:.1f},{y:.1f}")
        f.write(f'<polyline points="{" ".join(i_points)}" fill="none" stroke="blue" stroke-width="1.5" stroke-dasharray="4,2"/>\n')

        # Legend
        f.write(f'<rect x="{width-150}" y="50" width="100" height="40" fill="white" stroke="#ccc" fill-opacity="0.8"/>\n')
        f.write(f'<line x1="{width-140}" y1="65" x2="{width-110}" y2="65" stroke="red" stroke-width="2"/>\n')
        f.write(f'<text x="{width-105}" y="70" font-family="sans-serif" font-size="12">V_bat</text>\n')
        f.write(f'<line x1="{width-140}" y1="80" x2="{width-110}" y2="80" stroke="blue" stroke-width="1.5" stroke-dasharray="4,2"/>\n')
        f.write(f'<text x="{width-105}" y="85" font-family="sans-serif" font-size="12">I_bat</text>\n')

        # Time labels
        f.write(f'<text x="{width/2}" y="{height-20}" text-anchor="middle" font-family="sans-serif" font-size="14">Time (seconds)</text>\n')

        f.write('</svg>')

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: plot_results.py input.log output.svg title")
    else:
        generate_svg(sys.argv[1], sys.argv[2], sys.argv[3])
