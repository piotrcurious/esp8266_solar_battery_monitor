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
                parts = line.strip().split(" ")[1].split(",")
                if len(parts) >= 4:
                    times.append(float(parts[0]) / 1000.0) # ms to s
                    modes.append(int(parts[1]))
                    v_bats.append(float(parts[2]))
                    i_bats.append(float(parts[3]))

    if not times:
        print("No telemetry data found.")
        return

    width = 800
    height = 400
    padding = 60

    t_min, t_max = min(times), max(times)
    v_min, v_max = min(v_bats), max(v_bats)
    i_min, i_max = min(i_bats), max(i_bats)

    # Ensure some range
    if t_max == t_min: t_max += 1
    if v_max == v_min: v_max += 1; v_min -= 1
    if i_max == i_min: i_max += 1; i_min -= 1

    def scale(val, src_min, src_max, dst_min, dst_max):
        return dst_min + (val - src_min) * (dst_max - dst_min) / (src_max - src_min)

    with open(output_svg, 'w') as f:
        f.write(f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">\n')
        f.write(f'<rect width="100%" height="100%" fill="#f8f8f8"/>\n')
        f.write(f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="20">{title}</text>\n')

        # Grid lines
        f.write(f'<line x1="{padding}" y1="{height-padding}" x2="{width-padding}" y2="{height-padding}" stroke="black" stroke-width="2"/>\n')
        f.write(f'<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{height-padding}" stroke="black" stroke-width="2"/>\n')

        # Vbatt path (Red)
        v_points = []
        for t, v in zip(times, v_bats):
            x = scale(t, t_min, t_max, padding, width-padding)
            y = scale(v, v_min, v_max, height-padding, padding)
            v_points.append(f"{x},{y}")
        f.write(f'<polyline points="{" ".join(v_points)}" fill="none" stroke="red" stroke-width="1.5"/>\n')
        f.write(f'<text x="{padding-5}" y="{padding}" text-anchor="end" font-family="Arial" font-size="12" fill="red">{v_max:.1f}V</text>\n')
        f.write(f'<text x="{padding-5}" y="{height-padding}" text-anchor="end" font-family="Arial" font-size="12" fill="red">{v_min:.1f}V</text>\n')

        # Ibatt path (Blue)
        i_points = []
        for t, i in zip(times, i_bats):
            x = scale(t, t_min, t_max, padding, width-padding)
            y = scale(i, i_min, i_max, height-padding, padding)
            i_points.append(f"{x},{y}")
        f.write(f'<polyline points="{" ".join(i_points)}" fill="none" stroke="blue" stroke-width="1" stroke-dasharray="2,2"/>\n')
        f.write(f'<text x="{width-padding+5}" y="{padding}" text-anchor="start" font-family="Arial" font-size="12" fill="blue">{i_max/1000:.1f}A</text>\n')
        f.write(f'<text x="{width-padding+5}" y="{height-padding}" text-anchor="start" font-family="Arial" font-size="12" fill="blue">{i_min/1000:.1f}A</text>\n')

        # Time labels
        f.write(f'<text x="{width/2}" y="{height-10}" text-anchor="middle" font-family="Arial" font-size="14">Time (seconds)</text>\n')

        f.write('</svg>')

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: plot_results.py input.log output.svg title")
    else:
        generate_svg(sys.argv[1], sys.argv[2], sys.argv[3])
