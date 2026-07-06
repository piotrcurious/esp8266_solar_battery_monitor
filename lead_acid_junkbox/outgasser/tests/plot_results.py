import sys
import os

# Mode map from firmware
# 2: CHAR_PARASITIC
# 5: CHARGE_BULK
# 6: OUTGAS_PULSE_TEST
# 7: CHARGE_FLOAT

def generate_svg(data, output_svg, title, zoom_mode=None):
    if not data:
        return

    times = [d['t'] for d in data]
    modes = [d['m'] for d in data]
    v_bats = [d['v'] for d in data]
    i_bats = [d['i'] for d in data]

    if zoom_mode:
        # Filter for a specific mode range if requested
        filtered = [d for d in data if d['m'] == zoom_mode]
        if not filtered:
            return
        times = [d['t'] for d in filtered]
        v_bats = [d['v'] for d in filtered]
        i_bats = [d['i'] for d in filtered]

    # Downsample
    step = max(1, len(times) // 1000)
    times = times[::step]
    v_bats = v_bats[::step]
    i_bats = i_bats[::step]

    width = 1000
    height = 500
    padding = 70

    t_min, t_max = min(times), max(times)
    v_min, v_max = min(v_bats), max(v_bats)
    i_min, i_max = min(i_bats), max(i_bats)

    if v_max == v_min: v_max += 0.1; v_min -= 0.1
    if i_max == i_min: i_max += 0.1; i_min -= 0.1

    def scale(val, src_min, src_max, dst_min, dst_max):
        return dst_min + (val - src_min) * (dst_max - dst_min) / (src_max - src_min)

    with open(output_svg, 'w') as f:
        f.write(f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">\n')
        f.write(f'<rect width="100%" height="100%" fill="#ffffff"/>\n')
        f.write(f'<text x="{width/2}" y="35" text-anchor="middle" font-family="sans-serif" font-size="22" font-weight="bold">{title}</text>\n')

        # Grid lines
        for i in range(6):
            v_val = v_min + (v_max - v_min) * i / 5
            y = scale(v_val, v_min, v_max, height-padding, padding)
            f.write(f'<line x1="{padding}" y1="{y}" x2="{width-padding}" y2="{y}" stroke="#eee" stroke-width="1"/>\n')
            f.write(f'<text x="{padding-5}" y="{y+4}" text-anchor="end" font-family="sans-serif" font-size="12" fill="red">{v_val:.2f}V</text>\n')

        for i in range(6):
            i_val = i_min + (i_max - i_min) * i / 5
            y = scale(i_val, i_min, i_max, height-padding, padding)
            f.write(f'<text x="{width-padding+5}" y="{y+4}" text-anchor="start" font-family="sans-serif" font-size="12" fill="blue">{i_val:.2f}A</text>\n')

        # V_bat path
        v_pts = " ".join([f"{scale(t, t_min, t_max, padding, width-padding):.1f},{scale(v, v_min, v_max, height-padding, padding):.1f}" for t, v in zip(times, v_bats)])
        f.write(f'<polyline points="{v_pts}" fill="none" stroke="red" stroke-width="2.5" opacity="0.8"/>\n')

        # I_bat path
        i_pts = " ".join([f"{scale(t, t_min, t_max, padding, width-padding):.1f},{scale(i, i_min, i_max, height-padding, padding):.1f}" for t, i in zip(times, i_bats)])
        f.write(f'<polyline points="{i_pts}" fill="none" stroke="blue" stroke-width="1.5" stroke-dasharray="5,3"/>\n')

        # Axis
        f.write(f'<line x1="{padding}" y1="{height-padding}" x2="{width-padding}" y2="{height-padding}" stroke="#333" stroke-width="2"/>\n')
        f.write(f'<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{height-padding}" stroke="#333" stroke-width="2"/>\n')
        f.write(f'<line x1="{width-padding}" y1="{padding}" x2="{width-padding}" y2="{height-padding}" stroke="#333" stroke-width="2"/>\n')

        # Time labels
        for i in range(6):
            t_val = t_min + (t_max - t_min) * i / 5
            x = scale(t_val, t_min, t_max, padding, width-padding)
            f.write(f'<text x="{x}" y="{height-padding+20}" text-anchor="middle" font-family="sans-serif" font-size="12">{t_val:.0f}s</text>\n')

        # Legend
        f.write(f'<g transform="translate({width-180}, 60)">\n')
        f.write(f'<rect width="120" height="50" fill="white" fill-opacity="0.9" stroke="#ccc"/>\n')
        f.write(f'<line x1="10" y1="15" x2="40" y2="15" stroke="red" stroke-width="3"/>\n')
        f.write(f'<text x="45" y="20" font-family="sans-serif" font-size="14">Voltage</text>\n')
        f.write(f'<line x1="10" y1="35" x2="40" y2="35" stroke="blue" stroke-width="2" stroke-dasharray="5,3"/>\n')
        f.write(f'<text x="45" y="40" font-family="sans-serif" font-size="14">Current</text>\n')
        f.write(f'</g>\n')

        f.write('</svg>')

def main():
    if len(sys.argv) < 3:
        print("Usage: plot_results.py input.log output_prefix")
        return

    log_file = sys.argv[1]
    out_prefix = sys.argv[2]

    data = []
    with open(log_file, 'r') as f:
        for line in f:
            if line.startswith("TELE:"):
                try:
                    p = line.split(" ")[1].split(",")
                    data.append({'t': float(p[0])/1000, 'm': int(p[1]), 'v': float(p[2]), 'i': float(p[3])/1000})
                except: continue

    if not data:
        return

    # 1. Overall Cycle
    generate_svg(data, f"{out_prefix}_full.svg", f"Full Charge Cycle: {out_prefix}")

    # 2. Characterization Zoom (Mode 6: OUTGAS_PULSE_TEST)
    generate_svg(data, f"{out_prefix}_pulses.svg", f"Outgassing Characterization Detail (Zoom): {out_prefix}", zoom_mode=6)

if __name__ == "__main__":
    main()
