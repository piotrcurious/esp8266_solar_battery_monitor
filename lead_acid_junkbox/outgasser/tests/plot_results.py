import sys
import os

# Mode map from firmware
# 1: IDLE
# 2: CHAR_PARASITIC_WAIT
# 3: CHAR_PARASITIC
# 4: PASSIVE_FORMATION_TEST
# 5: WAIT_AFTER_OUTGASSING
# 6: CHARGE_BULK
# 7: OUTGAS_PULSE_TEST
# 8: CHARGE_FLOAT
# 9: CHARGE_DONE
# 10: FAULT
# 11: LOW_INPUT_SLEEP

def generate_svg(data, output_svg, title, zoom_mode=None):
    if not data:
        return

    if zoom_mode:
        # Filter for pulse test or parasitic phases to see the transitions
        # Include mode 2/3 (Parasitic) and 7 (Pulse Test)
        filtered = [d for d in data if d['m'] in [2, 3, 7]]
    else:
        filtered = data

    if not filtered:
        return

    # To handle X-axis continuity, split data into segments where time gap > 1s
    segments = []
    if filtered:
        curr_seg = [filtered[0]]
        for i in range(1, len(filtered)):
            # Gap detection
            if (filtered[i]['t'] - filtered[i-1]['t']) > 2.0:
                segments.append(curr_seg)
                curr_seg = []
            curr_seg.append(filtered[i])
        segments.append(curr_seg)

    # Global min/max for scaling across all segments
    all_times = [d['t'] for d in filtered]
    all_v = [d['v'] for d in filtered]
    all_i = [d['i'] for d in filtered]
    all_ratio = [d['r'] for d in filtered if 'r' in d]

    t_min, t_max = min(all_times), max(all_times)
    v_min, v_max = min(all_v), max(all_v)
    i_min, i_max = min(all_i), max(all_i)
    r_min, r_max = (min(all_ratio), max(all_ratio)) if all_ratio else (0, 1.2)

    if v_max == v_min: v_max += 0.1; v_min -= 0.1
    if i_max == i_min: i_max += 0.1; i_min -= 0.1
    if t_max == t_min: t_max += 1.0

    width = 1000
    height = 600
    padding = 75

    def scale(val, src_min, src_max, dst_min, dst_max):
        if src_max == src_min: return dst_min
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
            f.write(f'<text x="{padding-10}" y="{y+4}" text-anchor="end" font-family="sans-serif" font-size="12" fill="red">{v_val:.2f}V</text>\n')

        for i in range(6):
            i_val = i_min + (i_max - i_min) * i / 5
            y = scale(i_val, i_min, i_max, height-padding, padding)
            f.write(f'<text x="{width-padding+10}" y="{y+4}" text-anchor="start" font-family="sans-serif" font-size="12" fill="blue">{i_val:.2f}A</text>\n')

        # Ratio axis (Green) if zoom
        if zoom_mode == 7:
            for i in range(3):
                r_val = 0.5 + i * 0.25
                y = scale(r_val, 0, 1.25, height-padding, padding)
                f.write(f'<line x1="{padding}" y1="{y}" x2="{width-padding}" y2="{y}" stroke="green" stroke-width="0.5" stroke-dasharray="2,2" opacity="0.3"/>\n')
                f.write(f'<text x="{padding + 40}" y="{y-2}" text-anchor="start" font-family="sans-serif" font-size="10" fill="green">Eff:{r_val:.2f}</text>\n')

        # Plot each segment separately to avoid jump lines
        for seg in segments:
            # Downsample segment for size
            step = max(1, len(seg) // 800)
            seg = seg[::step]

            v_pts = " ".join([f"{scale(d['t'], t_min, t_max, padding, width-padding):.1f},{scale(d['v'], v_min, v_max, height-padding, padding):.1f}" for d in seg])
            f.write(f'<polyline points="{v_pts}" fill="none" stroke="red" stroke-width="2.5" opacity="0.8"/>\n')

            i_pts = " ".join([f"{scale(d['t'], t_min, t_max, padding, width-padding):.1f},{scale(d['i'], i_min, i_max, height-padding, padding):.1f}" for d in seg])
            f.write(f'<polyline points="{i_pts}" fill="none" stroke="blue" stroke-width="1.5" stroke-dasharray="5,3"/>\n')

            if zoom_mode == 7:
                 r_pts = " ".join([f"{scale(d['t'], t_min, t_max, padding, width-padding):.1f},{scale(d['r'], 0, 1.25, height-padding, padding):.1f}" for d in seg if 'r' in d])
                 if r_pts:
                     f.write(f'<polyline points="{r_pts}" fill="none" stroke="green" stroke-width="1.2" opacity="0.6"/>\n')

        # Axis
        f.write(f'<line x1="{padding}" y1="{height-padding}" x2="{width-padding}" y2="{height-padding}" stroke="#333" stroke-width="2"/>\n')
        f.write(f'<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{height-padding}" stroke="#333" stroke-width="2"/>\n')
        f.write(f'<line x1="{width-padding}" y1="{padding}" x2="{width-padding}" y2="{height-padding}" stroke="#333" stroke-width="2"/>\n')

        # Time labels
        for i in range(6):
            t_val = t_min + (t_max - t_min) * i / 5
            x = scale(t_val, t_min, t_max, padding, width-padding)
            f.write(f'<text x="{x}" y="{height-padding+25}" text-anchor="middle" font-family="sans-serif" font-size="12">{t_val:.0f}s</text>\n')

        # Legend
        f.write(f'<g transform="translate({width-200}, 60)">\n')
        f.write(f'<rect width="140" height="70" fill="white" fill-opacity="0.9" stroke="#ccc"/>\n')
        f.write(f'<line x1="10" y1="15" x2="40" y2="15" stroke="red" stroke-width="3"/>\n')
        f.write(f'<text x="45" y="20" font-family="sans-serif" font-size="14">Voltage</text>\n')
        f.write(f'<line x1="10" y1="35" x2="40" y2="35" stroke="blue" stroke-width="2" stroke-dasharray="5,3"/>\n')
        f.write(f'<text x="45" y="40" font-family="sans-serif" font-size="14">Current</text>\n')
        if zoom_mode == 7:
            f.write(f'<line x1="10" y1="55" x2="40" y2="55" stroke="green" stroke-width="1.2"/>\n')
            f.write(f'<text x="45" y="60" font-family="sans-serif" font-size="14">Efficiency</text>\n')
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
                    d = {'t': float(p[0])/1000, 'm': int(p[1]), 'v': float(p[2]), 'i': float(p[3])/1000}
                    if len(p) >= 9:
                        d['r'] = float(p[7])
                        d['c'] = float(p[8])
                    data.append(d)
                except: continue

    if not data:
        return

    generate_svg(data, f"{out_prefix}_full.svg", f"Full Charge Cycle: {out_prefix}")
    generate_svg(data, f"{out_prefix}_pulses.svg", f"Outgassing Characterization Detail (Zoom): {out_prefix}", zoom_mode=7)

if __name__ == "__main__":
    main()
