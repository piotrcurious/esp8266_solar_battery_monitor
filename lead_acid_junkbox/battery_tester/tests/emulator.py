class LeadAcidCell:
    def __init__(self, capacity_ah=10.0, initial_soc=0.5):
        self.capacity_ah = capacity_ah
        self.soc = initial_soc
        self.internal_resistance = 0.02  # Ohms
        self.v_nominal = 2.0
        self.v_full = 2.15
        self.v_empty = 1.9

    def get_ocv(self):
        # Simple linear OCV model
        return self.v_empty + (self.v_full - self.v_empty) * self.soc

    def update(self, current_a, dt_s):
        # Charge/Discharge
        # current_a > 0 means charging
        # current_a < 0 means discharging

        # Coulombic efficiency
        eff = 0.95 if current_a > 0 else 1.0

        delta_soc = (current_a * dt_s * eff) / (self.capacity_ah * 3600.0)
        self.soc = max(0.0, min(1.0, self.soc + delta_soc))

        # Gassing effect (simplified)
        # Above 2.35V per cell, voltage rises faster
        ocv = self.get_ocv()
        gassing_voltage = 0.0
        if self.soc > 0.95 and current_a > 0:
            # Overcharge increases voltage significantly as water electrolyzes
            # Plateau effect: voltage rises to a certain point and then levels off or rises very slowly
            gassing_voltage = 0.3 * (self.soc - 0.95) / 0.05

        return ocv + gassing_voltage + (current_a * self.internal_resistance)

class LeadAcidBattery:
    def __init__(self, num_cells=3, capacity_ah=10.0, initial_soc=0.5):
        self.cells = [LeadAcidCell(capacity_ah, initial_soc) for _ in range(num_cells)]
        self.current = 0.0

    def step(self, current_a, dt_s):
        self.current = current_a
        v_total = 0.0
        for cell in self.cells:
            v_total += cell.update(current_a, dt_s)
        return v_total

    def get_soc(self):
        return sum(cell.soc for cell in self.cells) / len(self.cells)
