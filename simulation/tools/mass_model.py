#!/usr/bin/env python3
"""Lumped-mass model of the real drone -> mass, centre of mass and inertia for model.sdf.

Every component is a box or a rod with its mass and position in the base_link
frame (x forward, y left, z up, origin on top of the arms at the frame centre).
Positions and sizes come from the CAD mesh (simulation/models/mi_drone_base/meshes),
masses are measured on the scale, taken from datasheets, or estimated from the
CAD geometry. The landing gear takes whatever mass is left so the total matches
the scale exactly.

    python3 simulation/tools/mass_model.py
"""
import math

G = 9.81

# Measured on a kitchen scale
TOTAL_MASS = 0.970     # whole drone, battery and props included
BATTERY_MASS = 0.200   # Tattu 4S 2200 mAh 45C
ROTOR_MASS = 0.033     # one motor + prop nut + prop
ROTOR_XY = 0.1228      # motor axis position, from CAD (347 mm diagonal)

# Propulsion data used for the hover estimate (see mi_drone/model.sdf)
MOTOR_CONSTANT = 1.278e-6   # N/(rad/s)^2
W_MIN, W_MAX = 400.0, 2700.0  # rad/s, SIM_GZ_EC_MIN / MAX


def box(m, a, b, c):
    """Inertia of a solid box with edges a (x), b (y), c (z) about its centre."""
    return [[m * (b * b + c * c) / 12, 0, 0],
            [0, m * (a * a + c * c) / 12, 0],
            [0, 0, m * (a * a + b * b) / 12]]


def rod(m, length, direction):
    """Inertia of a thin rod along a unit direction about its centre: mL^2/12 (I - u u^T)."""
    n = math.sqrt(sum(d * d for d in direction))
    u = [d / n for d in direction]
    k = m * length * length / 12
    return [[k * ((i == j) - u[i] * u[j]) for j in range(3)] for i in range(3)]


def shift(m, inertia, d):
    """Parallel axis theorem: inertia about a point displaced by -d from the centre."""
    d2 = sum(x * x for x in d)
    return [[inertia[i][j] + m * ((i == j) * d2 - d[i] * d[j]) for j in range(3)] for i in range(3)]


def add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


DIAG = [(1, 1), (1, -1), (-1, 1), (-1, -1)]

# name, mass [kg], centre [m], inertia about own centre, source
parts = [
    ("Bateria Tattu 4S 2200", BATTERY_MASS, (-0.0019, 0.0014, -0.0282), box(BATTERY_MASS, 0.082, 0.033, 0.0375), "bascula"),
    ("Pixhawk 4", 0.0158, (-0.0011, -0.0006, 0.0080), box(0.0158, 0.084, 0.044, 0.012), "datasheet"),
    ("Raspberry Pi 3B+", 0.050, (0.0049, -0.0001, 0.0520), box(0.050, 0.085, 0.056, 0.017), "datasheet"),
    ("GPS Holybro M10", 0.032, (-0.0281, -0.0001, 0.0663), box(0.032, 0.050, 0.050, 0.014), "datasheet"),
    ("Modulo potencia PM02", 0.020, (0.0, 0.0, 0.0150), box(0.020, 0.040, 0.025, 0.010), "datasheet"),
    ("Telemetria 433 MHz + antena", 0.025, (0.0, 0.0650, 0.0200), box(0.025, 0.030, 0.020, 0.050), "datasheet"),
    ("Receptor, step-down, placa PWM", 0.0115, (0.0, 0.0, 0.0150), box(0.0115, 0.040, 0.030, 0.010), "datasheet"),
    ("Placa base (carbono 2 mm)", 0.041, (0.0, 0.0, -0.0060), box(0.041, 0.125, 0.125, 0.002), "CAD x 1.55 g/cm3"),
    ("Placa de apoyo (carbono 2 mm)", 0.041, (0.0, 0.0, 0.0010), box(0.041, 0.125, 0.125, 0.002), "CAD x 1.55 g/cm3"),
    ("Placa superior (carbono 1 mm)", 0.012, (0.0, 0.0, 0.0325), box(0.012, 0.095, 0.095, 0.001), "CAD x 1.55 g/cm3"),
    ("Placa bateria (carbono 1 mm)", 0.005, (0.0, 0.0, -0.0475), box(0.005, 0.062, 0.053, 0.001), "CAD x 1.55 g/cm3"),
    ("Piezas impresas (jaula, soportes)", 0.030, (0.0, 0.0, 0.0450), box(0.030, 0.092, 0.092, 0.030), "estimado"),
    ("Cableado, XT60, tornilleria", 0.060, (0.0, 0.0, 0.0000), box(0.060, 0.100, 0.100, 0.040), "estimado"),
]
for sx, sy in DIAG:
    parts.append(("Brazo carbono 5 mm", 0.025, (0.0825 * sx, 0.0825 * sy, -0.0025),
                  rod(0.025, 0.160, (sx, sy, 0)), "CAD x 1.55 g/cm3"))
    parts.append(("ESC", 0.007, (0.0898 * sx, 0.0898 * sy, 0.0050), box(0.007, 0.030, 0.015, 0.006), "datasheet"))
for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
    parts.append(("Columna aluminio 35 mm", 0.0025, (0.045 * sx, 0.045 * sy, 0.0170), rod(0.0025, 0.035, (0, 0, 1)), "estimado"))
    parts.append(("Columna aluminio 35 mm", 0.0025, (0.045 * sx, 0.030 * sy, 0.0170), rod(0.0025, 0.035, (0, 0, 1)), "estimado"))
    parts.append(("Columna aluminio 50 mm", 0.0025, (0.0256 * sx, 0.0207 * sy, -0.0270), rod(0.0025, 0.050, (0, 0, 1)), "estimado"))
for sy in (1, -1):
    parts.append(("Pared impresa", 0.005, (0.0, 0.042 * sy, 0.0169), box(0.005, 0.065, 0.009, 0.030), "estimado"))
    parts.append(("Pared impresa", 0.005, (0.042 * sy, 0.0, 0.0169), box(0.005, 0.009, 0.065, 0.030), "estimado"))

# Landing gear: whatever the scale says is left, split over mounts, struts and skids
base_mass = TOTAL_MASS - 4 * ROTOR_MASS
gear = base_mass - sum(p[1] for p in parts)
for sy in (1, -1):
    parts.append(("Tren: anclaje", 0.15 * gear, (0.0, 0.045 * sy, -0.0198), box(0.15 * gear, 0.034, 0.032, 0.026), "resto"))
    parts.append(("Tren: pata", 0.20 * gear, (0.0, 0.067 * sy, -0.0715), rod(0.20 * gear, 0.140, (0, 0.45 * sy, -1)), "resto"))
    parts.append(("Tren: patin", 0.15 * gear, (0.0, 0.093 * sy, -0.1447), rod(0.15 * gear, 0.160, (1, 0, 0)), "resto"))

mass = sum(p[1] for p in parts)
com = [sum(p[1] * p[2][i] for p in parts) / mass for i in range(3)]
inertia = [[0.0] * 3 for _ in range(3)]
for _, m, c, i_own, _ in parts:
    inertia = add(inertia, shift(m, i_own, [c[k] - com[k] for k in range(3)]))

# Rotor link: motor (28 g) + nut (1 g) + 5" tri-blade prop (4 g) = 33 g measured
m_prop, r_prop = 0.004, 0.0635
m_bell, r_bell = 0.012, 0.0135          # rotating bell, thin cylinder
m_stator, r_stator = 0.017, 0.011       # stator + nut, solid cylinder
rot_izz = m_prop * r_prop ** 2 / 3 + m_bell * r_bell ** 2 + m_stator * r_stator ** 2 / 2
rot_ixx = m_prop * r_prop ** 2 / 6 + m_prop * 0.02 ** 2 + 0.029 * (3 * 0.0139 ** 2 + 0.024 ** 2) / 12

# Whole vehicle about its own centre of mass, for reference
veh = shift(base_mass, inertia, [0, 0, 0])
veh_com = [com[0] * base_mass / TOTAL_MASS, com[1] * base_mass / TOTAL_MASS,
           (com[2] * base_mass + 4 * ROTOR_MASS * 0.012) / TOTAL_MASS]
veh = shift(base_mass, inertia, [com[k] - veh_com[k] for k in range(3)])
for sx, sy in DIAG:
    r = [ROTOR_XY * sx - veh_com[0], ROTOR_XY * sy - veh_com[1], 0.012 - veh_com[2]]
    veh = add(veh, shift(ROTOR_MASS, box(0, 0, 0, 0), r))

w_hover = math.sqrt(TOTAL_MASS * G / 4 / MOTOR_CONSTANT)
t_max = MOTOR_CONSTANT * W_MAX ** 2

print(f"{'Componente':38s} {'masa [g]':>9s}  fuente")
for name, m, _, _, src in parts:
    print(f"{name:38s} {m * 1000:9.1f}  {src}")
print(f"{'4 x rotor (motor+tuerca+helice)':38s} {4 * ROTOR_MASS * 1000:9.1f}  bascula")
print(f"{'TOTAL':38s} {(mass + 4 * ROTOR_MASS) * 1000:9.1f}")
print()
print(f"base_link: mass {mass:.3f} kg, CoM ({com[0]:.4f}, {com[1]:.4f}, {com[2]:.4f}) m")
print(f"  ixx {inertia[0][0]:.5f}  iyy {inertia[1][1]:.5f}  izz {inertia[2][2]:.5f}  kg m^2")
print(f"  ixy {inertia[0][1]:.2e}  ixz {inertia[0][2]:.2e}  iyz {inertia[1][2]:.2e}")
print(f"rotor: mass {ROTOR_MASS} kg, ixx = iyy {rot_ixx:.2e}, izz {rot_izz:.2e} kg m^2")
print(f"vehicle (all links): ixx {veh[0][0]:.5f}  iyy {veh[1][1]:.5f}  izz {veh[2][2]:.5f}  kg m^2")
print()
print(f"max thrust {4 * t_max:.1f} N total -> thrust/weight {4 * t_max / (TOTAL_MASS * G):.2f}")
print(f"hover: {TOTAL_MASS * G / 4:.3f} N per motor, w = {w_hover:.0f} rad/s, "
      f"throttle = {(w_hover - W_MIN) / (W_MAX - W_MIN):.3f}")
