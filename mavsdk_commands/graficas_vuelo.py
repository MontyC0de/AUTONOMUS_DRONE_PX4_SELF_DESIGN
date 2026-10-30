#!/usr/bin/env python3
"""
Gráficas y métricas de un vuelo de mision_avanzada.py (o de varios, superpuestos).

    python3 mavsdk_commands/graficas_vuelo.py vuelos/mision_XXXX.csv
    python3 mavsdk_commands/graficas_vuelo.py sim.csv real.csv --etiquetas Gemelo Real

Genera, junto al primer CSV, una carpeta con:
    planta.png        vista en planta de cada figura, referencia frente a vuelo
    trayectoria_3d.png
    altura_error.png  altura y error de seguimiento frente al tiempo
    resumen.txt       error RMS / máximo por figura, velocidad, batería
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

FIGURAS = ["Cuadrado", "Orbita", "Ocho", "Helice"]
TITULOS = {"Cuadrado": "Cuadrado 8 m", "Orbita": "Órbita r = 6 m",
           "Ocho": "Ocho 14 × 7 m", "Helice": "Hélice r = 4 m"}
TRANSICIONES = ("entrada", "a la esquina")

# Paleta de referencia (dataviz): slot 1 azul, slot 2 naranja, slot 3 aqua
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
REFERENCIA = "#8a8984"
TEXTO, TEXTO_2, REJILLA, FONDO = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def leer(ruta):
    with open(ruta) as f:
        filas = list(csv.DictReader(f))
    num = ["t", "sp_n", "sp_e", "sp_d", "sp_yaw", "n", "e", "d", "vn", "ve", "vd",
           "roll", "pitch", "yaw", "bat_v", "bat_pct"]
    datos = {k: np.array([float(r[k]) for r in filas]) for k in num}
    datos["tramo"] = np.array([r["tramo"] for r in filas])
    datos["figura"] = np.array([r["tramo"].split(":")[0] for r in filas])
    datos["transicion"] = np.array([r["tramo"].split(": ")[-1] in TRANSICIONES for r in filas])
    datos["error"] = np.sqrt((datos["n"] - datos["sp_n"]) ** 2 + (datos["e"] - datos["sp_e"]) ** 2
                             + (datos["d"] - datos["sp_d"]) ** 2)
    datos["vel"] = np.sqrt(datos["vn"] ** 2 + datos["ve"] ** 2 + datos["vd"] ** 2)
    return datos


def estilo(ax):
    ax.set_facecolor(FONDO)
    ax.grid(True, color=REJILLA, linewidth=0.8)
    ax.tick_params(colors=TEXTO_2, labelsize=9)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    for lado in ("left", "bottom"):
        ax.spines[lado].set_color(REJILLA)
    ax.xaxis.label.set_color(TEXTO_2)
    ax.yaxis.label.set_color(TEXTO_2)


def mascara(d, figura):
    return (d["figura"] == figura) & ~d["transicion"]


def resumen(vuelos, etiquetas):
    lineas = []
    for d, nombre in zip(vuelos, etiquetas):
        lineas.append(f"== {nombre}")
        lineas.append(f"{'Figura':10s} {'T [s]':>6s} {'RMS [cm]':>9s} {'max [cm]':>9s} {'v max [m/s]':>12s}")
        for fig in FIGURAS + ["Regreso"]:
            m = mascara(d, fig)
            if not m.any():
                continue
            e = d["error"][m]
            dur = d["t"][m][-1] - d["t"][m][0]
            lineas.append(f"{fig:10s} {dur:6.1f} {100 * math.sqrt(np.mean(e ** 2)):9.1f} "
                          f"{100 * e.max():9.1f} {d['vel'][m].max():12.2f}")
        e = d["error"]
        lineas.append(f"{'Total':10s} {d['t'][-1]:6.1f} {100 * math.sqrt(np.mean(e ** 2)):9.1f} "
                      f"{100 * e.max():9.1f} {d['vel'].max():12.2f}")
        bat = d["bat_pct"][~np.isnan(d["bat_pct"])]
        if len(bat):
            lineas.append(f"Batería: {bat[0]:.0f} % -> {bat[-1]:.0f} %  "
                          f"({d['bat_v'][0]:.2f} V -> {d['bat_v'][-1]:.2f} V)")
        inclinacion = np.degrees(np.arccos(np.cos(np.radians(d["roll"])) * np.cos(np.radians(d["pitch"]))))
        lineas.append(f"Inclinación máxima: {inclinacion.max():.1f}°")
        lineas.append("")
    return "\n".join(lineas)


def grafica_planta(vuelos, etiquetas, salida):
    fig, ejes = plt.subplots(2, 2, figsize=(10, 10), facecolor=FONDO)
    ref = vuelos[0]
    for ax, figura in zip(ejes.flat, FIGURAS):
        estilo(ax)
        m = mascara(ref, figura)
        ax.plot(ref["sp_e"][m], ref["sp_n"][m], color=REFERENCIA, linewidth=2,
                linestyle=(0, (4, 3)), label="Referencia")
        rms = []
        for d, nombre, color in zip(vuelos, etiquetas, SERIES):
            m = mascara(d, figura)
            ax.plot(d["e"][m], d["n"][m], color=color, linewidth=2, label=nombre)
            rms.append(f"{nombre} {100 * math.sqrt(np.mean(d['error'][m] ** 2)):.0f} cm")
        ax.set_title(TITULOS[figura], color=TEXTO, fontsize=12, loc="left",
                     fontweight="bold", pad=20)
        ax.text(0.0, 1.02, "Error RMS: " + " · ".join(rms), transform=ax.transAxes,
                color=TEXTO_2, fontsize=9, va="bottom", ha="left")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("Este [m]")
        ax.set_ylabel("Norte [m]")
    lineas, nombres = ejes[0, 0].get_legend_handles_labels()
    fig.legend(lineas, nombres, frameon=False, fontsize=10, labelcolor=TEXTO_2,
               loc="upper center", ncol=len(nombres))
    fig.tight_layout(h_pad=3, rect=(0, 0, 1, 0.96))
    fig.savefig(salida, dpi=160, facecolor=FONDO)
    plt.close(fig)


def grafica_3d(vuelos, etiquetas, salida):
    fig = plt.figure(figsize=(9, 7), facecolor=FONDO)
    ax = fig.add_subplot(projection="3d")
    ax.set_facecolor(FONDO)
    ref = vuelos[0]
    ax.plot(ref["sp_e"], ref["sp_n"], -ref["sp_d"], color=REFERENCIA, linewidth=1.5,
            linestyle=(0, (4, 3)), label="Referencia")
    for d, nombre, color in zip(vuelos, etiquetas, SERIES):
        ax.plot(d["e"], d["n"], -d["d"], color=color, linewidth=2, label=nombre)
    ax.set_xlabel("Este [m]", color=TEXTO_2)
    ax.set_ylabel("Norte [m]", color=TEXTO_2)
    ax.set_zlabel("Altura [m]", color=TEXTO_2)
    ax.tick_params(colors=TEXTO_2, labelsize=8)
    for eje in (ax.xaxis, ax.yaxis, ax.zaxis):
        eje.set_pane_color((0.99, 0.99, 0.98, 1.0))
        eje._axinfo["grid"]["color"] = REJILLA
    ax.set_box_aspect((1, 1, 0.5))
    ax.view_init(elev=28, azim=-60)
    ax.set_title("Misión completa", color=TEXTO, fontsize=12, loc="left", fontweight="bold")
    ax.legend(frameon=False, fontsize=9, labelcolor=TEXTO_2, loc="upper left")
    fig.tight_layout()
    fig.savefig(salida, dpi=160, facecolor=FONDO)
    plt.close(fig)


def grafica_altura_error(vuelos, etiquetas, salida):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True, facecolor=FONDO,
                                   gridspec_kw={"height_ratios": [1.3, 1]})
    ref = vuelos[0]
    for ax in (ax1, ax2):
        estilo(ax)
    ax1.plot(ref["t"], -ref["sp_d"], color=REFERENCIA, linewidth=2, linestyle=(0, (4, 3)),
             label="Referencia")
    for d, nombre, color in zip(vuelos, etiquetas, SERIES):
        ax1.plot(d["t"], -d["d"], color=color, linewidth=2, label=nombre)
        ax2.plot(d["t"], 100 * d["error"], color=color, linewidth=1.5, label=nombre)
    # Etiquetas de cada figura sobre su intervalo de tiempo
    for figura in FIGURAS:
        m = mascara(ref, figura)
        if m.any():
            t0, t1 = ref["t"][m][0], ref["t"][m][-1]
            for ax in (ax1, ax2):
                ax.axvspan(t0, t1, color=REJILLA, alpha=0.35, linewidth=0)
            ax1.text((t0 + t1) / 2, 1.0, TITULOS[figura].split(" ")[0], transform=ax1.get_xaxis_transform(),
                     ha="center", va="bottom", color=TEXTO_2, fontsize=9)
    ax1.set_ylabel("Altura [m]")
    ax2.set_ylabel("Error de seguimiento [cm]")
    ax2.set_xlabel("Tiempo [s]")
    ax1.legend(frameon=False, fontsize=9, labelcolor=TEXTO_2, loc="upper left", ncol=len(vuelos) + 1)
    fig.tight_layout()
    fig.savefig(salida, dpi=160, facecolor=FONDO)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Gráficas de vuelos de mision_avanzada.py")
    parser.add_argument("csv", nargs="+", help="uno o varios CSV (el primero da la referencia)")
    parser.add_argument("--etiquetas", nargs="+", help="nombre de cada vuelo en la leyenda")
    args = parser.parse_args()
    if len(args.csv) > len(SERIES):
        raise SystemExit(f"Máximo {len(SERIES)} vuelos superpuestos")
    etiquetas = args.etiquetas or [os.path.splitext(os.path.basename(c))[0] for c in args.csv]
    vuelos = [leer(c) for c in args.csv]

    carpeta = os.path.splitext(args.csv[0])[0]
    os.makedirs(carpeta, exist_ok=True)
    grafica_planta(vuelos, etiquetas, os.path.join(carpeta, "planta.png"))
    grafica_3d(vuelos, etiquetas, os.path.join(carpeta, "trayectoria_3d.png"))
    grafica_altura_error(vuelos, etiquetas, os.path.join(carpeta, "altura_error.png"))
    texto = resumen(vuelos, etiquetas)
    with open(os.path.join(carpeta, "resumen.txt"), "w") as f:
        f.write(texto)
    print(texto)
    print(f"Gráficas en {carpeta}/")


if __name__ == "__main__":
    main()
