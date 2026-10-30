#!/usr/bin/env python3
"""
Genera el mundo de Gazebo del campo de pruebas real a partir de sus coordenadas.

- Suelo: ortofoto PNOA del IGN (CC BY 4.0, © Instituto Geográfico Nacional)
  en dos capas: detalle de 204.8 m a 10 cm/píxel y contexto de 1.6 km a 80 cm/píxel.
- Árboles: encinas y eucaliptos 3D (malla Oak tree de Gazebo Fuel, CC0) sobre las copas
  detectadas en la ortofoto.
- Mundo: origen en el punto de despegue, con su latitud, longitud, altitud y
  campo magnético (WMM2025), así PX4 y QGroundControl sitúan el dron en el sitio real.

Herramienta de desarrollo; el mundo generado ya está en el repo. Dependencias:
    pip install numpy pillow scipy pyproj pygeomag

Uso:
    python3 simulation/tools/campo_pruebas.py 36.5117778 -4.6959167 --elevacion 20
"""
import argparse
import datetime
import json
import math
import os
import random
import shutil
import urllib.request

import numpy as np
from PIL import Image
from pyproj import Transformer
from pygeomag import GeoMag
from scipy import ndimage

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WMS = ("https://www.ign.es/wms-inspire/pnoa-ma?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap"
       "&LAYERS=OI.OrthoimageCoverage&STYLES=&CRS=EPSG:25830&BBOX={},{},{},{}"
       "&WIDTH={}&HEIGHT={}&FORMAT=image/jpeg")
FUEL = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models")
CAPAS = {"detalle": (204.8, 2048, 0.0), "contexto": (1638.4, 2048, -0.3)}  # lado [m], px, z

DESPLAZAMIENTO_ESTE = 2.5   # m, del lado en sombra (oeste) al centro de la copa
PASO = 5.0                  # m entre árboles dentro de una copa continua
OAK_ANCHO, OAK_ALTO = 10.6, 6.5     # copa y altura de la malla Oak tree [m]
EUCALIPTO_ALTO = 15.0              # m, los de la pinada del suroeste


def descargar_ortofotos(x, y, carpeta):
    for nombre, (lado, px, _) in CAPAS.items():
        url = WMS.format(x - lado / 2, y - lado / 2, x + lado / 2, y + lado / 2, px, px)
        ruta = os.path.join(carpeta, f"{nombre}.jpg")
        urllib.request.urlretrieve(url, ruta)
        print(f"ortofoto {nombre}: {lado} m, {lado / px * 100:.0f} cm/px")


def detectar_arboles(ruta, mpp, tipo_en, paso_m=PASO, area_min=4.0, excluir=0.0, radio=1e9):
    """Copas = zonas oscuras no marrones; se descartan las que caen en asfalto.

    excluir: semilado [m] del cuadrado central que ya cubre otra capa;
    radio: distancia máxima [m] al despegue.
    """
    img = Image.open(ruta).convert("RGB")
    a = np.asarray(img).astype(float)
    v = a.mean(axis=2)
    sat = np.asarray(img.convert("HSV"))[..., 1] / 255.0
    copa = (v < 75) & (a[..., 1] + 6 >= a[..., 0])
    copa = ndimage.binary_fill_holes(ndimage.binary_closing(ndimage.binary_opening(copa, iterations=3), iterations=4))
    # Asfalto medido en la ortofoto: saturación ~0.03; la tierra seca está por encima de 0.12
    asfalto = (sat < 0.06) & (v > 90) & (v < 180)
    lab, _ = ndimage.label(copa)
    h, w = copa.shape
    rng = random.Random(7)
    r_asf = int(3.0 / mpp)

    arboles = []
    for i, sl in enumerate(ndimage.find_objects(lab), start=1):
        blob = lab[sl] == i
        area = blob.sum() * mpp ** 2
        if area < area_min:
            continue
        if area < max(60, 3 * paso_m ** 2):
            cy, cx = ndimage.center_of_mass(blob)
            puntos = [(cx + sl[1].start, cy + sl[0].start, 2 * math.sqrt(area / math.pi) + 2)]
        else:
            paso = int(round(paso_m / mpp))
            ys, xs = np.mgrid[sl[0].start:sl[0].stop:paso, sl[1].start:sl[1].stop:paso]
            puntos = [(xx, yy, paso_m + 1) for yy, xx in zip(ys.ravel() + paso // 2, xs.ravel() + paso // 2)
                      if yy < h and xx < w and lab[yy, xx] == i]
        for px, py, diam in puntos:
            ventana = asfalto[max(0, int(py) - r_asf):int(py) + r_asf, max(0, int(px) - r_asf):int(px) + r_asf]
            if ventana.mean() > 0.4:
                continue
            x = (px - w / 2) * mpp + DESPLAZAMIENTO_ESTE
            y = (h / 2 - py) * mpp
            if (abs(x) < excluir and abs(y) < excluir) or math.hypot(x, y) > radio:
                continue
            tipo = tipo_en(x, y)
            diam = min(max(diam, 4.0), 12.0)
            escala = {"eucalipto": diam / OAK_ANCHO,
                      "arbusto": min(diam, 5.0) / OAK_ANCHO,
                      "roble": diam / OAK_ANCHO}[tipo]
            # Eucalyptus: same crown width, stretched to its real height
            escala_z = EUCALIPTO_ALTO / OAK_ALTO if tipo == "eucalipto" else escala
            arboles.append({"tipo": tipo, "x": round(x, 2), "y": round(y, 2), "escala": round(escala, 3),
                            "escala_z": round(escala_z, 3), "yaw": round(rng.uniform(0, 2 * math.pi), 3)})
    return arboles


def escribir_suelo(carpeta_mallas):
    for nombre, (lado, _, _) in CAPAS.items():
        m = lado / 2
        with open(os.path.join(carpeta_mallas, f"suelo_{nombre}.mtl"), "w") as f:
            f.write(f"newmtl {nombre}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd ../materials/textures/{nombre}.jpg\n")
        with open(os.path.join(carpeta_mallas, f"suelo_{nombre}.obj"), "w") as f:
            f.write(f"mtllib suelo_{nombre}.mtl\n")
            f.write(f"v {-m} {-m} 0\nv {m} {-m} 0\nv {m} {m} 0\nv {-m} {m} 0\n")
            f.write("vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n")
            f.write(f"usemtl {nombre}\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def copiar_arboles(carpeta_modelo):
    for especie, version, malla, texturas in (("oak tree", 7, "oak_tree.dae", ("bark_diffuse.png", "branch_diffuse.png")),):
        origen = os.path.join(FUEL, especie, str(version))
        if not os.path.isdir(origen):
            os.system(f'gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/{especie}"')
        corto = especie.split()[0]
        shutil.copy(os.path.join(origen, "meshes", malla), os.path.join(carpeta_modelo, "meshes", malla))
        destino = os.path.join(carpeta_modelo, "materials", "textures", corto)
        os.makedirs(destino, exist_ok=True)
        for t in texturas:
            shutil.copy(os.path.join(origen, "materials", "textures", t), os.path.join(destino, t))


def visual_arbol(nombre, i, t):
    malla, corto, hojas = "oak_tree.dae", "oak", "branch_diffuse.png"
    pose = f"{t['x']} {t['y']} 0 0 0 {t['yaw']}"
    s, sz = t["escala"], t["escala_z"]
    xml = []
    for sub, tex, doble in (("Branch", hojas, "true"), ("Bark", "bark_diffuse.png", "false")):
        xml.append(f"""      <visual name="arbol{i}_{sub.lower()}">
        <pose>{pose}</pose>
        <geometry><mesh><uri>model://{nombre}/meshes/{malla}</uri><submesh><name>{sub}</name></submesh><scale>{s} {s} {sz}</scale></mesh></geometry>
        <material><double_sided>{doble}</double_sided><diffuse>1 1 1</diffuse>
          <pbr><metal><albedo_map>model://{nombre}/materials/textures/{corto}/{tex}</albedo_map></metal></pbr></material>
      </visual>""")
    # Tronco como colisión, para que el gemelo no atraviese los árboles
    xml.append(f"""      <collision name="arbol{i}_tronco">
        <pose>{t['x']} {t['y']} {1.5 * sz * 2} 0 0 0</pose>
        <geometry><cylinder><radius>{max(0.15, 0.35 * s):.2f}</radius><length>{3.0 * sz * 2:.2f}</length></cylinder></geometry>
      </collision>""")
    return "\n".join(xml)


def escribir_modelo(nombre, carpeta, arboles, lat, lon):
    capas = "\n".join(f"""      <visual name="suelo_{c}">
        <pose>0 0 {z} 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://{nombre}/meshes/suelo_{c}.obj</uri></mesh></geometry>
      </visual>""" for c, (_, _, z) in CAPAS.items())
    vis_arboles = "\n".join(visual_arbol(nombre, i, t) for i, t in enumerate(arboles))
    with open(os.path.join(carpeta, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generado por simulation/tools/campo_pruebas.py. Origen = punto de despegue
     ({lat:.7f}, {lon:.7f}). Ortofoto PNOA (c) Instituto Geografico Nacional, CC BY 4.0.
     Arboles: Gazebo Fuel OpenRobotics Oak tree / Pine Tree, CC0. -->
<sdf version="1.9">
  <model name="{nombre}">
    <static>true</static>
    <link name="suelo">
      <collision name="suelo">
        <geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
      </collision>
{capas}
    </link>
    <link name="arboles">
{vis_arboles}
    </link>
  </model>
</sdf>
""")
    with open(os.path.join(carpeta, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{nombre}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Antonio Montero Diaz</name></author>
  <description>Campo de pruebas real ({lat:.5f}, {lon:.5f}): ortofoto PNOA (c) IGN CC BY 4.0 y arboles 3D.</description>
</model>
""")


def escribir_mundo(nombre, ruta, lat, lon, elevacion, campo):
    este, norte, abajo = campo
    with open(ruta, "w") as f:
        f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Campo de pruebas real. Generado por simulation/tools/campo_pruebas.py -->
<sdf version="1.9">
  <world name="{nombre}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <!-- WMM2025 en el punto de despegue, marco ENU [T] -->
    <magnetic_field>{este:.4e} {norte:.4e} {-abajo:.4e}</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.16 0.38 0.78 1</background>
      <shadows>true</shadows>
    </scene>
    <include>
      <uri>model://{nombre}</uri>
    </include>
    <!-- Sol desde el sureste, como en la ortofoto (sombras hacia el noroeste) -->
    <light name="sol" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.45 0.3 -0.84</direction>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat:.7f}</latitude_deg>
      <longitude_deg>{lon:.7f}</longitude_deg>
      <elevation>{elevacion}</elevation>
    </spherical_coordinates>
  </world>
</sdf>
""")


def tipo_campo_pruebas(x, y):
    """Eucaliptos al suroeste, matorral en el talud norte, encinas sueltas en el resto."""
    if x < -25 and y < -10:
        return "eucalipto"
    if y > 68:
        return "arbusto"
    return "roble"


def main():
    p = argparse.ArgumentParser(description="Mundo de Gazebo del campo de pruebas real")
    p.add_argument("lat", type=float)
    p.add_argument("lon", type=float)
    p.add_argument("--elevacion", type=float, default=0.0, help="altitud del despegue [m]")
    p.add_argument("--nombre", default="campo_pruebas")
    args = p.parse_args()

    modelo = os.path.join(RAIZ, "models", args.nombre)
    for sub in ("meshes", "materials/textures"):
        os.makedirs(os.path.join(modelo, sub), exist_ok=True)
    x, y = Transformer.from_crs("EPSG:4326", "EPSG:25830", always_xy=True).transform(args.lon, args.lat)
    print(f"despegue: UTM30N {x:.1f} E, {y:.1f} N")

    descargar_ortofotos(x, y, os.path.join(modelo, "materials", "textures"))
    lado, px, _ = CAPAS["detalle"]
    arboles = detectar_arboles(os.path.join(modelo, "materials", "textures", "detalle.jpg"),
                               lado / px, tipo_campo_pruebas)
    # Arboleda del horizonte: árboles de la ortofoto de contexto alrededor del detalle
    lado_c, px_c, _ = CAPAS["contexto"]
    arboles += detectar_arboles(os.path.join(modelo, "materials", "textures", "contexto.jpg"),
                                lado_c / px_c, lambda x, y: "roble", paso_m=12.0, area_min=30.0,
                                excluir=lado / 2, radio=600.0)
    with open(os.path.join(modelo, "arboles.json"), "w") as f:
        json.dump(arboles, f, indent=1)
    print("árboles:", {k: sum(t["tipo"] == k for t in arboles) for k in ("roble", "eucalipto", "arbusto")})

    escribir_suelo(os.path.join(modelo, "meshes"))
    copiar_arboles(modelo)
    escribir_modelo(args.nombre, modelo, arboles, args.lat, args.lon)

    hoy = datetime.date.today()
    año = hoy.year + (hoy.timetuple().tm_yday - 1) / 365.25
    m = GeoMag(coefficients_file="wmm/WMM_2025.COF").calculate(
        glat=args.lat, glon=args.lon, alt=args.elevacion / 1000, time=año)
    campo = (m.y * 1e-9, m.x * 1e-9, m.z * 1e-9)   # este, norte, abajo [T]
    print(f"campo magnético: D {m.d:.2f}°, I {m.i:.2f}°, F {m.f:.0f} nT")

    os.makedirs(os.path.join(RAIZ, "worlds"), exist_ok=True)
    escribir_mundo(args.nombre, os.path.join(RAIZ, "worlds", f"{args.nombre}.sdf"),
                   args.lat, args.lon, args.elevacion, campo)
    print(f"mundo: simulation/worlds/{args.nombre}.sdf")


if __name__ == "__main__":
    main()
