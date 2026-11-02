"""Render world for the mission video: the test site plus two cameras, the drone and its props.

usage: mundo_render.py simulation/worlds/campo_pruebas.sdf render.sdf

The world is renamed "render", steps 1/30 s per update (one video frame) and runs only when
render_mision.py steps it. cam_main is a small wide view, cam_seg the 1080p camera that
render_mision.py places as the fixed or the follow camera. Needs modelos_render/ in
GZ_SIM_RESOURCE_PATH.
"""
import sys

ORIGEN, DESTINO = sys.argv[1], sys.argv[2]
s = open(ORIGEN).read()
s = s.replace('<world name="campo_pruebas">', '<world name="render">')
s = s.replace("<max_step_size>0.004</max_step_size>", "<max_step_size>0.0333333333</max_step_size>")
s = s.replace("<real_time_update_rate>250</real_time_update_rate>", "<real_time_update_rate>0</real_time_update_rate>")
s = s.replace("      <shadows>true</shadows>\n    </scene>",
              "      <shadows>true</shadows>\n"
              "      <sky><clouds><speed>0</speed><humidity>0.35</humidity><mean_size>0.4</mean_size></clouds></sky>\n"
              "    </scene>")


def camara(nombre, hfov, ancho, alto):
    return f"""
    <model name="{nombre}"><static>true</static><pose>2 2 1.5 0 0 0</pose>
      <link name="l"><sensor name="s" type="camera"><topic>/{nombre}</topic><update_rate>60</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{ancho}</width><height>{alto}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>3000</far></clip></camera></sensor></link></model>"""


extra = camara("cam_main", 0.95, 320, 180) + camara("cam_seg", 0.8, 1920, 1080) + """
    <include><name>dron</name><uri>model://dron_render</uri><static>true</static><pose>0 0 0.16 0 0 0</pose></include>
    <include><name>h0</name><uri>model://helice_ccw</uri><pose>0.1228 -0.1228 0.16 0 0 0</pose></include>
    <include><name>h1</name><uri>model://helice_ccw</uri><pose>-0.1228 0.1228 0.16 0 0 0</pose></include>
    <include><name>h2</name><uri>model://helice_cw</uri><pose>0.1228 0.1228 0.16 0 0 0</pose></include>
    <include><name>h3</name><uri>model://helice_cw</uri><pose>-0.1228 -0.1228 0.16 0 0 0</pose></include>
"""
open(DESTINO, "w").write(s.replace("  </world>", extra + "  </world>"))
print("mundo de render en", DESTINO)
