#!/usr/bin/env python3
"""
Rutina de pick and place para el UR5e usando pymoveit2.
RBTC - Programa 3
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as robot

# ---------------------------------------------------------------------------
# Definicion de las rutinas. Cada una es:
#   pick:   punto de recogida (x, y, z) en metros, respecto a base_link
#   place:  punto de llegada
#   altura_segura: cuanto se sube por encima de pick/place para los puntos
#                  intermedios (evita golpear la mesa al viajar)
# Orientacion fija: herramienta mirando hacia abajo.
# ---------------------------------------------------------------------------
QUAT_ABAJO = [1.0, 0.0, 0.0, 0.0]  # [x, y, z, w] -- ajustar si no es la que necesitas

RUTINAS = [
    {
        "pick":  [0.40,  0.30, 0.05],
        "place": [0.50,  0.00, 0.05],
        "altura_segura": 0.20,
    },
    {
        "pick":  [0.40, -0.30, 0.05],
        "place": [0.50,  0.15, 0.05],
        "altura_segura": 0.20,
    },
]

PAUSA_EN_PUNTOS_INTERMEDIOS = 0.5  # segundos


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("rbtc_pick_place_node")

        cb_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cb_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # "Efector final" simulado: True = cerrado/agarrando, False = abierto
        self.gripper_pub = self.create_publisher(Bool, "gripper_cmd", 10)

        # Entrada digital simulada para repetir el programa
        self._digital_input_alto = False
        self.create_subscription(
            Bool, "digital_input", self._cb_entrada_digital, 10, callback_group=cb_group
        )

        self._contador_payload = 0

    def _cb_entrada_digital(self, msg: Bool):
        if msg.data:
            self._digital_input_alto = True

    def mover_a(self, xyz, etiqueta=""):
        self.get_logger().info(f"Moviendo a {etiqueta}: {xyz}")
        self.moveit2.move_to_pose(position=xyz, quat_xyzw=QUAT_ABAJO, cartesian=False)
        self.moveit2.wait_until_executed()

    def activar_efector(self, cerrar: bool, xyz_objeto=None):
        msg = Bool()
        msg.data = cerrar
        self.gripper_pub.publish(msg)

        if cerrar:
            # Simula tomar el objeto: agrega una caja y la adjunta al efector
            nombre = f"payload_{self._contador_payload}"
            self._nombre_payload_actual = nombre
            self.moveit2.add_collision_box(
                id=nombre,
                size=[0.04, 0.04, 0.04],
                position=xyz_objeto,
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            )
            self.moveit2.attach_collision_object(id=nombre)
            self.get_logger().info(f"Efector CERRADO, payload '{nombre}' tomado")
        else:
            # Simula soltar el objeto
            nombre = getattr(self, "_nombre_payload_actual", None)
            if nombre is not None:
                self.moveit2.detach_collision_object(id=nombre)
                self.moveit2.remove_collision_object(id=nombre)
                self.get_logger().info(f"Efector ABIERTO, payload '{nombre}' liberado")
            self._contador_payload += 1

    def ejecutar_rutina(self, rutina, indice):
        pick = rutina["pick"]
        place = rutina["place"]
        h = rutina["altura_segura"]

        sobre_pick = [pick[0], pick[1], pick[0] * 0 + pick[2] + h]
        sobre_place = [place[0], place[1], place[2] + h]

        self.get_logger().info(f"--- Rutina {indice}: pick {pick} -> place {place} ---")

        self.mover_a(sobre_pick, "sobre el punto de recogida")
        time.sleep(PAUSA_EN_PUNTOS_INTERMEDIOS)

        self.mover_a(pick, "punto de recogida")
        self.activar_efector(cerrar=True, xyz_objeto=pick)

        self.mover_a(sobre_pick, "sobre el punto de recogida (subiendo)")
        time.sleep(PAUSA_EN_PUNTOS_INTERMEDIOS)

        self.mover_a(sobre_place, "sobre el punto de llegada")
        time.sleep(PAUSA_EN_PUNTOS_INTERMEDIOS)

        self.mover_a(place, "punto de llegada")
        self.activar_efector(cerrar=False)

        self.mover_a(sobre_place, "sobre el punto de llegada (subiendo)")

    def esperar_entrada_digital(self):
        self.get_logger().info(
            "Rutinas terminadas. Esperando /digital_input en alto para repetir..."
        )
        self._digital_input_alto = False
        while not self._digital_input_alto:
            time.sleep(0.1)

    def ciclo_principal(self):
        while rclpy.ok():
            for i, rutina in enumerate(RUTINAS, start=1):
                self.ejecutar_rutina(rutina, i)
            self.esperar_entrada_digital()


def main():
    rclpy.init()
    nodo = PickAndPlace()

    hilo_ejecutor = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo_ejecutor.start()

    try:
        nodo.ciclo_principal()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
