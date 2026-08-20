#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
esp_bridge.py  •  Единый WebSocket: LiDAR + Odometry (из ESP) + /cmd_vel (в ESP)  (ROS 2 Jazzy)

* WebSocket от ESP32  →  /scan   (sensor_msgs/LaserScan)
*                     →  /odom + TF odom→base_link  (nav_msgs/Odometry)
* WebSocket  ← /cmd_vel (geometry_msgs/Twist), бинарный фрейм 4 байта (int16 L, int16 R)

ИЗМЕНЕНИЯ vs предыдущей версии:
  - Убран HTTP-поллинг /state (create_timer(0.10, poll_state)). Это было
    узким местом: пересоздание TCP-соединения на каждый запрос забивало
    пул PCB на ESP32 и давало деградацию частоты (10 Гц → 3 Гц) и рваную
    одометрию.
  - Одометрия теперь приходит тем же WebSocket-потоком, что и лидар,
    с частотой, заданной прошивкой (ODOM_PERIOD_MS на ESP32, по
    умолчанию 50 Гц), без HTTP round-trip на каждый цикл.
  - Пакеты от ESP различаются по 1-байтовому префиксу типа:
      0x01 = одометрия, payload = 5 x float32 LE (x, y, th, speedL_mm, speedR_mm)
      0x02 = скан лидара, payload = как раньше (N x 20 байт кадров)
    Оба пакета: [тип][payload][CRC16 Modbus от (тип+payload), 2 байта LE]
  - Исправлен баг: linear.x раньше брался только с левого колеса
    (js["speed"]["left"]), теперь — среднее по обоим колёсам.
  - /cmd_vel больше НЕ шлётся через HTTP GET /setSpeed. Прошивка уже
    умела принимать бинарные WS-фреймы длиной 4 байта (int16 L, int16 R) —
    теперь нода их и отправляет, через тот же persistent WS-сокет, что
    принимает скан+одометрию. Это убирает HTTP полностью из горячего
    пути в обе стороны: нет накладных расходов заголовков HTTP, нет
    риска исчерпания TCP PCB на ESP32 даже при частой публикации
    /cmd_vel (навигация/телеop на 10-30+ Гц).

Дополнительные параметры конфигурируют датчик:

  lidar_yaw_deg     (double, ° CW) – повернуть облако
  lidar_mirror      (bool)         – отразить (лево↔право)
  lidar_xyz         (double[3])    – смещение лидара в базе  [м]
  lidar_rpy_deg     (double[3])    – ориентация лидара       [° R P Y]

При старте узел публикует *статический* трансформ base_link→laser,
поэтому RViz/ Nav2 сразу видят полную цепочку `odom → base_link → laser`.
"""

import asyncio
import math
import struct
import threading
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformStamped,
)
import websockets

# ――― константы LDS-пакетов ―――――――――――――――――――――――――――――――――――――――――――――――
FRAME_FMT = "<HHHHHHHHHH"          # 20 B – угол начала, угол конца, 8×дистанция
FRAME_SZ = 20
MAX_PKTS = 64                      # макс. пакетов на оборот

# ――― константы протокола WS (тип пакета) ――――――――――――――――――――――――――――――――
ODOM_PKT_TYPE = 0x01
SCAN_PKT_TYPE = 0x02
ODOM_FMT = "<5f"                   # x, y, th, speedL(mm/s), speedR(mm/s)
ODOM_SZ = struct.calcsize(ODOM_FMT)

CMD_FMT = "<hh"                    # left, right — int16 LE (в ESP: L затем R)

BASE_MM = 600.0                     # база робота (колёс), мм


def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    """Считает CRC-16 (алгоритм Modbus, полином 0xA001) по всей строке байт.

    Args:
        buf: байты, по которым считается контрольная сумма.
        crc: начальное значение аккумулятора (по умолчанию 0xFFFF —
             стандартный старт для Modbus CRC-16).

    Returns:
        Итоговое 16-битное значение CRC.

    Должен быть побитово идентичен реализации crc16() на стороне
    прошивки ESP32 — иначе все входящие пакеты будут браковаться как
    повреждённые (см. _handle_packet). Используется как для проверки
    входящих пакетов (скан/одометрия), так и мог бы использоваться для
    исходящих (сейчас команды /cmd_vel отправляются без CRC — см.
    cmd_cb/_send_cmd).
    """
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def q_yaw(yaw: float) -> Quaternion:
    """Строит кватернион поворота ТОЛЬКО вокруг оси Z (курс/yaw).

    Args:
        yaw: угол курса в радианах.

    Returns:
        geometry_msgs/Quaternion с ненулевыми только z и w (roll и
        pitch считаются нулевыми) — этого достаточно для 2D-одометрии
        колёсного робота, который не наклоняется/не переворачивается.

    Используется в publish_odom() для заполнения ориентации в /odom
    и в связанном с ним TF odom→base_link.
    """
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Полное преобразование углов Эйлера (roll, pitch, yaw) в кватернион.

    Args:
        roll: крен вокруг оси X, в радианах.
        pitch: тангаж вокруг оси Y, в радианах.
        yaw: курс вокруг оси Z, в радианах.

    Returns:
        geometry_msgs/Quaternion, эквивалентный повороту на данные
        roll/pitch/yaw (порядок вращений — стандартный ZYX intrinsic).

    В отличие от q_yaw() (только курс), эта функция нужна там, где
    датчик/база могут быть смонтированы под произвольным углом в
    пространстве — используется в _publish_static_tf() для трансформа
    base_link→laser, заданного параметром lidar_rpy_deg, и для
    (тождественного) трансформа map→odom.
    """
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


# ―――― основной узел ―――――――――――――――――――――――――――――――――――――――――――――――――――
class ESPBridge(Node):
    def __init__(self) -> None:
        """Конструктор ноды. Вызывается один раз при старте (rclpy.spin(ESPBridge())).

        Что делает, по порядку:
          1. Объявляет ROS-параметры (host/порт/путь WS, имена фреймов,
             диапазоны лидара, параметры монтажа лидара на раме) —
             все они настраиваемы через YAML/launch-файл или CLI
             (--ros-args -p name:=value), реальные значения читаются
             через self.get_parameter(...).value в местах использования.
          2. Собирает URL WebSocket (self.ws_url) из host/ws_port/ws_path.
          3. Создаёт паблишеры /scan и /odom, подписку на /cmd_vel.
          4. Инициализирует self.ws = None — сюда позже (в ws_task)
             будет записан живой объект соединения, как только он
             установится; используется в cmd_cb для отправки команд.
          5. Публикует статические трансформы base_link→laser и
             map→odom (см. _publish_static_tf) — публикуются один раз
             и остаются актуальными (latched) на весь срок жизни ноды.
          6. Запускает отдельный поток с собственным asyncio event loop
             (self.ws_loop) для WebSocket-обмена — это нужно, чтобы
             asyncio-корутины (websockets.connect/recv/send) не
             блокировали основной executor rclpy, который крутит
             колбэки (cmd_cb) и таймеры в своём потоке.
        """
        super().__init__("esp_bridge")

        # ── параметры подключения ──────────────────────────────────
        self.declare_parameter("host", "192.168.0.105")
        self.declare_parameter("ws_port", 80)
        self.declare_parameter("ws_path", "/ws")

        # ── имена фреймов и диапазон LiDAR-a ───────────────────────
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 16.0)

        # ── настройка ориентации LiDAR-а ───────────────────────────
        self.declare_parameter("lidar_yaw_deg", 180.0)        # сдвиг облака CW
        self.declare_parameter("lidar_mirror", True)          # зеркально (True/False)
        self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.10]) # положение на раме
        self.declare_parameter("lidar_rpy_deg", [0.0, 0.0, 0.0])  # ориентация

        # ── формируем URL WebSocket ――
        host = self.get_parameter("host").value
        port = self.get_parameter("ws_port").value
        path = self.get_parameter("ws_path").value
        self.ws_url = f"ws://{host}:{port}{path}"

        # ── ROS паблишеры/сабскрайберы ────────────────────────────
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        # активное WS-соединение (устанавливается/сбрасывается в ws_task,
        # используется из cmd_cb для отправки команд бинарным фреймом)
        self.ws = None

        # динамический TF (odom→base_link) + статический (base_link→laser)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # ― запускаем отдельный event-loop для WebSocket (лидар + одометрия) ―
        self.ws_loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._run_ws_loop, args=(self.ws_loop,), daemon=True
        ).start()

        # HTTP /state больше не опрашивается — одометрия идёт через WS.
        # (poll_state удалён вместе с create_timer(0.10, self.poll_state))

        self.get_logger().info(f"LiDAR+Odom WebSocket →  {self.ws_url}")

    # =============================================================
    #                 статический TF  base_link → laser
    # =============================================================
    def _publish_static_tf(self) -> None:
        """Публикует статические (latched) трансформы, вызывается один раз из __init__.

        Не принимает и не возвращает ничего — работает через
        self.static_br.sendTransform().

        Публикует два независимых трансформа:
          1. base_link → laser — положение и ориентация лидара
             относительно базы робота, заданные параметрами
             lidar_xyz (смещение в метрах) и lidar_rpy_deg (углы
             Эйлера в градусах, переводятся в радианы и в кватернион
             через rpy_to_quat). Это то, что позволяет RViz/Nav2
             корректно спроецировать облако точек лидара (кадр
             "laser") в систему координат робота.
          2. map → odom — тождественный (нулевой) трансформ. Нужен,
             чтобы полная цепочка TF map→odom→base_link→laser была
             валидна сразу после старта, даже если в системе ещё нет
             отдельного узла локализации (AMCL/SLAM), который обычно
             и владеет трансформом map→odom в полноценном стеке Nav2.
             Если такой узел появится — эту публикацию стоит убрать,
             иначе возникнет конфликт (два источника одного трансформа).

        Оба трансформа публикуются через StaticTransformBroadcaster —
        это значит, что они "залатчены" и новые подписчики (например,
        только что запущенный RViz) получат их сразу, без ожидания
        следующей публикации.
        """
        xyz = self.get_parameter("lidar_xyz").value
        rpy_deg = self.get_parameter("lidar_rpy_deg").value

        roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
        q = rpy_to_quat(roll, pitch, yaw)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter("base_frame").value
        tf.child_frame_id = self.get_parameter("frame_id").value
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation = q

        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF base_link → laser")

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = rpy_to_quat(0.0, 0.0, 0.0)

        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF map → odom")

    # =============================================================
    #          WebSocket (единый поток: скан + одометрия + команды)
    # =============================================================
    def _run_ws_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Точка входа отдельного потока: запускает asyncio event loop навсегда.

        Args:
            loop: новый event loop, созданный в __init__
                  (asyncio.new_event_loop()) специально для этого потока.

        Вызывается как target= у threading.Thread(daemon=True) в __init__.
        Делает loop "текущим" для этого потока (asyncio.set_event_loop)
        и запускает на нём ws_task() до его завершения — на практике
        ws_task() завершается только когда rclpy.ok() становится False
        (т.е. при остановке ноды), поэтому этот метод фактически живёт
        всё время работы ноды.
        Поток daemon=True — значит, он не помешает процессу завершиться,
        если основной поток (rclpy.spin) закончит работу.
        """
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ws_task())

    async def ws_task(self) -> None:
        """Основная корутина: держит WS-соединение с ESP32 и разбирает входящие пакеты.

        Крутится в цикле, пока rclpy.ok() (т.е. пока нода жива):
          1. Пытается установить WS-соединение по self.ws_url.
          2. При успехе — отключает алгоритм Нейгла на сокете
             (TCP_NODELAY), логирует подключение, сохраняет объект
             соединения в self.ws (чтобы cmd_cb мог им пользоваться
             для отправки команд из другого потока).
          3. В цикле читает входящие бинарные фреймы (await ws.recv())
             и передаёт их на разбор в _handle_packet(). Нечисловые
             (текстовые) фреймы игнорируются.
          4. Если соединение обрывается или не удаётся установить —
             ловит исключение, логирует предупреждение, ждёт 1 секунду
             и пробует переподключиться (внешний while).
          5. В блоке finally всегда обнуляет self.ws — это важно:
             без этого cmd_cb мог бы попытаться слать данные в уже
             закрытый/невалидный объект соединения между моментом
             обрыва связи и следующим успешным reconnect'ом.

        Ничего не принимает и не возвращает — работает через побочные
        эффекты (публикация ROS-сообщений внутри _handle_packet →
        publish_scan/publish_odom, обновление self.ws).
        """
        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.ws_url, max_size=None, ping_interval=None
                ) as ws:
                    sock: socket.socket = ws.transport.get_extra_info("socket")
                    if sock:
                        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.get_logger().info("WS connected")
                    self.ws = ws

                    while rclpy.ok():
                        buf = await ws.recv()
                        if not isinstance(buf, (bytes, bytearray)):
                            continue
                        self._handle_packet(buf)

            except Exception as e:
                self.get_logger().warn(f"WS error: {e}")
                await asyncio.sleep(1.0)
            finally:
                self.ws = None

    def _handle_packet(self, buf: bytes) -> None:
        """Разбирает один входящий бинарный WS-пакет от ESP32 и маршрутизирует его.

        Args:
            buf: сырые байты одного WS-фрейма, как есть от ws.recv().
                 Ожидаемый формат (задан прошивкой): [1 байт тип]
                 [payload][2 байта CRC16 LE от (тип+payload)].

        Returns:
            None. Побочный эффект — вызов publish_scan()/publish_odom()
            при валидном пакете, либо просто выход (пакет отброшен)
            при слишком коротком буфере или несовпадении CRC.

        Логика:
          1. Если пакет короче 3 байт (1 тип + 2 CRC минимум) — не
             может быть валидным, выходим сразу.
          2. Разделяем на pkt_type (первый байт), payload (всё между
             типом и CRC) и crc_recv (последние 2 байта, little-endian).
          3. Пересчитываем CRC16 по всем байтам ДО контрольной суммы
             (buf[:-2], т.е. тип+payload) и сравниваем с полученной.
             Несовпадение — типичный признак повреждения при передаче
             по Wi-Fi; пакет отбрасывается с предупреждением в лог,
             ничего не публикуется (лучше пропустить один скан/кадр
             одометрии, чем опубликовать мусор).
          4. По типу пакета вызываем соответствующий обработчик:
             SCAN_PKT_TYPE → publish_scan(payload),
             ODOM_PKT_TYPE → publish_odom(payload).
             Любой другой (пока не существующий) тип молча
             игнорируется — сделано специально, чтобы в будущем можно
             было добавить новые типы пакетов без поломки совместимости
             со старой версией ноды.
        """
        if len(buf) < 1 + 2:
            return

        pkt_type = buf[0]
        payload = buf[1:-2]
        crc_recv = int.from_bytes(buf[-2:], "little")

        if crc16(buf[:-2]) != crc_recv:
            self.get_logger().warn("CRC mismatch, dropping packet")
            return

        if pkt_type == SCAN_PKT_TYPE:
            self.publish_scan(payload)
        elif pkt_type == ODOM_PKT_TYPE:
            self.publish_odom(payload)
        # неизвестные типы молча игнорируем (совместимость на будущее)

    # =============================================================
    #          /cmd_vel  →  бинарный WS-фрейм (4 байта: int16 L, int16 R)
    # =============================================================
    def cmd_cb(self, msg: Twist) -> None:
        """Колбэк подписки на /cmd_vel — превращает Twist в команду для ESP32.

        Args:
            msg: geometry_msgs/Twist. Используются только msg.linear.x
                 (линейная скорость вперёд, м/с) и msg.angular.z
                 (угловая скорость, рад/с) — остальные поля (linear.y/z,
                 angular.x/y) игнорируются, т.к. это дифференциальный
                 привод (двумя колёсами), а не омниколёсная база.

        Returns:
            None. Побочный эффект — постановка задачи на отправку
            бинарного фрейма в очередь asyncio event loop'а (ws_loop).

        Логика:
          1. Переводит linear.x из м/с в мм/с (v_mm) — прошивка ожидает
             целевые скорости именно в мм/с (см. tgtL/tgtR на ESP).
          2. Раскладывает linear+angular в скорости левого/правого
             колёс по формуле дифференциального привода:
               left  = v_mm - w * BASE_MM / 2
               right = v_mm + w * BASE_MM / 2
             (BASE_MM — база робота в мм, см. константу выше).
          3. Обрезает left/right в диапазон int16 ([-32768, 32767]) —
             защита от переполнения при формировании бинарного пакета
             (struct.pack с форматом "h" бросит исключение на выходе
             за эти границы).
          4. Если WS-соединение сейчас не установлено (self.ws is None,
             например, идёт переподключение после обрыва связи) —
             просто пропускает этот кадр команды. Ничего страшного:
             прошивка сама обнулит скорости по CMD_TIMEOUT (100 мс),
             если новые команды не будут приходить.
          5. Иначе — упаковывает left/right в 4 байта (CMD_FMT = "<hh")
             и планирует их отправку через _send_cmd() в event loop'е
             WS-потока (asyncio.run_coroutine_threadsafe — безопасный
             способ вызвать корутину из другого потока/loop'а, т.к.
             сам cmd_cb выполняется в потоке ROS-executor'а, а не в
             asyncio-потоке).
        """
        v_mm = msg.linear.x * 1000.0
        w = msg.angular.z
        left = int(v_mm - w * BASE_MM / 2.0)
        right = int(v_mm + w * BASE_MM / 2.0)
        left = max(-32768, min(32767, left))
        right = max(-32768, min(32767, right))

        if self.ws is None:
            # соединение ещё не установлено/переустанавливается — просто
            # пропускаем кадр, CMD_TIMEOUT на ESP сам обнулит скорость,
            # если команды не будет идти достаточно долго
            return

        payload = struct.pack(CMD_FMT, left, right)
        # cmd_cb вызывается в потоке ROS-executor'а, а ws живёт в
        # отдельном asyncio event loop — отправляем потокобезопасно
        asyncio.run_coroutine_threadsafe(self._send_cmd(payload), self.ws_loop)

    async def _send_cmd(self, payload: bytes) -> None:
        """Корутина, реально отправляющая бинарный фрейм команды в сокет.

        Args:
            payload: уже упакованные 4 байта (int16 left, int16 right),
                     сформированные в cmd_cb через struct.pack(CMD_FMT, ...).

        Returns:
            None. Выполняется ВНУТРИ event loop'а WS-потока (запущена
            через run_coroutine_threadsafe из cmd_cb) — поэтому здесь
            безопасно напрямую обращаться к объекту WS-соединения.

        Отдельная маленькая корутина нужна именно потому, что cmd_cb —
        обычная (синхронная) функция-колбэк ROS, а операция отправки
        в WS (ws.send) — асинхронная и должна выполняться в "своём"
        event loop'е (self.ws_loop), а не там, где её вызвали.
        Если отправка не удалась (например, соединение как раз оборвалось
        между проверкой self.ws is None в cmd_cb и этим моментом) —
        ошибка ловится и логируется как предупреждение, не роняя ноду.
        """
        ws = self.ws
        if ws is None:
            return
        try:
            await ws.send(payload)
        except Exception as e:
            self.get_logger().warn(f"cmd send fail: {e}")

    # =============================================================
    #               преобразуем пакеты LDS → LaserScan
    # =============================================================
    def publish_scan(self, raw: bytes) -> None:
        """Собирает один полный оборот лидара из бинарных кадров и публикует /scan.

        Args:
            raw: payload SCAN-пакета (уже без типа и CRC) — последовательность
                 20-байтовых кадров, каждый кадр = struct FRAME_FMT
                 (start_angle_raw, end_angle_raw, 8 x distance_raw),
                 сформированных прошивкой в lidarTask().

        Returns:
            None. Побочный эффект — публикация sensor_msgs/LaserScan
            в топик /scan (self.scan_pub.publish(...)). Если данных
            слишком мало для осмысленного скана (pkt_cnt < 38, то есть
            меньше ~270° покрытия) — просто выходит без публикации.

        Как собирается скан из кадров:
          1. Каждый кадр содержит начальный и конечный угол (в сотых
             долях градуса, переводятся в float делением на 100) и
             8 замеров дистанции с равномерным шагом внутри этого
             углового интервала.
          2. Дистанция 0 в кадре — это "невалидное измерение" (в
             прошивке degradируется по quality < INTENSITY_MIN),
             переводится в float("inf") — стандартный способ сообщить
             ROS'у "измерения нет" в LaserScan.
          3. Углы аккумулируются с "разворачиванием" через переменную
             off — компенсация перехода через 0°/360° между кадрами
             (лидар крутится непрерывно, а мы получаем углы по модулю
             360°, поэтому нужно отслеживать полные обороты).
          4. Если включён параметр lidar_mirror — все три списка
             (углы/дистанции/интенсивности) разворачиваются, а знак
             углов меняется — так компенсируется зеркальное
             (обратное) физическое направление вращения/монтажа лидара.
          5. Применяется дополнительный поворот на lidar_yaw_deg
             (компенсация ориентации монтажа лидара на раме).
          6. Углы нормализуются так, чтобы первый угол был <= 180° —
             иначе angle_min/angle_max в LaserScan получились бы
             "перевёрнутыми" относительно стандартного диапазона.
          7. Формируется и публикуется LaserScan с вычисленными
             angle_min/angle_max/angle_increment (по факту собранных
             данных, не по константам) и списками ranges/intensities.
        """
        pkt_cnt = len(raw) // FRAME_SZ
        if pkt_cnt < 38:  # < ~270°
            return

        ang, rng, inten = [], [], []
        prev = None
        off = 0.0

        for i in range(pkt_cnt):
            s, e, *dist = struct.unpack_from(FRAME_FMT, raw, i * FRAME_SZ)
            s /= 100.0
            e /= 100.0
            if e < s:
                e += 360.0

            for j, d in enumerate(dist):
                a = s + (e - s) * j / 7
                if prev is not None and a + off < prev - 300:
                    off += 360.0
                a += off
                prev = a

                ang.append(a)
                rng.append(d / 1000.0 if d else float("inf"))
                inten.append(1.0 if d else 0.0)

        if self.get_parameter("lidar_mirror").value:
            ang.reverse()
            rng.reverse()
            inten.reverse()
            ang = [-a for a in ang]

        yaw_deg = self.get_parameter("lidar_yaw_deg").value
        if yaw_deg:
            ang = [a + yaw_deg for a in ang]

        if ang[0] > 180.0:
            ang = [a - 360.0 for a in ang]

        rad = [math.radians(a) for a in ang]

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.get_parameter("frame_id").value
        scan.angle_min = rad[0]
        scan.angle_max = rad[-1]
        scan.angle_increment = (rad[-1] - rad[0]) / (len(rad) - 1)
        scan.range_min = self.get_parameter("range_min").value
        scan.range_max = self.get_parameter("range_max").value
        scan.ranges = rng
        scan.intensities = inten
        self.scan_pub.publish(scan)

    # =============================================================
    #          пакет одометрии (WS push) → /odom + TF
    # =============================================================
    def publish_odom(self, raw: bytes) -> None:
        """Разбирает пакет одометрии от ESP32 и публикует /odom + TF odom→base_link.

        Args:
            raw: payload ODOM-пакета (уже без типа и CRC) — ровно
                 ODOM_SZ байт, формат ODOM_FMT = "<5f":
                 (x, y, th, speedL_mm, speedR_mm). x/y в метрах,
                 th в радианах, speedL/speedR в мм/с — соответствует
                 тому, что шлёт sendOdomPacket() на стороне прошивки.

        Returns:
            None. Побочные эффекты:
              - публикация nav_msgs/Odometry в топик /odom
              - публикация динамического TF odom→base_link через
                self.tf_br.sendTransform(...)
            Если длина raw не совпадает с ожидаемой ODOM_SZ (признак
            рассинхрона протокола или битого пакета, который каким-то
            образом прошёл проверку CRC) — публикация пропускается,
            в лог пишется предупреждение.

        Расчёт twist (скоростей):
          - linear.x = среднее арифметическое скоростей левого и
            правого колеса (speed_left_m + speed_right_m) / 2 —
            стандартная кинематика дифференциального привода.
            ВАЖНО: раньше здесь была ошибка — бралась скорость только
            левого колеса, из-за чего при поворотах (когда left и right
            различаются) linear.x был неверным.
          - angular.z = (speed_right_mm - speed_left_mm) / BASE_MM —
            разница скоростей колёс, делённая на базу робота (в мм,
            т.к. speed_*_mm тоже в мм) — даёт угловую скорость в рад/с
            (для малых углов это стандартное приближение diff-drive).

        stamp (od.header.stamp) берётся как ТЕКУЩЕЕ время получения
        пакета нодой (self.get_clock().now()), а не время, когда
        одометрия была реально посчитана на ESP32 — на ESP32 сейчас
        нет синхронизированных с ROS часов/своей метки времени в
        пакете. При высокой и стабильной частоте WS-пуша (50 Гц)
        разница обычно пренебрежимо мала, но это стоит иметь в виду
        при точной работе с временными синхронизациями (например,
        EKF из robot_localization может быть чувствителен к этому).
        """
        if len(raw) != ODOM_SZ:
            self.get_logger().warn(f"Odom packet size mismatch: {len(raw)} != {ODOM_SZ}")
            return

        x, y, th, speed_left_mm, speed_right_mm = struct.unpack(ODOM_FMT, raw)

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self.get_parameter("odom_frame").value
        od.child_frame_id = self.get_parameter("base_frame").value
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation = q_yaw(th)

        # ИСПРАВЛЕНО: раньше linear.x бралось только с левого колеса.
        # Теперь — среднее по обоим, как и положено для дифференциального привода.
        speed_left_m = speed_left_mm / 1000.0
        speed_right_m = speed_right_mm / 1000.0
        od.twist.twist.linear.x = (speed_left_m + speed_right_m) / 2.0
        od.twist.twist.angular.z = (speed_right_mm - speed_left_mm) / BASE_MM

        self.odom_pub.publish(od)

        tf = TransformStamped()
        tf.header.stamp = od.header.stamp
        tf.header.frame_id = od.header.frame_id
        tf.child_frame_id = od.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = od.pose.pose.orientation
        self.tf_br.sendTransform(tf)


# ――― точка входа ――――――――――――――――――――――――――――――――――――――――――――――――――
def main(args=None) -> None:
    """Стандартная точка входа ROS2-ноды (регистрируется в setup.py как entry_point).

    Args:
        args: аргументы командной строки, передаваемые в rclpy.init()
              (обычно None — rclpy сам разбирает sys.argv).

    Returns:
        None. Блокируется на rclpy.spin(...) до остановки ноды
        (Ctrl+C, сигнал завершения от launch-системы и т.п.), после
        чего корректно завершает работу клиента ROS (rclpy.shutdown()).

    Порядок:
      1. rclpy.init(args) — инициализация клиентской библиотеки ROS2.
      2. Создаётся ESPBridge() — при этом уже в конструкторе
         запускается фоновый поток с WS-соединением (см. __init__).
      3. rclpy.spin(...) — основной цикл обработки колбэков/таймеров
         ROS в текущем (главном) потоке; WS-обмен продолжает жить
         параллельно в своём собственном потоке/event loop'е.
      4. rclpy.shutdown() — вызывается после выхода из spin (нода
         остановлена), освобождает ресурсы клиента ROS2.
    """
    rclpy.init(args=args)
    rclpy.spin(ESPBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
