import time
import math
import argparse
import struct

from inavmspapi import MultirotorControl, TCPTransmitter
from agrotechsimapi import SimClient


def get_altitude(sim_client):
    try:
        kin_data = sim_client.get_kinametics_data()
        if kin_data and "location" in kin_data:
            return float(kin_data["location"][2])
    except Exception:
        pass
    return None


def get_physics_xy(sim_client):
    try:
        kin_data = sim_client.get_kinametics_data()
        if kin_data and "location" in kin_data:
            return float(kin_data["location"][0]), float(kin_data["location"][1])
    except Exception:
        pass
    return None, None


def get_yaw(sim_client):
    try:
        kin = sim_client.get_kinametics_data()
        qx, qy, qz, qw = kin["orientation"]
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception:
        return 0.0


def get_gps(control):
    try:
        if control.send_RAW_msg(MultirotorControl.MSPCodes["MSP_RAW_GPS"], data=[]):
            dataHandler = control.receive_msg()
            control.process_recv_data(dataHandler)
            lat = control.GPS_DATA["lat"]
            lon = control.GPS_DATA["lon"]
            if lat != 0 and lon != 0:
                return lat, lon
    except Exception:
        pass
    return None, None


# Константы змейки
SNAKE_START_LAT = 454276736
SNAKE_START_LON = 396638176
SNAKE_RIGHT_LON = 396587360
SNAKE_END_LAT = 454412384  # ← изменено
SNAKE_END_LON = 396585696
SNAKE_LAT_STEP = 3000


def build_snake_waypoints(
    home_lat,
    home_lon,
    alt_cm,
    speed,
    resume_lat=None,
    resume_lon=None,
    recharge_every=7,
):
    START_LON = SNAKE_START_LON
    RIGHT_LON = SNAKE_RIGHT_LON
    END_LAT = SNAKE_END_LAT
    LAT_STEP = SNAKE_LAT_STEP

    if resume_lat is None:
        resume_lat = SNAKE_START_LAT
    if resume_lon is None:
        resume_lon = SNAKE_START_LON

    wps = []
    wp_num = 1
    left_hits = 0
    lat = resume_lat
    on_left = resume_lon == START_LON

    wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
    wp_num += 1

    wps.append([wp_num, 1, resume_lat, resume_lon, alt_cm, speed, 0, 0, 0])
    wp_num += 1

    while lat <= END_LAT:
        if on_left:
            wps.append([wp_num, 1, lat, RIGHT_LON, alt_cm, speed, 0, 0, 0])
            wp_num += 1

            lat += LAT_STEP
            wps.append([wp_num, 1, lat, RIGHT_LON, alt_cm, speed, 0, 0, 0])
            wp_num += 1

            if lat > END_LAT:
                wps.append(
                    [wp_num, 1, SNAKE_END_LAT, SNAKE_END_LON, alt_cm, speed, 0, 0, 0]
                )
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
                return wps, lat, RIGHT_LON, True

            wps.append([wp_num, 1, lat, START_LON, alt_cm, speed, 0, 0, 0])
            wp_num += 1
            left_hits += 1

            if left_hits >= recharge_every:
                wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
                return wps, lat + LAT_STEP, START_LON, False

            lat += LAT_STEP
            wps.append([wp_num, 1, lat, START_LON, alt_cm, speed, 0, 0, 0])
            wp_num += 1
            left_hits += 1

            if lat > END_LAT:
                wps.append(
                    [wp_num, 1, SNAKE_END_LAT, SNAKE_END_LON, alt_cm, speed, 0, 0, 0]
                )
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
                return wps, lat, START_LON, True

            if left_hits >= recharge_every:
                wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
                return wps, lat, START_LON, False

        else:
            lat += LAT_STEP
            wps.append([wp_num, 1, lat, RIGHT_LON, alt_cm, speed, 0, 0, 0])
            wp_num += 1

            if lat > END_LAT:
                wps.append(
                    [wp_num, 1, SNAKE_END_LAT, SNAKE_END_LON, alt_cm, speed, 0, 0, 0]
                )
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
                return wps, lat, RIGHT_LON, True

            wps.append([wp_num, 1, lat, START_LON, alt_cm, speed, 0, 0, 0])
            wp_num += 1
            left_hits += 1
            on_left = True

            if left_hits >= recharge_every:
                wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
                wp_num += 1
                wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
                return wps, lat + LAT_STEP, START_LON, False

    wps.append([wp_num, 1, SNAKE_END_LAT, SNAKE_END_LON, alt_cm, speed, 0, 0, 0])
    wp_num += 1
    wps.append([wp_num, 1, home_lat, home_lon, alt_cm, speed, 0, 0, 0])
    wp_num += 1
    wps.append([wp_num, 1, home_lat, home_lon, 0, 0, 0, 0, 165])
    return wps, lat, START_LON, True


def send_waypoints_in_batches(control, waypoints):
    print(f"\nЗАГРУЗКА МАРШРУТА ({len(waypoints)} точек):")
    for wp in waypoints:
        binary_data = struct.pack("<BBIIIHHHB", *wp)
        control.send_RAW_msg(MultirotorControl.MSPCodes["MSP_SET_WP"], binary_data)
        time.sleep(0.5)
        print(
            f"  WP{wp[0]:2d} | lat={wp[2]} lon={wp[3]} | alt={wp[4]/100:.0f}м | флаг={wp[8]}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inav_host", type=str, default="127.0.0.1")
    parser.add_argument("--inav_port", type=int, default=5762)
    parser.add_argument(
        "--recharge_every", type=int, default=7, help="Количество полос до посадки"
    )
    args = parser.parse_args()

    tcp_transmitter = TCPTransmitter((args.inav_host, args.inav_port))
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)
    sim_client = SimClient(address=args.inav_host, port=8080)

    print("=" * 60)
    print("СТАРТ МИССИИ — РЕЖИМ ЗМЕЙКИ")
    print("=" * 60)

    start_x, start_y = None, None
    for _ in range(10):
        start_x, start_y = get_physics_xy(sim_client)
        if start_x is not None:
            break
        time.sleep(0.5)

    if start_x is None:
        print("  Ошибка получения физики. Выход.")
        return

    print("\nСЧИТЫВАНИЕ КООРДИНАТ БАЗЫ...")
    home_lat, home_lon = None, None
    for attempt in range(10):
        home_lat, home_lon = get_gps(control)
        if home_lat is not None:
            print(f"  База: lat={home_lat}, lon={home_lon}")
            break
        print(f"  Попытка {attempt + 1}/10...")
        time.sleep(1.0)

    if home_lat is None:
        print("  Не удалось получить GPS. Завершение.")
        tcp_transmitter.disconnect()
        return

    ALT_FLY_CM = 5000
    SPEED = 2000
    resume_lat = None
    resume_lon = None
    segment_num = 1

    print(f"\nЗМЕЙКА:")
    print(f"  Перезарядка каждые {args.recharge_every} полос")
    print(f"  Диапазон lat: {SNAKE_START_LAT} → {SNAKE_END_LAT}")
    print(f"  Конечная точка: lat={SNAKE_END_LAT}, lon={SNAKE_END_LON}")

    while True:
        print(f"\n{'='*60}")
        if segment_num == 1:
            print("ПЕРВЫЙ СЕГМЕНТ — ЗАГРУЗКА МАРШРУТА")
        else:
            print(
                f"СЕГМЕНТ #{segment_num} | возобновление: lat={resume_lat}, lon={resume_lon}"
            )
        print(f"{'='*60}")

        # DISARM перед загрузкой
        control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000])
        control.receive_msg()
        time.sleep(1.0)

        waypoints, next_lat, next_lon, finished = build_snake_waypoints(
            home_lat,
            home_lon,
            ALT_FLY_CM,
            SPEED,
            resume_lat,
            resume_lon,
            args.recharge_every,
        )

        print(f"  Точек в сегменте: {len(waypoints)}")
        send_waypoints_in_batches(control, waypoints)

        print("\nИНИЦИАЛИЗАЦИЯ...")
        control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000])
        control.receive_msg()
        time.sleep(1.0)

        control.send_RAW_RC([1500, 1500, 1000, 1500, 2000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.5)
        print("  Автопилот активирован")

        print("\nВЗЛЁТ (цель: 50м)...")
        control.send_RAW_RC([1500, 1500, 1900, 1500, 2000, 1000, 1000])
        control.receive_msg()

        takeoff_start = time.time()
        while time.time() - takeoff_start < 30:
            altitude = get_altitude(sim_client)
            if altitude is not None:
                print(f"\r  Высота: {altitude:.1f} м / 50 м", end="")
                if altitude >= 45.0:
                    print(f"\n  Высота {altitude:.1f} м достигнута")
                    break
                elif altitude >= 20.0:
                    control.send_RAW_RC([1500, 1500, 1750, 1500, 2000, 1000, 1000])
                    control.receive_msg()
            time.sleep(0.1)
        else:
            print(f"\n  Таймаут взлёта, продолжаем...")

        control.send_RAW_RC([1500, 1500, 1500, 1500, 2000, 1000, 1000])
        control.receive_msg()

        print("\nЗАПУСК МИССИИ...")
        control.send_RAW_RC([1500, 1500, 1500, 1500, 2000, 1000, 2000])
        control.receive_msg()
        mission_start = time.time()
        print("  Миссия запущена!")

        precise_landing = False
        mission_complete = False

        print("\nПОЛЁТ (ЗМЕЙКА):")

        while not mission_complete:
            elapsed = time.time() - mission_start
            altitude = get_altitude(sim_client)

            if not precise_landing:
                if altitude is not None and altitude < 15.0 and elapsed > 20:
                    print(f"\n\nПЕРЕХВАТ: начинаю точную посадку... ({elapsed:.0f}с)")
                    precise_landing = True
                    control.send_RAW_RC([1500, 1500, 1500, 1500, 2000, 1000, 1500])
                    control.receive_msg()
                    time.sleep(0.5)
                else:
                    alt_str = f"{altitude:.1f}м" if altitude else "н/д"
                    mins = int(elapsed // 60)
                    secs = int(elapsed % 60)
                    print(f"\r  {mins:02d}:{secs:02d} | Высота: {alt_str}", end="")

            else:
                curr_x, curr_y = get_physics_xy(sim_client)
                yaw = get_yaw(sim_client)

                if curr_x is not None and curr_y is not None:
                    dx = start_x - curr_x
                    dy = start_y - curr_y
                    err_x = math.cos(yaw) * dx + math.sin(yaw) * dy
                    err_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                    kP = 150
                    rc_pitch = max(1350, min(1650, int(1500 + err_x * kP)))
                    rc_roll = max(1350, min(1650, int(1500 - err_y * kP)))
                    dist_to_center = math.hypot(dx, dy)
                    if dist_to_center < 0.5:
                        rc_throttle = (
                            1250 if altitude is not None and altitude > 5.0 else 1320
                        )
                    else:
                        rc_throttle = 1500
                    control.send_RAW_RC(
                        [rc_roll, rc_pitch, rc_throttle, 1500, 2000, 1000, 1500]
                    )
                    print(
                        f"\r  Посадка | Откл: {dist_to_center:.2f}м | Высота: {altitude:.2f}м",
                        end="",
                    )

                control.receive_msg()

                if altitude is not None and altitude <= 0.35:
                    total = time.time() - mission_start
                    mins, secs = int(total // 60), int(total % 60)
                    print(f"\n\n  ПРИЗЕМЛИЛСЯ! Высота: {altitude:.2f}м")
                    print(f"  Время сегмента: {mins:02d}:{secs:02d}")
                    mission_complete = True

                time.sleep(0.05)
                continue

            if elapsed > 1800:
                print(f"\n  Общий таймаут миссии.")
                break
            time.sleep(0.5)

        if finished:
            print("\n" + "=" * 60)
            print("ВСЯ ЗМЕЙКА ЗАВЕРШЕНА!")
            break

        print(f"\nПЕРЕЗАРЯДКА #{segment_num}")
        print(f"  Сохранена позиция: lat={next_lat}, lon={next_lon}")
        resume_lat = next_lat
        resume_lon = next_lon
        segment_num += 1

    print("\n" + "=" * 60)
    print("ВЫКЛЮЧЕНИЕ ДВИГАТЕЛЕЙ")
    for _ in range(5):
        control.send_RAW_RC([1500, 1500, 900, 1500, 1000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.5)

    print("МИССИЯ ЗАВЕРШЕНА!")
    tcp_transmitter.disconnect()


if __name__ == "__main__":
    main()
