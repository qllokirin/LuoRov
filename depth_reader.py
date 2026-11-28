#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BPM-TTR / BPM-TTS 深度传感器（RS485 Modbus-RTU）实时深度读取脚本。

传感器接线：红线 VCC(实测必须 >=20V，推荐 24V，范围 20~32V，12V 无法工作！)、蓝线 GND、黄线 485A、白线 485B。
串口默认：9600, 8, N, 1，从站地址 1，功能码 03H。

寄存器（大端 IEEE-754 float，4 字节 / 2 个寄存器）：
    0x0000  主变量（压力/液位，单位由 0x000A 给出）
    0x0002  量程百分比 %
    0x0004  等效电流 mA
    0x0006  温度 C
    0x0008  传感器原始电桥值 mV
    0x000A  主变量单位代码（unsigned short，12 = kPa）

水深换算：h = (P - P0) * 1000 / (rho * g)，P/P0 单位 kPa，h 单位 m。

用法：
    python depth_reader.py                          # COM13，每 0.2 s 打印一次
    python depth_reader.py --zero                   # 启动时在空气中采 10 次取平均作为水面零点
    python depth_reader.py -p COM5 -b 9600 -i 0.5   # 换串口 / 波特率 / 打印间隔
    python depth_reader.py --rho 1025               # 海水密度（默认淡水 998）
    python depth_reader.py --count 5                # 只打印 5 次，便于自检
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

import serial

# 主变量单位代码 -> 换算到 kPa 的系数（见协议附录 1）
UNIT_FACTORS = {
    11: ("Pa", 0.001),
    12: ("kPa", 1.0),
    7: ("bar", 100.0),
    8: ("mbar", 0.1),
    237: ("MPa", 1000.0),
    240: ("mH2O", 9.80665),
    45: ("m", 9.80665),          # 以米水柱表示液位
    4: ("mmH2O", 0.00980665),
    48: ("cm", 0.0980665),       # 以厘米水柱表示液位
    49: ("mm", 0.00980665),      # 以毫米水柱表示液位
}
DEFAULT_UNIT_FACTOR = 1.0        # 未知单位时按 kPa 处理

REG_BLOCK_START = 0x0000
REG_BLOCK_COUNT = 11             # 0x0000 ~ 0x000A，一次读完所有动态量


def crc16_modbus(data: bytes) -> bytes:
    """标准 CRC-16/MODBUS，返回低字节在前的 2 字节。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


@dataclass
class Sample:
    pressure_kpa: float
    percent: float
    current_ma: float
    temp_c: float
    bridge_mv: float
    unit_code: int
    unit_name: str


class DepthSensor:
    """深度传感器 Modbus-RTU 客户端。"""

    def __init__(self, port: str, baud: int = 9600, slave: int = 1,
                 timeout: float = 0.3, retries: int = 2) -> None:
        self.port = port
        self.slave = slave
        self.timeout = timeout
        self.retries = retries
        self.ser = serial.Serial(
            port, baud, bytesize=8, parity="N", stopbits=1, timeout=timeout
        )
        time.sleep(0.2)                      # 等 USB-RS485 转换器稳定
        self.ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:                    # noqa: BLE001
            pass

    def _read_exact(self, size: int, deadline: float) -> bytes:
        buf = b""
        while len(buf) < size:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            self.ser.timeout = remain
            chunk = self.ser.read(size - len(buf))
            if not chunk:
                break
            buf += chunk
        return buf

    def _transact(self, addr: int, count: int) -> bytes:
        """发一条 03H 读保持寄存器请求，返回数据区（不含地址/功能码/字节数/CRC）。"""
        req = struct.pack(">BBHH", self.slave, 0x03, addr, count)
        req += crc16_modbus(req)
        expected = 5 + count * 2

        self.ser.reset_input_buffer()
        self.ser.write(req)
        deadline = time.monotonic() + self.timeout
        resp = self._read_exact(expected, deadline)

        if len(resp) >= 5 and resp[1] & 0x80:
            raise IOError(f"传感器异常响应 0x{resp[2]:02X}: {resp.hex(' ')}")
        if len(resp) != expected:
            raise TimeoutError(
                f"响应不完整：期望 {expected} 字节，收到 {len(resp)} 字节"
                + (f"（{resp.hex(' ')}）" if resp else "")
            )
        if resp[0] != self.slave:
            raise IOError(f"从站地址不匹配: {resp[0]}")
        if resp[1] != 0x03:
            raise IOError(f"功能码不匹配: 0x{resp[1]:02X}")
        if resp[2] != count * 2:
            raise IOError(f"字节数不匹配: {resp[2]}")
        if crc16_modbus(resp[:-2]) != resp[-2:]:
            raise ValueError(f"CRC 校验失败: {resp.hex(' ')}")
        return resp[3:-2]

    def _transact_retry(self, addr: int, count: int) -> bytes:
        last_err: Exception | None = None
        for _ in range(self.retries + 1):
            try:
                return self._transact(addr, count)
            except (TimeoutError, IOError, ValueError) as err:
                last_err = err
                time.sleep(0.05)
        raise last_err  # type: ignore[misc]

    def read_sample(self) -> Sample:
        data = self._transact_retry(REG_BLOCK_START, REG_BLOCK_COUNT)
        pv, pct, cur, temp, mv = struct.unpack(">fffff", data[:20])
        unit_code = struct.unpack(">H", data[20:22])[0]
        name, factor = UNIT_FACTORS.get(unit_code, ("?", DEFAULT_UNIT_FACTOR))
        return Sample(pv * factor, pct, cur, temp, mv, unit_code, name)


def fmt_depth(depth_m: float) -> str:
    return f"{depth_m:7.3f} m"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="BPM-TTR/BPM-TTS RS485 深度传感器实时深度打印",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", default="COM16", help="串口号")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="波特率")
    parser.add_argument("-s", "--slave", type=int, default=1, help="Modbus 从站地址")
    parser.add_argument("-i", "--interval", type=float, default=0.2, help="打印间隔（秒）")
    parser.add_argument("--rho", type=float, default=998.0, help="水体密度 kg/m^3（海水 1025）")
    parser.add_argument("--g", type=float, default=9.80665, help="重力加速度 m/s^2")
    parser.add_argument("--p0", type=float, default=0.0,
                        help="水面零点压力（kPa）；表压型传感器在空气中约为 0")
    parser.add_argument("--zero", nargs="?", type=int, const=10, default=None,
                        metavar="N", help="启动时采集 N 次平均值作为水面零点（默认 10 次）")
    parser.add_argument("--count", type=int, default=0, help="打印次数，0 表示一直运行")
    parser.add_argument("--timeout", type=float, default=0.3, help="单帧超时（秒）")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        sensor = DepthSensor(args.port, args.baud, args.slave, args.timeout)
    except serial.SerialException as err:
        print(f"无法打开串口 {args.port}：{err}", file=sys.stderr)
        return 1
    factor_kpa_per_m = args.rho * args.g / 1000.0   # kPa / m

    print(f"串口 {args.port} @ {args.baud} 8N1 | 从站 {args.slave} | "
          f"rho={args.rho:g} kg/m^3 | 每米水深 {factor_kpa_per_m:.4f} kPa")

    try:
        first = sensor.read_sample()
    except Exception as err:  # noqa: BLE001
        print(f"读取失败：{err}", file=sys.stderr)
        sensor.close()
        return 1

    p0 = args.p0
    if args.zero is not None:
        n = max(1, args.zero)
        values: list[float] = []
        for _ in range(n * 3):
            if len(values) >= n:
                break
            try:
                values.append(sensor.read_sample().pressure_kpa)
            except Exception as err:  # noqa: BLE001
                print(f"标定采样失败，重试：{err}", file=sys.stderr)
                time.sleep(0.05)
        if values:
            p0 = sum(values) / len(values)
            print(f"零点标定：{len(values)}/{n} 次平均 P0 = {p0:.4f} kPa")
    print(f"当前单位 {first.unit_name} (代码 {first.unit_code}) | 零点 P0 = {p0:.4f} kPa")
    print("-" * 78)

    count = 0
    fails = 0
    depths: list[float] = []
    start = time.monotonic()

    try:
        while args.count == 0 or count < args.count:
            tick = time.monotonic()
            try:
                sample = sensor.read_sample()
            except Exception as err:  # noqa: BLE001
                fails += 1
                print(f"[{time.strftime('%H:%M:%S')}] 读取失败({fails})：{err}")
                time.sleep(max(0.0, args.interval - (time.monotonic() - tick)))
                continue

            depth_m = (sample.pressure_kpa - p0) * 1000.0 / (args.rho * args.g)
            depths.append(depth_m)
            count += 1
            print(
                f"[{time.strftime('%H:%M:%S')}] 深度 {fmt_depth(depth_m)}"
                f" | 压力 {sample.pressure_kpa:9.3f} {sample.unit_name}"
                f" | 温度 {sample.temp_c:6.2f} C"
                f" | {sample.percent:6.2f} %"
                f" | {sample.current_ma:5.2f} mA"
            )
            time.sleep(max(0.0, args.interval - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        sensor.close()

    elapsed = time.monotonic() - start
    if depths:
        print("-" * 78)
        print(f"共成功 {count} 次 / 失败 {fails} 次，运行 {elapsed:.1f} s | "
              f"深度 min {min(depths):.3f} m, max {max(depths):.3f} m, "
              f"平均 {sum(depths) / len(depths):.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
