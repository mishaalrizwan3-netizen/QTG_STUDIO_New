"""Read-only BU0836X Interface 2 HID axes for throttle alignment testing."""
import math
import threading
import time

USAGES = {'throttle': 0x35, 'propeller': 0x36, 'mixture': 0x37}


def percentages(raw, limits):
    """Preserve the physical axis direction and expose throttle demand separately."""
    result = {}
    for name in USAGES:
        low, high = limits[name]
        value = float(raw[name])
        if not math.isfinite(value) or high <= low or not low <= value <= high:
            raise ValueError('Invalid BU0836X ' + name + ' value or range')
        result[name + '_raw'] = value
        result[name + '_pct'] = 100. * (value - low) / (high - low)
    result['throttle_demand_pct'] = 100. - result['throttle_pct']
    return result


class Reader:
    def __init__(self, *, devices=None, clock=time.monotonic):
        try:
            import pywinusb.hid as hid
            from pywinusb.hid.core import HidP_Input
        except ImportError as exc:
            raise RuntimeError('BU0836X reading requires pywinusb on the Sim PC') from exc
        self.clock, self.lock = clock, threading.Lock()
        self.latest = None
        candidates = devices if devices is not None else hid.find_all_hid_devices()
        matches = [d for d in candidates if 'bu0836' in (d.product_name or '').lower()
                   and 'interface 2' in (d.product_name or '').lower()]
        if len(matches) != 1:
            raise RuntimeError(f'Expected one BU0836X Interface 2, found {len(matches)}')
        self.device = matches[0]
        self.device.open()
        try:
            reports = self.device.find_input_reports()
            wanted = {(1 << 16) | usage for usage in USAGES.values()}
            matches = [report for report in reports if wanted.issubset(set(report.keys()))]
            if len(matches) != 1:
                raise RuntimeError('BU0836X Interface 2 does not expose a unique Rz, Slider and Dial report')
            self.report = matches[0]
            limits = {}
            for name, usage in USAGES.items():
                caps = [cap for cap in self.device.usages_storage.get(HidP_Input, [])
                        if cap.report_id == self.report.report_id and cap.is_value
                        and cap.usage_page == 1 and ((not cap.is_range and cap.usage == usage)
                            or (cap.is_range and cap.usage_min <= usage <= cap.usage_max))]
                if len(caps) != 1 or caps[0].logical_max <= caps[0].logical_min:
                    raise RuntimeError('BU0836X ' + name + ' logical range is unavailable')
                limits[name] = (caps[0].logical_min, caps[0].logical_max)
            self.limits = limits
            self.device.set_raw_data_handler(self._receive)
        except Exception:
            self.device.close()
            raise

    def _receive(self, data):
        if not data or data[0] != self.report.report_id:
            return
        try:
            self.report.set_raw_data(data)
            raw = {name: self.report[(1 << 16) | usage].value for name, usage in USAGES.items()}
            value = percentages(raw, self.limits)
            with self.lock:
                self.latest = dict(value, receipt_time=self.clock())
        except Exception:
            with self.lock:
                self.latest = None

    def poll(self):
        if not self.device.is_plugged():
            raise RuntimeError('BU0836X Interface 2 disconnected')
        with self.lock:
            return dict(self.latest) if self.latest is not None else None

    def close(self):
        self.device.close()
