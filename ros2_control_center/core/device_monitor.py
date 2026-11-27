from pathlib import Path


def devices():
    devices_input = Path('/sys/class/input')
    controller = any(
        name.is_file() and any(word in name.read_text(errors='replace').lower()
                               for word in ('wireless controller', 'dualshock', 'gamepad'))
        for name in devices_input.glob('event*/device/name')
    )
    return {
        'USB-CAN (can0)': Path('/sys/class/net/can0').exists(),
        'DM3519 / 足回り': None,
        'コントローラー': controller or any(Path('/dev/input').glob('js*')),
        'LiDAR・IMU 用シリアル': bool(list(Path('/dev/serial/by-id').glob('*')) or
                                 list(Path('/dev').glob('ttyACM*')) or list(Path('/dev').glob('ttyUSB*'))),
        'カメラ': any(Path('/dev').glob('video*')),
    }


def inventory():
    rows = []
    for path in sorted(Path('/sys/bus/usb/devices').glob('*')):
        fields = {}
        for field in ('idVendor', 'idProduct', 'product', 'serial'):
            try:
                fields[field] = (path / field).read_text(errors='replace').strip()
            except OSError:
                fields[field] = ''
        if fields['idVendor'] and fields['idVendor'] != '1d6b':
            rows.append(f"{fields['idVendor']}:{fields['idProduct']}  {fields['product']}  {fields['serial']}".strip())
    for directory in ('/dev/serial/by-id', '/dev/v4l/by-id', '/dev/input/by-id'):
        paths = sorted(Path(directory).glob('*'))
        rows.extend(f'{path} → {path.resolve()}' for path in paths if path.exists())
    return rows
