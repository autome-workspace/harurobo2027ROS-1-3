"""Correlated, read-only fault-detail requests; never clear or write history."""
FAULTS = {
    1: '保存データの形式・CRC異常',
    2: 'CANバスオフ復帰履歴',
    8: '計数サンプリング遅延',
    9: 'CAN受信キュー溢れ',
    10: '設定保存・照合失敗',
    11: 'E-stop受信履歴',
}


class FaultReader:
    def __init__(self, interval=1., timeout=.2, max_age=1.5):
        self.interval, self.timeout, self.max_age = interval, timeout, max_age
        self.sequence = 0
        self.reset()

    def reset(self):
        self.pending = None
        self.details = {}
        self.last_request = {}
        self.bits = {}

    def observe(self, channel, fault):
        if self.bits.get(channel) != fault:
            self.details.pop(channel, None)
        self.bits[channel] = fault

    def request(self, channels, now):
        if self.pending is not None:
            if now-self.pending[2] <= self.timeout:
                return None
            self.pending = None
        eligible = [ch for ch in channels if self.bits.get(ch) and
                    now-self.last_request.get(ch, float('-inf')) >= self.interval]
        if not eligible:
            return None
        channel = min(eligible, key=lambda ch:self.last_request.get(ch, float('-inf')))
        self.sequence = (self.sequence+1) & 0xff
        self.pending = channel, self.sequence, now
        self.last_request[channel] = now
        # Operation 0, item 5, no write/clear/save/reset operation.
        return bytes([channel, 0, 5, 0, 0, 0, 0, self.sequence])

    def failed(self):
        if self.pending is not None:
            self.details.pop(self.pending[0], None)
        self.pending = None

    def reply(self, data, now):
        if self.pending is None or len(data) != 8:
            return False
        channel, sequence, sent = self.pending
        if (data[0] != channel or data[1] >> 6 != 0 or data[2] != 5 or data[7] != sequence
                or not 0 <= now-sent <= self.timeout):
            return False
        self.pending = None
        if data[1] & 63:
            self.details.pop(channel, None)
        else:
            self.details[channel] = int.from_bytes(data[3:7], 'big'), now
        return True

    def describe(self, channel, fault, now):
        if not fault:
            return {'fault_detail':0, 'fault_detail_age':0., 'fault_level':'ok', 'fault_text':'受信正常'}
        detail = self.details.get(channel)
        if detail is None or not 0 <= now-detail[1] <= self.max_age or detail[0] == 0:
            return {'fault_detail':None, 'fault_detail_age':None,
                    'fault_level':'warning', 'fault_text':'異常履歴あり（詳細確認中）'}
        mask, at = detail
        if mask == 1:
            level, text = 'info', '再起動履歴のみ（受信正常）'
        else:
            labels = ['再起動履歴'] if mask & 1 else []
            labels += [label for bit,label in FAULTS.items() if mask & (1<<bit)]
            unknown = mask & ~sum(1<<bit for bit in (0,*FAULTS))
            if unknown:
                labels.append(f'未定義異常bit 0x{unknown:08X}')
            level, text = 'warning', '・'.join(labels)+f'（0x{mask:08X}）'
        return {'fault_detail':mask, 'fault_detail_age':now-at, 'fault_level':level, 'fault_text':text}
