from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, ChoicesSetting

# Define commands for ADXL367
READ_REGISTER_CMD = 0x0B
WRITE_REGISTER_CMD = 0x0A
READ_FIFO_CMD = 0x0D

COMMANDS = {
    READ_REGISTER_CMD: "Read Register",
    WRITE_REGISTER_CMD: "Write Register",
    READ_FIFO_CMD: "Read FIFO"
}

# Correct Offset Registers for ADXL367
OFFSET_REGISTERS = {
    0x33: "X Offset",
    0x34: "Y Offset",
    0x35: "Z Offset"
}

class FakeFrame:
    def __init__(self, t, time=None):
        self.type = t
        self.start_time = time
        self.end_time = time
        self.data = {}

class ADXL367Analyzer(HighLevelAnalyzer):
    decode_level = ChoicesSetting(choices=('Everything', 'Only Data', 'Only Errors', 'Only Control'))

    result_types = {
        'read_register': {
            'format': 'Read Register 0x{{data.register}}: {{data.value}}'
        },
        'write_register': {
            'format': 'Write Register 0x{{data.register}}: {{data.value}}'
        },
        'read_fifo': {
            'format': 'Read FIFO: 0x{{data.value}}'
        },
        'set_offset': {
            'format': 'Set {{data.offset_axis}}: 0x{{data.value}}'
        },
        'error': {
            'format': 'Error: {{data.error}}'
        }
    }

    def __init__(self):
        self._start_time = None
        self._mosi_data = None
        self._miso_data = None
        self._empty_result_count = 0

    def decode(self, frame: AnalyzerFrame):
        frames = []
        if frame.type == "data":
            data = frame.data["data"]
            cs = data >> 15
            if self._last_time:
                diff = frame.start_time - self._last_time
            else:
                diff = self._fastest_cs
            diff = float(diff * 1_000_000_000)

            self._fastest_cs = min(diff * 4, self._fastest_cs)
            if diff > self._fastest_cs and cs == 0:
                if self._transaction > 0:
                    frames.append(FakeFrame("disable", self._last_time))

                frames.append(FakeFrame("enable", frame.start_time))

                self._transaction += 1
                self._clock_count = 0
                if not self._continuous:
                    self._command = 0
                    self._quad_start = None
                    self._dummy = 0

                    self._mosi_out = 0
                    self._miso_in = 0
                else:
                    self._clock_count = 8
                    f = FakeFrame("result")
                    f.data["mosi"] = [self._command]
                    f.data["miso"] = [0]
                    frames.append(f)

            self._last_time = frame.start_time

            if cs == 1:
                return None

            if self._quad_start is None or self._clock_count < self._quad_start:
                self._mosi_out = self._mosi_out << 1 | (data & 0x1)
                self._miso_in = self._miso_in << 1 | ((data >> 1) & 0x1)
                if self._clock_count % 8 == 7:
                    if self._clock_count == 7:
                        self._command = self._mosi_out

                    f = FakeFrame("result")
                    f.data["mosi"] = [self._mosi_out]
                    f.data["miso"] = [self._miso_in]
                    frames.append(f)
                    self._mosi_out = 0
                    self._miso_in = 0
            else:
                self._quad_data = (self._quad_data << 4 | data & 0xf)
                if self._clock_count % 2 == 1:
                    f = FakeFrame("result")
                    f.data["mosi"] = [self._quad_data]
                    f.data["miso"] = [self._quad_data]
                    frames.append(f)
                    self._quad_data = 0

            self._clock_count += 1
        else:
            print("non data!")
            frames = [frame]

        output = None
        for fake_frame in frames:
            if fake_frame.type == "enable":
                self._start_time = fake_frame.start_time
                self._miso_data = bytearray()
                self._mosi_data = bytearray()
            elif fake_frame.type == "result":
                if self._miso_data is None or self._mosi_data is None:
                    if self._empty_result_count == 0:
                        print(fake_frame)
                    self._empty_result_count += 1
                    continue
                self._miso_data.extend(fake_frame.data["miso"])
                self._mosi_data.extend(fake_frame.data["mosi"])
            elif fake_frame.type == "disable":
                if not self._miso_data or not self._mosi_data:
                    continue
                command = self._mosi_data[0]
                frame_type = None
                frame_data = {"command": command}
                if command in COMMANDS:
                    if command == READ_REGISTER_CMD:
                        if len(self._mosi_data) > 1:
                            register = self._mosi_data[1]
                            if register in [0x00, 0x01, 0x02] and len(self._miso_data) > 2:
                                value = self._miso_data[2]
                                frame_data["register"] = f'{register:02X}'
                                frame_data["value"] = f'{value:02X}'
                                frame_type = "read_device_id"
                            else:
                                frame_data["register"] = f'{register:02X}'
                                frame_data["value"] = f'{self._miso_data[2]:02X}' if len(self._miso_data) > 2 else "N/A"
                                frame_type = "read_register"

                    elif command == WRITE_REGISTER_CMD:
                        if len(self._mosi_data) > 2:
                            register = self._mosi_data[1]
                            value = self._mosi_data[2]
                            frame_data["register"] = f'{register:02X}'
                            frame_data["value"] = f'{value:02X}'
                            if register in OFFSET_REGISTERS:
                                frame_data["offset_axis"] = OFFSET_REGISTERS[register]
                                frame_type = "set_offset"
                            else:
                                frame_type = "write_register"

                    elif command == READ_FIFO_CMD:
                        if len(self._miso_data) > 1:
                            value = self._miso_data[1]
                            frame_data["value"] = f'{value:02X}'
                            frame_type = "read_fifo"

                if frame_type is None:
                    frame_type = "error"
                    frame_data = {"error": "Unknown command"}

                if frame_type:
                    our_frame = AnalyzerFrame(frame_type,
                                              self._start_time,
                                              fake_frame.end_time,
                                              frame_data)
                self._miso_data = None
                self._mosi_data = None
                if self.decode_level == 'Only Data' and frame_type == "control_command":
                    continue
                if self.decode_level == 'Only Errors' and frame_type != "error":
                    continue
                if self.decode_level == "Only Control" and frame_type != "control_command":
                    continue
                output = our_frame
        return output
