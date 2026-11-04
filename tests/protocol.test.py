"""BP2F packet checks without importing pyserial or opening a device."""
from pathlib import Path
import runpy,json
ROOT=Path(__file__).resolve().parents[1]
bridge=runpy.run_path(str(ROOT/'host/mobile-distance/bridge.py'),run_name='protocol_fixture')
crc8=bridge['crc8'];packet=bridge['distance_packet']
assert crc8(b'123456789')==0xf4
for seq in [0,1,127,255]:
    for mm in [500,625,750]:
        data=packet(mm,seq)
        assert len(data)==8 and data[:2]==bytes([0xa5,0x5a])
        assert data[2]==seq and data[3]==0x20 and int.from_bytes(data[4:6],'little')==mm and data[6]==0
        assert crc8(data[:7])==data[7]
        corrupted=bytearray(data);corrupted[4]^=1
        assert crc8(corrupted[:7])!=corrupted[7]
print(json.dumps({'passed':True,'CRC8_known_vector':True,'cases':12,'serial_opened':False}))
