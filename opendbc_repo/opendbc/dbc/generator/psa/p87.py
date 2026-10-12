"""P87 uses the common AEE2010 messages with an eight-byte driver-pedal frame."""
from pathlib import Path
import re


def generate():
  common = (Path(__file__).resolve().parents[2] / 'psa_aee2010_r3.dbc').read_text()
  pedal = '''BO_ 552 Dyn5_CMM: 8 CMM___Motor_SG
 SG_ P334_ACCPed_Position : 16|8@1+ (0.5,0) [0|100] "%" Vector__XXX
 SG_ CHECKSUM : 24|4@1+ (1,0) [0|15] "" Vector__XXX
 SG_ COUNTER : 28|4@1+ (1,0) [0|15] "" Vector__XXX

'''
  content, count = re.subn(r'BO_ 552 Dyn5_CMM:.*?(?=BO_ )', pedal, common, flags=re.S)
  assert count == 1
  return {'psa_p87.dbc': content}
