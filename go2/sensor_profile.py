"""我方实机传感器的参数与可复现扫描方向；安装外参暂沿用原位置。"""
import json
from pathlib import Path
import numpy as np

PROFILE_PATH = Path(__file__).resolve().parents[1] / 'cfg/our_go2_sensors.json'
PROFILE = json.loads(PROFILE_PATH.read_text())

def mid360_directions(scan_index=0):
    """按黄金比例相位生成非固定线束扫描近似，不冒充厂商专有扫描轨迹。"""
    cfg = PROFILE['lidar']
    count = int(cfg['points_per_second'] / cfg['scan_hz'])
    i = np.arange(count, dtype=np.float64) + scan_index * count
    azimuth = ((i * 0.6180339887498949) % 1.0) * 360.0 - 180.0
    elevation = -7.0 + ((i * 0.4142135623730951) % 1.0) * 59.0
    return azimuth.astype(np.float32), elevation.astype(np.float32)

def write_mid360_usda(path):
    """生成独立本地 RTX OmniLidar 资产，无需修改 conda/Isaac 扩展。"""
    az, el = mid360_directions()
    count = len(az)
    def array(values):
        return '[' + ','.join(str(round(float(x), 6)) for x in values) + ']'
    lines = ['#usda 1.0', '(defaultPrim = "MID360")',
             'def OmniLidar "MID360" (prepend apiSchemas = ["OmniSensorGenericLidarCoreAPI"])', '{']
    values = {
        'float nearRangeM': .1, 'float farRangeM': 70., 'uint maxReturns': 1,
        'float avgPowerW': .002, 'float minReflectance': .1,
        'float focusDistM': .05, 'uint pulseTimeNs': 6,
        'float rangeAccuracyM': 0., 'float rangeResolutionM': .001,
        'float azimuthErrorMean': 0., 'float azimuthErrorStd': 0.,
        'float elevationErrorMean': 0., 'float elevationErrorStd': 0.,
        'uint scanRateBaseHz': 10, 'uint reportRateBaseHz': 10,
        'uint numberOfEmitters': count, 'uint numberOfChannels': count,
        'uint rangeCount': 1, 'uint numLines': 1, 'uint stateResolutionStep': 1,
        'bool skipDroppingInvalidPoints': 'false',
        'token scanType': '"SOLID_STATE"', 'token rayType': '"IDEALIZED"',
        'token intensityMappingType': '"LINEAR"', 'token intensityProcessing': '"NORMALIZATION"',
        'token rotationDirection': '"CW"', 'token outputFrameOfReference': '"SENSOR"',
    }
    for key, value in values.items():
        typ, name = key.split()
        lines.append(f'    {typ} omni:sensor:Core:{name} = {value}')
    lines.append(f'    uint[] omni:sensor:Core:numRaysPerLine = [{count}]')
    for typ, name, values in [
        ('float','azimuthDeg',az), ('float','elevationDeg',el),
        ('uint','fireTimeNs',np.arange(count,dtype=np.int64)*5000),
        ('uint','channelId',np.arange(count)+1), ('uint','bank',np.zeros(count,dtype=int)),
        ('uint','rangeId',np.zeros(count,dtype=int)),
        ('float','distanceCorrectionM',np.zeros(count)), ('float','focalDistM',np.zeros(count)),
        ('float','focalSlope',np.zeros(count)), ('float','horOffsetM',np.zeros(count)),
        ('float','vertOffsetM',np.zeros(count)), ('float','reportRateDiv',np.ones(count))]:
        text = '['+','.join(str(int(x)) for x in values)+']' if typ == 'uint' else array(values)
        lines.append(f'    {typ}[] omni:sensor:Core:emitterState:s001:{name} = {text}')
    lines.extend(['    string omni:sensor:modelName = "OurGo2_MID360_coverage_approximation"', '}'])
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text('\n'.join(lines)+'\n')
