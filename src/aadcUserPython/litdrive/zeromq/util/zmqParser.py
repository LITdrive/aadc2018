"""
    zmqParser
    Parses from and to ZMQ
    Author: Philipp Seidl
    date: 2018.09.20

"""

fDict = {'tJuryStruct': 'bh',
 'tDriverStruct': 'hh',
 'tSignalValue': 'If',
 'tBoolSignalValue': 'I?',
 'tWheelData': 'IIb',
 'tInerMeasUnitData': 'Ifffffffff',
 'tRoadSignExt': 'hfff',
 'tPosition': 'fffff',
 'tObstacle': 'ff',
 'tTrafficSign': 'hfff',
 'tParkingSpace': 'hffH',
 'tUltrasonicStruct': 'IfIfIfIfIf',
 'tVoltageStruct': 'IfIfIfIfIfIfIfIfIfIf',
 'tPolarCoordiante': 'ff',
 'tLaserScannerData': 'Iff',
 'tClassification': 'sQd',
 'tVirtualPoint': 'dddd'}
 
cNameDict = {'tJuryStruct': ['i16ActionID', 'i16ManeuverEntry'],
 'tDriverStruct': ['i16StateID', 'i16ManeuverEntry'],
 'tSignalValue': ['ui32ArduinoTimestamp', 'f32Value'],
 'tBoolSignalValue': ['ui32ArduinoTimestamp', 'bValue'],
 'tWheelData': ['ui32ArduinoTimestamp', 'ui32WheelTach', 'i8WheelDir'],
 'tInerMeasUnitData': ['ui32ArduinoTimestamp',
  'f32A_x',
  'f32A_y',
  'f32A_z',
  'f32G_x',
  'f32G_y',
  'f32G_z',
  'f32M_x',
  'f32M_y',
  'f32M_z'],
 'tRoadSignExt': ['i16Identifier', 'f32Imagesize', 'af32TVec', 'af32RVec'],
 'tPosition': ['f32x', 'f32y', 'f32radius', 'f32speed', 'f32heading'],
 'tObstacle': ['f32x', 'f32y'],
 'tTrafficSign': ['i16Identifier', 'f32x', 'f32y', 'f32angle'],
 'tParkingSpace': ['i16Identifier', 'f32x', 'f32y', 'ui16Status'],
 'tUltrasonicStruct': ['ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value'],
 'tVoltageStruct': ['ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value',
  'ui32ArduinoTimestamp',
  'f32Value'],
 'tPolarCoordiante': ['f32Radius', 'f32Angle'],
 'tLaserScannerData': ['ui32Size', 'f32Radius', 'f32Angle'],
 'tClassification': ['className', 'classId', 'probValue'],
 'tVirtualPoint': ['f64x', 'f64y', 'f64Heading', 'f64Speed']}

import struct

def parseFromZMQ(data, data_types):
    """
        Parses the ZMQ data given the data-types and the data in form of a list
        Returns a list
    """
    parsed_data = []
    for i, data_type in enumerate(data_types):
        parsed_data.append( struct.unpack(fDict[data_type], data[i]) )
    return parsed_data

def parseToZMQ(data, data_type):
    """
        Parses one data_type to a 
        Returns
    """
    parsed_data = struct.pack(fDict[data_type], *data)
    return parsed_data

def parseToZMQMultiple(data, data_types):
    """
        Parses multiple ... too lazy to write doc
    """
    parsed_data=[]
    for i, data_type in enumerate(data_types):
        parsed_data.append( parseToZMQ(data[i], data_type))
    return parsed_data
    
def getColumnNames(data_type):
    return cNameDict[data_type]

def getColumnNamesMultiple(data_types):
    cNames = []
    for data_type in data_types:
        cNames.append(getColumnNames(data_type))
    return cNames