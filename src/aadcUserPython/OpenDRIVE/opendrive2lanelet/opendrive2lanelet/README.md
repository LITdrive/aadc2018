# OpenDRIVE 2 Lanelets - Converter

## Requirements

- Python 3.x
- numpy
- opendriveparser

## Usage

```python
from lxml import etree
from opendriveparser import parse_opendrive
from opendrive2lanelet import Network

fh = open("input_opendrive.xodr", 'r')
openDrive = parse_opendrive(etree.parse(fh).getroot())
fh.close()

roadNetwork = Network()
roadNetwork.loadOpenDrive(openDrive)

scenario = roadNetwork.exportCommonRoadScenario()

# Write CommonRoad scenario to file
fh = open("output_commonroad_file.xml", "wb")
fh.write(scenario.export_to_string())
fh.close()
```

## Author

Stefan Urban