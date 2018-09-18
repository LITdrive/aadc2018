# OpenDRIVE parser

## Usage

```python
from lxml import etree
from opendriveparser import parse_opendrive

fh = open("input_opendrive.xodr", 'r')
openDrive = parse_opendrive(etree.parse(fh).getroot())
fh.close()

# Now do stuff with the data
for road in openDrive.roads:
    print("Road ID: {}".format(road.id))
```

## Author

Stefan Urban