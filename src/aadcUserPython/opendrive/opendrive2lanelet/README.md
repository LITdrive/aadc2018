# OpenDRIVE 2 Lanelets - Converter

We provide the code for an OpenDRIVE ([www.opendrive.org](http://www.opendrive.org)) to lanelets ([www.mrt.kit.edu/software/liblanelet](https://www.mrt.kit.edu/software/libLanelet/libLanelet.html)) converter, which has been introduced in our [paper](https://mediatum.ub.tum.de/doc/1449005/1449005.pdf): M. Althoff, S. Urban, and M. Koschi, "Automatic Conversion of Road Networks from OpenDRIVE to Lanelets," in Proc. of the IEEE International Conference on Service Operations and Logistics, and Informatics, 2018.

## Installation

The following python packages have to be available:
- Python 3.x
- numpy
- scipy
- lxml
- PyQt5 only for GUI

If needed, the converter libraries can be installed using ```pip```:

```bash
git clone https://gitlab.com/commonroad/commonroad.gitlab.io.git
cd tools/ opendrive2lanelet && pip install .
```

## Example OpenDRIVE Files

Download example files from: http://opendrive.org/download.html

## Usage

### Using our provided GUI

Additional requirement: PyQt5. Start the GUI with ```python gui.py```

![GUI screenshot](gui_screenshot.png "Screenshot of converter GUI")

### Using the library in your own scripts

```python
from lxml import etree
from opendriveparser import parse_opendrive
from opendrive2lanelet import Network

# Import, parse and convert OpenDRIVE file
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


## Known Problems

- When trying to use the gui.py under Wayland, the following error occurs:
  ```
  This application failed to start because it could not find or load the Qt platform plugin "wayland" in "".
  Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
  Reinstalling the application may fix this problem.
  ```
  Set the platform to *xcb* using this command: ```export QT_QPA_PLATFORM="xcb"```

## ToDo

- When CommonRoad distinguishes lane types, the OpenDRIVE types have to be mapped accordingly and added to the lanelet output.

## Author

Stefan Urban