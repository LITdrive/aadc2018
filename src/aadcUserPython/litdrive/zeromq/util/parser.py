import struct

from os.path import dirname, join, abspath
from xml.etree import ElementTree

AADC_DESCRIPTION = abspath(join(dirname(__file__), r'../../../../../description/aadc.description'))

baseTypes = {
    'tBool': '?',  # bool (1 byte)
    'tChar': 'c',  # char (1 byte)
    'tUInt8': 'B',  # unsigned char (1 byte)
    'tInt8': 'b',  # signed char (1 byte)
    'tUInt16': 'H',  # unsigned short (2 bytes)
    'tInt16': 'h',  # signed short (2 bytes)
    'tUInt32': 'I',  # unsigned int (4 bytes)
    'tInt32': 'i',  # signed int (4 bytes)
    'tUInt64': 'Q',  # unsigned long (8 bytes)
    'tInt64': 'q',  # signed long (8 bytes)
    'tFloat32': 'f',  # float (4 bytes)
    'tFloat64': 'd'  # double (8 bytes)
}


def _read_structs(nodes):
    """ Read the <struct> xml node """
    data = {}

    # read struct definitions
    for node in nodes:
        elements = [(e.attrib['name'], e.attrib['type']) for e in node]
        data[node.attrib['name']] = elements

    def __resolve_nested_definition(elements, i):
        name_, type_ = elements[i]
        if type_ not in baseTypes:
            resolved = data[type_].copy()
            elements[i] = {name_: resolved}
            for j in range(len(resolved)):
                __resolve_nested_definition(resolved, j)

    # resolve nested definitions
    for name, elements in data.items():
        for i in range(len(elements)):
            __resolve_nested_definition(elements, i)

    return data


def _build_format_string(node):
    """ Build the format string for struct.pack() and struct.unpack() """
    fmt = ""

    for element in node:
        if isinstance(element, dict):
            for key in element:
                fmt += _build_format_string(element[key])
        else:
            fmt += baseTypes[element[1]]

    return fmt


def pack(data, nested=True, dtype=None, fmt_str=None):
    """ Pack the data into its binary format. Set the nested parameter if you provide a nested list. """
    if data is None:
        return b''

    if not nested:
        data = [item for sublist in data for item in sublist]

    return struct.pack(fmt_str if fmt_str else FORMATS[dtype], *data)


def unpack(data, dtype=None, fmt_str=None):
    """ Unpack the binary data into a dictionary """
    if len(data) == 0:
        return None

    return struct.unpack(fmt_str if fmt_str else FORMATS[dtype], data)


def unpack_dict(data, dtype=None, fmt_str=None):
    tuples = unpack(data, dtype, fmt_str)

    def __unpack_dict(elements, i, result):
        for element in elements:
            if isinstance(element, dict):
                for key in element:
                    result[key] = {}
                    i = __unpack_dict(element[key], i, result[key])
            else:
                result[element[0]] = tuples[i]
                i += 1
        return i

    data = {}
    __unpack_dict(NODES[dtype], 0, data)
    return data


# read description file
tree = ElementTree.parse(AADC_DESCRIPTION)

# parse structs
NODES = {}
for childs in tree.getroot():
    if childs.tag == 'structs':
        NODES = _read_structs(childs)

# cache format strings
FORMATS = {}
for node in NODES:
    FORMATS[node] = _build_format_string(NODES[node])
