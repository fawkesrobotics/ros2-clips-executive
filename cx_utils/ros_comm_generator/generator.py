#!/usr/bin/env python3
# Licensed under GPLv2. See LICENSE file. Copyright Carologistics.
import argparse
import re
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from jinja2 import Template


def to_camel_case(s):
    words = s.split("_")  # Split the string into words based on underscores
    camel_case_words = [word.capitalize() for word in words[0:]]
    camel_case_string = "".join(camel_case_words)
    return camel_case_string


def to_upper_case(s):
    words = s.split("_")  # Split the string into words based on underscores
    upper_case_words = [word.upper() for word in words]
    upper_case_words = "".join(upper_case_words)
    return upper_case_words


# Dictionary for mapping ROS2 types to CLIPS types
clips_types = {
    "bool": "BOOLEAN",
    "byte": "INTEGER",
    "char": "STRING",
    "float32": "FLOAT",
    "float64": "FLOAT",
    "int8": "INTEGER",
    "uint8": "INTEGER",
    "int16": "INTEGER",
    "uint16": "INTEGER",
    "int32": "INTEGER",
    "uint32": "INTEGER",
    "int64": "INTEGER",
    "uint64": "INTEGER",
    "string": "STRING",
    "wstring": "STRING",
}

# Dictionary for mapping ROS2 types to C++ types
cpp_types = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "char",
    "float32": "float",
    "float64": "double",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "string": "std::string",
    "wstring": "std::u16string",
}

# Capture pattern for fields
pattern = r"\b(?P<type>(?:bool|byte|char|float32|float64|int8|uint8|int16|uint16|int32|uint32|int64|uint64|string|wstring))\b(?P<restrictions>(?:<=\d+))?(?P<array>(?:\[\d*(?P<length>(?:<=\d+))?\]))?\s+(?P<name>[a-z]\w*)\b"  # noqa
regex = re.compile(pattern)

# Parse arguments
parser = argparse.ArgumentParser(
    prog="generator", description="This program is a generator for ROS2 communication in the ROS2 CX project"
)

parser.add_argument("file")
parser.add_argument("name")
parser.add_argument("message_type")
parser.add_argument("message_include")
parser.add_argument("-s", "--subscriber", action="store_true")
parser.add_argument("-p", "--publisher", action="store_true")
parser.add_argument("-r", "--requester", action="store_true")
parser.add_argument("-i", "--include_path_prefix")

args = parser.parse_args()

if args.subscriber and args.publisher:
    print("Error: Cannot be both subscriber and publisher")
    exit(1)

if args.subscriber and args.requester:
    print("Error: Cannot be both subscriber and requester")
    exit(1)

if args.publisher and args.requester:
    print("Error: Cannot be both publisher and requester")
    exit(1)

if not args.subscriber and not args.publisher and not args.requester:
    print("Error: Must be either subscriber or publisher or requester")
    exit(1)


# Open file
try:
    f = open(args.file, "r")
except FileNotFoundError:
    print("Error: File not found")
    exit(1)
except OSError as e:
    print(f"Error: {e}")
    exit(1)

# Read file
lines = f.readlines()

# Find fields and store them in a dictionary
fields = {}
response_fields = {}
in_response = False
for line in lines:
    if line.startswith("---") and args.requester:
        in_response = True
        continue
    match = regex.match(line)
    if match:
        type_part = match.group("type")
        restrictions_part = match.group("restrictions")
        array_part = match.group("array")
        lengths_part = match.group("length")
        name_part = match.group("name")
        field = {
            "name": name_part,
            "type": type_part,
            "clips_type": clips_types[type_part],
            "cpp_type": cpp_types[type_part],
            "array": (True if array_part is not None else False),
            "restrictions": restrictions_part,
            "lengths": lengths_part,
        }
        if in_response:
            response_fields[name_part] = field
        else:
            fields[name_part] = field

time_string = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

package_dir = get_package_share_directory("cx_utils")

if args.subscriber:
    with open(package_dir + "/templates/" + "subscriber_source.jinja") as s, open(
        package_dir + "/templates/" + "subscriber_header.jinja"
    ) as h, open(to_camel_case(args.name) + ".cpp", "w") as source_file, open(
        to_camel_case(args.name) + ".hpp", "w"
    ) as header_file:
        tmpl = Template(s.read())
        out = tmpl.render(
            name_camel=to_camel_case(args.name),
            message_type=args.message_type,
            subscriber_name=args.name,
            slots=fields.values(),
            gen_date=time_string,
            include_path_prefix=("cx_features" if args.include_path_prefix is None else args.include_path_prefix),
        )
        source_file.write(out)
        tmpl = Template(h.read())
        out = tmpl.render(
            name_upper=to_upper_case(args.name),
            name_camel=to_camel_case(args.name),
            message_type=args.message_type,
            subscriber_name=args.name,
            slots=fields.values(),
            gen_date=time_string,
            include_path_prefix=("cx_features" if args.include_path_prefix is None else args.include_path_prefix),
            message_include_path=args.message_include,
        )
        header_file.write(out)

if args.publisher:
    with open(package_dir + "/templates/" + "publisher_source.jinja") as s, open(
        package_dir + "/templates/" + "publisher_header.jinja"
    ) as h, open(to_camel_case(args.name) + ".cpp", "w") as source_file, open(
        to_camel_case(args.name) + ".hpp", "w"
    ) as header_file:
        tmpl = Template(s.read())
        out = tmpl.render(
            name_camel=to_camel_case(args.name),
            message_type=args.message_type,
            publisher_name=args.name,
            slots=fields.values(),
            gen_date=time_string,
            include_path_prefix=("cx_features" if args.include_path_prefix is None else args.include_path_prefix),
        )
        source_file.write(out)
        tmpl = Template(h.read())
        out = tmpl.render(
            name_upper=to_upper_case(args.name),
            name_camel=to_camel_case(args.name),
            message_type=args.message_type,
            publisher_name=args.name,
            slots=fields.values(),
            gen_date=time_string,
            include_path_prefix=("cx_features" if args.include_path_prefix is None else args.include_path_prefix),
            message_include_path=args.message_include,
        )
        header_file.write(out)

if args.requester:
    with open(package_dir + "/templates/" + "requester_source.jinja") as s, open(
        package_dir + "/templates/" + "requester_header.jinja"
    ) as h, open(to_camel_case(args.name) + ".cpp", "w") as source_file, open(
        to_camel_case(args.name) + ".hpp", "w"
    ) as header_file:
        tmpl = Template(s.read())
        out = tmpl.render(
            name_camel=to_camel_case(args.name),
            message_type=args.message_type,
            requester_name=args.name,
            slots=fields.values(),
            response_slots=response_fields.values(),
            gen_date=time_string,
            include_path_prefix=("cx_features" if args.include_path_prefix is None else args.include_path_prefix),
        )
        source_file.write(out)
        tmpl = Template(h.read())
        out = tmpl.render(
            name_upper=to_upper_case(args.name),
            name_camel=to_camel_case(args.name),
            message_type=args.message_type,
            requester_name=args.name,
            slots=fields.values(),
            gen_date=time_string,
            include_path_prefix=("cx_features" if args.include_path_prefix is None else args.include_path_prefix),
            message_include_path=args.message_include,
        )
        header_file.write(out)
