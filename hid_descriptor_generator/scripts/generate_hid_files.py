#!/usr/bin/env python3
# Copyright 2025 Adnan Saood
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
HID Descriptor Generator from YAML Schema.

Generates: C header, URDF, controller YAML, and launch file.
"""
import argparse
import yaml
from pathlib import Path
from textwrap import dedent

# Type information
TYPE_INFO = {
    'bool': {'bits': 1, 'signed': False, 'float': False},
    'int8': {'bits': 8, 'signed': True, 'float': False},
    'uint8': {'bits': 8, 'signed': False, 'float': False},
    'int16': {'bits': 16, 'signed': True, 'float': False},
    'uint16': {'bits': 16, 'signed': False, 'float': False},
    'int32': {'bits': 32, 'signed': True, 'float': False},
    'uint32': {'bits': 32, 'signed': False, 'float': False},
    'float32': {'bits': 32, 'signed': True, 'float': True},
    'float64': {'bits': 64, 'signed': True, 'float': True},
}


def parse_schema(schema_path):
    """Parse YAML schema file."""
    with open(schema_path) as f:
        return yaml.safe_load(f)


def build_hid_descriptor(fields, report_id):
    """Generate HID report descriptor bytes."""
    desc = []

    # Usage Page (Vendor Defined 0xFF00)
    desc.extend([0x06, 0x00, 0xFF])
    # Usage (0x01)
    desc.extend([0x09, 0x01])
    # Collection (Application)
    desc.extend([0xA1, 0x01])

    # Report ID (if non-zero)
    if report_id:
        desc.extend([0x85, report_id & 0xFF])

    total_bits = 0

    for field in fields:
        field_type = field['type']
        count = field.get('count', 1)

        if field_type not in TYPE_INFO:
            raise ValueError(f"Unsupported type: {field_type}")

        type_info = TYPE_INFO[field_type]
        bits = type_info['bits']

        # Logical min/max
        if type_info['signed']:
            logical_min = -(2**(bits-1))
            logical_max = (2**(bits-1)) - 1
        elif type_info['float']:
            logical_min = -32768
            logical_max = 32767
        else:
            logical_min = 0
            logical_max = (2**bits) - 1

        # Logical Minimum (use extended format for large values)
        if logical_min >= -128 and logical_min <= 127:
            desc.extend([0x15, logical_min & 0xFF])
        else:
            desc.extend([0x17,
                        logical_min & 0xFF,
                        (logical_min >> 8) & 0xFF,
                        (logical_min >> 16) & 0xFF,
                        (logical_min >> 24) & 0xFF])

        # Logical Maximum
        if logical_max >= -128 and logical_max <= 127:
            desc.extend([0x25, logical_max & 0xFF])
        else:
            desc.extend([0x27,
                        logical_max & 0xFF,
                        (logical_max >> 8) & 0xFF,
                        (logical_max >> 16) & 0xFF,
                        (logical_max >> 24) & 0xFF])

        # Report Size
        desc.extend([0x75, bits & 0xFF])
        # Report Count
        desc.extend([0x95, count & 0xFF])
        # Input (Data, Var, Abs)
        desc.extend([0x81, 0x02])

        total_bits += bits * count

    # Pad to byte boundary if needed
    if total_bits % 8 != 0:
        pad_bits = 8 - (total_bits % 8)
        desc.extend([0x75, pad_bits])  # Report Size
        desc.extend([0x95, 0x01])      # Report Count
        desc.extend([0x81, 0x03])      # Input (Const, Var, Abs)
        total_bits += pad_bits

    # End Collection
    desc.extend([0xC0])

    payload_bytes = total_bits // 8
    on_wire_bytes = payload_bytes + (1 if report_id else 0)

    return bytes(desc), payload_bytes, on_wire_bytes


def generate_c_header(schema, desc_bytes, payload_bytes, on_wire_bytes, package_name):
    """Generate C header with HID descriptor."""
    device_name = schema.get('device_name', 'hid_device')
    report_id = schema.get('report_id', 1)

    # Format descriptor as C array
    desc_lines = []
    for i in range(0, len(desc_bytes), 16):
        chunk = desc_bytes[i:i+16]
        line = '  ' + ', '.join(f'0x{b:02X}' for b in chunk)
        desc_lines.append(line)
    desc_array = ',\n'.join(desc_lines)

    # Generate field offset information
    field_info_lines = []
    byte_offset = 0
    for field in schema['fields']:
        field_name = field['name']
        field_type = field['type']
        count = field.get('count', 1)
        type_info = TYPE_INFO[field_type]
        field_bytes = (type_info['bits'] * count + 7) // 8

        field_info_lines.append(
            f"  /* {field_name}: offset={byte_offset}, type={field_type}, "
            f"count={count}, bytes={field_bytes} */"
        )
        byte_offset += field_bytes

    field_info = '\n'.join(field_info_lines)

    guard = f"{package_name.upper()}_HID_DESCRIPTOR_H_"

    header = dedent(f"""\
    #ifndef {guard}
    #define {guard}

    #include <stdint.h>

    #ifdef __cplusplus
    extern "C" {{
    #endif

    /**
     * Auto-generated HID Report Descriptor
     * Device: {device_name}
     * Report ID: {report_id}
     * Payload bytes: {payload_bytes}
     * On-wire bytes: {on_wire_bytes} (includes report ID)
     */

    static const uint8_t HID_REPORT_DESCRIPTOR[] = {{
    {desc_array}
    }};

    static const uint16_t HID_REPORT_DESCRIPTOR_LEN = sizeof(HID_REPORT_DESCRIPTOR);
    static const uint8_t  HID_REPORT_ID = {report_id}U;
    static const uint16_t HID_PAYLOAD_BYTES = {payload_bytes}U;
    static const uint16_t HID_ON_WIRE_BYTES = {on_wire_bytes}U;

    /**
     * Field layout in payload (after report ID):
     */
    {field_info}

    #ifdef __cplusplus
    }} /* extern "C" */
    #endif

    #endif /* {guard} */
    """)

    return header


def generate_urdf(schema, payload_bytes, package_name):
    """Generate URDF with state interfaces matching the HID report and optional command interfaces."""
    device_name = schema.get('device_name', 'hid_device')
    vendor_id = schema.get('vendor_id', '0xCAFE')
    product_id = schema.get('product_id', '0x4000')
    sensor_name = schema.get('sensor_name', 'hid_input')
    input_report_id = schema.get('input_report_id', schema.get('report_id', 1))
    output_report_id = schema.get('output_report_id', 2)

    # Build state interface type map
    state_types = []
    state_types.append('uint8')  # report_id is always uint8

    for field in schema['fields']:
        field_type = field.get('type', 'uint8')
        count = field.get('count', 1)
        for _ in range(count):
            state_types.append(field_type)

    state_types_str = ','.join(state_types)

    # Generate state interfaces from 'fields' or 'inputs'
    state_interfaces = ['      <state_interface name="report_id"/>']

    for field in schema['fields']:
        field_name = field['name']
        count = field.get('count', 1)

        if count == 1:
            state_interfaces.append(f'      <state_interface name="{field_name}"/>')
        else:
            for i in range(count):
                state_interfaces.append(f'      <state_interface name="{field_name}_{i}"/>')

    state_interfaces_str = '\n'.join(state_interfaces)

    # Check if we have outputs (command interfaces)
    has_outputs = 'outputs' in schema and len(schema['outputs']) > 0

    if has_outputs:
        # Generate command interfaces
        gpio_name = schema.get('gpio_name', 'signal_params')
        command_interfaces = []
        command_types = []

        for field in schema['outputs']:
            field_name = field['name']
            field_type = field.get('type', 'float32')
            count = field.get('count', 1)

            for i in range(count):
                command_types.append(field_type)
                if count == 1:
                    command_interfaces.append(f'      <command_interface name="{field_name}"/>')
                else:
                    command_interfaces.append(f'      <command_interface name="{field_name}_{i}"/>')

        command_interfaces_str = '\n'.join(command_interfaces)
        command_types_str = ','.join(command_types)

        # URDF with both sensor (state) and gpio (command)
        urdf = dedent(f"""\
<?xml version="1.0"?>
<!-- Auto-generated URDF from HID schema -->
<robot name="{package_name}_robot">
  <ros2_control name="HIDSystem" type="system">
    <hardware>
      <plugin>hid_hardware/HidHardware</plugin>
      <param name="vendor_id">{vendor_id}</param>
      <param name="product_id">{product_id}</param>
      <param name="device_name">{device_name}</param>
      <param name="input_report_id">{input_report_id}</param>
      <param name="output_report_id">{output_report_id}</param>
      <param name="state_types">{state_types_str}</param>
      <param name="command_types">{command_types_str}</param>
    </hardware>
    <sensor name="{sensor_name}">
{state_interfaces_str}
    </sensor>
    <gpio name="{gpio_name}">
{command_interfaces_str}
    </gpio>
  </ros2_control>
</robot>
""")
    else:
        # URDF with only sensor (state interfaces)
        urdf = dedent(f"""\
<?xml version="1.0"?>
<!-- Auto-generated URDF from HID schema -->
<robot name="{package_name}_robot">
  <ros2_control name="HIDSystem" type="system">
    <hardware>
      <plugin>hid_hardware/HidHardware</plugin>
      <param name="vendor_id">{vendor_id}</param>
      <param name="product_id">{product_id}</param>
      <param name="device_name">{device_name}</param>
      <param name="input_report_id">{input_report_id}</param>
      <param name="state_types">{state_types_str}</param>
    </hardware>
    <sensor name="{sensor_name}">
{state_interfaces_str}
    </sensor>
  </ros2_control>
</robot>
""")

    return urdf


def generate_controller_yaml(schema):
    """Generate controller YAML configuration."""
    sensor_name = schema.get('sensor_name', 'hid_input')
    frame_id = schema.get('frame_id', 'hid_frame')
    update_rate = schema.get('update_rate', 250)
    publish_rate = schema.get('publish_rate', 100.0)  # Ensure float
    device_name = schema.get('device_name', 'hid_device')
    vendor_id = schema.get('vendor_id', '0xCAFE')
    product_id = schema.get('product_id', '0x4000')

    yaml_content = dedent(f"""\
    controller_manager:
      ros__parameters:
        update_rate: {update_rate}

        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster

        dummy_pose_broadcaster:
          type: dummy_pose_broadcaster/DummyPoseBroadcaster

        hid_generic_broadcaster:
          type: hid_generic_broadcaster/HidGenericBroadcaster

        hid_diagnostic_broadcaster:
          type: hid_diagnostic_broadcaster/HidDiagnosticBroadcaster

    dummy_pose_broadcaster:
      ros__parameters:
        sensor_name: "{sensor_name}"
        frame_id: "{frame_id}"

    hid_generic_broadcaster:
      ros__parameters:
        sensor_name: "{sensor_name}"
        publish_rate: {publish_rate:.1f}

    hid_diagnostic_broadcaster:
      ros__parameters:
        sensor_name: "{sensor_name}"
        publish_rate: 1.0
        device_name: "{device_name}"
        vendor_id: "{vendor_id}"
        product_id: "{product_id}"
    """)

    return yaml_content


def generate_launch_file(schema, package_name):
    """Generate launch file TEMPLATE (user must customize controller spawning)."""
    frame_id = schema.get('frame_id', 'hid_frame')

    launch = dedent(f"""\
    from launch import LaunchDescription
    from launch.substitutions import Command, FindExecutable
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    import os

    def generate_launch_description():
        pkg_share = get_package_share_directory('{package_name}')
        urdf_file = os.path.join(pkg_share, 'urdf', 'hid_robot.urdf.xacro')
        yaml_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

        robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_file])

        return LaunchDescription([
            # Static transform from world to HID frame
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_hid_frame',
                arguments=['0', '0', '0', '0', '0', '0', 'world', '{frame_id}'],
                output='both'
            ),

            # ros2_control node
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[{{'robot_description': robot_description}}, yaml_file],
                output='both'
            ),

            # Joint state broadcaster (always needed)
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='both'
            ),

            # TODO: Add controller spawners based on your needs
            # Available controllers (configured in controllers.yaml):
            #   - dummy_pose_broadcaster (6DOF specific PoseStamped publisher)
            #   - hid_generic_broadcaster (universal Float64MultiArray publisher)
            #
            # Example for pose broadcaster:
            # Node(
            #     package='controller_manager',
            #     executable='spawner',
            #     arguments=['dummy_pose_broadcaster', '--controller-manager', '/controller_manager'],
            #     output='both'
            # ),
            #
            # Example for generic broadcaster:
            # Node(
            #     package='controller_manager',
            #     executable='spawner',
            #     arguments=['hid_generic_broadcaster', '--controller-manager', '/controller_manager'],
            #     output='both'
            # ),
        ])
    """)

    return launch


def main():
    parser = argparse.ArgumentParser(description='Generate HID files from YAML schema')
    parser.add_argument('--schema', required=True, help='Path to YAML schema file')
    parser.add_argument('--output-dir', required=True, help='Output directory')
    parser.add_argument('--package-name', required=True, help='Package name')
    args = parser.parse_args()

    # Parse schema
    schema = parse_schema(args.schema)

    # Validate schema - support both old 'fields' and new 'inputs/outputs' formats
    if 'fields' not in schema and 'inputs' not in schema:
        raise ValueError("Schema must contain either 'fields' list (old format) or 'inputs' list (new format)")

    # Convert new format to old format for backward compatibility
    if 'inputs' in schema:
        # For now, we only process 'inputs' as state interfaces
        # 'outputs' are command interfaces and handled differently
        schema['fields'] = schema['inputs']
        # Store report IDs if provided
        if 'input_report_id' not in schema and 'report_id' not in schema:
            schema['report_id'] = 1
        elif 'input_report_id' in schema:
            schema['report_id'] = schema['input_report_id']

    # Build HID descriptor
    report_id = schema.get('report_id', 1)
    desc_bytes, payload_bytes, on_wire_bytes = build_hid_descriptor(schema['fields'], report_id)

    # Output paths
    out_dir = Path(args.output_dir)
    include_dir = out_dir / 'include' / args.package_name
    urdf_dir = out_dir / 'urdf'
    config_dir = out_dir / 'config'
    launch_dir = out_dir / 'launch'

    # Create directories
    include_dir.mkdir(parents=True, exist_ok=True)
    urdf_dir.mkdir(parents=True, exist_ok=True)
    config_dir.mkdir(parents=True, exist_ok=True)
    launch_dir.mkdir(parents=True, exist_ok=True)

    # Generate files
    header = generate_c_header(schema, desc_bytes, payload_bytes, on_wire_bytes, args.package_name)
    (include_dir / 'hid_descriptor.h').write_text(header)

    urdf = generate_urdf(schema, payload_bytes, args.package_name)
    (urdf_dir / 'hid_robot.urdf.xacro').write_text(urdf)

    yaml_content = generate_controller_yaml(schema)
    (config_dir / 'controllers.yaml').write_text(yaml_content)

    launch = generate_launch_file(schema, args.package_name)
    (launch_dir / 'hid.launch.py').write_text(launch)

    print(f"Generated HID files for {args.package_name}")
    print(f"  Header: {include_dir / 'hid_descriptor.h'}")
    print(f"  URDF: {urdf_dir / 'hid_robot.urdf.xacro'}")
    print(f"  Config: {config_dir / 'controllers.yaml'}")
    print(f"  Launch: {launch_dir / 'hid.launch.py'}")
    print(f"  Report size: {on_wire_bytes} bytes (Report ID: {report_id}, Payload: {payload_bytes} bytes)")


if __name__ == '__main__':
    main()
