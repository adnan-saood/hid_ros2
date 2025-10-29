HID ROS 2 Documentation
=======================

**A schema-driven ROS 2 hardware interface for USB Human Interface Devices (HID)**

HID ROS 2 is a comprehensive framework for integrating USB HID devices with ROS 2 through the ``ros2_control`` infrastructure. It provides a schema-driven approach where a single YAML file defines your device's interface, automatically generating all necessary code for both the ROS 2 side and embedded firmware.

.. image:: https://img.shields.io/badge/License-Apache%202.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
   :alt: License

.. image:: https://github.com/adnan-saood/hid_ros2/actions/workflows/ci.yaml/badge.svg
   :target: https://github.com/adnan-saood/hid_ros2/actions/workflows/ci.yaml
   :alt: CI Status

Key Features
------------

* **Single Source of Truth**: Define your device interface once in YAML, generate everything else automatically
* **Type-Aware Hardware Interface**: Native support for ``uint8/16/32``, ``int8/16/32``, and ``float32/64`` with proper byte conversion
* **Bidirectional Communication**: Both input (sensor data) and output (commands) over USB HID
* **Code Generation**: Automatic generation of URDF, controller configs, launch files, and C headers for firmware
* **Developer Tools**: Command-line tools for schema validation, live device inspection, and descriptor generation
* **Full ros2_control Integration**: Works seamlessly with existing ROS 2 controllers and tools
* **Zero Manual Configuration**: No hand-written URDF or configuration files required

Quick Links
-----------

* `GitHub Repository <https://github.com/adnan-saood/hid_ros2>`_
* `Issue Tracker <https://github.com/adnan-saood/hid_ros2/issues>`_
* `Discussions <https://github.com/adnan-saood/hid_ros2/discussions>`_

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   installation
   quickstart
   tutorial_custom_device
   basic_concepts

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   schema_reference
   code_generation
   hardware_interface
   controllers
   developer_tools

.. toctree::
   :maxdepth: 2
   :caption: Examples

   examples/dummy_pose
   examples/signal_generator

.. toctree::
   :maxdepth: 2
   :caption: Advanced Topics

   advanced/hid_report_descriptors
   advanced/firmware_integration
   advanced/custom_controllers
   advanced/type_system
   advanced/debugging

.. toctree::
   :maxdepth: 2
   :caption: Development

   contributing
   architecture
   api_reference

.. toctree::
   :maxdepth: 1
   :caption: Additional Resources

   troubleshooting
   faq
   changelog

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
