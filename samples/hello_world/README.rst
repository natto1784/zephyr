.. _eqep-capture-sample:

.. zephyr:code-sample:: eqep_capture
   :name: EQEP Capture

   Demonstrate EQEP quadrature encoder functionality using Zephyr's counter subsystem.

Overview
********

This sample demonstrates the use of Enhanced Quadrature Encoder Pulse (EQEP)
functionality through Zephyr's counter subsystem. It showcases position counting,
direction detection, frequency calculation, and interrupt handling for quadrature
encoder signals.

The sample is based on the TI MCU+ SDK EQEP capture example, adapted to use
Zephyr APIs and demonstrate the TI AM3352 EQEP counter driver.

Features Demonstrated
*********************

* **Position Counting**: Read encoder position and detect changes
* **Direction Detection**: Determine clockwise vs counter-clockwise rotation
* **Frequency Calculation**: Calculate encoder rotation frequency
* **Alarm Callbacks**: Periodic position monitoring using counter alarms
* **Counter Operations**: Start, stop, get/set values, top value configuration
* **Advanced Features**: Guard period configuration and top value management

Requirements
************

* TI AM64x, AM243x, or AM62L board with EQEP peripheral support
* Quadrature encoder connected to EQEP pins (A, B, Index, Strobe)
* Board configuration with EQEP device tree nodes enabled

Hardware Setup
**************

Connect a quadrature encoder to the EQEP pins on your board:

AM243x EVM:
  * EQEP0_A: Pin N16 (configured via pinctrl)
  * EQEP0_B: Pin N17 (configured via pinctrl)
  * EQEP0_I: Pin R20 (Index signal, optional)
  * EQEP0_S: Pin R19 (Strobe signal, optional)

AM62L EVM:
  * EQEP0_A: Pin P23 (configured via pinctrl)
  * EQEP0_B: Pin P22 (configured via pinctrl)
  * EQEP0_I: Pin N22 (Index signal, optional)
  * EQEP0_S: Pin N23 (Strobe signal, optional)

Building and Running
********************

To build for AM243x EVM:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: am243x_evm_am2434_r5f0_0
   :goals: build
   :compact:

To build for AM62L EVM:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: am62l_evm_am62l3_a53
   :goals: build
   :compact:

Sample Output
=============

.. code-block:: console

   EQEP Capture Sample Application Started
   Board: am243x_evm_am2434_r5f0_0
   EQEP device: eqep@23200000

   === EQEP Basic Operations Test ===
   EQEP Counter frequency: 200000000 Hz
   EQEP Counter top value: 4294967295
   EQEP Counting direction: UP
   EQEP Counter started
   Initial position: 0

   === EQEP Position Monitoring Test ===
   Monitoring EQEP position for 5 seconds...
   Rotate the encoder to see position changes
   Alarm 1: Position = 0, Change = 0, Direction = UP
   Alarm 2: Position = 24, Change = 24, Direction = UP
   Alarm 3: Position = 48, Change = 24, Direction = UP
   Position monitoring complete. Total alarms: 10

   === EQEP Frequency Calculation Test ===
   Rotate encoder at constant speed for frequency measurement...
   Measurement results:
     Start position: 48
     End position: 148
     Position change: 100
     Time period: 2000 ms
     Calculated frequency: 12.50 Hz
     Direction: Clockwise

   === EQEP Advanced Features Test ===
   Guard period not supported or failed to read
   Current top value: 4294967295
   Top value configuration not supported
   EQEP Counter stopped

   === EQEP Capture Sample Complete ===
   All tests completed successfully!
   Connect a quadrature encoder to the EQEP pins and run again to see live data.

Troubleshooting
***************

If the sample fails to find the EQEP device:

1. Verify that the board's device tree includes EQEP nodes
2. Check that the EQEP node is enabled (`status = "okay"`)
3. Ensure pinctrl configuration is correct for your board
4. Verify that the TI AM3352 EQEP driver is enabled in Kconfig

If position readings are always zero:

1. Check encoder connections to EQEP pins
2. Verify encoder power supply
3. Check pinctrl mux mode settings
4. Test with a known working encoder

References
**********

* TI AM3352 Enhanced Quadrature Encoder Pulse (EQEP) Module
* Zephyr Counter Subsystem Documentation
* TI MCU+ SDK EQEP Examples
