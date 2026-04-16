# visualization

PyQt6-based visualization and debugging tools for the virtual gamepad workflow. The current toolchain provides:

- LCM connection management
- a virtual gamepad publisher
- a lightweight message diagnostic script

## Dependencies

- Python 3.7+
- PyQt6
- lcm

To install the required dependencies, run the following command:
```bash
pip3 install -r requirements.txt
```

## Usage

```bash
python3 robot_debugger.py
```

Available widgets/components:

1. `LcmManagerWidget`: connect and disconnect from the target LCM URL.
2. `VirtualGamepadWidget`: publish virtual gamepad messages for debugging.
3. `diagnose_gamepad.py`: basic LCM/message diagnostics for the virtual gamepad channel.
