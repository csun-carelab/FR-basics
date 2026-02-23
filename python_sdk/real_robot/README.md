# Real Robot Mode

This mode allows direct control of the physical FR5 robot using the Fairino Python SDK.

## Setup

1. Ensure the FR5 robot is powered on and connected to the network.

2. Set the robot IP address (default: 192.168.58.2). You can modify the IP in `first_example.py` if needed.

3. Install dependencies:
   ```bash
   pip install -r ../requirements.txt
   ```

4. Enable the robot (refer to robot teach pendant).

## Running the Example

```bash
python first_example.py
```

This will:
- Connect to the robot at 192.168.58.2
- Read current joint positions
- Move the first joint by +15 degrees
- Close the connection

## Safety Notes

- Ensure the robot is in a safe state before running.
- Monitor the robot during operation.
- The robot should be enabled and not in error state.

## Troubleshooting

- Check network connectivity to robot IP.
- Verify Fairino SDK installation.
- Ensure robot is enabled on teach pendant.