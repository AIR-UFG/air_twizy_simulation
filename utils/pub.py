import can
import cantools
import time

try:
    dbc_file_path = '/root/can/streetdrone_can.dbc'  # Update this path as needed
    db = cantools.database.load_file(dbc_file_path)

    can_bus = can.interface.Bus(channel='can2', interface='socketcan')

    message_name = 'StreetDrone_Data_2'  # Adjust according to your DBC file
    message = db.get_message_by_name(message_name)

    # Control variables (set these to 1 to turn on, 0 to turn off)
    left_indicator = 1   # Left Indicator On/Off
    right_indicator = 1  # Right Indicator On/Off
    side_lights = 1      # Side Lights On/Off
    dipped_lights = 1    # Dipped Lights On/Off
    main_beam = 1        # Main Beam On/Off
    reverse_light = 1    # Reverse Light On/Off
    brake_light = 1      # Brake Light On/Off
    horn = 1             # Horn On/Off

    # Encode the data with all light controls
    data = message.encode({
        'Left_Indicator_B': left_indicator,
        'Right_Indicator_B': right_indicator,
        'Side_Lights_B': side_lights,
        'Dipped_Lights_B': dipped_lights,
        'Main_Beam_B': main_beam,
        'Reverse_Light_B': reverse_light,
        'Brake_Light_B': brake_light,
        'Horn_B': horn,  # Activate horn
        'PRND_Actual_Zs': 0,  # Neutral or default value
        'Remote_Auto_Mode_Ack_B': 0  # Neutral or default value
    })

    can_message = can.Message(arbitration_id=message.frame_id, data=data, is_extended_id=False)

    print(f"Encoded data: {data.hex()}")  # Print encoded data for debugging
    print(f"Sending {message_name} with all lights set...")

    # Continuously send the CAN message
    while True:
        can_bus.send(can_message)
        print("Message sent!")
        time.sleep(0.1)  # Send the message every 100ms (adjust as needed)

finally:
    can_bus.shutdown()
    print("CAN bus shutdown completed.")
