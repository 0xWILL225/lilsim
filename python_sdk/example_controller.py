#!/usr/bin/env python3
"""
Example: Simple Sinusoidal Controller

This script demonstrates a basic controller that makes the car drive
in a sinusoidal pattern.

Prerequisites:
1. Start lilsim: ./build/debug/app/lilsim
2. Enable ZMQ in Admin panel
3. Set to SYNC mode with control period = 10
4. Run this script: python example_controller.py
"""

import sys
import time
import numpy as np
sys.path.insert(0, '.')

from lilsim import LilsimClient, MarkerType
from lilsim.utils import state_to_dict, create_circle

def main():
    print("=" * 60)
    print("Sinusoidal Controller Example")
    print("=" * 60)
    
    # Connect to simulator
    print("\nConnecting to simulator...")
    client = LilsimClient(host="localhost")
    client.connect()
    print("✓ Connected")
    
    # Set to sync mode
    print("\nSetting sync mode (K=10)...")
    success = client.set_mode(sync=True, control_period=10)
    if not success:
        print("❌ Failed to set sync mode. Make sure ZMQ is enabled!")
        return
    print("✓ Sync mode enabled")
    
    # Statistics
    stats = {
        'calls': 0,
        'max_v': 0.0,
        'positions': []
    }
    
    # Define the controller
    def sinusoidal_controller(request):
        """Controller that drives in a sinusoidal pattern."""
        stats['calls'] += 1
        
        # Extract state
        car = request.scene.car
        tick = request.header.tick
        
        # Track statistics
        stats['max_v'] = max(stats['max_v'], car.v)
        stats['positions'].append((car.pos.x, car.pos.y))
        
        # Sinusoidal steering
        frequency = 0.005  # Hz
        amplitude = 0.4    # rad
        steer_angle = amplitude * np.sin(tick * frequency)
        
        # Constant acceleration
        ax = 3.0  # m/s^2
        
        # Print status every 100 calls
        if stats['calls'] % 100 == 0:
            print(f"[{stats['calls']:4d}] "
                  f"tick={tick:5d}, "
                  f"pos=({car.pos.x:6.2f}, {car.pos.y:6.2f}), "
                  f"v={car.v:5.2f} m/s, "
                  f"steer={steer_angle:5.2f} rad")
        
        # Visualize current position with a marker every 50 calls
        if stats['calls'] % 50 == 0:
            marker = create_circle(
                ns="trajectory",
                id=stats['calls'] // 50,
                x=car.pos.x,
                y=car.pos.y,
                radius=0.2,
                color=(0, 255, 255, 150),  # Cyan with transparency
                ttl_sec=10.0  # Marker disappears after 10 seconds
            )
            client.publish_markers([marker])
        
        return (steer_angle, ax)
    
    # Register the controller
    print("\nRegistering controller...")
    client.register_sync_controller(sinusoidal_controller)
    
    # Start the client (begins control loop)
    client.start()
    print("✓ Controller active\n")
    
    print("The car should now be driving in a sinusoidal pattern.")
    print("Cyan markers show the trajectory (10s TTL).")
    print("Press Ctrl+C to stop...\n")
    
    # Run until interrupted
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nStopping controller...")
    
    # Print final statistics
    print("\n" + "=" * 60)
    print("Controller Statistics")
    print("=" * 60)
    print(f"Total control calls: {stats['calls']}")
    print(f"Maximum velocity:    {stats['max_v']:.2f} m/s")
    
    if len(stats['positions']) >= 2:
        # Calculate total distance
        positions = np.array(stats['positions'])
        distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
        total_distance = np.sum(distances)
        print(f"Distance traveled:   {total_distance:.2f} m")
    
    # Cleanup
    client.stop()
    client.close()
    print("\n✓ Controller stopped and disconnected")

if __name__ == "__main__":
    main()

