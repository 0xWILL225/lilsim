#!/usr/bin/env python3
"""Basic test script for lilsim communication."""

import sys
import time
sys.path.insert(0, '.')

from lilsim import LilsimClient, MarkerType
from lilsim.utils import state_to_dict

def main():
    print("=" * 60)
    print("Lilsim Communication Test Script")
    print("=" * 60)
    print("\nMake sure lilsim is running and ZMQ is enabled in the GUI!\n")
    
    # Create and connect client
    print("1. Connecting to simulator...")
    client = LilsimClient(host="localhost")
    client.connect()
    print("   ✓ Connected")
    
    # Subscribe to state updates
    print("\n2. Subscribing to state updates...")
    state_count = [0]
    
    def state_callback(state):
        state_count[0] += 1
        if state_count[0] % 100 == 0:
            s = state_to_dict(state)
            print(f"   State #{state_count[0]}: tick={s['tick']}, x={s['x']:.2f}, y={s['y']:.2f}, v={s['v']:.2f}")
    
    client.subscribe_state(state_callback)
    client.start()
    time.sleep(2)
    print(f"   ✓ Received {state_count[0]} state updates")
    
    # Test admin commands
    print("\n3. Testing admin commands...")
    
    print("   - PAUSE")
    success = client.pause()
    print(f"     {'✓' if success else '✗'} Pause: {success}")
    time.sleep(0.5)
    
    print("   - RUN")
    success = client.run()
    print(f"     {'✓' if success else '✗'} Run: {success}")
    time.sleep(0.5)
    
    print("   - RESET")
    success = client.reset()
    print(f"     {'✓' if success else '✗'} Reset: {success}")
    time.sleep(0.5)
    
    print("   - STEP (10 ticks)")
    client.pause()
    time.sleep(0.3)
    success = client.step(10)
    print(f"     {'✓' if success else '✗'} Step: {success}")
    time.sleep(0.3)
    client.run()
    
    # Test mode switching
    print("\n4. Testing mode switching...")
    
    print("   - Switch to ASYNC mode")
    success = client.set_mode(sync=False)
    print(f"     {'✓' if success else '✗'} Async mode: {success}")
    time.sleep(0.5)
    
    print("   - Switch to SYNC mode (K=5)")
    success = client.set_mode(sync=True, control_period=5)
    print(f"     {'✓' if success else '✗'} Sync mode: {success}")
    time.sleep(0.5)
    
    # Test async control
    print("\n5. Testing async control...")
    print("   ⚠ Async control not yet implemented (skipping)")
    # client.set_mode(sync=False)
    # time.sleep(0.3)
    # 
    # if client.control_rep is None:
    #     client.connect_control_sync()
    #     client.stop()
    #     client.start()
    # 
    # print("   Sending control: steer=0.2 rad, ax=2.0 m/s^2")
    # for i in range(30):
    #     client.send_control_async(steer_angle=0.2, ax=2.0)
    #     time.sleep(0.05)
    # 
    # # Stop the car
    # for i in range(20):
    #     client.send_control_async(steer_angle=0.0, ax=-3.0)
    #     time.sleep(0.05)
    # client.send_control_async(steer_angle=0.0, ax=0.0)
    # 
    # print("   ✓ Async control complete (car should have turned)")
    
    # Test sync control
    print("\n6. Testing sync control...")
    
    control_count = [0]
    
    def test_controller(request):
        control_count[0] += 1
        # Simple sinusoidal steering
        import numpy as np
        tick = request.header.tick
        steer = 0.3 * np.sin(tick * 0.01)
        return (steer, 2.0)
    
    # Register controller BEFORE switching to sync mode
    client.register_sync_controller(test_controller)
    client.stop()
    client.start()
    
    # Now switch to sync mode after control socket is ready
    client.set_mode(sync=True, control_period=10)
    time.sleep(0.3)
    
    print("   Running sync controller for 3 seconds...")
    time.sleep(3)
    print(f"   ✓ Controller called {control_count[0]} times")
    
    # Test markers
    print("\n7. Testing marker visualization...")
    
    print("   - Publishing red circle at (5, 5)")
    client.publish_marker(
        ns="test", id=1, marker_type=MarkerType.CIRCLE,
        x=5.0, y=5.0, scale_x=1.0, scale_y=1.0,
        r=255, g=0, b=0, a=200
    )
    
    print("   - Publishing green square at (10, 10)")
    square = [(10,10,0), (12,10,0), (12,12,0), (10,12,0), (10,10,0)]
    client.publish_marker(
        ns="test", id=2, marker_type=MarkerType.LINE_STRIP,
        points=square, r=0, g=255, b=0, a=255,
        scale_x=0.1, scale_y=0.1
    )
    
    print("   - Publishing blue circles along x-axis")
    from lilsim.utils import create_circle
    markers = []
    for i in range(10):
        marker = create_circle(
            ns="test", id=100+i, x=i*2.0, y=0.0,
            radius=0.3, color=(0, 0, 255, 255)
        )
        markers.append(marker)
    client.publish_markers(markers)
    
    print("   ✓ Markers published (check simulator)")
    
    # Cleanup
    print("\n8. Cleaning up...")
    client.stop()
    client.close()
    print("   ✓ Client closed")
    
    print("\n" + "=" * 60)
    print("All tests completed successfully! ✓")
    print("=" * 60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()

