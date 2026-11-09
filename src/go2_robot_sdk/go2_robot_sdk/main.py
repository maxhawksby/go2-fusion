# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Main entry point for Go2 Robot SDK
Implements clean architecture with real-time data publishing
"""

import asyncio
import time
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .presentation.go2_driver_node import Go2DriverNode


# Waits between reconnect tries: doubling from the first up to the most.
INITIAL_BACKOFF_SEC = 5.0
MAX_BACKOFF_SEC = 60.0
# A connection that stayed up this long counts as good, so the next drop
# starts the waits over instead of continuing to double.
STABLE_CONNECTION_SEC = 30.0


def backoff_after(previous_sec, uptime_sec):
    """Return how long to wait before the next try, given the last wait and
    how long the connection that just dropped had been up."""
    if uptime_sec >= STABLE_CONNECTION_SEC:
        return INITIAL_BACKOFF_SEC
    return min(previous_sec * 2, MAX_BACKOFF_SEC)


async def run_robot_connections(node: Go2DriverNode):
    """Keep the robot connected, retrying with backoff on failure.

    A transient blip (the robot's WebRTC handshake, http://<ip>:9991/con_notify,
    has been observed to time out for several seconds at a time) used to
    propagate out of this function, which made main_async()'s
    asyncio.wait(..., FIRST_COMPLETED) treat it as a reason to cancel the
    ROS2 spin task and shut the whole node down - so one brief network hiccup
    permanently killed the entire driver until a human noticed and manually
    restarted the container. Retrying here instead keeps the node alive
    through anything but a truly fatal, unrecoverable error.
    """
    backoff_sec = INITIAL_BACKOFF_SEC
    while True:
        tasks = []
        connected_at = None
        try:
            # Connect to robots
            await node.connect_robots()
            connected_at = time.monotonic()

            # Start control loops for each robot
            for i in range(len(node.config.robot_ip_list)):
                robot_id = str(i)
                task = asyncio.create_task(node.run_robot_control_loop(robot_id))
                tasks.append(task)

            # Wait for all tasks to complete (they only do via cancellation,
            # e.g. node shutdown, or by raising on a disconnect)
            await asyncio.gather(*tasks)
            return

        except Exception as e:
            # Stop the other robots' loops too; the retry restarts them all.
            for task in tasks:
                task.cancel()
            node.get_logger().error(f"Error in robot connections: {e}.")
            uptime = 0.0 if connected_at is None else time.monotonic() - connected_at
            backoff_sec = backoff_after(backoff_sec, uptime)
            node.get_logger().error(f"Retrying in {backoff_sec:.0f}s.")
            await asyncio.sleep(backoff_sec)


async def spin_node(node: Go2DriverNode):
    """Run ROS2 node in a separate thread"""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    def spin_thread():
        try:
            executor.spin()
        except Exception as e:
            node.get_logger().error(f"Error in ROS2 spin: {e}")
        finally:
            executor.shutdown()

    # Start in a separate thread
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()

    try:
        # Wait for node to finish
        while rclpy.ok():
            await asyncio.sleep(0.1)
    finally:
        thread.join(timeout=1.0)


async def main_async():
    """Main asynchronous function"""
    # Initialize ROS2
    rclpy.init()

    try:
        # Create node with current event loop
        current_loop = asyncio.get_running_loop()
        node = Go2DriverNode(event_loop=current_loop)
        
        # Run ROS2 node and robot connections in parallel
        ros_task = asyncio.create_task(spin_node(node))
        robot_task = asyncio.create_task(run_robot_connections(node))

        # Wait for any task to complete
        done, pending = await asyncio.wait(
            [ros_task, robot_task],
            return_when=asyncio.FIRST_COMPLETED
        )

        # Cancel unfinished tasks
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        # Check results of completed tasks
        for task in done:
            try:
                await task
            except Exception as e:
                node.get_logger().error(f"Task failed: {e}")

    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt")
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Resource cleanup
        try:
            # Disconnect from robots
            if 'node' in locals() and hasattr(node, 'webrtc_adapter'):
                for robot_id in node.webrtc_adapter.connections:
                    await node.webrtc_adapter.disconnect(robot_id)
            
            # Close unfinished tasks
            tasks = [t for t in asyncio.all_tasks() if not t.done()]
            if tasks:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)

        except Exception as e:
            print(f"Error during cleanup: {e}")
        finally:
            rclpy.shutdown()


def main():
    """Entry point"""
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\nNode terminated by keyboard interrupt")
    except Exception as e:
        print(f"Fatal error in main: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main() 