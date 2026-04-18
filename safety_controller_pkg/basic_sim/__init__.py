"""basic_sim — a lightweight 2D demo simulator for the safety controller.

This subpackage is a standalone visualization tool. It mocks just enough of
rclpy / sensor_msgs / ackermann_msgs to run the real `safety_node.py`
unchanged inside a pygame window, driven by WASD input.

Nothing inside the parent `safety_controller_pkg` depends on this module
— it is strictly a demo harness.

Entry point:
    python3 -m safety_controller_pkg.basic_sim.run_sim
"""
