"""
Victor Sim Hardware Package

This package provides a simulated hardware interface for the Victor robot
with Python API for external simulators to communicate via ROS topics.
"""

from .robot_state_api_new import VictorSimulatorAPI, create_victor_simulator

__all__ = ['VictorSimulatorAPI', 'create_victor_simulator']
