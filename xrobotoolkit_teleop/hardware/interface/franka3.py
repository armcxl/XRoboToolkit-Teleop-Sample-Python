from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

DEFAULT_FRANKA3_IP = "172.16.0.2"
DEFAULT_FRANKA3_HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
DEFAULT_GRIPPER_OPEN_WIDTH = 0.08
DEFAULT_GRIPPER_CLOSED_WIDTH = 0.0


class Franka3DriverBase(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_joint_positions(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_joint_velocities(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def command_joint_positions(self, joint_positions: np.ndarray) -> None:
        raise NotImplementedError

    @abstractmethod
    def command_gripper_width(self, width: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def go_home(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class MockFranka3Driver(Franka3DriverBase):
    def __init__(
        self,
        dt: float = 0.02,
        q_home: Optional[np.ndarray] = None,
        open_width: float = DEFAULT_GRIPPER_OPEN_WIDTH,
        closed_width: float = DEFAULT_GRIPPER_CLOSED_WIDTH,
    ):
        self.dt = dt
        self.q_home = DEFAULT_FRANKA3_HOME_Q if q_home is None else q_home
        self.open_width = open_width
        self.closed_width = closed_width
        self._joint_positions = self.q_home.copy()
        self._joint_velocities = np.zeros_like(self._joint_positions)
        self._gripper_width = self.open_width

    def connect(self) -> None:
        time.sleep(0.1)

    def get_joint_positions(self) -> np.ndarray:
        return self._joint_positions.copy()

    def get_joint_velocities(self) -> np.ndarray:
        return self._joint_velocities.copy()

    def command_joint_positions(self, joint_positions: np.ndarray) -> None:
        joint_positions = np.asarray(joint_positions)
        self._joint_velocities = (joint_positions - self._joint_positions) / max(self.dt, 1e-6)
        self._joint_positions = joint_positions.copy()

    def command_gripper_width(self, width: float) -> None:
        width = float(np.clip(width, self.closed_width, self.open_width))
        self._gripper_width = width

    def go_home(self) -> None:
        self.command_joint_positions(self.q_home)
        self.command_gripper_width(self.open_width)

    def close(self) -> None:
        time.sleep(0.05)


class Franka3Interface:
    def __init__(
        self,
        robot_ip: str = DEFAULT_FRANKA3_IP,
        dt: float = 0.02,
        driver: Optional[Franka3DriverBase] = None,
        use_mock: bool = False,
        q_home: Optional[np.ndarray] = None,
        open_width: float = DEFAULT_GRIPPER_OPEN_WIDTH,
        closed_width: float = DEFAULT_GRIPPER_CLOSED_WIDTH,
    ):
        self.robot_ip = robot_ip
        if driver is None:
            if not use_mock:
                raise ValueError(
                    "Provide a Franka3DriverBase implementation or set use_mock=True for testing."
                )
            driver = MockFranka3Driver(
                dt=dt,
                q_home=q_home,
                open_width=open_width,
                closed_width=closed_width,
            )
        self.driver = driver

    def connect(self) -> None:
        self.driver.connect()

    def get_joint_positions(self) -> np.ndarray:
        return self.driver.get_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        return self.driver.get_joint_velocities()

    def set_joint_positions(self, joint_positions: np.ndarray) -> None:
        self.driver.command_joint_positions(joint_positions)

    def set_gripper_width(self, width: float) -> None:
        self.driver.command_gripper_width(width)

    def go_home(self) -> None:
        self.driver.go_home()

    def close(self) -> None:
        self.driver.close()
