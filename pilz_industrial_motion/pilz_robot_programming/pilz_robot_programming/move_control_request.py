# Copyright (c) 2018 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""API for easy usage of Pilz robot commands."""

import logging
import threading
from enum import Enum

logger = logging.getLogger(__name__)


class MoveControlAction(Enum):
    """Actions to control the execution of a move command."""
    STOP = 1
    PAUSE = 2
    RESUME = 3

    # triggered automatically
    MOTION_STOPPED = 4
    MOTION_RESUMED = 5


class _MoveControlState(Enum):
    NO_REQUEST = 1
    STOP_REQUESTED = 2
    PAUSE_REQUESTED = 3
    RESUME_REQUESTED = 4


class _MoveControlStateMachine(object):
    """Implements a light weight finite state machine for controlling the execution of a move command."""

    def __init__(self):
        self._state_lock = threading.RLock()
        self._state_cv = threading.Condition(self._state_lock)
        self.__state = _MoveControlState.NO_REQUEST

        self._fsm = {
            (_MoveControlState.NO_REQUEST, MoveControlAction.PAUSE): _MoveControlState.PAUSE_REQUESTED,
            (_MoveControlState.NO_REQUEST, MoveControlAction.RESUME): _MoveControlState.NO_REQUEST,
            (_MoveControlState.NO_REQUEST, MoveControlAction.STOP): _MoveControlState.STOP_REQUESTED,

            (_MoveControlState.PAUSE_REQUESTED, MoveControlAction.STOP): _MoveControlState.STOP_REQUESTED,
            (_MoveControlState.PAUSE_REQUESTED, MoveControlAction.PAUSE): _MoveControlState.PAUSE_REQUESTED,
            (_MoveControlState.PAUSE_REQUESTED, MoveControlAction.RESUME): _MoveControlState.RESUME_REQUESTED,

            (_MoveControlState.RESUME_REQUESTED, MoveControlAction.STOP): _MoveControlState.STOP_REQUESTED,
            (_MoveControlState.RESUME_REQUESTED, MoveControlAction.PAUSE): _MoveControlState.PAUSE_REQUESTED,
            (_MoveControlState.RESUME_REQUESTED, MoveControlAction.RESUME): _MoveControlState.RESUME_REQUESTED,
            (_MoveControlState.RESUME_REQUESTED, MoveControlAction.MOTION_RESUMED): _MoveControlState.NO_REQUEST,

            (_MoveControlState.STOP_REQUESTED, MoveControlAction.STOP): _MoveControlState.STOP_REQUESTED,
            (_MoveControlState.STOP_REQUESTED, MoveControlAction.PAUSE): _MoveControlState.STOP_REQUESTED,
            (_MoveControlState.STOP_REQUESTED, MoveControlAction.RESUME): _MoveControlState.STOP_REQUESTED,
            (_MoveControlState.STOP_REQUESTED, MoveControlAction.MOTION_STOPPED): _MoveControlState.NO_REQUEST,
        }

    @property
    def state(self):
        with self._state_lock:
            return self.__state

    @state.setter
    def state(self, state):
        logger.error("State cannot be set directly, please use switch() instead.")

    def switch(self, action):
        """Switch the state according to current state and action."""
        assert isinstance(action, MoveControlAction), \
            "Unknown type of action, only MoveControlAction is allowed."
        with self._state_lock:
            logger.info("Switching state from %s to %s",
                        self.__state.name, self._fsm[(self.__state, action)].name)
            self.__state = self._fsm[(self.__state, action)]
            self._state_cv.notify_all()

    def wait_for_resume(self):
        """Helper function for waiting for resume."""
        with self._state_cv:
            while self.state == _MoveControlState.PAUSE_REQUESTED:
                self._state_cv.wait()
        return self.state == _MoveControlState.RESUME_REQUESTED

    def __enter__(self):
        self._state_lock.acquire()

    def __exit__(self, type, value, tb):
        self._state_lock.release()
