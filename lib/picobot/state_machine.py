"""
State Machine module for organizing robot behavior.

Students learn state machines in Lab 07, then use this
helper class to build complex behaviors.
"""

import time


class StateMachine:
    """
    Simple finite state machine for robot behavior.

    Usage:
        sm = StateMachine(robot)

        sm.add_state("IDLE", action=robot.stop)
        sm.add_state("FOLLOW", action=robot.line_follow_step)
        sm.add_state("TURN", action=lambda: robot.turn_degrees(90))

        sm.add_transition("IDLE", "FOLLOW", condition=lambda: button_pressed)
        sm.add_transition("FOLLOW", "TURN", condition=robot.at_junction)
        sm.add_transition("TURN", "FOLLOW", condition=lambda: abs(robot.heading) > 85)

        sm.run()
    """

    def __init__(self, robot=None):
        """
        Create state machine.

        Args:
            robot: Robot instance (optional, for convenience)
        """
        self.robot = robot

        self._states = {}
        self._transitions = []
        self._current_state = None
        self._state_entry_time = 0
        self._history = []

    def add_state(self, name, action=None, on_enter=None, on_exit=None):
        """
        Add a state to the machine.

        Args:
            name: State name (string)
            action: Function to call every loop while in state
            on_enter: Function to call once when entering state
            on_exit: Function to call once when leaving state
        """
        self._states[name] = {
            'action': action,
            'on_enter': on_enter,
            'on_exit': on_exit,
        }

        # First state added becomes initial state
        if self._current_state is None:
            self._current_state = name
            self._state_entry_time = time.ticks_ms()

    def add_transition(self, from_state, to_state, condition):
        """
        Add a transition between states.

        Args:
            from_state: Source state name
            to_state: Destination state name
            condition: Function returning True when transition should happen
        """
        self._transitions.append({
            'from': from_state,
            'to': to_state,
            'condition': condition,
        })

    def set_state(self, new_state):
        """
        Force transition to a new state.

        Args:
            new_state: State name to transition to
        """
        if new_state not in self._states:
            print(f"WARNING: Unknown state '{new_state}'")
            return

        if new_state == self._current_state:
            return

        # Exit current state
        old_state = self._current_state
        if old_state and self._states[old_state]['on_exit']:
            self._states[old_state]['on_exit']()

        # Log transition
        duration = self.time_in_state()
        self._history.append({
            'from': old_state,
            'to': new_state,
            'time': time.ticks_ms(),
            'duration': duration,
        })

        if len(self._history) > 100:
            self._history.pop(0)

        print(f"[{time.ticks_ms():8d}] {old_state} ({duration}ms) → {new_state}")

        # Enter new state
        self._current_state = new_state
        self._state_entry_time = time.ticks_ms()

        if self._states[new_state]['on_enter']:
            self._states[new_state]['on_enter']()

        # Update LEDs if robot available
        if self.robot and hasattr(self.robot, 'leds'):
            self.robot.leds.show_state(new_state)

    def step(self):
        """
        Execute one step of the state machine.

        Call this in your main loop.
        """
        if self._current_state is None:
            return

        # Run current state action
        state_info = self._states[self._current_state]
        if state_info['action']:
            state_info['action']()

        # Check transitions
        for trans in self._transitions:
            if trans['from'] == self._current_state:
                if trans['condition']():
                    self.set_state(trans['to'])
                    break  # Only one transition per step

    def run(self, loop_period_ms=20):
        """
        Run the state machine continuously.

        Args:
            loop_period_ms: Time between steps
        """
        print(f"State machine running. Initial state: {self._current_state}")
        print("Press Ctrl+C to stop")

        try:
            while True:
                self.step()
                time.sleep(loop_period_ms / 1000)

        except KeyboardInterrupt:
            print("\nState machine stopped")
            if self.robot:
                self.robot.stop()

    @property
    def state(self):
        """Current state name."""
        return self._current_state

    def time_in_state(self):
        """Milliseconds since entering current state."""
        return time.ticks_diff(time.ticks_ms(), self._state_entry_time)

    def get_history(self, last_n=10):
        """
        Get recent state transition history.

        Args:
            last_n: Number of transitions to return

        Returns:
            List of transition records
        """
        return self._history[-last_n:]

    def print_history(self):
        """Print state transition history."""
        print("\nState History:")
        print("-" * 50)
        for h in self._history:
            print(f"  {h['from']:15} → {h['to']:15} ({h['duration']}ms)")


class StateBuilder:
    """
    Fluent builder for creating state machines.

    Usage:
        sm = (StateBuilder(robot)
            .state("IDLE").action(robot.stop)
            .state("FOLLOW").action(robot.line_follow_step)
            .transition("IDLE", "FOLLOW").when(start_button)
            .transition("FOLLOW", "IDLE").when(robot.line_lost)
            .build())

        sm.run()
    """

    def __init__(self, robot=None):
        self._sm = StateMachine(robot)
        self._current_state = None
        self._current_transition = None

    def state(self, name):
        """Start defining a state."""
        self._current_state = name
        self._sm.add_state(name)
        return self

    def action(self, func):
        """Set action for current state."""
        if self._current_state:
            self._sm._states[self._current_state]['action'] = func
        return self

    def on_enter(self, func):
        """Set on_enter for current state."""
        if self._current_state:
            self._sm._states[self._current_state]['on_enter'] = func
        return self

    def on_exit(self, func):
        """Set on_exit for current state."""
        if self._current_state:
            self._sm._states[self._current_state]['on_exit'] = func
        return self

    def transition(self, from_state, to_state):
        """Start defining a transition."""
        self._current_transition = (from_state, to_state)
        return self

    def when(self, condition):
        """Set condition for current transition."""
        if self._current_transition:
            self._sm.add_transition(
                self._current_transition[0],
                self._current_transition[1],
                condition
            )
        return self

    def build(self):
        """Return the built state machine."""
        return self._sm
