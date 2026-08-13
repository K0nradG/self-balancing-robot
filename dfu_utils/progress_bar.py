#!/usr/bin/env python3

import sys


class ProgressBar:
    def __init__(self, total_steps=100, width=50, description="") -> None:

        self.total_steps = total_steps
        self.current_step = 0
        self.width = width
        self.description = description
        self._last_print_length = 0

    def update(self, step, description=None) -> None:

        self.current_step = step
        if description:
            self.description = description

        self.current_step = min(self.current_step, self.total_steps)
        percentage = (self.current_step / self.total_steps) * 100
        filled_length = int(self.width * self.current_step // self.total_steps)
        bar = "█" * filled_length + "░" * (self.width - filled_length)

        output = f"\r{self.description} [{bar}] {percentage:5.1f}%"
        if len(output) < self._last_print_length:
            output += " " * (self._last_print_length - len(output))
        self._last_print_length = len(output)

        sys.stdout.write(output)
        sys.stdout.flush()

    def finish(self, message="\n") -> None:

        sys.stdout.write("\r" + " " * self._last_print_length + "\r")
        sys.stdout.write(message)
        sys.stdout.flush()
