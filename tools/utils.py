"""Defines a set of convenient utilities to use in scripts."""
import time
from typing import Callable

class DebugLoop:

    """Loops a function and prints statistics."""

    def __init__(
            self,
            callback: Callable[[], bool],
            iters_per_print: int = 1000
    ):
        """
        Initializes the loop variables.

        Arguments:
            callback: called for each iteration of the loop. This callable
                takes no arguments and should return True if successful else,
                False.
            iters_per_print: number of iterations between prints.
        """
        self._callback = callback
        self._iters_per_print = iters_per_print

    def loop(self):
        """Loops the callback. Can be escaped with ctrl^c."""
        start_time = time.time()
        errors = 0
        iters = 0
        try:
            while True:
                errors += 0 if self._callback() else 1
                iters += 1

                if iters % self._iters_per_print == 0:
                    now = time.time()
                    diff = (now - start_time)
                    start_time = now

                    freq = self._iters_per_print / diff
                    error_rate = errors / self._iters_per_print * 100
                    print(f'Loop frequency is {round(freq, 2)}hz at '
                          f'{round(error_rate, 2)}% error rate.')

                    errors = 0
        except KeyboardInterrupt:
            print()  # Clear line immediately after the ctrl-c
            print(f"Interrupted. Loop exiting. Completed {iters} iterations.")
            pass
