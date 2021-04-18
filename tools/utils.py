"""Defines a set of convenient utilities to use in scripts."""
import time
from typing import Callable


class DebugLoop:
    """Loops a function and prints statistics."""
    def __init__(self,
                 callback: Callable[[], bool],
                 iters_per_print: int = 1000):
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

        self._errors = 0
        self._iters = 0
        self._iter_start_time = None

    def _loop_func(self):
        self._errors += 0 if self._callback() else 1
        self._iters += 1

        if self._iters % self._iters_per_print == 0:
            self._print_func()

    def _print_func(self):
        now = time.time()
        diff = (now - self._iter_start_time)

        freq = self._iters_per_print / diff
        error_rate = self._errors / self._iters_per_print * 100
        print(f'Loop frequency is {round(freq, 2)}hz at '
              f'{round(error_rate, 2)}% error rate.')

        # Reset statistics variables.
        self._start_time = now
        self._errors = 0

    def loop(self):
        """Loops the callback. Can be escaped with ctrl^c."""
        self._iter_start_time = time.time()
        try:
            while True:
                self._loop_func()
        except KeyboardInterrupt:
            print()  # Clear line immediately after the ctrl-c
            print(f"Interrupted. Loop exiting. Completed {sel._iters} iterations.")
            pass
