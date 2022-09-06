"""Defines a set of convenient utilities to use in scripts."""
import time
from typing import Callable
from collections import Counter


class DebugLoop:
    """Loops a function and prints statistics."""

    def __init__(
        self,
        callback: Callable[[], bool],
        num_iters: int,
        iters_per_print: int = 1000,
    ):
        """
        Initializes the loop variables.

        Arguments:
            callback: called for each iteration of the loop. This callable
                takes no arguments and should return 0 if successful and an
                error code otherwise (this will be used to report statistics).
            num_iters: number of iterations to run before exiting.
            iters_per_print: number of iterations between prints.
        """
        self._callback = callback
        self._num_iters = num_iters
        self._iters_per_print = iters_per_print

        self._errors = {}
        self._cummulative_errors = Counter({})
        self._iters = 0
        self._iter_start_time = None

    def _loop_func(self):
        err = self._callback()
        assert type(err) is int

        if err != 0:
            if err not in self._errors:
                self._errors[err] = 1
            else:
                self._errors[err] += 1
        self._iters += 1

        if self._iters % self._iters_per_print == 0:
            self._print_func()
            self._iter_start_time = time.time()

    def _print_error_codes(self, errors):
        total_errors = sum(errors.values())
        for error_code, count in sorted(errors.items()):
            code_errors = count / total_errors * 100
            print(
                f"Error code: {error_code} was {round(code_errors, 2)}% of "
                f"errors."
            )

    def _print_func(self):
        now = time.time()
        diff = now - self._iter_start_time

        freq = self._iters_per_print / diff
        total_errors = sum(self._errors.values())
        error_rate = total_errors / self._iters_per_print * 100
        print(
            f"Loop frequency is {round(freq, 2)}hz at "
            f"({total_errors}/{self._iters_per_print})"
            f"{round(error_rate, 2)}% error rate."
        )

        self._print_error_codes(self._errors)

        # Reset statistics variables.
        self._start_time = now
        self._cummulative_errors += Counter(self._errors)
        self._errors = {}

    def loop(self):
        """Loops the callback. Can be escaped with ctrl^c."""
        self._iter_start_time = time.time()
        try:
            while True:
                self._loop_func()
                if self._iters >= self._num_iters and self._num_iters != 0:
                    break
        except KeyboardInterrupt:
            print()  # Clear line immediately after the ctrl-c
            print(
                f"Interrupted. Loop exiting. Completed {self._iters} "
                f"iterations."
            )
            print(f"Cummultive errors:")
            self._print_error_codes(self._cummulative_errors)
            pass
