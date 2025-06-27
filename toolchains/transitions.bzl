# The specific platform label you want to transition to.
_CORTEX_M4_PLATFORM = "@bazel_embedded//platforms:cortex_m4_fpu"

def _switch_to_cortex_m4_impl(settings, attr):
  """Implementation function for the transition.

  This function ignores the incoming settings and unconditionally sets
  the target platform(s) to the desired Cortex M4 platform.
  """
  # The "//command_line_option:platforms" setting expects a list of platform labels.
  # Even though we only have one, we wrap it in a list.
  return {"//command_line_option:platforms": [_CORTEX_M4_PLATFORM]}

# Create the transition object
switch_to_cortex_m4 = transition(
    implementation = _switch_to_cortex_m4_impl,
    # This transition doesn't need to read any input settings.
    inputs = [],
    # This transition modifies the target platform(s).
    outputs = ["//command_line_option:platforms"],
)
