/**
 * Single editable human-input config.
 * Button indices live ONLY in GAMEPAD_MAP. Other modules must reference these
 * names, not numeric Standard Gamepad indices.
 *
 * ASSUMPTION (not verified on a physical DualShock): the W3C "standard" gamepad
 * map. Chrome/Firefox typically expose a DualShock 4 this way over USB, but
 * Bluetooth, Steam Input, and the macOS Game Controller framework can differ.
 * Confirm on hardware before trusting L1/R1/L2/R2 or the D-pad.
 */
export const GAMEPAD_MAP = {
  buttons: {
    L1: 4,
    R1: 5,
    L2: 6,
    R2: 7,
    DpadUp: 12,
    DpadDown: 13,
    DpadLeft: 14,
    DpadRight: 15,
  },
  axes: {
    leftX: 0,
    leftY: 1,
    rightX: 2,
    rightY: 3,
  },
  /**
   * Standard Gamepad Y is often inverted: physical stick UP reports -1.
   * Multiply the raw Y axis by this constant so stick UP becomes +vertical,
   * which the mapper treats as +X joint command.
   * ASSUMPTION: unverified on a physical DualShock.
   */
  AXIS_Y_SIGN: -1,
};

/** abs(x) <= DEADZONE becomes 0; outside is rescaled so ±1 still reaches ±1. */
export const DEADZONE = 0.08;

/** Global HUMAN AUTHORITY. Effective command is mapped_ctrl * authority, clipped to ctrlrange. */
export const DEFAULT_AUTHORITY = 0.10;

/** Discrete levels. D-pad Up/Down and the on-screen buttons step this list (clamped, no wrap). */
export const AUTHORITY_LEVELS = [0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1];

/** Browser send rate. Not coupled to the MuJoCo timestep. */
export const HUMAN_CMD_HZ = 50;

/**
 * Mirror of physics/control_console.py HUMAN_CMD_STALE_S (0.200 s).
 * The runtime enforces the timeout; this constant is for docs and tests.
 */
export const HUMAN_STALE_MS = 200;
