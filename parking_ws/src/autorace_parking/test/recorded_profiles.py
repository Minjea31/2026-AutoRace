"""Policies used by recordings made before the arbitrary-empty-cell course."""
from dataclasses import replace


def fixed_empty_recording(config):
    """The November 7–15 fixtures used known empty cell 4 and point obstacles.

    Keep those historical assertions independent of the live YAML's newer
    five-occupied-cell selection and vehicle-box collision policy.
    """
    return replace(config, single_empty_slot=False, fixed_parking_slot_id=4,
                   fixed_slot_empty_known=True, block_occupied_slots=False)
