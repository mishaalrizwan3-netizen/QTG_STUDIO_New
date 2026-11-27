"""Confirm Prepar3D is paused before offering Start Recording."""


def confirm_ready_pause(sim):
    if sim is None or not sim.is_connected():
        raise RuntimeError('Prepar3D disconnected before recording readiness.')
    sim.pause_sim()
    if not sim.wait_for_paused(3.0):
        raise RuntimeError('Prepar3D did not confirm pause after initialization.')
