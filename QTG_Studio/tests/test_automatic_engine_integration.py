"""Offline GUI/IPC/transport/recording integration. Never loads a native DLL."""
import ctypes as C
import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend.automatic_engine import AutomaticEngine, METHOD, run
from interfaces.engine_measurements import Measurements, CHANNELS


def conditions(decel=False):
    return dict(alt_ft=755 if decel else 2000, ias_kt=0 if decel else 80,
                lat=32.23, lon=74.12, heading_deg=48.4, pitch_deg=0., bank_deg=0.,
                flaps=0., throttle=100 if decel else 5, temperature_c=15., on_ground=decel)


class EngineIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.now = 0.
        self.sim = Mock(spec=['is_connected', 'pause_sim', 'wait_for_paused', 'set_initial_throttle',
                              'queue_position_airborne', 'set_initial_flaps', 'get_global_weather_metar',
                              'set_global_weather_metar', 'unpause_sim', 'set_replay_throttle', 'stop', 'connect_and_run'])
        self.sim.is_connected.return_value = True
        for method in ('wait_for_paused', 'set_initial_throttle', 'queue_position_airborne',
                       'set_initial_flaps', 'set_global_weather_metar', 'set_replay_throttle'):
            getattr(self.sim, method).return_value = True
        self.sim.get_global_weather_metar.return_value = 'GLOB 000000Z 00000KT 9999 SKC 15/10 Q1013'
        self.published = []
        self.counter = 0
        self.workers = []
        self.addCleanup(self._cleanup_workers)

    def _cleanup_workers(self):
        for w in self.workers:
            w.close()

    def context(self, decel=False):
        return dict(mode='auto', control_method=METHOD, test_reference='1.f.(2)' if decel else '1.f.(1)',
                    title='Engine deceleration' if decel else 'Engine acceleration', initial_conditions=conditions(decel),
                    result_directory=str(self.root/'results'), command_directory=str(self.root/'commands'),
                    stop_file=str(self.root/'stop'), status_file=str(self.root/'status.json'))

    def worker(self, decel=False, **kwargs):
        w = AutomaticEngine(self.context(decel), self.sim, self.published.append, clock=lambda: self.now, **kwargs)
        self.workers.append(w)
        return w

    def frame(self, worker, *, rpm=None, throttle=None, brake=1, **kwargs):
        self.counter += 1
        result = dict(worker.cfg, rpm=rpm if rpm is not None else (2700. if worker.deceleration else 1700.),
                      throttle=worker.cfg['throttle'] if throttle is None else throttle,
                      parking_brake=brake, ground_speed=0., sim_time=self.now,
                      sequence=self.counter, receipt_time=self.now)
        result.update(kwargs)
        return result

    def ready(self, worker):
        worker.step(self.frame(worker, throttle=5))
        self.now += 1
        worker.step(self.frame(worker, throttle=5))
        self.now += 1
        worker.step(self.frame(worker, throttle=5))
        for _ in range(5):
            self.now += 1
            worker.step(self.frame(worker))
        self.assertEqual(worker.stage, 'READY TO RECORD', worker.details)
        self.assertTrue(worker.status()['can_record'])
        self.assertFalse(worker.rows)

    def complete(self, worker):
        self.ready(worker)
        worker.command(dict(action='record'))
        start = self.now
        for seconds in range(1, 6):
            self.now = start + seconds
            worker.step(self.frame(worker))
            if seconds < 5:
                self.sim.set_replay_throttle.assert_not_called()
        self.sim.set_replay_throttle.assert_called_once_with(5. if worker.deceleration else 100.)
        for seconds in range(6, 16):
            self.now = start + seconds
            worker.step(self.frame(worker, rpm=970. if worker.deceleration else 2700.,
                                   throttle=5. if worker.deceleration else 100.))
        self.assertEqual(worker.stage, 'Completed', worker.details)

    def test_both_engine_tests_gui_contract_through_real_csv_and_pdf(self):
        for decel in (False, True):
            with self.subTest(deceleration=decel):
                self.sim.reset_mock()
                worker = self.worker(decel)
                self.complete(worker)
                paths = list(map(Path, worker.outputs))
                self.assertEqual(len(paths), 3)
                self.assertTrue(next(p for p in paths if p.suffix == '.pdf').read_bytes().startswith(b'%PDF'))
                with next(p for p in paths if p.suffix == '.csv').open() as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 15)
                metadata = json.loads(next(p for p in paths if p.suffix == '.json').read_text())
                self.assertEqual(metadata['result'], 'COMPLETE')
                self.assertEqual(metadata['events_s']['throttle_command'], 5.)
                self.assertEqual(metadata['events_s']['recording_stop'], 15.)
                self.assertEqual(metadata['initial_conditions']['throttle'], 100. if decel else 5.)
                self.assertFalse(worker.status()['can_record'])

    def test_brake_off_or_unknown_never_allows_initial_full_throttle(self):
        for brake in (0, None, float('nan')):
            self.sim.reset_mock()
            worker = self.worker(True)
            worker.step(self.frame(worker, throttle=5, brake=brake))
            self.now += 16
            worker.step(self.frame(worker, throttle=5, brake=brake))
            self.assertEqual(worker.stage, 'FAILED')
            self.assertNotIn(unittest.mock.call(100.), self.sim.set_initial_throttle.call_args_list)
            self.assertIn('parking brake', worker.details)

    def test_deceleration_engages_brake_and_waits_for_fresh_on_before_full_throttle(self):
        from backend.parking_brake import GroundParkingBrake
        send = Mock()
        worker = self.worker(True)
        worker.ground_brake = GroundParkingBrake(send)
        worker.step(self.frame(worker, throttle=5, brake=0))
        self.now += .1
        worker.step(self.frame(worker, throttle=5, brake=0))
        send.assert_called_once_with()
        self.assertNotIn(unittest.mock.call(100.), self.sim.set_initial_throttle.call_args_list)
        self.now += .1
        worker.step(self.frame(worker, throttle=5, brake=1))
        self.assertNotIn(unittest.mock.call(100.), self.sim.set_initial_throttle.call_args_list)
        self.now += .1
        worker.step(self.frame(worker, throttle=5, brake=1))
        self.sim.set_initial_throttle.assert_any_call(100.)

    def test_deceleration_continues_recording_with_ground_movement(self):
        worker = self.worker(True)
        self.ready(worker)
        worker.command(dict(action='record'))
        self.now += 1
        worker.step(self.frame(worker, brake=1, ground_speed=8.))
        self.assertEqual(worker.phase, 'RECORD')
        self.assertEqual(worker.rows[-1]['Ground Speed (kt)'], 8.)

    def test_ground_movement_does_not_block_verified_parking_brake(self):
        worker = self.worker(True)
        worker.step(self.frame(worker, throttle=5, brake=1, ground_speed=4.))
        self.now += .1
        worker.step(self.frame(worker, throttle=5, brake=1, ground_speed=4.))
        self.sim.unpause_sim.assert_called_once()
        self.now += .1
        worker.step(self.frame(worker, throttle=5, brake=1, ground_speed=4.))
        self.sim.set_initial_throttle.assert_any_call(100.)

    def test_brake_loss_while_ready_or_recording_removes_power(self):
        for record in (False, True):
            worker = self.worker(True)
            self.ready(worker)
            if record:
                worker.command(dict(action='record'))
            self.now += .1
            worker.step(self.frame(worker, brake=0))
            self.assertEqual(worker.stage, 'FAILED')
            self.sim.set_replay_throttle.assert_called_with(5.)

    def test_stale_duplicate_pause_disconnect_or_invalid_measurements_abort(self):
        for fault in ('stale', 'duplicate', 'paused', 'disconnect', 'nan'):
            self.sim.is_connected.return_value = True
            worker = self.worker()
            self.ready(worker)
            worker.command(dict(action='record'))
            old = self.frame(worker)
            worker.step(old)
            self.now += 2
            frame = self.frame(worker)
            if fault == 'stale': frame['receipt_time'] -= 2
            if fault == 'duplicate': frame = old
            if fault == 'paused': frame['sim_time'] = old['sim_time']
            if fault == 'disconnect': self.sim.is_connected.return_value = False
            if fault == 'nan': frame['rpm'] = float('nan')
            worker.step(frame)
            self.assertEqual(worker.stage, 'FAILED', fault)

    def test_stop_during_baseline_saves_incomplete_results_without_step(self):
        worker = self.worker()
        self.ready(worker)
        worker.command(dict(action='record'))
        self.now += 1
        worker.step(self.frame(worker))
        worker.command(dict(action='stop'))
        self.assertEqual(worker.stage, 'ABORTED')
        self.assertEqual(len(worker.outputs), 3)
        self.sim.set_replay_throttle.assert_not_called()
        worker.command(dict(action='record'))
        self.assertEqual(worker.phase, 'DONE')

    def test_rejected_step_and_timeout_save_failed_results(self):
        for reject in (False, True):
            worker = self.worker()
            self.ready(worker)
            worker.command(dict(action='record'))
            self.sim.set_replay_throttle.return_value = not reject
            for _ in range(36):
                self.now += 1
                worker.step(self.frame(worker, throttle=5 if worker.phase == 'RECORD' and 'throttle_command' not in worker.sequence.events else 100))
            self.assertEqual(worker.result, 'FAILED' if reject else 'NOT STABILIZED')
            self.assertEqual(len(worker.outputs), 3)

    def test_no_step_until_actual_start_recording_and_shutdown_is_idempotent(self):
        worker = self.worker(True)
        self.ready(worker)
        for _ in range(10):
            self.now += 1
            worker.step(self.frame(worker))
        self.sim.set_replay_throttle.assert_not_called()
        worker.command(dict(action='shutdown'))
        count = len(self.sim.mock_calls)
        worker.command(dict(action='shutdown'))
        self.assertTrue(worker.closed)
        self.assertEqual(worker.stage, 'Closed')
        self.assertEqual(len(self.sim.mock_calls), count+1)  # status reads connection only

    def test_idle_reapplied_after_position_reset_and_verified_before_unpause(self):
        worker = self.worker()
        worker.step(self.frame(worker, throttle=5))
        self.assertEqual(self.sim.set_initial_throttle.call_args_list, [unittest.mock.call(5.)])
        self.sim.unpause_sim.assert_not_called()
        self.now += .1
        worker.step(self.frame(worker, throttle=40))
        self.assertEqual(self.sim.set_initial_throttle.call_args_list, [unittest.mock.call(5.), unittest.mock.call(5.)])
        self.sim.unpause_sim.assert_not_called()
        self.now += .1
        worker.step(self.frame(worker, throttle=5))
        self.assertEqual(worker.phase, 'PREPARE')
        self.sim.unpause_sim.assert_called_once()
        self.now += .1
        worker.step(self.frame(worker, throttle=5))
        self.assertEqual(worker.phase, 'READY')
        self.assertEqual(worker.stage, 'READY TO RECORD')
        worker.command(dict(action='record'))
        self.assertEqual(self.sim.unpause_sim.call_count, 2)

    def test_persistent_nonidle_readback_blocks_recording(self):
        worker = self.worker()
        worker.step(self.frame(worker, throttle=5))
        self.now += 1
        worker.step(self.frame(worker, throttle=40))
        self.now += 15
        worker.step(self.frame(worker, throttle=40))
        self.assertEqual(worker.stage, 'FAILED')
        self.assertIn('throttle 5%', worker.details)
        self.sim.unpause_sim.assert_not_called()

    def test_initialization_waits_for_fresh_measurement_after_blocking_call(self):
        worker = self.worker()
        worker.step(self.frame(worker, throttle=5))
        self.now = 2.
        worker.step(None)
        self.assertEqual(worker.phase, 'PREPARE')
        self.assertIn('fresh engine measurements', worker.details)
        self.now = 16.
        worker.step(None)
        self.assertEqual(worker.stage, 'FAILED')

    def test_throttle_correction_and_unpause_do_not_wait_for_temperature(self):
        worker = self.worker()
        worker.step(self.frame(worker, throttle=5))
        self.now += .1
        worker.step(self.frame(worker, throttle=90, temperature_c=30))
        self.sim.set_initial_throttle.assert_called_with(5.)
        self.sim.unpause_sim.assert_not_called()
        self.now += .1
        worker.step(self.frame(worker, throttle=5, temperature_c=30))
        self.sim.unpause_sim.assert_called_once()
        self.now += .1
        worker.step(self.frame(worker, throttle=5, temperature_c=30))
        self.assertEqual(worker.phase, 'PREPARE')
        self.assertIn('temperature_c', worker.details)
        self.now += .1
        worker.step(self.frame(worker, throttle=5, temperature_c=15))
        self.assertEqual(worker.phase, 'READY')

    def test_initialization_does_not_wait_for_engine_rpm_to_settle(self):
        worker = self.worker()
        worker.step(self.frame(worker, throttle=5, rpm=2700))
        self.now = 1.
        worker.step(self.frame(worker, throttle=5, rpm=2700))
        self.now = 2.
        worker.step(self.frame(worker, throttle=5, rpm=2000.))
        self.assertEqual(worker.stage, 'READY TO RECORD')
        self.sim.pause_sim.assert_called()

    def test_deceleration_never_issues_full_throttle_without_brake_after_idle_reapply(self):
        worker = self.worker(True)
        worker.step(self.frame(worker, throttle=5))
        self.now += .1
        worker.step(self.frame(worker, throttle=40, brake=1))
        self.now += .1
        worker.step(self.frame(worker, throttle=5, brake=0))
        self.assertNotIn(unittest.mock.call(100.), self.sim.set_initial_throttle.call_args_list)
        self.sim.unpause_sim.assert_not_called()

    def test_startup_displays_connecting_and_initiates_positioning(self):
        worker = self.worker()
        self.assertEqual(worker.stage, 'Connecting')
        self.assertEqual(worker.phase, 'CONNECT')
        self.assertEqual(worker.status()['stage'], 'Connecting')
        self.assertFalse(worker.status()['can_record'])
        # Telemetry frame provided: initial conditions applied immediately
        worker.step(self.frame(worker, throttle=5))
        self.assertEqual(worker.phase, 'PREPARE')
        self.sim.set_initial_throttle.assert_called_with(5.0)
        self.sim.pause_sim.assert_called()
        self.sim.unpause_sim.assert_not_called()

    def test_connection_timeout_when_p3d_never_becomes_ready(self):
        ctx = self.context()
        ctx['startup_timeout'] = 120.0
        worker = AutomaticEngine(ctx, self.sim, self.published.append, clock=lambda: self.now)
        # Simulator not connected
        self.sim.is_connected.return_value = False
        self.now = 119.0
        worker.step(self.frame(worker, throttle=5))
        self.assertEqual(worker.stage, 'Connecting')
        # Timeout reached at 120s
        self.now = 120.0
        worker.step(self.frame(worker, throttle=5))
        self.assertEqual(worker.stage, 'FAILED')
        self.assertIn('Timed out waiting for Sim Flight Model to connect', worker.details)

    def test_genuine_simulator_pause_during_recording_aborts(self):
        worker = self.worker()
        self.ready(worker)  # gets to READY TO RECORD
        worker.command(dict(action='record'))
        self.assertEqual(worker.phase, 'RECORD')
        self.assertEqual(self.sim.unpause_sim.call_count, 2)
        worker.step(self.frame(worker))
        # Advance clock by 2s without advancing sim_time
        self.now += 2.0
        worker.step(self.frame(worker, sim_time=worker.last_sim_time))
        self.assertEqual(worker.stage, 'FAILED')
        self.assertIn('Simulator paused or simulation time stopped advancing', worker.details)

    def test_deceleration_baseline_starting_power_check(self):
        # Deceleration requires starting RPM near 2700 during 5s baseline
        worker = self.worker(True)
        self.ready(worker)
        worker.command(dict(action='record'))
        for t in range(5):
            self.now += 1.0
            worker.step(self.frame(worker, throttle=100, rpm=2400))  # Low RPM (< 2625)
        self.assertEqual(worker.result, 'NOT READY')
        self.assertIn('Engine RPM did not reach required starting power', worker.details)

    def test_worker_stop_priority_and_transport_cleanup(self):
        context = self.context()
        directory = Path(context['command_directory'])
        directory.mkdir()
        for index, action in enumerate(('record', 'initialize', 'shutdown')):
            (directory/f'{index}.json').write_text(json.dumps(dict(action=action)))
        reader = Mock()
        with patch('backend.automatic_engine.threading.Thread') as thread:
            result = run(context, self.published.append, factory=lambda: self.sim, reader_factory=lambda: reader, sleep=Mock())
        self.assertEqual(result, 0)
        self.sim.set_initial_throttle.assert_not_called()
        reader.poll.assert_not_called()
        reader.close.assert_called_once()
        self.sim.stop.assert_called_once()
        thread.return_value.join.assert_called_once()

    def test_finished_worker_does_not_poll_stale_reader(self):
        context = self.context()
        directory = Path(context['command_directory'])
        directory.mkdir()
        reader = Mock()
        controller = Mock()
        controller.closed = False
        controller.phase = 'CONNECT'
        def step(_frame):
            controller.phase = 'DONE'
        def command(_command):
            controller.closed = True
        controller.step.side_effect = step
        controller.command.side_effect = command
        def sleep(_seconds):
            (directory / 'shutdown.json').write_text(json.dumps({'action':'shutdown'}))
        with patch('backend.automatic_engine.threading.Thread'), \
             patch('backend.automatic_engine.AutomaticEngine', return_value=controller):
            result = run(context, self.published.append, factory=lambda: self.sim,
                         reader_factory=lambda: reader, sleep=sleep)
        self.assertEqual(result, 0)
        reader.poll.assert_called_once()

    def test_worker_startup_failure_cleans_owned_transport(self):
        with patch('backend.automatic_engine.threading.Thread'):
            code = run(self.context(), self.published.append, factory=lambda: self.sim,
                       reader_factory=Mock(side_effect=RuntimeError('reader unavailable')))
        self.assertEqual(code, 1)
        self.sim.stop.assert_called_once()
        self.assertEqual(self.published[-1]['stage'], 'FAILED')

    def test_launch_context_and_process_route_without_base_or_reference(self):
        import qtg_common_engine as engine
        spec = SimpleNamespace(reference='1.f.(1)', title='Engine acceleration')
        with patch.object(engine.subprocess, 'Popen') as popen:
            _, context = engine.launch('auto', spec, self.root/'sessions', control_method=METHOD,
                                       initial_conditions=conditions())
        self.assertEqual(context['control_method'], METHOD)
        self.assertEqual(context['duration_s'], 35)
        self.assertIsNone(context['reference_csv'])
        self.assertIn('--brunner-test', popen.call_args.args[0])
        context_path = next((self.root/'sessions').glob('*_context.json'))
        with patch.object(engine, 'load_base', side_effect=AssertionError('BRUNNER base must not load')), \
             patch('backend.automatic_engine.run', return_value=0) as automatic, \
             patch.object(engine.sys, 'stdout'), patch.object(engine.sys, 'stderr'):
            self.assertEqual(engine.run(context_path), 0)
        automatic.assert_called_once()


class MeasurementProtocolTests(unittest.TestCase):
    def reader(self):
        reader = object.__new__(Measurements)
        reader.pending, reader.sequence, reader.latest, reader.error = 100, 0, None, ''
        reader.object_id, reader.requested_at = None, 11.9
        reader.clock = lambda: 12.
        return reader

    def packet(self, request=100, brake=1, object_id=2):
        packet = C.create_string_buffer(40 + 8*len(CHANNELS))
        header = C.cast(packet, C.POINTER(C.c_uint32))
        for i, value in enumerate((len(packet), 0, 8, request, object_id, 1, 0, 1, 1, len(CHANNELS))):
            header[i] = value
        values = C.cast(C.addressof(packet)+40, C.POINTER(C.c_double))
        values[2] = brake
        return packet

    def test_only_correlated_complete_valid_brake_response_advances(self):
        for request, brake, truncate, accepts in ((100, 1, False, True), (99, 1, False, False),
                                                  (100, .5, False, False), (100, 1, True, False)):
            reader = self.reader()
            packet = self.packet(request, brake)
            reader._receive(C.addressof(packet), 40 if truncate else len(packet), None)
            self.assertEqual(reader.sequence, int(accepts))
            if accepts:
                self.assertEqual(reader.latest['receipt_time'], 12.)
                self.assertEqual(reader.latest['parking_brake'], 1.)
                self.assertEqual(reader.latest['object_id'], 2)
                reader._receive(C.addressof(packet), len(packet), None)
                self.assertEqual(reader.sequence, 1)

    def test_correlated_object_is_pinned_and_aircraft_change_fails(self):
        reader = self.reader()
        packet = self.packet(object_id=2)
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 1)
        reader.pending, reader.requested_at = 101, 11.9
        packet = self.packet(request=101, object_id=2)
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 2)
        reader.pending, reader.requested_at = 102, 11.9
        packet = self.packet(request=102, object_id=3)
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 2)
        self.assertIn('User aircraft changed', reader.error)

    def test_delayed_response_never_refreshes_brake_or_pins_object(self):
        reader = self.reader()
        reader.requested_at = 10.
        packet = self.packet()
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 0)
        self.assertIsNone(reader.object_id)
        self.assertIsNone(reader.pending)
        self.assertEqual(reader.error, '')

    def test_expired_request_is_replaced_and_fresh_response_is_accepted(self):
        reader = self.reader()
        now = [12.]
        reader.clock = lambda: now[0]
        reader.requested_at = 10.
        reader.request_counter = 100
        reader.latest = {'receipt_time': 10.}
        reader.handle = C.c_void_p(1)
        reader.callback = None
        reader.dll = SimpleNamespace(
            SimConnect_CallDispatch=lambda *_: 0,
            SimConnect_RequestDataOnSimObject=lambda *_: 0)
        self.assertIsNone(reader.poll())
        self.assertEqual(reader.pending, 101)
        self.assertEqual(reader.requested_at, 12.)
        now[0] = 12.1
        packet = self.packet(request=101)
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 1)
        self.assertEqual(reader.latest['receipt_time'], 12.1)

    def test_discarded_initialization_response_cannot_refresh_measurements(self):
        reader = self.reader()
        reader.discard_pending()
        packet = self.packet()
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 0)
        self.assertIsNone(reader.latest)
        self.assertEqual(reader.error, '')


class GuiWiringTests(unittest.TestCase):
    def host(self, decel=False):
        from backend.registry import get_test
        cfg = conditions(decel)
        reference = '1.f.(2)' if decel else '1.f.(1)'
        values = {'Flt cond': 'Ground' if decel else 'Approach or Landing', 'Alt[ft]': str(cfg['alt_ft']),
                  'IAS[kts]': str(cfg['ias_kt']), 'Latitude[deg]': str(cfg['lat']), 'Longitude[deg]': str(cfg['lon']),
                  'Heading[deg]': '48.4', 'Pitch[deg]': '0', 'Bank[deg]': '0', 'Flaps': '0',
                  'Throttle[%]': str(cfg['throttle']), 'Temperature[C]': '15'}
        fields = {key: Mock(get=lambda value=value: value) for key, value in values.items()}
        host = SimpleNamespace(_common_duration_var=Mock(get=lambda: '21'),
            _test_settings={reference: (fields, {})}, _test_defaults={reference: values},
            _apply_test_settings=Mock(return_value=True), _common_process=None,
            common_record_button=Mock(), common_status=Mock(), after=Mock(), _poll_common_test=Mock())
        return host, get_test(reference)

    def test_gui_automatic_launches_real_backend_contract_without_reference(self):
        import main
        import qtg_common_engine as engine
        for decel in (False, True):
            host, spec = self.host(decel)
            with patch.object(engine, 'launch', return_value=(Mock(), {})) as launch:
                main.App._launch_common_test(host, 'auto', spec)
            launch.assert_called_once()
            self.assertEqual(launch.call_args.kwargs['control_method'], METHOD)
            self.assertIsNone(launch.call_args.args[3])
            host.common_record_button.configure.assert_called_with(state='disabled')

    def test_gui_drains_old_worker_before_switching_to_throttle_step(self):
        import main
        import qtg_common_engine as engine
        host, spec = self.host()
        host._common_process = Mock(poll=Mock(return_value=None))
        host._common_context = dict(control_method='replay')
        host._common_last_state = dict(stage='Completed')
        host._common_command = Mock()
        host._wait_common_worker_exit = Mock()
        with patch.object(engine, 'launch') as launch:
            main.App._launch_common_test(host, 'auto', spec)
        launch.assert_not_called()
        host._common_command.assert_called_once_with('shutdown')
        host._wait_common_worker_exit.assert_called_once()

    def test_gui_preserves_worker_failure_reason_after_exit(self):
        import main
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'status.json'
            path.write_text(json.dumps(dict(test_reference='1.f.(1)', stage='FAILED',
                                            details='Measurement response timed out', outputs=[])))
            host = SimpleNamespace(_common_process=Mock(poll=Mock(return_value=1), returncode=1),
                _common_context=dict(status_file=str(path), test_reference='1.f.(1)'),
                _common_display_spec=SimpleNamespace(reference='1.f.(1)'),
                _common_states={}, _render_common_state=Mock(), common_status=Mock(),
                common_record_button=Mock())
            host.common_status.winfo_exists.return_value = True
            main.App._poll_common_test(host)
            self.assertIn('Measurement response timed out', host.common_status.configure.call_args.kwargs['text'])

    def test_gui_cleanup_hides_manual_reference_for_automatic_engine(self):
        import main
        frame = Mock()
        frame.winfo_exists.return_value = True
        frame.winfo_manager.return_value = True
        host = SimpleNamespace(
            common_status=Mock(winfo_exists=Mock(return_value=True)),
            _common_display_spec=SimpleNamespace(reference='1.f.(1)'),
            _common_view_vars={k: Mock() for k in ('connection', 'throttle', 'engine', 'flight', 'attitude', 'controls', 'recording', 'progress', 'performance', 'recording_indicator')},
            _common_view_labels={'recording_indicator': Mock()},
            common_progress=Mock(cget=Mock(return_value='determinate')),
            common_record_button=Mock(),
            common_output_var=Mock(),
            common_reference_frame=frame,
            common_bar=Mock(winfo_exists=Mock(return_value=True))
        )
        state = dict(test_reference='1.f.(1)', mode='auto', control_method='engine_step', stage='Connecting')
        main.App._render_common_state(host, state)
        frame.pack_forget.assert_called_once()


class TelemetryAndReportingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.now = 0.
        self.sim = Mock()
        self.sim.is_connected.return_value = True
        self.sim.get_telemetry.return_value = {
            'vertical_speed': 150.0, 'total_weight': 1200.0, 'cg_percent': 26.5,
            'prop_rpms': [2695.0], 'engine_manifold_pressure_psi_1': 14.2
        }
        self.published = []

    def test_flight_dynamics_and_controls_telemetry_in_status(self):
        mock_brunner = Mock()
        mock_brunner.sock = True
        mock_brunner._connected = True
        mock_brunner.read_axis.side_effect = lambda axis: (0.15, 12.4, 88.0) if axis == 0x01 else ((-0.05, 5.2, 44.0) if axis == 0x02 else (0.0, 0.0, 0.0))

        context = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                       title='Engine acceleration', initial_conditions=conditions(False),
                       result_directory=str(self.root/'results'))
        worker = AutomaticEngine(context, self.sim, self.published.append, clock=lambda: self.now, brunner=mock_brunner)
        worker.latest = dict(rpm=976.42, throttle=5.04, ias_kt=80.2, alt_ft=2000.0,
                             pitch_deg=2.1, bank_deg=-1.5, heading_deg=48.4,
                             vertical_speed=150.0, total_weight=1200.0, cg_percent=26.5,
                             prop_rpm=976.0, manifold_pressure_psi=14.0)

        st = worker.status()
        self.assertEqual(st['engine_text'], "Engine RPM: 976.4 | Prop RPM: 976.0")
        self.assertEqual(st['throttle_text'], "Throttle: 5.0%")
        self.assertIn("80.2 kt", st['flight_text'])
        self.assertIn("2000 ft", st['flight_text'])
        self.assertIn("+2.1°", st['attitude_text'])
        self.assertIn("-1.5°", st['attitude_text'])
        self.assertIn("12.4 N", st['controls_text'])
        self.assertNotIn("88.0", st['controls_text'])
        self.assertEqual(st['telemetry']['vertical_speed'], 150.0)
        self.assertEqual(st['telemetry']['total_weight'], 1200.0)
        self.assertEqual(st['telemetry']['cg_percent'], 26.5)

    def test_selected_bank_pid_is_visible_during_acceleration(self):
        from frontend.main import format_active_pid_status

        mock_brunner = Mock()
        mock_brunner.sock = True
        mock_brunner._connected = True
        mock_brunner.read_axis.side_effect = lambda axis: (0.0, 0.0, 0.0)
        context = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                       title='Engine acceleration', initial_conditions=conditions(False),
                       pid_conditions={'target_bank':0.0},
                       active_pids={'airspeed':False,'bank':True,'yaw':False},
                       result_directory=str(self.root/'results'))
        worker = AutomaticEngine(context, self.sim, self.published.append,
                                 clock=lambda: self.now, brunner=mock_brunner)
        worker.phase = 'RECORD'
        worker.latest = dict(ias_kt=80.0, alt_ft=2000.0, pitch_deg=2.0,
                             bank_deg=0.6, heading_deg=48.4)
        worker.pid_assist.update(self.now, worker.latest, mock_brunner)
        state = worker.status()
        self.assertEqual(state['pid_runtime']['command_axes'], ['roll'])
        self.assertIn('Bank PID: RUNNING, holding', format_active_pid_status(state))
        self.assertIn('measured 0.6°', format_active_pid_status(state))

    def test_unavailable_telemetry_displays_na_instead_of_dashes(self):
        context = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                       title='Engine acceleration', initial_conditions=conditions(False),
                       result_directory=str(self.root/'results'))
        self.sim.get_telemetry.return_value = {}
        mock_brunner = Mock(sock=None)
        worker = AutomaticEngine(context, self.sim, self.published.append, clock=lambda: self.now, brunner=mock_brunner)
        worker.latest = {}
        st = worker.status()
        self.assertEqual(st['flight_text'], "Airspeed / Altitude: N/A")
        self.assertEqual(st['attitude_text'], "Pitch / Bank / Heading: N/A")
        self.assertEqual(st['controls_text'], "CLS2sim Pitch / Roll / Rudder: N/A")
        self.assertEqual(st['engine_text'], "Engine RPM: N/A")
        self.assertEqual(st['throttle_text'], "Throttle: N/A")

    def test_live_recording_timer_and_final_duration(self):
        context = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                       title='Engine acceleration', initial_conditions=conditions(False),
                       result_directory=str(self.root/'results'))
        worker = AutomaticEngine(context, self.sim, self.published.append, clock=lambda: self.now, brunner=None)
        worker.stage = 'READY TO RECORD'
        worker.phase = 'READY'
        st = worker.status()
        self.assertIn("00:00", st['recording_text'])

        worker.latest = dict(receipt_time=0.0, sim_time=0.0)
        worker.command(dict(action='record'))
        self.assertEqual(worker.phase, 'RECORD')
        self.now = 4.2
        st = worker.status()
        self.assertIn("00:04", st['recording_text'])
        self.assertIn("4.2 s", st['progress_text'])

        self.now = 16.5
        worker.rows = [{'Time (s)': 16.5}]
        worker.finish('COMPLETE', 'RPM stabilized for 3.0s')
        st = worker.status()
        self.assertIn("00:16 (16.5 s)", st['recording_text'])

    def test_prop_rpm_and_manifold_pressure_saved_in_csv_rows(self):
        context = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                       title='Engine acceleration', initial_conditions=conditions(False),
                       result_directory=str(self.root/'results'))
        worker = AutomaticEngine(context, self.sim, self.published.append, clock=lambda: self.now, brunner=Mock(sock=None))
        self.now = 1.0
        worker.latest = dict(receipt_time=1.0, sim_time=1.0)
        worker.stage = 'READY TO RECORD'
        worker.phase = 'READY'
        worker.command(dict(action='record'))

        frame = dict(worker.cfg, rpm=2650.0, throttle=100.0, parking_brake=1, ground_speed=0.0,
                     alt_ft=2000.0, ias_kt=80.0, sim_time=1.0, sequence=1, receipt_time=1.0,
                     prop_rpm=2648.0, manifold_pressure_psi=14.2)
        worker.step(frame)
        self.assertEqual(len(worker.rows), 1)
        row = worker.rows[0]
        self.assertIn('Propeller 1 RPM', row)
        self.assertEqual(row['Propeller 1 RPM'], 2648.0)
        self.assertIn('Engine Manifold Pressure (psi)', row)
        self.assertEqual(row['Engine Manifold Pressure (psi)'], 14.2)
        self.assertIn('Manifold Pressure (inHg)', row)
        self.assertAlmostEqual(row['Manifold Pressure (inHg)'], 14.2 * 2.0360206576, places=4)

    def test_initial_page_samples_stabilized_baseline_conditions(self):
        from reports.plot_graphs import _safe_initial, _fmt
        df_decel = {
            'Time (s)': [0.0, 1.0, 2.5, 4.8, 5.0, 6.0],
            'Throttle 1 Position (%)': [100.0, 100.0, 100.0, 100.0, 5.0, 5.0],
            'Engine RPM': [972.0, 1400.0, 2200.0, 2700.0, 2600.0, 1200.0],
            'Propeller 1 RPM': [970.0, 1400.0, 2200.0, 2700.0, 2600.0, 1200.0],
            'Engine Manifold Pressure (psi)': [14.0, 14.1, 14.2, 14.2, 6.0, 6.0]
        }
        import pandas as pd
        df = pd.DataFrame(df_decel)
        meta = {'event_time_s': 5.0}

        rpm = _safe_initial(df, 'engine_rpm', metadata=meta)
        throt = _safe_initial(df, 'throttle_pct', metadata=meta)
        self.assertEqual(rpm, 2700.0)
        self.assertEqual(throt, 100.0)

        self.assertEqual(_fmt(float('nan')), "N/A")

    def test_height_agl_in_channels_definition(self):
        from interfaces.engine_measurements import CHANNELS
        agl_channel = next((ch for ch in CHANNELS if ch[0] == 'height_agl_ft'), None)
        self.assertIsNotNone(agl_channel)
        self.assertEqual(agl_channel[1], 'PLANE ALT ABOVE GROUND')
        self.assertEqual(agl_channel[2], 'feet')

    def test_height_agl_recorded_in_csv_rows_ground_and_airborne(self):
        # 1. Ground test (Deceleration): height near zero (e.g. 0.5 ft)
        context_decel = dict(mode='auto', control_method=METHOD, test_reference='1.f.(2)',
                             title='Engine deceleration', initial_conditions=conditions(True),
                             result_directory=str(self.root/'results_decel'))
        worker_decel = AutomaticEngine(context_decel, self.sim, self.published.append, clock=lambda: self.now, brunner=Mock(sock=None))
        self.now = 1.0
        worker_decel.latest = dict(receipt_time=1.0, sim_time=1.0)
        worker_decel.stage, worker_decel.phase = 'READY TO RECORD', 'READY'
        worker_decel.command(dict(action='record'))

        frame_ground = dict(worker_decel.cfg, rpm=2650.0, throttle=100.0, parking_brake=1, ground_speed=0.0,
                            alt_ft=755.0, ias_kt=0.0, sim_time=1.0, sequence=1, receipt_time=1.0,
                            height_agl_ft=0.4)
        worker_decel.step(frame_ground)
        self.assertEqual(len(worker_decel.rows), 1)
        row_ground = worker_decel.rows[0]
        self.assertIn('Height AGL (ft)', row_ground)
        self.assertEqual(row_ground['Height AGL (ft)'], 0.4)
        self.assertEqual(row_ground['Altitude MSL (ft)'], 755.0)

        # 2. Airborne test (Acceleration): real height AGL (e.g. 1245.0 ft)
        context_accel = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                             title='Engine acceleration', initial_conditions=conditions(False),
                             result_directory=str(self.root/'results_accel'))
        worker_accel = AutomaticEngine(context_accel, self.sim, self.published.append, clock=lambda: self.now, brunner=Mock(sock=None))
        worker_accel.latest = dict(receipt_time=1.0, sim_time=1.0)
        worker_accel.stage, worker_accel.phase = 'READY TO RECORD', 'READY'
        worker_accel.command(dict(action='record'))

        frame_airborne = dict(worker_accel.cfg, rpm=1700.0, throttle=5.0, parking_brake=0, ground_speed=80.0,
                              alt_ft=2000.0, ias_kt=80.0, sim_time=1.0, sequence=1, receipt_time=1.0,
                              height_agl_ft=1245.0)
        worker_accel.step(frame_airborne)
        self.assertEqual(len(worker_accel.rows), 1)
        row_airborne = worker_accel.rows[0]
        self.assertIn('Height AGL (ft)', row_airborne)
        self.assertEqual(row_airborne['Height AGL (ft)'], 1245.0)
        self.assertEqual(row_airborne['Altitude MSL (ft)'], 2000.0)

    def test_height_agl_never_substitutes_altitude_msl(self):
        context = dict(mode='auto', control_method=METHOD, test_reference='1.f.(1)',
                       title='Engine acceleration', initial_conditions=conditions(False),
                       result_directory=str(self.root/'results'))
        self.sim.get_telemetry.return_value = {}
        worker = AutomaticEngine(context, self.sim, self.published.append, clock=lambda: self.now, brunner=Mock(sock=None))
        self.now = 1.0
        worker.latest = dict(receipt_time=1.0, sim_time=1.0)
        worker.stage, worker.phase = 'READY TO RECORD', 'READY'
        worker.command(dict(action='record'))

        # frame has alt_ft = 2000.0, but height_agl_ft is missing/None
        frame = dict(worker.cfg, rpm=1700.0, throttle=5.0, parking_brake=0, ground_speed=80.0,
                     alt_ft=2000.0, ias_kt=80.0, sim_time=1.0, sequence=1, receipt_time=1.0)
        worker.step(frame)
        row = worker.rows[0]
        import math
        self.assertTrue(math.isnan(row['Height AGL (ft)']))
        self.assertNotEqual(row['Height AGL (ft)'], 2000.0)
        self.assertIn('height_agl_ft', worker.diagnostics)
        self.assertIn('PLANE ALT ABOVE GROUND', worker.diagnostics['height_agl_ft'])

    def test_height_agl_pdf_renders_with_measured_data(self):
        from reports.plot_graphs import _render_report
        import pandas as pd
        csv_path = self.root / 'test_agl.csv'
        pdf_path = self.root / 'test_agl.pdf'
        t = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        df = pd.DataFrame({
            'Time (s)': t,
            'Throttle 1 Position (%)': [5.0]*5 + [100.0]*6,
            'Engine RPM': [970.0]*5 + [2700.0]*6,
            'Propeller 1 RPM': [970.0]*5 + [2695.0]*6,
            'Altitude MSL (ft)': [2000.0]*11,
            'Height AGL (ft)': [1245.0]*11,
            'Indicated Airspeed (kts)': [80.0]*11,
            'Engine Manifold Pressure (psi)': [14.0]*11,
        })
        df.to_csv(csv_path, index=False)
        _render_report(sim_path=csv_path, ref_path=None, out_path=pdf_path,
                       metadata={'mode': 'auto', 'kind': 'acceleration', 'event_time_s': 5.0})
        self.assertTrue(pdf_path.exists())
        self.assertGreater(pdf_path.stat().st_size, 1000)


if __name__ == '__main__':
    unittest.main()
