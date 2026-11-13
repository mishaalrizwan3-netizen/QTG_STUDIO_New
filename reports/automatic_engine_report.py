"""Measured engine Automatic CSV, provenance and PDF; never opens a viewer."""
import csv
from datetime import datetime
import json
from pathlib import Path
import time


def save(context, rows, sequence, result, reason, diagnostics=None):
    from backend.artifacts import test_directory
    folder = Path(context['result_directory'])
    folder.mkdir(parents=True, exist_ok=True)
    prefix = test_directory(folder, context['test_reference'], context['title']).name
    stem = folder / (prefix + '_auto_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_' + str(time.time_ns()))
    csv_path = stem.with_suffix('.csv')
    outputs = []
    try:
        with csv_path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        outputs.append(str(csv_path))
        origin = sequence.events['recording_start']
        event_time_s = sequence.events.get('throttle_onset', sequence.events.get('throttle_command', origin + sequence.settings.baseline_seconds)) - origin
        metadata = dict(context, result=result, reason=reason,
                        status='complete' if result == 'COMPLETE' else result.lower(),
                        mode='auto',
                        event_time_s=event_time_s,
                        events_s={key: stamp-origin for key, stamp in sequence.events.items()},
                        sequence_settings=vars(sequence.settings),
                        measurement_source='Correlated SimConnect callback; monotonic receipt clock',
                        validation='Response sequence only; no aircraft reference conformity evaluation',
                        diagnostics=dict(diagnostics or {}))
        metadata_path = stem.with_suffix('.qtg_test.json')
        metadata_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
        outputs.append(str(metadata_path))
        pdf_path = stem.with_name(stem.name + '_graphs.pdf')
        from reports.plot_graphs import _render_report
        _render_report(sim_path=csv_path, ref_path=None, out_path=pdf_path,
                       titt_results=None, metadata=metadata,
                       selected_graphs=context.get('selected_graphs'))
        outputs.append(str(pdf_path))
        return outputs
    except Exception as exc:
        exc.outputs = outputs
        raise
