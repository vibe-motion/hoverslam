from coldgas.rng import Mulberry32
from coldgas.spec import spec_hash, TASKS, NOZZLE_IDS
from coldgas.tasks import load_task, sample_initial


def test_spec_hash_matches_js(node):
    js = node("import { specHash } from './src/flight/spechash.js'; console.log(JSON.stringify(await specHash()));")
    assert js == spec_hash() and len(js) == 16


def test_mulberry32_matches_js(node):
    js = node("import { mulberry32 } from './src/flight/rng.js'; const out = [];"
              "for (const s of [0, 1, 7, 4294967295, -3, 123456789]) { const r = mulberry32(s); out.push(Array.from({length: 50}, r)); }"
              "console.log(JSON.stringify(out));")
    for s, seq in zip([0, 1, 7, 4294967295, -3, 123456789], js):
        r = Mulberry32(s)
        assert [r() for _ in range(50)] == seq


def test_tasks_resolve_and_sample_like_js(node):
    js = node("import { loadTask, sampleInitial } from './src/flight/tasks.js'; import { TASKS } from './src/core/spec.js';"
              "const out = {}; for (const id of Object.keys(TASKS)) { const t = loadTask(id);"
              "out[id] = { task: t, init: Array.from({length: 25}, (_, s) => sampleInitial(t, s)) }; } console.log(JSON.stringify(out));")
    assert set(js) == set(TASKS)
    for tid, v in js.items():
        t = load_task(tid)
        assert t == v['task']
        assert [sample_initial(t, s) for s in range(25)] == v['init']
    pad = load_task('precision-pad')
    assert pad['success']['max_vertical_speed'] == 2.0 and pad['success']['max_pad_distance'] == 1.5
    assert pad['success']['max_horizontal_speed'] == 0.5
    assert pad['metrics'][:2] == ['success', 'touchdown_vertical_speed'] and pad['metrics'][-1] == 'pad_distance'
    assert load_task('tumble-recovery')['initial']['presets'] == ['roof', 'corner', 'side']
    assert len(NOZZLE_IDS) == 10
