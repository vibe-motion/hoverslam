// Standard experiment result content, shared by the interactive overlay and offline film.
// New experiments supply title, success, metric rows, optional details and note; styling stays in the renderers.
export function resultTemplate({ title, success, metrics = [], details = [], note = '' }) {
  return { title, color: success ? '#93d3b8' : '#ff5a4e', lines: [...metrics, ...details].filter(Boolean), note };
}

export function flightResult(v) {
  const c = v.crit || {};
  const mark = (value, limit, unit, digits = 2) => `${value.toFixed(digits)}${unit}${limit !== undefined ? (value <= limit ? ' ✓' : ' ✗') : ''}`;
  const landing = `触地垂直速度 ${mark(v.vs, c.max_vertical_speed, ' m/s')}   ·   倾角 ${mark(v.tilt, c.max_tilt_deg, '°', 1)}   ·   角速度 ${mark(v.rate, c.max_angular_rate, ' rad/s')}   ·   ${v.hull ? '车身触地 ✗' : '四轮着地 ✓'}${v.hold !== undefined && c.upright_hold ? `   ·   ${c.upright_hold} s 内最大倾角 ${mark(v.hold, c.max_tilt_deg, '°', 0)}` : ''}`;
  return resultTemplate({ title: verdictLabel(v), success: v.success, metrics: [landing, padVerdict(v)], details: [v.stop], note: v.note });
}

function padVerdict(v, separator = '   ·   ') {
  if (!Number.isFinite(v.padDistance) && !Number.isFinite(v.horizontalSpeed)) return '';
  const c = v.crit || {}, metric = (label, value, limit, unit) => {
    if (!Number.isFinite(value)) return null;
    const threshold = Number.isFinite(limit) ? ` / ${limit.toFixed(1)}${unit}` : '';
    const mark = Number.isFinite(limit) ? (value <= limit ? ' ✓' : ' ✗') : '';
    return `${label} ${value.toFixed(2)}${unit}${threshold}${mark}`;
  };
  return [metric('距靶心', v.padDistance, c.max_pad_distance, ' m'), metric('水平速度', v.horizontalSpeed, c.max_horizontal_speed, ' m/s')].filter(Boolean).join(separator);
}
/** 软着陆 = task success; 硬着陆 = the touchdown itself broke a limit; 着陆后翻车 = soft touchdown, then a hull strike / roll-over. */
export function verdictLabel(v) {
  if (v.success) return v.crit?.max_pad_distance !== undefined ? '定点降落' : v.crit?.stop_speed !== undefined ? '软着陆 · 刹停' : '软着陆';
  if (v.kerb) return '冲上路缘';
  const c = v.crit || {};
  const hardTd = (c.max_vertical_speed !== undefined && v.vs > c.max_vertical_speed) || (c.max_tilt_deg !== undefined && v.tilt > c.max_tilt_deg) || (c.max_angular_rate !== undefined && v.rate > c.max_angular_rate);
  const missedPad = (Number.isFinite(c.max_pad_distance) && Number.isFinite(v.padDistance) && v.padDistance > c.max_pad_distance)
    || (Number.isFinite(c.max_horizontal_speed) && Number.isFinite(v.horizontalSpeed) && v.horizontalSpeed > c.max_horizontal_speed);
  if (!hardTd && missedPad) return '定点降落未达标';
  return hardTd ? '硬着陆' : '着陆后翻车';
}
