/**
 * Animation Debugger + Facial Morph Target Inspector (EXP002.1-B).
 * A collapsible in-place panel; plain DOM, no framework. It only reads
 * snapshots and calls the handlers it is given, so it never owns state.
 * Pointer/keyboard events inside the panel stay inside it (camera and the
 * on-screen sticks are unaffected).
 */
function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text != null) node.textContent = text;
  return node;
}
const f2 = (v) => (Number.isFinite(v) ? v.toFixed(2) : '–');
const f3 = (v) => (Number.isFinite(v) ? v.toFixed(3) : '–');

export function mountDebugger(parent, handlers, { open = true } = {}) {
  const root = el('aside', 'anim-debug');
  root.id = 'anim-debug';
  root.dataset.open = open ? '1' : '0';
  const toggle = el('button', 'ad-toggle', '');
  toggle.type = 'button';
  toggle.setAttribute('aria-controls', 'anim-debug-body');
  const body = el('div', 'ad-body');
  body.id = 'anim-debug-body';
  root.append(toggle, body);
  const setOpen = (v) => {
    root.dataset.open = v ? '1' : '0';
    toggle.textContent = v ? 'ANIMATION DEBUG ▾' : 'ANIMATION DEBUG ▸';
    toggle.setAttribute('aria-expanded', String(v));
  };
  setOpen(open);
  toggle.addEventListener('click', () => setOpen(root.dataset.open !== '1'));
  // Keep panel interaction away from OrbitControls / sticks / movement keys.
  for (const type of ['pointerdown', 'wheel', 'touchstart']) root.addEventListener(type, (e) => e.stopPropagation(), { passive: true });

  // ---- Live readout ----
  const live = el('dl', 'ad-live');
  const fields = {};
  for (const [key, label] of [
    ['state', 'State'], ['mode', 'Mode'], ['active', 'Active clips'], ['timeScale', 'timeScale'],
    ['source', 'Input'], ['vector', 'Input vector'], ['speed', 'Speed'], ['position', 'Position'],
    ['yaw', 'Facing'], ['run', 'Run'],
  ]) {
    const dt = el('dt', null, label); const dd = el('dd'); dd.dataset.field = key;
    live.append(dt, dd); fields[key] = dd;
  }
  const weights = el('div', 'ad-weights');

  // ---- Controls ----
  const controls = el('div', 'ad-controls');
  const pause = el('button', 'ad-btn', 'Pause'); pause.type = 'button'; pause.dataset.action = 'pause';
  const modeBtn = el('button', 'ad-btn', 'Mode: Auto'); modeBtn.type = 'button'; modeBtn.dataset.action = 'mode';
  const row1 = el('div', 'ad-row'); row1.append(pause, modeBtn);
  const speedLabel = el('label', 'ad-slider');
  const speedText = el('span', null, 'Playback ×1.00');
  const speed = el('input'); Object.assign(speed, { type: 'range', min: '0.1', max: '2', step: '0.05', value: '1' });
  speed.setAttribute('aria-label', 'Playback speed'); speed.dataset.action = 'speed';
  speedLabel.append(speedText, speed);
  const row2 = el('div', 'ad-row');
  const select = el('select', 'ad-select'); select.setAttribute('aria-label', 'Animation clip'); select.dataset.action = 'clip';
  const play = el('button', 'ad-btn', 'Preview'); play.type = 'button'; play.dataset.action = 'preview';
  row2.append(select, play);
  const row3 = el('div', 'ad-row ad-checks');
  const follow = el('label', null); const followBox = el('input'); followBox.type = 'checkbox'; followBox.checked = true; followBox.dataset.action = 'follow';
  follow.append(followBox, ' camera follow');
  const runLock = el('label', null); const runBox = el('input'); runBox.type = 'checkbox'; runBox.dataset.action = 'runlock';
  runLock.append(runBox, ' run lock (Caps)');
  row3.append(follow, runLock);
  const status = el('div', 'ad-status');
  controls.append(row1, speedLabel, row2, row3, status);

  // ---- Clip list ----
  const clipBox = el('details', 'ad-section'); clipBox.open = false;
  const clipSummary = el('summary', null, 'Clips');
  const clipList = el('ul', 'ad-clips');
  clipBox.append(clipSummary, clipList);

  // ---- Face ----
  const face = el('details', 'ad-section ad-face');
  const faceSummary = el('summary', null, 'Face · morph targets');
  const faceTools = el('div', 'ad-row');
  const reset = el('button', 'ad-btn', 'Reset Face'); reset.type = 'button'; reset.dataset.action = 'reset-face';
  const filter = el('input', 'ad-filter'); Object.assign(filter, { type: 'search', placeholder: 'filter…' }); filter.setAttribute('aria-label', 'Filter morph targets');
  faceTools.append(reset, filter);
  const faceList = el('div', 'ad-morphs');
  face.append(faceSummary, faceTools, faceList);

  body.append(live, weights, controls, clipBox, face);
  parent.append(root);

  pause.addEventListener('click', () => handlers.onPause?.());
  modeBtn.addEventListener('click', () => handlers.onMode?.());
  speed.addEventListener('input', () => handlers.onSpeed?.(Number(speed.value)));
  play.addEventListener('click', () => handlers.onPreview?.(select.value));
  select.addEventListener('change', () => handlers.onSelect?.(select.value));
  followBox.addEventListener('change', () => handlers.onFollow?.(followBox.checked));
  runBox.addEventListener('change', () => handlers.onRunLock?.(runBox.checked));
  reset.addEventListener('click', () => { handlers.onResetFace?.(); for (const s of faceList.querySelectorAll('input')) { s.value = '0'; s.nextSibling.textContent = '0.00'; } });
  filter.addEventListener('input', () => {
    const q = filter.value.trim().toLowerCase();
    for (const row of faceList.children) row.hidden = !!q && !row.dataset.name.toLowerCase().includes(q);
  });

  function setClips(list) {
    select.replaceChildren(...list.map((c) => { const o = el('option', null, `${c.name}${c.loop ? '' : ' (once)'}`); o.value = c.name; return o; }));
    clipList.replaceChildren(...list.map((c) => {
      const li = el('li'); li.dataset.clip = c.name;
      li.textContent = `${c.name} · ${c.duration.toFixed(2)} s · ${c.loop ? 'loop' : 'once'}${c.natural ? ` · ${c.natural.toFixed(3)} m/s` : ''}`;
      return li;
    }));
    clipSummary.textContent = `Clips · ${list.length} available`;
  }

  function setMorphs(names, getWeight, setWeight, meshName) {
    faceSummary.textContent = `Face · ${names.length} morph targets${meshName ? ` (${meshName})` : ''}`;
    faceList.replaceChildren(...names.map((name, i) => {
      const row = el('label', 'ad-morph'); row.dataset.name = name;
      const n = el('span', 'ad-morph-name', `${String(i).padStart(2, '0')} ${name}`);
      const s = el('input'); Object.assign(s, { type: 'range', min: '0', max: '1', step: '0.01', value: String(getWeight(name) || 0) });
      s.dataset.morph = name; s.setAttribute('aria-label', `morph ${name}`);
      const v = el('span', 'ad-morph-val', f2(getWeight(name) || 0));
      s.addEventListener('input', () => { setWeight(name, Number(s.value)); v.textContent = f2(Number(s.value)); });
      row.append(n, s, v);
      return row;
    }));
  }

  let lastPaint = 0;
  function update(d, now = performance.now()) {
    if (root.dataset.open !== '1' || now - lastPaint < 100) return;
    lastPaint = now;
    const a = d.anim;
    fields.state.textContent = a.state + (a.mode === 'manual' && a.previewClip ? ` · ${a.previewClip}` : '');
    fields.mode.textContent = a.mode === 'auto' ? 'Automatic' : 'Manual';
    const visible = a.clips.filter((c) => c.weight > 0.001).sort((x, y) => y.weight - x.weight);
    fields.active.textContent = visible.map((c) => `${c.name}${c.finished ? ' (holding)' : ''}`).join(', ') || '—';
    fields.timeScale.textContent = visible.map((c) => `${c.name} ×${f2(c.timeScale)}`).join(', ') || '—';
    fields.source.textContent = d.input.source;
    fields.vector.textContent = `(${f2(d.input.x)}, ${f2(d.input.y)}) |${f2(Math.hypot(d.input.x, d.input.y))}| gait ${f2(d.input.gait)}`;
    fields.speed.textContent = `${f3(Math.abs(d.move.speed))} m/s ${d.move.speed < -1e-3 ? 'back' : d.move.speed > 1e-3 ? 'fwd' : ''} (target ${f3(Math.abs(d.move.targetSpeed))}) · measured ${f3(d.measuredSpeed)}`;
    fields.position.textContent = `x ${f2(d.move.position.x)}  z ${f2(d.move.position.z)} m`;
    fields.yaw.textContent = `${(d.move.yaw * 180 / Math.PI).toFixed(0)}°`;
    fields.run.textContent = `${d.run.shift ? 'Shift ' : ''}${d.run.capsLock ? 'CapsLock ' : ''}${!d.run.shift && !d.run.capsLock ? 'off' : 'on'}`;
    weights.replaceChildren(...a.clips.filter((c) => c.weight > 0.001 || c.target > 0).map((c) => {
      const r = el('div', 'ad-wrow'); r.dataset.clip = c.name;
      const bar = el('i'); bar.style.width = `${(c.weight * 100).toFixed(1)}%`;
      r.append(el('span', null, c.name), el('b', null, f2(c.weight)), bar);
      return r;
    }));
    pause.textContent = a.paused ? 'Resume' : 'Pause';
    modeBtn.textContent = a.mode === 'auto' ? 'Mode: Auto' : 'Mode: Manual';
    speedText.textContent = `Playback ×${f2(a.speed)}`;
    if (Number(speed.value) !== a.speed && document.activeElement !== speed) speed.value = String(a.speed);
    runBox.checked = d.run.capsLock;
    status.textContent = a.warnings.length ? a.warnings.join(' · ') : '';
    for (const li of clipList.children) li.dataset.active = visible.some((c) => c.name === li.dataset.clip) ? '1' : '0';
  }

  return { root, update, setClips, setMorphs, setOpen, get open() { return root.dataset.open === '1'; } };
}
