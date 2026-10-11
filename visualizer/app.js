"use strict";
(() => {
  const G = RadarGeometry;
  const $ = id => document.getElementById(id);
  const plan = $("plan"), key = "ld2460-room-visualizer-v1";
  const defaults = () => ({ heading: 0, points: [{ x: -3, y: 0 }, { x: 3, y: 0 }, { x: 2, y: 5 }, { x: -2, y: 5 }],
    person: { x: 0, y: 2 }, showPerson: true, distance: 6, start: -60, end: 60, coverage: true });
  let state = defaults(), transform, dragging = null, inputsValid = true, personInputsValid = true, storageAvailable = true;
  const fmt = n => (Math.abs(n) < .05 ? 0 : n).toFixed(1);
  const numeric = (n, min, max) => Number.isFinite(n) && n >= min && n <= max;
  try {
    const saved = JSON.parse(localStorage.getItem(key));
    if (saved && numeric(saved.heading, -180, 180) && Array.isArray(saved.points) && saved.points.length === 4 &&
        saved.points.every(p => p && numeric(p.x, -12, 12) && numeric(p.y, -12, 12)) &&
        numeric(saved.distance, .1, 12) && numeric(saved.start, -180, 180) && numeric(saved.end, -180, 180) &&
        saved.start < saved.end && typeof saved.coverage === "boolean") {
      state = { ...defaults(), ...saved, points: saved.points.map(p => ({ x: G.snap(p.x), y: G.snap(p.y) })),
        person: saved.person && numeric(saved.person.x, -12, 12) && numeric(saved.person.y, -12, 12) ?
          { x: G.snap(saved.person.x), y: G.snap(saved.person.y) } : defaults().person,
        showPerson: typeof saved.showPerson === "boolean" ? saved.showPerson : true };
    }
  } catch { storageAvailable = false; }

  function persist() {
    try { localStorage.setItem(key, JSON.stringify(state)); storageAvailable = true; }
    catch { storageAvailable = false; }
    $("save-note").textContent = storageAvailable ? "Settings saved in this browser. No data is sent anywhere." :
      "Browser storage is unavailable. Keep this tab open or copy your corner parameters.";
  }
  function populate() {
    $("heading").value = $("heading-range").value = state.heading;
    state.points.forEach((p, i) => ["x", "y"].forEach(axis => $(`p${i + 1}-${axis}`).value = fmt(p[axis])));
    $("distance").value = state.distance;
    $("start-angle").value = state.start;
    $("end-angle").value = state.end;
    $("show-coverage").checked = state.coverage;
    ["x", "y"].forEach(axis => $(`person-${axis}`).value = fmt(state.person[axis]));
    $("show-person").checked = state.showPerson;
  }
  function svg(parent, tag, attrs, content) {
    const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [name, value] of Object.entries(attrs)) el.setAttribute(name, value);
    if (content !== undefined) el.textContent = content;
    parent.appendChild(el);
    return el;
  }
  function sectorPoints() {
    const points = [{ x: 0, y: 0 }];
    const steps = Math.max(2, Math.ceil((state.end - state.start) / 3));
    for (let i = 0; i <= steps; i++) {
      const a = (state.start + (state.end - state.start) * i / steps) * Math.PI / 180;
      points.push({ x: state.distance * Math.sin(a), y: state.distance * Math.cos(a) });
    }
    return points;
  }
  function fit() {
    const points = [...state.points, { x: -2, y: 0 }, { x: 2, y: 0 }, { x: 0, y: -1.5 }, { x: 0, y: 2.5 }];
    if (state.coverage) points.push(...sectorPoints());
    if (state.showPerson && personInputsValid) points.push(state.person);
    const world = points.map(p => G.rotate(p, state.heading));
    const minX = Math.min(...world.map(p => p.x)) - 1.2, maxX = Math.max(...world.map(p => p.x)) + 1.2;
    const minY = Math.min(...world.map(p => p.y)) - 1.2, maxY = Math.max(...world.map(p => p.y)) + 1.2;
    const scale = Math.min(820 / (maxX - minX), 600 / (maxY - minY));
    transform = { scale, cx: (minX + maxX) / 2, cy: (minY + maxY) / 2 };
  }
  function screen(p) { return { x: 450 + (p.x - transform.cx) * transform.scale, y: 340 - (p.y - transform.cy) * transform.scale }; }
  function radarScreen(p) { return screen(G.rotate(p, state.heading)); }
  function fromScreen(p) { return G.unrotate({ x: (p.x - 450) / transform.scale + transform.cx,
    y: (340 - p.y) / transform.scale + transform.cy }, state.heading); }
  function pointer(event) {
    const point = plan.createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
    return fromScreen(point.matrixTransform(plan.getScreenCTM().inverse()));
  }
  function path(points) { return points.map((p, i) => { const s = radarScreen(p); return `${i ? "L" : "M"}${s.x},${s.y}`; }).join(" ") + " Z"; }
  function drawGrid() {
    const group = $("grid");
    const minX = Math.ceil(transform.cx - 450 / transform.scale), maxX = Math.floor(transform.cx + 450 / transform.scale);
    const minY = Math.ceil(transform.cy - 380 / transform.scale), maxY = Math.floor(transform.cy + 340 / transform.scale);
    for (let x = minX; x <= maxX; x++) {
      const s = screen({ x, y: 0 });
      svg(group, "line", { x1: s.x, y1: 0, x2: s.x, y2: 720, class: "grid-line" });
    }
    for (let y = minY; y <= maxY; y++) {
      const s = screen({ x: 0, y });
      svg(group, "line", { x1: 0, y1: s.y, x2: 900, y2: s.y, class: "grid-line" });
    }
    // A fixed length, independent of heading, makes the grid's metre scale explicit.
    svg(group, "path", { d: `M30,28 v6 h${transform.scale} v-6`, fill: "none", stroke: "#89a896", "stroke-width": 2 });
    svg(group, "text", { x: 30 + transform.scale / 2, y: 21, "text-anchor": "middle", class: "grid-label" }, "1 m");
  }
  function drawAxes() {
    const group = $("axes"), origin = radarScreen({ x: 0, y: 0 });
    const length = Math.max(1.5, Math.min(3, 145 / transform.scale));
    const vectors = [
      { p: { x: length, y: 0 }, color: "#ffbe79", marker: "x", label: "+X", positive: true },
      { p: { x: -length, y: 0 }, color: "#ffbe79", marker: "x", label: "−X" },
      { p: { x: 0, y: length }, color: "#65dfba", marker: "y", label: "+Y · forward", positive: true },
      { p: { x: 0, y: -length }, color: "#65dfba", marker: "y", label: "−Y · behind" }
    ];
    for (const v of vectors) {
      const end = radarScreen(v.p);
      svg(group, "line", { x1: origin.x, y1: origin.y, x2: end.x, y2: end.y, stroke: v.color,
        "stroke-width": v.positive ? 2 : 1.3, "stroke-dasharray": v.positive ? "none" : "5 5",
        "marker-end": `url(#arrow-${v.marker})`, opacity: v.positive ? 1 : .7 });
      const dx = end.x - origin.x, dy = end.y - origin.y;
      const norm = Math.hypot(dx, dy), offset = 26;
      svg(group, "text", { x: end.x + dx / norm * offset, y: end.y + dy / norm * offset + (v.marker === "x" ? 23 : 4),
        fill: v.color, "text-anchor": Math.abs(dx) < 10 ? "middle" : dx > 0 ? "start" : "end", class: "axis-label" }, v.label);
    }
    const groupDevice = svg($("device"), "g", { transform: `translate(${origin.x} ${origin.y}) rotate(${state.heading})` });
    svg(groupDevice, "rect", { x: -17, y: -11, width: 34, height: 22, rx: 6, fill: "#e4f0e8", stroke: "#111b17", "stroke-width": 3 });
    svg(groupDevice, "path", { d: "M-9,-5 H9", stroke: "#1d8060", "stroke-width": 3, "stroke-linecap": "round" });
    svg(groupDevice, "path", { d: "M-4,-16 L0,-22 L4,-16", fill: "none", stroke: "#65dfba", "stroke-width": 2 });
    svg($("device"), "text", { x: origin.x, y: origin.y + 32, "text-anchor": "middle", class: "axis-label", fill: "#d5eadd" }, "RADAR (0, 0)");
  }
  function exportText() {
    return "# Radar coordinates in metres; plan heading is not a device setting.\nld2460:\n  tracking:\n" +
      state.points.map((p, i) => ["x", "y"].map(axis =>
        `    point_${i + 1}_${axis}:\n      initial_value: ${fmt(p[axis])}`).join("\n")).join("\n") + "\n";
  }
  function drawPerson(roomError) {
    const status = $("person-status");
    status.classList.toggle("error", !personInputsValid);
    if (!personInputsValid) {
      status.textContent = "Enter person X and Y in 0.1 m steps, between −12 and +12 m.";
      return;
    }
    const p = state.person, distance = Math.hypot(p.x, p.y);
    const position = roomError ? "Room needs correction" : G.contains(state.points, p) ? "Inside room" : "Outside room";
    status.textContent = `${state.showPerson ? position : "Marker hidden"} · ${distance.toFixed(2)} m from radar`;
    if (!state.showPerson) return;
    const pos = radarScreen(p), origin = radarScreen({ x: 0, y: 0 });
    svg($("person-ray"), "line", { x1: origin.x, y1: origin.y, x2: pos.x, y2: pos.y,
      stroke: "#88cbff", "stroke-width": 1.5, "stroke-dasharray": "4 6", "pointer-events": "none" });
    const group = svg($("person"), "g", { class: "person-marker", transform: `translate(${pos.x} ${pos.y})`,
      tabindex: "0", role: "button", "aria-label": `Person, X ${fmt(p.x)}, Y ${fmt(p.y)} metres. Arrow keys move you on the plan.` });
    svg(group, "circle", { r: 28, fill: "transparent" });
    svg(group, "circle", { r: 13, fill: "#203c50", stroke: "#88cbff", "stroke-width": 2, class: "person-ring" });
    svg(group, "circle", { cx: 0, cy: -4, r: 3, fill: "#c0e5ff" });
    svg(group, "path", { d: "M-5 6 V4 C-5 -1 5 -1 5 4 V6 Z", fill: "#c0e5ff" });
    svg(group, "text", { x: 18, y: -18, class: "person-label" }, "YOU");
  }
  function render(refit = true) {
    if (refit || !transform) fit();
    for (const id of ["grid", "coverage", "room", "person-ray", "axes", "device", "corners", "person", "probe"]) $(id).replaceChildren();
    drawGrid();
    if (state.coverage) svg($("coverage"), "path", { d: path(sectorPoints()), fill: "#6ebd92", "fill-opacity": .07,
      stroke: "#6ba487", "stroke-width": 1.5, "stroke-dasharray": "6 6" });
    const error = inputsValid ? G.validate(state.points) : "Enter all coordinates in 0.1 m steps, between −12 and +12 m.";
    svg($("room"), "path", { d: path(state.points), fill: error ? "#ec916b" : "#65dfba", "fill-opacity": error ? .08 : .13,
      stroke: error ? "#ffad8c" : "#65dfba", "stroke-width": 2.5 });
    drawAxes();
    state.points.forEach((p, i) => {
      const pos = radarScreen(p);
      const group = svg($("corners"), "g", { class: "corner", "data-index": i, tabindex: "0", role: "button",
        "aria-label": `P${i + 1}, X ${fmt(p.x)}, Y ${fmt(p.y)} metres. Arrow keys move this corner on the plan.` });
      svg(group, "circle", { cx: pos.x, cy: pos.y, r: 28, fill: "transparent", opacity: 0 });
      svg(group, "circle", { cx: pos.x, cy: pos.y, r: 7 });
      const label = svg(group, "text", { x: pos.x + 15, y: pos.y - 13 });
      svg(label, "tspan", {}, `P${i + 1}`);
      svg(label, "tspan", { x: pos.x + 15, dy: 17, "font-size": 11, "font-weight": 400 }, `(${fmt(p.x)}, ${fmt(p.y)})`);
    });
    drawPerson(error);
    $("status").classList.toggle("error", !!error);
    $("status").textContent = error || "✓ Valid room · corners follow the perimeter";
    $("behind").hidden = !state.points.some(p => p.y < 0);
    $("area").textContent = error ? "Room needs correction" : `${G.area(state.points).toFixed(2)} m² room area`;
    $("copy").disabled = !!error;
    $("parameters").value = error ? `# ${error}` : exportText();
    persist();
  }
  function readPoints() {
    const points = state.points.map((_, i) => ({ x: $(`p${i + 1}-x`).valueAsNumber, y: $(`p${i + 1}-y`).valueAsNumber }));
    inputsValid = points.every((p, i) => ["x", "y"].every(axis => $(`p${i + 1}-${axis}`).validity.valid && Number.isFinite(p[axis])));
    if (inputsValid) state.points = points.map(p => ({ x: G.snap(p.x), y: G.snap(p.y) }));
    render();
  }
  state.points.forEach((_, i) => ["x", "y"].forEach(axis => $(`p${i + 1}-${axis}`).addEventListener("input", readPoints)));
  function readPerson() {
    personInputsValid = ["x", "y"].every(axis => $(`person-${axis}`).validity.valid && Number.isFinite($(`person-${axis}`).valueAsNumber));
    if (personInputsValid) state.person = { x: G.snap($("person-x").valueAsNumber), y: G.snap($("person-y").valueAsNumber) };
    render();
  }
  ["x", "y"].forEach(axis => $(`person-${axis}`).addEventListener("input", readPerson));
  $("show-person").addEventListener("change", () => { state.showPerson = $("show-person").checked; render(); });
  function readHeading(source) {
    if (!source.validity.valid || !Number.isFinite(source.valueAsNumber)) return;
    state.heading = source.valueAsNumber;
    $("heading").value = $("heading-range").value = state.heading;
    render();
  }
  ["heading", "heading-range"].forEach(id => $(id).addEventListener("input", () => readHeading($(id))));
  function readSector() {
    const distance = $("distance").valueAsNumber, start = $("start-angle").valueAsNumber, end = $("end-angle").valueAsNumber;
    const valid = ["distance", "start-angle", "end-angle"].every(id => $(id).validity.valid && Number.isFinite($(id).valueAsNumber)) && start < end;
    $("sector-error").hidden = valid;
    $("sector-error").textContent = "Use a range of 0.1–12 m and angles from −180° to +180°, with From < To. The last valid overlay is shown.";
    if (valid) { Object.assign(state, { distance, start, end }); render(); }
  }
  ["distance", "start-angle", "end-angle"].forEach(id => $(id).addEventListener("input", readSector));
  $("show-coverage").addEventListener("change", () => { state.coverage = $("show-coverage").checked; render(); });
  $("fit").addEventListener("click", () => render());
  $("reset").addEventListener("click", () => {
    state = defaults(); inputsValid = personInputsValid = true; $("sector-error").hidden = true;
    populate(); render();
  });
  $("copy").addEventListener("click", async () => {
    try {
      await navigator.clipboard.writeText(exportText());
      $("save-note").textContent = "Copied all eight radar corner parameters.";
    } catch {
      document.querySelector("details.export").open = true;
      $("parameters").focus(); $("parameters").select();
      $("save-note").textContent = "Parameters selected below. Press Ctrl+C (or ⌘C) to copy.";
    }
  });
  function moveCorner(i, p) {
    state.points[i] = { x: G.snap(p.x), y: G.snap(p.y) };
    // Do not replace incomplete edits to other corners during a drag.
    ["x", "y"].forEach(axis => $(`p${i + 1}-${axis}`).value = fmt(state.points[i][axis]));
    inputsValid = state.points.every((_, j) => ["x", "y"].every(axis => $(`p${j + 1}-${axis}`).validity.valid && Number.isFinite($(`p${j + 1}-${axis}`).valueAsNumber)));
    if (inputsValid) state.points = state.points.map((_, j) => ({ x: G.snap($(`p${j + 1}-x`).valueAsNumber), y: G.snap($(`p${j + 1}-y`).valueAsNumber) }));
    render(false);
  }
  function movePerson(p) {
    state.person = { x: G.snap(p.x), y: G.snap(p.y) };
    personInputsValid = true;
    ["x", "y"].forEach(axis => $(`person-${axis}`).value = fmt(state.person[axis]));
    render(false);
  }
  plan.addEventListener("pointerdown", event => {
    const target = event.target.closest(".corner, .person-marker");
    if (!target || (event.pointerType === "mouse" && event.button !== 0)) return;
    event.preventDefault();
    dragging = { person: target.classList.contains("person-marker"), index: Number(target.dataset.index), pointer: event.pointerId };
    plan.setPointerCapture(event.pointerId);
  });
  plan.addEventListener("pointermove", event => {
    const p = pointer(event);
    if (dragging && event.pointerId === dragging.pointer) {
      if (dragging.person) movePerson(p);
      else moveCorner(dragging.index, p);
    }
    const error = G.validate(state.points);
    const inside = inputsValid && !error && G.contains(state.points, p);
    $("cursor").textContent = `Radar X ${fmt(p.x)} m / Y ${fmt(p.y)} m${inputsValid && !error ? inside ? " · inside room" : " · outside room" : ""}`;
    $("probe").replaceChildren();
    if (!dragging) {
      const pos = radarScreen(p);
      svg($("probe"), "circle", { cx: pos.x, cy: pos.y, r: 3, fill: inside ? "#65dfba" : "#9cb7a7", "pointer-events": "none" });
    }
  });
  function finishDrag(event) {
    if (!dragging || event.pointerId !== dragging.pointer) return;
    dragging = null;
    if (plan.hasPointerCapture(event.pointerId)) plan.releasePointerCapture(event.pointerId);
    render();
  }
  plan.addEventListener("pointerup", finishDrag);
  plan.addEventListener("pointercancel", finishDrag);
  plan.addEventListener("lostpointercapture", event => { if (dragging) finishDrag(event); });
  plan.addEventListener("pointerleave", () => {
    if (!dragging) { $("probe").replaceChildren(); $("cursor").textContent = "Move over the plan to read radar X / Y."; }
  });
  plan.addEventListener("keydown", event => {
    const target = event.target.closest(".corner, .person-marker");
    const directions = { ArrowLeft: [-.1, 0], ArrowRight: [.1, 0], ArrowUp: [0, .1], ArrowDown: [0, -.1] };
    if (!target || !directions[event.key]) return;
    event.preventDefault();
    const direction = directions[event.key];
    const delta = G.unrotate({ x: direction[0], y: direction[1] }, state.heading);
    if (target.classList.contains("person-marker")) {
      movePerson({ x: state.person.x + delta.x, y: state.person.y + delta.y });
      plan.querySelector(".person-marker").focus();
    } else {
      const i = Number(target.dataset.index);
      moveCorner(i, { x: state.points[i].x + delta.x, y: state.points[i].y + delta.y });
      plan.querySelector(`[data-index="${i}"]`).focus();
    }
  });
  populate(); render();
})();
