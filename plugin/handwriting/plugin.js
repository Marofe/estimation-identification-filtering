/*!
 * handwriting - a minimal, self-contained handwriting/annotation plugin for reveal.js
 * Draw over the current slide with mouse, touch, or stylus. Ink is stored per slide
 * and redrawn on navigation/resize. No external dependencies (no Font Awesome, no CDN).
 *
 * Usage: include this script anywhere after dist/reveal.js (it self-attaches to Reveal).
 *
 * Shortcuts:  C = turn ON (toolbar + pen)   Esc = turn OFF   E = eraser   X = next colour   Z = undo
 * Toolbar (bottom-left): hidden by default. Press C to show it and start drawing;
 * press Esc to hide it. (When hidden, Esc falls through to Reveal's slide overview.)
 */
(function () {
  'use strict';
  if (window.__handwritingReady) return;
  window.__handwritingReady = true;

  var COLORS = ['#d81e1e', '#1e5fd8', '#0a9a0a', '#f0a000', '#111111'];
  var PEN_W = 3, ERASE_W = 24;

  var state = { on: false, erase: false, colorIdx: 0, drawing: false, cur: null, visible: true };
  var strokes = {};              // slideKey -> [ {color,width,erase,pts:[{x,y}...normalized]} ]
  var key = '0-0';
  var canvas, ctx, bar, penBtn, eraseBtn, swatch, dpr = window.devicePixelRatio || 1;

  // ---------- styles (injected, no external CSS needed) ----------
  function injectCSS() {
    var s = document.createElement('style');
    s.textContent =
      '.hw-canvas{position:fixed;inset:0;width:100vw;height:100vh;z-index:9000;' +
        'pointer-events:none;touch-action:none;}' +
      '.hw-canvas.hw-active{pointer-events:auto;cursor:crosshair;}' +
      '.hw-bar{position:fixed;left:14px;bottom:14px;z-index:9001;display:flex;gap:6px;' +
        'align-items:center;background:rgba(255,255,255,.94);border:1px solid #c9c9c9;' +
        'border-radius:11px;padding:5px 7px;box-shadow:0 3px 12px rgba(0,0,0,.18);' +
        'font-family:system-ui,-apple-system,sans-serif;user-select:none;}' +
      '.hw-bar button{width:32px;height:32px;border:1px solid #cfcfcf;border-radius:8px;' +
        'background:#fff;cursor:pointer;font-size:16px;line-height:1;display:flex;' +
        'align-items:center;justify-content:center;padding:0;color:#222;}' +
      '.hw-bar button:hover{background:#f1f1f1;}' +
      '.hw-bar button.hw-sel{outline:2px solid #333;background:#eee;}' +
      '.hw-swatch{width:32px;height:32px;border:1px solid #cfcfcf;border-radius:8px;' +
        'cursor:pointer;box-shadow:inset 0 0 0 3px #fff;}';
    document.head.appendChild(s);
  }

  // ---------- geometry helpers ----------
  function W() { return window.innerWidth; }
  function H() { return window.innerHeight; }
  function norm(e) { return { x: e.clientX / W(), y: e.clientY / H() }; }
  function denorm(p) { return { x: p.x * W(), y: p.y * H() }; }

  function sizeCanvas() {
    dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(W() * dpr);
    canvas.height = Math.round(H() * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    redraw();
  }

  // ---------- drawing ----------
  function applyStyle(stroke) {
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    if (stroke.erase) {
      ctx.globalCompositeOperation = 'destination-out';
      ctx.strokeStyle = 'rgba(0,0,0,1)';
    } else {
      ctx.globalCompositeOperation = 'source-over';
      ctx.strokeStyle = stroke.color;
    }
    ctx.lineWidth = stroke.width;
  }

  function drawStroke(stroke) {
    if (!stroke.pts.length) return;
    applyStyle(stroke);
    ctx.beginPath();
    var p0 = denorm(stroke.pts[0]);
    ctx.moveTo(p0.x, p0.y);
    for (var i = 1; i < stroke.pts.length; i++) {
      var p = denorm(stroke.pts[i]);
      ctx.lineTo(p.x, p.y);
    }
    if (stroke.pts.length === 1) { ctx.lineTo(p0.x + 0.1, p0.y + 0.1); }
    ctx.stroke();
    ctx.globalCompositeOperation = 'source-over';
  }

  function redraw() {
    ctx.clearRect(0, 0, W(), H());
    var list = strokes[key] || [];
    for (var i = 0; i < list.length; i++) drawStroke(list[i]);
  }

  // ---------- pointer handlers ----------
  function onDown(e) {
    if (!state.on) return;
    e.preventDefault();
    state.drawing = true;
    state.cur = { color: COLORS[state.colorIdx], width: state.erase ? ERASE_W : PEN_W,
                  erase: state.erase, pts: [norm(e)] };
    try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
    drawStroke(state.cur);
  }
  function onMove(e) {
    if (!state.drawing) return;
    e.preventDefault();
    state.cur.pts.push(norm(e));
    // incremental segment for smooth live feedback
    applyStyle(state.cur);
    var n = state.cur.pts.length;
    var a = denorm(state.cur.pts[n - 2]), b = denorm(state.cur.pts[n - 1]);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    ctx.globalCompositeOperation = 'source-over';
  }
  function onUp(e) {
    if (!state.drawing) return;
    state.drawing = false;
    (strokes[key] = strokes[key] || []).push(state.cur);
    state.cur = null;
  }

  // ---------- toolbar ----------
  function mkBtn(txt, title, fn) {
    var b = document.createElement('button');
    b.innerHTML = txt; b.title = title; b.onclick = fn; return b;
  }
  function buildBar() {
    bar = document.createElement('div'); bar.className = 'hw-bar';
    penBtn = mkBtn('&#9998;', 'Pen — draw over slide (C)', function () {
      if (state.on && !state.erase) { setPen(false); } else { state.erase = false; setPen(true); }
    });
    eraseBtn = mkBtn('&#9109;', 'Eraser (E)', function () { toggleErase(); });
    swatch = document.createElement('div'); swatch.className = 'hw-swatch'; swatch.title = 'Colour (X)';
    swatch.onclick = cycleColor;
    var undoBtn = mkBtn('&#8630;', 'Undo (Z)', undo);
    var clearBtn = mkBtn('&#128465;', 'Clear this slide', clearSlide);
    bar.appendChild(penBtn); bar.appendChild(eraseBtn); bar.appendChild(swatch);
    bar.appendChild(undoBtn); bar.appendChild(clearBtn);
    bar.style.display = 'none';            // hidden until the user presses C
    document.body.appendChild(bar);
    updateSwatch(); syncButtons();
  }
  function barVisible() { return bar && bar.style.display !== 'none'; }
  function setBar(show) {                  // C turns ON, Esc turns OFF
    if (bar) bar.style.display = show ? 'flex' : 'none';
    if (show) { state.erase = false; setPen(true); }
    else { setPen(false); }
    syncButtons();
  }
  function updateSwatch() { swatch.style.background = COLORS[state.colorIdx]; }
  function syncButtons() {
    penBtn.classList.toggle('hw-sel', state.on && !state.erase);
    eraseBtn.classList.toggle('hw-sel', state.on && state.erase);
  }

  // ---------- actions ----------
  function setPen(on) {
    state.on = on;
    canvas.classList.toggle('hw-active', on);
    syncButtons();
  }
  function toggleErase(force) {
    state.erase = (typeof force === 'boolean') ? force : !state.erase;
    if (state.erase) setPen(true); else syncButtons();
    syncButtons();
  }
  function cycleColor() {
    state.colorIdx = (state.colorIdx + 1) % COLORS.length;
    state.erase = false; setPen(true); updateSwatch(); syncButtons();
  }
  function undo() { if (strokes[key] && strokes[key].length) { strokes[key].pop(); redraw(); } }
  function clearSlide() { strokes[key] = []; redraw(); }

  // ---------- keyboard ----------
  function onKey(e) {
    var t = (e.target && e.target.tagName) || '';
    if (t === 'INPUT' || t === 'TEXTAREA' || e.metaKey || e.ctrlKey || e.altKey) return;
    var k = e.key.toLowerCase();

    if (k === 'c') { e.stopPropagation(); setBar(true); return; }   // C = turn ON
    if (k === 'escape') {                                           // Esc = turn OFF
      if (barVisible()) { e.preventDefault(); e.stopPropagation(); setBar(false); }
      return;                        // when already off, let Reveal handle Esc (overview)
    }

    switch (k) {
      case 'e': e.stopPropagation(); toggleErase(); break;
      case 'x': e.stopPropagation(); cycleColor(); break;
      case 'z': e.stopPropagation(); undo(); break;
    }
  }

  // ---------- setup ----------
  function setup(deck) {
    injectCSS();
    canvas = document.createElement('canvas'); canvas.className = 'hw-canvas';
    document.body.appendChild(canvas);
    ctx = canvas.getContext('2d');
    sizeCanvas();
    buildBar();

    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerup', onUp);
    canvas.addEventListener('pointercancel', onUp);
    window.addEventListener('resize', sizeCanvas);
    document.addEventListener('keydown', onKey, true);

    function setKey() {
      var idx = deck.getIndices();
      key = idx.h + '-' + idx.v;
      redraw();
    }
    deck.on('slidechanged', setKey);
    deck.on('ready', setKey);
    setKey();
  }

  // attach to Reveal whether or not it is already initialized
  function attach() {
    if (!window.Reveal) { window.addEventListener('load', attach); return; }
    if (Reveal.isReady && Reveal.isReady()) setup(Reveal);
    else Reveal.on('ready', function () { setup(Reveal); });
  }
  attach();
})();
