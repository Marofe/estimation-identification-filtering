/*!
 * handwriting - a minimal, self-contained handwriting/annotation plugin for reveal.js
 * Draw over the current slide with mouse, touch, or stylus. Ink is stored per slide
 * and redrawn on navigation/resize. No external dependencies (no Font Awesome, no CDN).
 *
 * Usage: include this script anywhere after dist/reveal.js (it self-attaches to Reveal).
 *
 * Shortcuts:
 *   C   = show controls (toolbar + pen)      Esc = hide controls
 *   E   = eraser         X = next colour      Z = undo
 *   1..9,0 = pick colour directly            N  = pop controls out into a new window
 *
 * NEW:
 *   - Full colour palette shown at once (click any swatch to pick it).
 *   - "Pop out" (N) opens the controls in a SEPARATE window you can drag to a second
 *     monitor, so the toolbar never overlays the projected slides. The drawing surface
 *     stays on the presentation; only the controls move.
 */
(function () {
  'use strict';
  if (window.__handwritingReady) return;
  window.__handwritingReady = true;

  // Full palette — all shown at once. Edit / reorder freely.
  var COLORS = [
    { c: '#d81e1e', name: 'Red' },
    { c: '#f0640a', name: 'Orange' },
    { c: '#f2c200', name: 'Yellow' },
    { c: '#0a9a0a', name: 'Green' },
    { c: '#00a79d', name: 'Teal' },
    { c: '#1e5fd8', name: 'Blue' },
    { c: '#7b2fd8', name: 'Purple' },
    { c: '#d81e9a', name: 'Magenta' },
    { c: '#111111', name: 'Black' },
    { c: '#ffffff', name: 'White' }
  ];
  var WIDTHS = [{ w: 2, name: 'S' }, { w: 4, name: 'M' }, { w: 7, name: 'L' }];
  var ERASE_W = 26;

  var state = { on: false, erase: false, colorIdx: 0, widthIdx: 1, drawing: false, cur: null };
  var strokes = {};              // slideKey -> [ {color,width,erase,pts:[{x,y}...normalized]} ]
  var key = '0-0';
  var canvas, ctx, bar, penBtn, eraseBtn, dpr = window.devicePixelRatio || 1;
  var swatchEls = [];            // in-page palette swatches
  var widthEls = [];
  var ctrlWin = null;            // popped-out control window
  var ctrlEls = null;            // references to popup elements for live sync

  function penW() { return WIDTHS[state.widthIdx].w; }
  function curColor() { return COLORS[state.colorIdx].c; }

  // ---------- styles (injected, no external CSS needed) ----------
  function injectCSS() {
    var s = document.createElement('style');
    s.textContent =
      '.hw-canvas{position:fixed;inset:0;width:100vw;height:100vh;z-index:9000;' +
        'pointer-events:none;touch-action:none;}' +
      '.hw-canvas.hw-active{pointer-events:auto;cursor:crosshair;}' +
      '.hw-bar{position:fixed;left:14px;bottom:14px;z-index:9001;display:flex;gap:6px;' +
        'align-items:center;background:rgba(255,255,255,.96);border:1px solid #c9c9c9;' +
        'border-radius:11px;padding:6px 8px;box-shadow:0 3px 12px rgba(0,0,0,.18);' +
        'font-family:system-ui,-apple-system,sans-serif;user-select:none;flex-wrap:wrap;max-width:96vw;}' +
      '.hw-bar button{width:32px;height:32px;border:1px solid #cfcfcf;border-radius:8px;' +
        'background:#fff;cursor:pointer;font-size:16px;line-height:1;display:flex;' +
        'align-items:center;justify-content:center;padding:0;color:#222;}' +
      '.hw-bar button:hover{background:#f1f1f1;}' +
      '.hw-bar button.hw-sel{outline:2px solid #333;background:#eee;}' +
      '.hw-sep{width:1px;height:26px;background:#ddd;margin:0 2px;}' +
      '.hw-pal{display:flex;gap:4px;align-items:center;}' +
      '.hw-sw{width:26px;height:26px;border-radius:7px;cursor:pointer;border:1px solid #bbb;' +
        'box-shadow:inset 0 0 0 2px #fff;transition:transform .06s;}' +
      '.hw-sw:hover{transform:scale(1.08);}' +
      '.hw-sw.hw-sel{outline:3px solid #333;outline-offset:1px;box-shadow:none;}' +
      '.hw-wbtn{font-weight:700;}';
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
    state.cur = { color: curColor(), width: state.erase ? ERASE_W : penW(),
                  erase: state.erase, pts: [norm(e)] };
    try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
    drawStroke(state.cur);
  }
  function onMove(e) {
    if (!state.drawing) return;
    e.preventDefault();
    state.cur.pts.push(norm(e));
    applyStyle(state.cur);
    var n = state.cur.pts.length;
    var a = denorm(state.cur.pts[n - 2]), b = denorm(state.cur.pts[n - 1]);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    ctx.globalCompositeOperation = 'source-over';
  }
  function onUp() {
    if (!state.drawing) return;
    state.drawing = false;
    (strokes[key] = strokes[key] || []).push(state.cur);
    state.cur = null;
  }

  // ---------- toolbar (in-page) ----------
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

    // width selector
    var widthWrap = document.createElement('div'); widthWrap.className = 'hw-pal';
    widthEls = [];
    WIDTHS.forEach(function (wd, i) {
      var b = mkBtn(wd.name, 'Pen size ' + wd.name, function () { setWidth(i); });
      b.className += ' hw-wbtn';
      widthWrap.appendChild(b); widthEls.push(b);
    });

    // full palette (all colours at once)
    var pal = document.createElement('div'); pal.className = 'hw-pal';
    swatchEls = [];
    COLORS.forEach(function (col, i) {
      var sw = document.createElement('div'); sw.className = 'hw-sw';
      sw.style.background = col.c; sw.title = col.name + ' (' + ((i + 1) % 10) + ')';
      sw.onclick = function () { setColor(i); };
      pal.appendChild(sw); swatchEls.push(sw);
    });

    var undoBtn = mkBtn('&#8630;', 'Undo (Z)', undo);
    var clearBtn = mkBtn('&#128465;', 'Clear this slide', clearSlide);
    var popBtn = mkBtn('&#8599;', 'Pop controls into a new window (N) — move to a 2nd monitor', openControls);

    bar.appendChild(penBtn); bar.appendChild(eraseBtn);
    bar.appendChild(sep()); bar.appendChild(widthWrap);
    bar.appendChild(sep()); bar.appendChild(pal);
    bar.appendChild(sep()); bar.appendChild(undoBtn); bar.appendChild(clearBtn);
    bar.appendChild(sep()); bar.appendChild(popBtn);
    bar.style.display = 'none';
    document.body.appendChild(bar);
    updateAll();
  }
  function sep() { var d = document.createElement('div'); d.className = 'hw-sep'; return d; }
  function barVisible() { return bar && bar.style.display !== 'none'; }
  function setBar(show) {
    if (bar) bar.style.display = show ? 'flex' : 'none';
    if (show) { state.erase = false; setPen(true); }
    else if (!(ctrlWin && !ctrlWin.closed)) { setPen(false); }  // keep drawing if popup controls it
    updateAll();
  }

  // ---------- actions ----------
  function setPen(on) { state.on = on; canvas.classList.toggle('hw-active', on); updateAll(); }
  function toggleErase(force) {
    state.erase = (typeof force === 'boolean') ? force : !state.erase;
    if (state.erase) setPen(true);
    updateAll();
  }
  function setColor(i) {
    state.colorIdx = i; state.erase = false; setPen(true); updateAll();
  }
  function setWidth(i) { state.widthIdx = i; state.erase = false; setPen(true); updateAll(); }
  function cycleColor() { setColor((state.colorIdx + 1) % COLORS.length); }
  function undo() { if (strokes[key] && strokes[key].length) { strokes[key].pop(); redraw(); } }
  function clearSlide() { strokes[key] = []; redraw(); }

  // ---------- sync all UIs (in-page bar + popup) ----------
  function updateAll() {
    // in-page
    if (penBtn) penBtn.classList.toggle('hw-sel', state.on && !state.erase);
    if (eraseBtn) eraseBtn.classList.toggle('hw-sel', state.on && state.erase);
    swatchEls.forEach(function (sw, i) {
      sw.classList.toggle('hw-sel', i === state.colorIdx && !state.erase);
    });
    widthEls.forEach(function (b, i) { b.classList.toggle('hw-sel', i === state.widthIdx); });
    // popup
    if (ctrlWin && !ctrlWin.closed && ctrlEls) {
      ctrlEls.pen.classList.toggle('sel', state.on && !state.erase);
      ctrlEls.erase.classList.toggle('sel', state.on && state.erase);
      ctrlEls.sw.forEach(function (sw, i) { sw.classList.toggle('sel', i === state.colorIdx && !state.erase); });
      ctrlEls.wd.forEach(function (b, i) { b.classList.toggle('sel', i === state.widthIdx); });
    }
  }

  // ---------- pop-out control window ----------
  function openControls() {
    if (ctrlWin && !ctrlWin.closed) { ctrlWin.focus(); return; }
    ctrlWin = window.open('', 'hwControls', 'width=380,height=560,menubar=no,toolbar=no,location=no,status=no');
    if (!ctrlWin) { alert('Pop-up blocked. Please allow pop-ups for this page, then press N again.'); return; }
    var d = ctrlWin.document;
    d.open();
    d.write('<!doctype html><html><head><meta charset="utf-8"><title>Ink Controls</title></head><body></body></html>');
    d.close();
    var st = d.createElement('style');
    st.textContent =
      'body{margin:0;font-family:system-ui,-apple-system,sans-serif;background:#eef0f3;color:#1b2230;padding:14px;}' +
      'h1{font-size:15px;margin:0 0 10px;color:#333;letter-spacing:.3px;}' +
      '.row{display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap;}' +
      'button{border:1px solid #c4c8cf;border-radius:9px;background:#fff;cursor:pointer;' +
        'padding:10px 12px;font-size:15px;color:#222;flex:1 1 auto;}' +
      'button:hover{background:#f4f4f6;}' +
      'button.sel{outline:2px solid #333;background:#e9e9ef;}' +
      '.grid{display:grid;grid-template-columns:repeat(5,1fr);gap:10px;margin-bottom:14px;}' +
      '.sw{height:52px;border-radius:10px;cursor:pointer;border:1px solid #b6b6b6;' +
        'box-shadow:inset 0 0 0 3px #fff;position:relative;transition:transform .06s;}' +
      '.sw:hover{transform:scale(1.05);}' +
      '.sw.sel{outline:4px solid #222;outline-offset:1px;box-shadow:none;}' +
      '.sw span{position:absolute;bottom:3px;left:0;right:0;text-align:center;font-size:10px;' +
        'color:#333;text-shadow:0 0 2px #fff;}' +
      '.lbl{font-size:12px;color:#666;text-transform:uppercase;letter-spacing:.6px;margin:2px 0 6px;}' +
      '.hint{font-size:12px;color:#888;margin-top:10px;line-height:1.5;}';
    d.head.appendChild(st);

    var body = d.body;
    var h = d.createElement('h1'); h.textContent = '✎ Ink controls'; body.appendChild(h);

    ctrlEls = { sw: [], wd: [] };

    // tools row
    var lbl1 = d.createElement('div'); lbl1.className = 'lbl'; lbl1.textContent = 'Tool'; body.appendChild(lbl1);
    var toolRow = d.createElement('div'); toolRow.className = 'row';
    ctrlEls.pen = pbtn(d, '✎ Pen', function () {
      if (state.on && !state.erase) { setPen(false); } else { state.erase = false; setPen(true); }
    });
    ctrlEls.erase = pbtn(d, '⌫ Eraser', function () { toggleErase(true); });
    var undoB = pbtn(d, '↩ Undo', undo);
    var clrB = pbtn(d, '🗑 Clear', clearSlide);
    toolRow.appendChild(ctrlEls.pen); toolRow.appendChild(ctrlEls.erase);
    toolRow.appendChild(undoB); toolRow.appendChild(clrB);
    body.appendChild(toolRow);

    // width row
    var lbl2 = d.createElement('div'); lbl2.className = 'lbl'; lbl2.textContent = 'Pen size'; body.appendChild(lbl2);
    var wRow = d.createElement('div'); wRow.className = 'row';
    WIDTHS.forEach(function (wd, i) {
      var b = pbtn(d, wd.name, function () { setWidth(i); });
      wRow.appendChild(b); ctrlEls.wd.push(b);
    });
    body.appendChild(wRow);

    // palette grid (all colours at once)
    var lbl3 = d.createElement('div'); lbl3.className = 'lbl'; lbl3.textContent = 'Colour'; body.appendChild(lbl3);
    var grid = d.createElement('div'); grid.className = 'grid';
    COLORS.forEach(function (col, i) {
      var sw = d.createElement('div'); sw.className = 'sw'; sw.style.background = col.c;
      sw.title = col.name;
      var sp = d.createElement('span'); sp.textContent = (i + 1) % 10; sw.appendChild(sp);
      sw.onclick = function () { setColor(i); };
      grid.appendChild(sw); ctrlEls.sw.push(sw);
    });
    body.appendChild(grid);

    // on-slide toolbar toggle + hint
    var lbl4 = d.createElement('div'); lbl4.className = 'lbl'; lbl4.textContent = 'On-slide toolbar'; body.appendChild(lbl4);
    var tRow = d.createElement('div'); tRow.className = 'row';
    var hideB = pbtn(d, 'Hide from slides', function () { setBar(false); });
    var showB = pbtn(d, 'Show on slides', function () { setBar(true); });
    tRow.appendChild(hideB); tRow.appendChild(showB); body.appendChild(tRow);

    var hint = d.createElement('div'); hint.className = 'hint';
    hint.innerHTML = 'Drag this window to your second monitor. Draw on the slides with the pen; ' +
      'these controls stay off the projected screen.<br>Keys still work on the slide window: ' +
      'E eraser · Z undo · X next colour · 1–9,0 pick colour.';
    body.appendChild(hint);

    // forward key shortcuts pressed while the popup is focused
    d.addEventListener('keydown', function (e) { handleKey(e, true); });
    ctrlWin.addEventListener('beforeunload', function () {
      ctrlWin = null; ctrlEls = null; setBar(true);
    });

    // popping out: enable pen and hide the on-slide bar so nothing overlays the slides
    state.erase = false; setPen(true);
    setBar(false);
    updateAll();
  }
  function pbtn(d, txt, fn) {
    var b = d.createElement('button'); b.textContent = txt; b.onclick = fn; return b;
  }

  // ---------- keyboard ----------
  function handleKey(e, fromPopup) {
    var t = (e.target && e.target.tagName) || '';
    if (t === 'INPUT' || t === 'TEXTAREA' || e.metaKey || e.ctrlKey || e.altKey) return;
    var k = e.key.toLowerCase();

    // number keys pick a colour directly (1..9 -> 0..8, 0 -> 10th)
    if (/^[0-9]$/.test(k)) {
      var idx = (k === '0') ? 9 : (parseInt(k, 10) - 1);
      if (idx < COLORS.length) { e.stopPropagation(); setColor(idx); }
      return;
    }
    if (k === 'c') { e.stopPropagation(); setBar(true); return; }
    if (k === 'n') { e.stopPropagation(); openControls(); return; }
    if (k === 'escape') {
      if (barVisible()) { e.preventDefault(); e.stopPropagation(); setBar(false); }
      return;
    }
    switch (k) {
      case 'e': e.stopPropagation(); toggleErase(); break;
      case 'x': e.stopPropagation(); cycleColor(); break;
      case 'z': e.stopPropagation(); undo(); break;
    }
  }

  // ---------- public API (used by the popped-out window) ----------
  window.__hwAPI = {
    setColor: setColor, setWidth: setWidth, setPen: setPen,
    toggleErase: toggleErase, undo: undo, clear: clearSlide,
    showBar: function (v) { setBar(v); }, colors: COLORS
  };

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
    document.addEventListener('keydown', function (e) { handleKey(e, false); }, true);
    window.addEventListener('beforeunload', function () { if (ctrlWin && !ctrlWin.closed) ctrlWin.close(); });

    function setKey() {
      var idx = deck.getIndices();
      key = idx.h + '-' + idx.v;
      redraw();
    }
    deck.on('slidechanged', setKey);
    deck.on('ready', setKey);
    setKey();
  }

  function attach() {
    if (!window.Reveal) { window.addEventListener('load', attach); return; }
    if (Reveal.isReady && Reveal.isReady()) setup(Reveal);
    else Reveal.on('ready', function () { setup(Reveal); });
  }
  attach();
})();
