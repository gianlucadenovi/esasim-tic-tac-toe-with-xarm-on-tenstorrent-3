// ── EsaSim block ──────────────────────────────────────────────────────
// Step-by-step guide (ws-step-by-step-guide) · version 2 · view, JavaScript: function render(ctx)
// One step of a procedure that a running block leads: what to do, whether the last action worked (✓/✗), and only the buttons that step takes.
// Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
// Saved: 2026-11-09
//
// Inputs   ctx.input("id") gives { files, stream }; ctx.rows("id"), ctx.json("id"), ctx.text("id")
//   data        Stream                    Data · several · optional · The live stream of a Telemetry block reading the block that leads the procedure
// Settings ctx.params.key
//   channel     text      Channel = "control"
//   to          text      Only to the block = ""
//   guide_channel text      Guide channel = "guide"
//   facts       textarea  Facts = ""
// EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
// ── end of EsaSim block header ────────────────────────────────────────

/* Step-by-step guide: shows one step of a procedure that a running machine block leads (its live "guide" item),
 * with only the buttons that step takes; a press is sent to the block on the controls channel, and the view says
 * it is waiting until the block answers with the next step (or the same step with a ✗ and why).
 *
 * The block sends, with ctx.telemetry("guide", {...}):
 *   seq       a number that grows with every new item (the view knows its press was answered)
 *   step      "Step 3 · Clear the encoder error"
 *   text      the instruction; lines starting with 1. 2. • become a list
 *   last      the result of the last action: "✓ …" (green) or "✗ …" (red)
 *   buttons   [{"label": "Done, next"}, {"label": "Send this command", "kind": "send"}]  (kind send: sends a command to a device)
 *   extra     ["Back", "Start over", "Finish"]   small buttons under the step
 *   choices   [{"name": "joint", "label": "Joint", "options": ["auto", "1"], "value": "auto"}]
 *   busy      a text while the block works (buttons are off)
 * A press sends {"control": label, "value": true}; a choice sends {"control": name, "value": option}. */
function render(ctx) {
  var p = ctx.params, channel = String(p.channel || 'control'), to = String(p.to || '').trim(), gch = String(p.guide_channel || 'guide');
  var facts = String(p.facts || '').split(/\r?\n/).map(function (l) { return l.trim(); }).filter(Boolean).map(function (l) {
    var k = l.indexOf('='), spec = (k < 0 ? l : l.slice(0, k)).trim(), label = k < 0 ? spec : l.slice(k + 1).trim(), d = spec.indexOf('.');
    return { ch: d < 0 ? spec : spec.slice(0, d), field: d < 0 ? '' : spec.slice(d + 1), label: label };
  });
  ctx.el.innerHTML =
    '<div class="viz-pad" style="display:flex;flex-direction:column;gap:14px;max-width:760px">' +
    '<div class="g-step" style="font:700 20px/1.25 Inter,system-ui,sans-serif"></div>' +
    '<div class="g-last" style="display:none;padding:10px 12px;border-radius:10px;font-weight:600;line-height:1.4"></div>' +
    '<div class="g-text" style="font-size:16px;line-height:1.55"></div>' +
    '<div class="g-choices" style="display:flex;flex-wrap:wrap;gap:10px"></div>' +
    '<div class="g-buttons" style="display:flex;flex-wrap:wrap;gap:10px"></div>' +
    '<div class="g-wait viz-small" style="min-height:1.3em;font-weight:600"></div>' +
    '<div class="g-extra" style="display:flex;flex-wrap:wrap;gap:8px;border-top:1px solid var(--viz-border,#d5dae1);padding-top:10px"></div>' +
    '<div class="g-facts viz-small viz-muted" style="display:flex;flex-direction:column;gap:3px"></div></div>';
  var $ = function (s) { return ctx.el.querySelector(s); };
  var elStep = $('.g-step'), elLast = $('.g-last'), elText = $('.g-text'), elCh = $('.g-choices'), elBtn = $('.g-buttons'),
      elWait = $('.g-wait'), elExtra = $('.g-extra'), elFacts = $('.g-facts');
  var shown = null, waitingFor = null, waitTimer = null;

  function active() { return !!(ctx.sample || (ctx.run && ctx.run.active)); }

  function send(name, value) {
    if (ctx.sample) { elWait.textContent = 'Preview: a press sends ' + name + '.'; return; }
    if (!active()) { elWait.textContent = 'Run the flow first: the steps come from it while it runs.'; return; }
    waitingFor = shown ? shown.seq : null;
    setButtons(false);
    elWait.style.color = '';
    elWait.textContent = 'Sent “' + name + '”. Waiting for the answer…';
    clearTimeout(waitTimer);
    waitTimer = setTimeout(function () {
      if (waitingFor !== null) { elWait.textContent = 'Still waiting for the answer (the arm can take up to 20 seconds after an E-stop)…'; }
    }, 6000);
    ctx.send(channel, { control: name, value: value }, to ? { node: to } : null).catch(function (e) {
      waitingFor = null; setButtons(true); elWait.style.color = '#c0392b'; elWait.textContent = 'Not sent: ' + e.message;
    });
  }

  function setButtons(on) {
    [].forEach.call(ctx.el.querySelectorAll('.g-buttons button, .g-extra button, .g-choices select'), function (b) { b.disabled = !on; });
  }

  function textHtml(t) {
    var esc = function (s) { return String(s).replace(/[&<>"]/g, function (c) { return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]; }); };
    var out = [], list = null;
    String(t || '').split(/\n/).forEach(function (line) {
      var m = /^\s*(\d+\.|•|-)\s+(.*)$/.exec(line);
      if (m) {
        var ordered = /\d/.test(m[1]);
        if (!list || list.ordered !== ordered) { if (list) out.push(list.ordered ? '</ol>' : '</ul>'); list = { ordered: ordered }; out.push(ordered ? '<ol style="margin:4px 0 4px 22px;padding:0">' : '<ul style="margin:4px 0 4px 22px;padding:0">'); }
        out.push('<li style="margin:4px 0">' + esc(m[2]) + '</li>');
      } else {
        if (list) { out.push(list.ordered ? '</ol>' : '</ul>'); list = null; }
        if (/^\s{4,}\S/.test(line)) out.push('<div style="font:600 17px/1.5 ui-monospace,Menlo,Consolas,monospace;margin:6px 0 6px 12px">' + esc(line.trim()) + '</div>');
        else if (line.trim()) out.push('<p style="margin:6px 0">' + esc(line) + '</p>');
      }
    });
    if (list) out.push(list.ordered ? '</ol>' : '</ul>');
    return out.join('');
  }

  function button(label, kind, small) {
    var b = document.createElement('button');
    b.className = small ? 'viz-btn' : 'viz-btn primary';
    b.textContent = label;
    if (!small) b.style.cssText = 'font-size:16px;padding:10px 18px;min-height:44px';
    if (kind === 'send') { b.style.background = '#c2410c'; b.style.borderColor = '#c2410c'; b.style.color = '#fff'; b.title = 'Sends a command to the device'; }
    b.onclick = function () { send(label, true); };
    return b;
  }

  function draw(g) {
    shown = g;
    elStep.textContent = g.step || '';
    var last = String(g.last || '').trim();
    if (last && last !== '—') {
      var ok = last.charAt(0) === '✓', bad = last.charAt(0) === '✗';
      elLast.style.display = '';
      elLast.style.background = ok ? 'rgba(22,163,74,.13)' : bad ? 'rgba(220,38,38,.13)' : 'rgba(100,116,139,.12)';
      elLast.style.color = ok ? '#15803d' : bad ? '#b91c1c' : '';
      elLast.textContent = last;
    } else { elLast.style.display = 'none'; elLast.textContent = ''; }
    elText.innerHTML = textHtml(g.text);
    elCh.innerHTML = '';
    (g.choices || []).forEach(function (c) {
      var r = document.createElement('label'); r.className = 'viz-row'; r.style.gap = '8px';
      var l = document.createElement('span'); l.className = 'viz-small'; l.textContent = c.label || c.name;
      var s = document.createElement('select'); s.className = 'viz-in';
      (c.options || []).forEach(function (o) { var op = document.createElement('option'); op.textContent = o; s.appendChild(op); });
      if (c.value != null) s.value = String(c.value);
      s.onchange = function () { send(c.name, s.value); };
      r.appendChild(l); r.appendChild(s); elCh.appendChild(r);
    });
    elBtn.innerHTML = '';
    (g.buttons || []).forEach(function (b) { elBtn.appendChild(button(typeof b === 'string' ? b : b.label, b && b.kind, false)); });
    elExtra.innerHTML = '';
    (g.extra || []).forEach(function (l) { elExtra.appendChild(button(l, null, true)); });
    elExtra.style.display = (g.extra || []).length ? 'flex' : 'none';
    clearTimeout(waitTimer);
    waitingFor = null;
    elWait.style.color = '';
    elWait.textContent = g.busy ? g.busy : '';
    setButtons(!g.busy && active());
  }

  function pick(d, field) { if (!field) return d; return field.split('.').reduce(function (o, k) { return o == null ? o : o[k]; }, d); }

  function update() {
    var lastBy = {};
    ctx.ports().forEach(function (port) {
      (ctx.input(port).stream || []).forEach(function (it) { if (!lastBy[it.channel] || it.t >= lastBy[it.channel].t) lastBy[it.channel] = it; });
    });
    var it = lastBy[gch], g = it && it.data && typeof it.data === 'object' ? it.data : null;
    if (ctx.sample && !g) g = { seq: 1, step: 'Step 3 · Clear the encoder error', last: '✓ Joint 1 chosen.', text: 'This sends the command that clears the error of joint 1:\n      H101 D0813 V2 I1\nNothing moves. Press Send this command.', buttons: [{ label: 'Send this command', kind: 'send' }], extra: ['Back', 'Start over', 'Finish'] };
    if (g) {
      var key = JSON.stringify(g);
      if (!shown || g.seq !== shown.seq || key !== shown._key) { g._key = key; draw(g); }
    } else if (!shown) {
      elStep.textContent = active() ? 'Waiting for the first step…' : 'Run the flow: its steps appear here.';
    }
    if (waitingFor === null && shown) setButtons(!shown.busy && active());
    elFacts.innerHTML = '';
    facts.forEach(function (f) {
      var r = lastBy[f.ch], v = r ? pick(r.data, f.field) : null;
      if (v == null || v === '') return;
      var d = document.createElement('div'); d.textContent = f.label + ': ' + (typeof v === 'object' ? JSON.stringify(v) : String(v)); elFacts.appendChild(d);
    });
  }
  ctx.on('update', update);
  ctx.on('run', update);
  update();
}
