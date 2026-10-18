// // ═══════════════════════════════════════════════════════════════════════
// //  app.js  —  do not edit unless you are changing bridge/rendering logic
// //
// //  Control panel layout is driven entirely by controls-config.js.
// // ═══════════════════════════════════════════════════════════════════════

// // ── Config ───────────────────────────────────────────────────────────
// const WS_URL = 'ws://localhost:8765';
// const RECONNECT_MS = 2000;
// const META_KEYS = new Set(['seq', 'ts_us']);  // hidden from telemetry table

// // ── State ────────────────────────────────────────────────────────────
// let ws = null;
// let pktCount = 0;
// let dropCount = 0;
// let lastSeq = -1;
// let pktThisSec = 0;
// let telemCreated = {};   // which table rows already exist

// // ═══════════════════════════════════════════════════════════════════════
// //  Control panel — build DOM from CONTROL_GROUPS (controls-config.js)
// // ═══════════════════════════════════════════════════════════════════════

// function buildControlPanel() {
// 	const container = document.getElementById('ctrl-groups');

// 	for (const group of CONTROL_GROUPS) {
// 		// ── Section wrapper ───────────────────────────────────────────────
// 		const groupDiv = document.createElement('div');
// 		groupDiv.className = 'ctrl-group';

// 		// ── Section heading ───────────────────────────────────────────────
// 		const title = document.createElement('div');
// 		title.className = 'group-title';
// 		title.textContent = group.label;
// 		groupDiv.appendChild(title);

// 		// ── One row per control ───────────────────────────────────────────
// 		for (const ctrl of group.controls) {
// 			const row = document.createElement('div');
// 			row.className = 'ctrl-row';

// 			// Short display label
// 			const labelEl = document.createElement('span');
// 			labelEl.className = 'ctrl-label';
// 			labelEl.textContent = ctrl.label;

// 			// Full ESP32 key name — dimmed, between label and input
// 			const keyEl = document.createElement('span');
// 			keyEl.className = 'ctrl-key';
// 			keyEl.textContent = ctrl.key;

// 			// Number input — data-key ties it to sendControl()
// 			const input = document.createElement('input');
// 			input.type = 'number';
// 			input.dataset.key = ctrl.key;
// 			input.step = ctrl.step;
// 			input.value = ctrl.default;

// 			row.appendChild(labelEl);
// 			row.appendChild(keyEl);
// 			row.appendChild(input);
// 			groupDiv.appendChild(row);
// 		}

// 		container.appendChild(groupDiv);
// 	}
// }

// // ═══════════════════════════════════════════════════════════════════════
// //  WebSocket
// // ═══════════════════════════════════════════════════════════════════════

// function connect() {
// 	ws = new WebSocket(WS_URL);

// 	ws.onopen = () => {
// 		setStatus(true);
// 		document.getElementById('send-btn').disabled = false;
// 		document.getElementById('req-btn').disabled = false;
// 	};

// 	ws.onclose = () => {
// 		setStatus(false);
// 		document.getElementById('send-btn').disabled = true;
// 		document.getElementById('req-btn').disabled = true;
// 		setTimeout(connect, RECONNECT_MS);
// 	};

// 	ws.onerror = () => ws.close();

// 	ws.onmessage = (ev) => {
// 		let data;
// 		try { data = JSON.parse(ev.data); } catch { return; }

// 		if (data._ctrl_values) {
// 			const vals = data._ctrl_values;
// 			let updated = 0, missing = 0;
// 			for (const [key, val] of Object.entries(vals)) {
// 				const inp = document.querySelector(`input[data-key="${key}"]`);
// 				if (inp) {
// 					inp.value = (typeof val === 'number') ? val : parseFloat(val);
// 					updated++;
// 				} else {
// 					missing++;
// 				}
// 			}
// 			showFeedback(
// 				`Received ${updated} values from ESP32` +
// 				(missing ? `  (${missing} unknown keys ignored)` : '')
// 			);
// 			return;
// 		}

// 		const seq = data.seq;
// 		if (typeof seq === 'number' && lastSeq >= 0) {
// 			const expected = (lastSeq + 1) >>> 0;
// 			if (seq !== expected) {
// 				const dropped = ((seq - lastSeq - 1) + 0x100000000) >>> 0;
// 				dropCount += dropped;
// 				document.getElementById('drop-count').textContent = dropCount;
// 			}
// 		}
// 		if (typeof seq === 'number') lastSeq = seq;

// 		pktCount++;
// 		pktThisSec++;
// 		document.getElementById('pkt-count').textContent = pktCount;

// 		const tbody = document.getElementById('telem-body');
// 		for (const [key, val] of Object.entries(data)) {
// 			if (META_KEYS.has(key)) continue;

// 			if (!telemCreated[key]) {
// 				const row = tbody.insertRow();
// 				row.id = `row_${key}`;
// 				row.insertCell(0).textContent = key;
// 				const cell = row.insertCell(1);
// 				cell.id = `tv_${key}`;
// 				telemCreated[key] = true;
// 			}

// 			const cell = document.getElementById(`tv_${key}`);
// 			if (cell) {
// 				cell.textContent = typeof val === 'number' ? val.toFixed(4) : val;
// 			}
// 		}
// 	};
// }


// // ═══════════════════════════════════════════════════════════════════════
// //  Send all controls to the FC
// // ═══════════════════════════════════════════════════════════════════════

// function sendControl() {
// 	if (!ws || ws.readyState !== WebSocket.OPEN) return;

// 	const inputs = document.querySelectorAll('input[data-key]');
// 	const ctrl = {};
// 	let valid = true;

// 	inputs.forEach(inp => {
// 		const v = parseFloat(inp.value);
// 		if (isNaN(v)) { valid = false; }
// 		ctrl[inp.dataset.key] = v;
// 	});

// 	if (!valid) {
// 		showFeedback('Invalid value in one of the inputs', true);
// 		return;
// 	}

// 	ws.send(JSON.stringify(ctrl));
// 	showFeedback('Sent  →  ' + JSON.stringify(ctrl));
// }

// function requestControlValues() {
// 	if (!ws || ws.readyState !== WebSocket.OPEN) {
// 		showFeedback('Not connected — cannot request', true);
// 		return;
// 	}
// 	ws.send(JSON.stringify({ "_req": "ctrl" }));
// 	showFeedback('Requested current values from ESP32...');
// }


// // ═══════════════════════════════════════════════════════════════════════
// //  UI helpers
// // ═══════════════════════════════════════════════════════════════════════

// function setStatus(connected) {
// 	const dot = document.getElementById('dot');
// 	const text = document.getElementById('conn-text');
// 	dot.className = connected ? 'dot dot-ok' : 'dot dot-bad';
// 	text.textContent = connected ? 'Connected' : 'Disconnected — retrying...';
// }

// let feedbackTimer = null;
// function showFeedback(msg, isError = false) {
// 	const el = document.getElementById('feedback');
// 	el.textContent = msg;
// 	el.className = isError ? 'feedback error' : 'feedback';
// 	clearTimeout(feedbackTimer);
// 	feedbackTimer = setTimeout(() => { el.textContent = ''; }, 3000);
// }

// // ── Rate counter — updates every second ──────────────────────────────
// setInterval(() => {
// 	document.getElementById('rate').textContent =
// 		ws && ws.readyState === WebSocket.OPEN ? pktThisSec : '—';
// 	pktThisSec = 0;
// }, 1000);

// // ═══════════════════════════════════════════════════════════════════════
// //  Init
// // ═══════════════════════════════════════════════════════════════════════

// buildControlPanel();
// connect();

// ═══════════════════════════════════════════════════════════════════════
//  app.js
// ═══════════════════════════════════════════════════════════════════════

const WS_URL       = 'ws://localhost:8765';
const RECONNECT_MS = 2000;
const META_KEYS    = new Set(['seq', 'ts_us']);

let ws          = null;
let pktCount    = 0;
let dropCount   = 0;
let lastSeq     = -1;
let pktThisSec  = 0;
let telemCreated = {};

// ── Build control panel ──────────────────────────────────────────────

function buildControlPanel() {
    const container = document.getElementById('ctrl-groups');

    for (const group of CONTROL_GROUPS) {
        const groupDiv = document.createElement('div');

        const title = document.createElement('b');
        title.textContent = group.label;
        groupDiv.appendChild(title);
        groupDiv.appendChild(document.createElement('br'));

        for (const ctrl of group.controls) {
            const row = document.createElement('div');

            const labelEl = document.createElement('span');
            labelEl.textContent = ctrl.label + '\u00a0';   // non-breaking space

            const input = document.createElement('input');
            input.type        = 'number';
            input.dataset.key = ctrl.key;
            input.step        = ctrl.step;
            input.value       = ctrl.default;

            // Per-field SEND button — placed before the input
            const sendBtn = document.createElement('button');
            sendBtn.textContent    = 'SEND';
            sendBtn.className      = 'per-field-send';
            sendBtn.disabled       = true;
            sendBtn.onclick        = () => sendOne(ctrl.key);

            row.appendChild(labelEl);
            row.appendChild(sendBtn);
            row.appendChild(input);
            groupDiv.appendChild(row);
        }

        groupDiv.appendChild(document.createElement('br'));
        container.appendChild(groupDiv);
    }
}

// ── WebSocket ────────────────────────────────────────────────────────

function connect() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        setStatus(true);
        document.getElementById('send-btn').disabled = false;
        document.getElementById('req-btn').disabled  = false;
        document.querySelectorAll('.per-field-send').forEach(b => b.disabled = false);
    };

    ws.onclose = () => {
        setStatus(false);
        document.getElementById('send-btn').disabled = true;
        document.getElementById('req-btn').disabled  = true;
        document.querySelectorAll('.per-field-send').forEach(b => b.disabled = true);
        setTimeout(connect, RECONNECT_MS);
    };

    ws.onerror = () => ws.close();

    ws.onmessage = (ev) => {
        let data;
        try { data = JSON.parse(ev.data); } catch { return; }

        // ── Ctrl echo from ESP32 ─────────────────────────────────────
        if (data._ctrl_values) {
            const vals = data._ctrl_values;
            let updated = 0, missing = 0;
            for (const [key, val] of Object.entries(vals)) {
                const inp = document.querySelector(`input[data-key="${key}"]`);
                if (inp) { inp.value = (typeof val === 'number') ? val : parseFloat(val); updated++; }
                else     { missing++; }
            }
            showFeedback(
                `Received ${updated} values from ESP32` +
                (missing ? `  (${missing} unknown keys ignored)` : '')
            );
            return;
        }

        // ── Telemetry packet ─────────────────────────────────────────
        const seq = data.seq;
        if (typeof seq === 'number' && lastSeq >= 0) {
            const expected = (lastSeq + 1) >>> 0;
            if (seq !== expected) {
                const dropped = ((seq - lastSeq - 1) + 0x100000000) >>> 0;
                dropCount += dropped;
                document.getElementById('drop-count').textContent = dropCount;
            }
        }
        if (typeof seq === 'number') lastSeq = seq;

        pktCount++;
        pktThisSec++;
        document.getElementById('pkt-count').textContent = pktCount;

        const tbody = document.getElementById('telem-body');
        for (const [key, val] of Object.entries(data)) {
            if (META_KEYS.has(key)) continue;

            if (!telemCreated[key]) {
                const row  = tbody.insertRow();
                row.id     = `row_${key}`;
                row.insertCell(0).textContent = key;
                const cell = row.insertCell(1);
                cell.id    = `tv_${key}`;
                telemCreated[key] = true;
            }

            const cell = document.getElementById(`tv_${key}`);
            if (cell) cell.textContent = typeof val === 'number' ? val.toFixed(4) : val;
        }
    };
}

// ── Send helpers ─────────────────────────────────────────────────────

// Send a single key — called by each row's SEND button
function sendOne(key) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const inp = document.querySelector(`input[data-key="${key}"]`);
    if (!inp) return;
    const v = parseFloat(inp.value);
    if (isNaN(v)) { showFeedback(`Invalid value for ${key}`, true); return; }
    const ctrl = { [key]: v };
    ws.send(JSON.stringify(ctrl));
    showFeedback('Sent  →  ' + JSON.stringify(ctrl));
}

// Send every control at once — called by SEND ALL button
function sendControl() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const ctrl  = {};
    let   valid = true;
    document.querySelectorAll('input[data-key]').forEach(inp => {
        const v = parseFloat(inp.value);
        if (isNaN(v)) valid = false;
        ctrl[inp.dataset.key] = v;
    });
    if (!valid) { showFeedback('Invalid value in one of the inputs', true); return; }
    ws.send(JSON.stringify(ctrl));
    showFeedback('Sent  →  ' + JSON.stringify(ctrl));
}

function requestControlValues() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        showFeedback('Not connected — cannot request', true);
        return;
    }
    ws.send(JSON.stringify({ '_req': 'ctrl' }));
    showFeedback('Requested current values from ESP32...');
}

// ── UI helpers ───────────────────────────────────────────────────────

function setStatus(connected) {
    document.getElementById('dot').style.color      = connected ? 'green' : 'red';
    document.getElementById('conn-text').textContent = connected
        ? 'Connected' : 'Disconnected — retrying...';
}

let feedbackTimer = null;
function showFeedback(msg, isError = false) {
    const el     = document.getElementById('feedback');
    el.textContent  = msg;
    el.style.color  = isError ? 'red' : 'green';
    clearTimeout(feedbackTimer);
    feedbackTimer = setTimeout(() => { el.textContent = ''; }, 3000);
}

setInterval(() => {
    document.getElementById('rate').textContent =
        ws && ws.readyState === WebSocket.OPEN ? pktThisSec : '—';
    pktThisSec = 0;
}, 1000);

buildControlPanel();
connect();