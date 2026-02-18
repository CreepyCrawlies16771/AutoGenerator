// ========================================
// Core Utilities and State Management
// ========================================

// Canvas and DOM elements
const canvas = document.getElementById('field');
const ctx = canvas.getContext('2d');
const output = document.getElementById('output');
const fieldMeters = 3.6576; // FTC field dimension in meters

// Global state
let ppm = canvas.width / fieldMeters;
let drawX = 0, drawY = 0, drawW = canvas.width, drawH = canvas.height;
let currentMode = 'robot'; // 'robot' or 'field'
let history = [];
let waypoints = [];
let markers = [];
let nextMarkerId = 1;
let start = { x: 0, y: 0, h: 0 };
let draggingIdx = -1;
let draggingHIdx = -1;
let draggingArcControlIdx = -1;
let setStartMode = false;
let addActionMode = false;
let isSimulating = false;
let selectedSegmentIdx = null;
let simSpeed = 1.0;
let arcSamples = 5;

// ========================================
// Math helpers
// ========================================

function radToDeg(r) { return r * 180 / Math.PI; }
function degToRad(d) { return d * Math.PI / 180; }
function wrapDeg(d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

// line intersection helper used for accurate arc control calculation
function lineIntersection(p0, d0, p1, d1) {
    const cross = d0.x * d1.y - d0.y * d1.x;
    if (Math.abs(cross) < 1e-6) return null;
    const dx = p1.x - p0.x;
    const dy = p1.y - p0.y;
    const t = (dx * d1.y - dy * d1.x) / cross;
    return { x: p0.x + t * d0.x, y: p0.y + t * d0.y };
}

// compute default control point (in meters) for an arc segment
function computeDefaultArcControl(prevWp, p) {
    const startX = prevWp.x;
    const startY = prevWp.y;
    const endX = p.x;
    const endY = p.y;
    const dx = endX - startX;
    const dy = endY - startY;
    const dist = Math.hypot(dx, dy);
    if (dist < 1e-6) {
        return { x: (startX + endX) / 2, y: (startY + endY) / 2 };
    }
    const segAngle = Math.atan2(dy, dx);
    const perpAngle = segAngle + Math.PI / 2;
    const controlDist = dist * 0.3;
    const midX = (startX + endX) / 2;
    const midY = (startY + endY) / 2;
    return {
        x: midX + Math.cos(perpAngle) * controlDist,
        y: midY + Math.sin(perpAngle) * controlDist
    };
}

// helper to compute an approximate robot trajectory for field-oriented paths
function computeFieldPath() {
    let pts = [start].concat(waypoints.filter(wp => !wp.isAction).map(wp => ({ x: wp.x, y: wp.y })));
    if (pts.length < 2) return [];
    for (let iter = 0; iter < 2; iter++) {
        const newPts = [];
        newPts.push(pts[0]);
        for (let i = 0; i < pts.length - 1; i++) {
            const p0 = pts[i];
            const p1 = pts[i + 1];
            newPts.push({ x: 0.75 * p0.x + 0.25 * p1.x, y: 0.75 * p0.y + 0.25 * p1.y });
            newPts.push({ x: 0.25 * p0.x + 0.75 * p1.x, y: 0.25 * p0.y + 0.75 * p1.y });
        }
        newPts.push(pts[pts.length - 1]);
        pts = newPts;
    }
    return pts.filter((p, i, a) => i === 0 || Math.hypot(p.x - a[i-1].x, p.y - a[i-1].y) > 1e-6);
}

// ========================================
// Coordinate conversion
// ========================================

function toPixels(mx, my) {
    return {
        x: drawX + (mx + fieldMeters/2) * ppm,
        y: drawY + drawH - (my + fieldMeters/2) * ppm
    };
}

function toMeters(px, py) {
    return {
        x: ((px - drawX) / ppm) - (fieldMeters / 2),
        y: ((drawY + drawH - py) / ppm) - (fieldMeters / 2)
    };
}

// ========================================
// State management
// ========================================

function saveState() {
    history.push({
        waypoints: JSON.parse(JSON.stringify(waypoints)),
        markers: JSON.parse(JSON.stringify(markers)),
        start: JSON.parse(JSON.stringify(start)),
        selectedSegmentIdx: selectedSegmentIdx
    });
    if (history.length > 20) history.shift();
}

function undo() {
    if (history.length === 0) return;
    const state = history.pop();
    waypoints = state.waypoints;
    markers = state.markers;
    start = state.start;
    selectedSegmentIdx = state.selectedSegmentIdx;
    draw();
    generateCode();
}

function setMode(mode) {
    currentMode = mode;
    const btns = document.querySelectorAll('.mode-btn');
    btns.forEach(b => b.classList.remove('active'));
    if (event && event.target) event.target.classList.add('active');
    
    const segmentLabel = document.getElementById('segmentLabel');
    const arcSpeedLabel = document.getElementById('arcSpeedLabel');
    const segmentSelect = document.getElementById('segmentType');
    const arcSpeedInput = document.getElementById('arcSpeed');
    
    if (mode === 'field') {
        segmentLabel.style.display = 'none';
        segmentSelect.style.display = 'none';
        arcSpeedLabel.style.display = 'none';
        arcSpeedInput.style.display = 'none';
    } else {
        segmentLabel.style.display = 'inline';
        segmentSelect.style.display = 'inline';
        arcSpeedLabel.style.display = 'inline';
        arcSpeedInput.style.display = 'inline';
    }
    
    draw();
    generateCode();
}

// ========================================
// Code output formatting with GitHub theme
// ========================================

function displayCodeOutput(code) {
    output.innerHTML = '';
    output.className = 'github-code-block';
    
    // Create a code element with Java syntax highlighting
    const codeEl = document.createElement('code');
    codeEl.className = 'language-java';
    codeEl.textContent = code;
    output.appendChild(codeEl);
    
    // Apply highlight.js if available for syntax highlighting
    if (typeof hljs !== 'undefined') {
        hljs.highlightElement(codeEl);
    }
}

// ========================================
// Field image loading
// ========================================

const fieldImg = new Image();
fieldImg.src = 'field.png';
fieldImg.onload = () => {
    const scale = Math.min(canvas.width / fieldImg.width, canvas.height / fieldImg.height);
    drawW = fieldImg.width * scale;
    drawH = fieldImg.height * scale;
    drawX = (canvas.width - drawW) / 2;
    drawY = (canvas.height - drawH) / 2;
    ppm = drawW / fieldMeters;
    draw();
};

// ========================================
// Helper functions for UI
// ========================================

function handlePosPixels(p) {
    const pos = toPixels(p.x, p.y);
    const r = 18;
    return { x: pos.x + Math.cos(p.h) * r, y: pos.y + Math.sin(p.h) * r };
}

function startHandlePosPixels() {
    const pos = toPixels(start.x, start.y);
    const r = 18;
    return { x: pos.x + Math.cos(start.h) * r, y: pos.y + Math.sin(start.h) * r };
}

function deleteSelectedPoint() {
    if (selectedSegmentIdx !== null && selectedSegmentIdx >= 0 && waypoints[selectedSegmentIdx]) {
        saveState();
        waypoints.splice(selectedSegmentIdx, 1);
        selectedSegmentIdx = null;
        draw(); generateCode();
    }
}

function clearPath() {
    if (confirm('Clear all waypoints?')) {
        saveState();
        waypoints = [];
        markers = [];
        start = { x: 0, y: 0, h: 0 };
        selectedSegmentIdx = null;
        draw();
        output.textContent = "";
    }
}

function toggleAddAction() { addActionMode = !addActionMode; draw(); }

function copyCode() { output.select(); document.execCommand('copy'); alert('Code copied!'); }

function toggleSetStart() { 
    if (currentMode !== 'robot') {
        setStartMode = !setStartMode; 
    }
}

// ========================================
// Drawing engine
// ========================================

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (fieldImg.complete && drawW > 0 && drawH > 0) {
        ctx.drawImage(fieldImg, drawX, drawY, drawW, drawH);
    }

    const spos = toPixels(start.x, start.y);
    ctx.fillStyle = "#10b981";
    ctx.beginPath(); ctx.arc(spos.x, spos.y, 8, 0, Math.PI * 2); ctx.fill();
    
    // Draw start heading handle in all modes
    const sh = startHandlePosPixels();
    ctx.strokeStyle = '#10b981'; ctx.beginPath(); ctx.moveTo(spos.x, spos.y); ctx.lineTo(sh.x, sh.y); ctx.stroke();
    ctx.fillStyle = '#065f46'; ctx.beginPath(); ctx.arc(sh.x, sh.y, 6, 0, Math.PI*2); ctx.fill();

    if (waypoints.length > 0) {
        const first = toPixels(waypoints[0].x, waypoints[0].y);
        ctx.strokeStyle = "#6366f1";
        ctx.beginPath(); ctx.moveTo(spos.x, spos.y); ctx.lineTo(first.x, first.y); ctx.stroke();
    }

    waypoints.forEach((p, i) => {
        const pos = toPixels(p.x, p.y);

        if (i === selectedSegmentIdx) {
            ctx.fillStyle = 'rgba(16, 185, 129, 0.3)';
            ctx.beginPath(); ctx.arc(pos.x, pos.y, 12, 0, Math.PI * 2); ctx.fill();
        }

        if (p.isAction) {
            ctx.fillStyle = '#a78bfa';
            ctx.beginPath();
            ctx.moveTo(pos.x, pos.y - 7);
            ctx.lineTo(pos.x + 7, pos.y);
            ctx.lineTo(pos.x, pos.y + 7);
            ctx.lineTo(pos.x - 7, pos.y);
            ctx.closePath(); ctx.fill();
            const m = markers.find(x => x.id === p.actionId);
            if (m) {
                ctx.fillStyle = '#fff'; ctx.font = '10px sans-serif'; ctx.fillText(m.name, pos.x + 10, pos.y + 3);
            }
        } else {
            const color = p.segmentType === 'strafe' ? '#eab308' : (p.segmentType === 'arc' ? '#06b6d4' : '#3b82f6');
            ctx.fillStyle = color;
            ctx.beginPath(); ctx.arc(pos.x, pos.y, 6, 0, Math.PI * 2); ctx.fill();
        }

        const prev = (i === 0) ? toPixels(start.x, start.y) : toPixels(waypoints[i-1].x, waypoints[i-1].y);
        const color = p.segmentType === 'strafe' ? '#eab308' : (p.segmentType === 'arc' ? '#06b6d4' : '#3b82f6');
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;

        if (currentMode === 'field' || p.segmentType === 'drive' || p.segmentType === 'strafe') {
            ctx.beginPath(); ctx.moveTo(prev.x, prev.y); ctx.lineTo(pos.x, pos.y); ctx.stroke();
        } else if (p.segmentType === 'arc') {
            const prevWp = (i === 0) ? start : waypoints[i-1];
            const startHeading = (i === 0) ? start.h : waypoints[i-1].h;
            let endHeading = p.h;
            const headingDelta = wrapDeg(radToDeg(endHeading - startHeading));
            
            const dx = p.x - prevWp.x;
            const dy = p.y - prevWp.y;
            const dist = Math.hypot(dx, dy);

            if (!p.arcControlPoint) {
                p.arcControlPoint = computeDefaultArcControl(prevWp, p);
            }

            const cpPix = toPixels(p.arcControlPoint.x, p.arcControlPoint.y);
            const controlX = cpPix.x;
            const controlY = cpPix.y;

            if (dist > 0.05 && Math.abs(headingDelta) > 0.01) {
                const startX = prev.x, startY = prev.y;
                const endX = pos.x, endY = pos.y;

                ctx.beginPath();
                ctx.moveTo(startX, startY);
                ctx.quadraticCurveTo(controlX, controlY, endX, endY);
                ctx.stroke();
            } else {
                ctx.beginPath();
                ctx.moveTo(prev.x, prev.y);
                ctx.lineTo(pos.x, pos.y);
                ctx.stroke();
            }
        }
        ctx.lineWidth = 1;

        if (!p.isAction && currentMode === 'robot') {
            const hp = handlePosPixels(p);
            ctx.strokeStyle = '#f59e0b'; ctx.beginPath(); ctx.moveTo(pos.x, pos.y); ctx.lineTo(hp.x, hp.y); ctx.stroke();
            ctx.fillStyle = '#b45309'; ctx.beginPath(); ctx.arc(hp.x, hp.y, 6, 0, Math.PI*2); ctx.fill();
            
            if (p.segmentType === 'arc' && p.arcControlPoint) {
                const acp = toPixels(p.arcControlPoint.x, p.arcControlPoint.y);
                ctx.strokeStyle = '#ec4899'; ctx.lineWidth = 2;
                ctx.beginPath(); ctx.moveTo(pos.x, pos.y); ctx.lineTo(acp.x, acp.y); ctx.stroke();
                ctx.fillStyle = '#be185d'; ctx.beginPath(); ctx.arc(acp.x, acp.y, 8, 0, Math.PI*2); ctx.fill();
                ctx.fillStyle = '#fce7f3'; ctx.font = 'bold 10px sans-serif'; ctx.fillText('C', acp.x - 3, acp.y + 3);
            }
        }
    });

    if (currentMode === 'field') {
        const preview = computeFieldPath();
        if (preview.length > 1) {
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 3;
            ctx.globalAlpha = 0.6;
            ctx.beginPath();
            const first = toPixels(preview[0].x, preview[0].y);
            ctx.moveTo(first.x, first.y);
            for (let j = 1; j < preview.length; j++) {
                const p = toPixels(preview[j].x, preview[j].y);
                ctx.lineTo(p.x, p.y);
            }
            ctx.stroke();
            ctx.globalAlpha = 1.0;
            ctx.lineWidth = 1;
        }
    }

    markers.forEach((a) => {
        const p = toPixels(a.x, a.y);
        ctx.strokeStyle = '#7c3aed'; ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(p.x, p.y - 7);
        ctx.lineTo(p.x + 7, p.y);
        ctx.lineTo(p.x, p.y + 7);
        ctx.lineTo(p.x - 7, p.y);
        ctx.closePath(); ctx.stroke();
        ctx.fillStyle = 'rgba(167,139,250,0.35)'; ctx.fill();
    });

    const btnAddAction = document.querySelector('button[onclick="toggleAddAction()"]');
    if (btnAddAction) {
        if (addActionMode) {
            btnAddAction.style.background = '#10b981';
            btnAddAction.style.color = '#001f13';
            btnAddAction.textContent = 'Add Action: Click field';
        } else {
            btnAddAction.style.background = '';
            btnAddAction.style.color = '';
            btnAddAction.textContent = 'Add Action';
        }
    }
}

// ========================================
// Event handlers
// ========================================

const btnAddAction = document.querySelector('button[onclick="toggleAddAction()"]');
const segmentTypeSelect = document.getElementById('segmentType');
const arcSpeedInput = document.getElementById('arcSpeed');
const arcSamplesInput = document.getElementById('arcSamples');
const simSpeedInput = document.getElementById('simSpeed');
const simSpeedLabel = document.getElementById('simSpeedLabel');

simSpeedInput.addEventListener('change', () => {
    simSpeed = parseFloat(simSpeedInput.value);
    simSpeedLabel.textContent = simSpeed.toFixed(1) + 'x';
});

arcSamplesInput.addEventListener('change', () => {
    arcSamples = parseInt(arcSamplesInput.value);
    generateCode();
});

canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const m = toMeters(px, py);

    const startPos = toPixels(start.x, start.y);
    const ds = Math.hypot(px - startPos.x, py - startPos.y);
    if (ds < 10) { draggingIdx = -2; return; }

    for (let i = 0; i < waypoints.length; i++) {
        const pos = toPixels(waypoints[i].x, waypoints[i].y);
        if (Math.hypot(px - pos.x, py - pos.y) < 10) {
            selectedSegmentIdx = i;
            if (waypoints[i].segmentType) segmentTypeSelect.value = waypoints[i].segmentType;
            if (waypoints[i].arcSpeed) arcSpeedInput.value = waypoints[i].arcSpeed;
            draggingIdx = i;
            return;
        }
        if (waypoints[i].segmentType === 'arc' && waypoints[i].arcControlPoint) {
            const acp = toPixels(waypoints[i].arcControlPoint.x, waypoints[i].arcControlPoint.y);
            if (Math.hypot(px - acp.x, py - acp.y) < 10) {
                draggingArcControlIdx = i;
                return;
            }
        }
        const hpos = handlePosPixels(waypoints[i]);
        if (Math.hypot(px - hpos.x, py - hpos.y) < 10) { draggingHIdx = i; return; }
    }

    // Check start heading handle in all modes
    const sh = startHandlePosPixels();
    if (Math.hypot(px - sh.x, py - sh.y) < 10) { draggingHIdx = -2; return; }

    if (setStartMode && currentMode !== 'robot') {
        saveState();
        start = { x: m.x, y: m.y, h: 0 };
        setStartMode = false;
        draw(); generateCode();
        return;
    }

    for (let i = 0; i < markers.length; i++) {
        const mp = toPixels(markers[i].x, markers[i].y);
        if (Math.hypot(px - mp.x, py - mp.y) < 10) {
            saveState();
            let insertAt = waypoints.length;
            for (let j = waypoints.length - 1; j >= 0; j--) {
                if (!waypoints[j].isAction) { insertAt = j + 1; break; }
            }
            waypoints.splice(insertAt, 0, { x: markers[i].x, y: markers[i].y, h: 0, isAction: true, actionId: markers[i].id });
            draw(); generateCode();
            return;
        }
    }

    if (addActionMode) {
        const name = prompt('Action name (function in AutoEngine):', 'customAction');
        if (name === null || name.trim() === '') { addActionMode = false; draw(); return; }
        const args = prompt('Arguments (comma separated, optional):', '');

        saveState();
        let found = null;
        for (let i = 0; i < markers.length; i++) {
            const d = Math.hypot(markers[i].x - m.x, markers[i].y - m.y);
            if (d < (5 / ppm) && markers[i].name === name.trim()) { found = markers[i]; break; }
        }

        if (found) {
            let insertAt = waypoints.length;
            for (let j = waypoints.length - 1; j >= 0; j--) {
                if (!waypoints[j].isAction) { insertAt = j + 1; break; }
            }
            waypoints.splice(insertAt, 0, { x: found.x, y: found.y, h: 0, isAction: true, actionId: found.id });
        } else {
            const id = nextMarkerId++;
            const marker = { id: id, x: m.x, y: m.y, name: name.trim(), args: args ? args.trim() : '' };
            markers.push(marker);
            let insertAt = waypoints.length;
            for (let j = waypoints.length - 1; j >= 0; j--) {
                if (!waypoints[j].isAction) { insertAt = j + 1; break; }
            }
            waypoints.splice(insertAt, 0, { x: m.x, y: m.y, h: 0, isAction: true, actionId: id });
        }

        addActionMode = false;
        draw(); generateCode();
        return;
    }

    saveState();
    const defaultSegmentType = currentMode === 'field' ? 'drive' : 'drive';
    waypoints.push({ x: m.x, y: m.y, h: 0, segmentType: defaultSegmentType, arcSpeed: 0.8 });
    selectedSegmentIdx = waypoints.length - 1;
    segmentTypeSelect.value = defaultSegmentType;
    arcSpeedInput.value = 0.8;
    draw();
    generateCode();
});

canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    let hoveringMarker = false;
    for (let i = 0; i < markers.length; i++) {
        const mp = toPixels(markers[i].x, markers[i].y);
        if (Math.hypot(px - mp.x, py - mp.y) < 10) { hoveringMarker = true; break; }
    }
    
    let hoveringWaypoint = false;
    for (let i = 0; i < waypoints.length; i++) {
        const pos = toPixels(waypoints[i].x, waypoints[i].y);
        if (Math.hypot(px - pos.x, py - pos.y) < 15) { hoveringWaypoint = true; break; }
    }
    
    if (hoveringMarker) canvas.style.cursor = addActionMode ? 'copy' : 'pointer';
    else if (hoveringWaypoint) canvas.style.cursor = 'move';
    else canvas.style.cursor = 'crosshair';

    if (draggingIdx === -1 && draggingHIdx === -1 && draggingArcControlIdx === -1) return;
    const m = toMeters(px, py);
    if (draggingIdx >= 0) {
        waypoints[draggingIdx].x = m.x;
        waypoints[draggingIdx].y = m.y;
        if (waypoints[draggingIdx].segmentType === 'arc') {
            waypoints[draggingIdx].arcControlPoint = null;
        }
        draw(); generateCode();
    } else if (draggingIdx === -2) {
        start.x = m.x; start.y = m.y; draw(); generateCode();
    } else if (draggingHIdx >= 0) {
        const pos = toPixels(waypoints[draggingHIdx].x, waypoints[draggingHIdx].y);
        waypoints[draggingHIdx].h = Math.atan2(py - pos.y, px - pos.x);
        draw(); generateCode();
    } else if (draggingHIdx === -2) {
        const pos = toPixels(start.x, start.y);
        start.h = Math.atan2(py - pos.y, px - pos.x);
        draw(); generateCode();
    } else if (draggingArcControlIdx >= 0) {
        const wp = waypoints[draggingArcControlIdx];
        wp.arcControlPoint = m;
        wp.h = Math.atan2(wp.y - m.y, wp.x - m.x);
        draw(); generateCode();
    }
});

window.addEventListener('mouseup', () => { 
    draggingIdx = -1; 
    draggingHIdx = -1; 
    draggingArcControlIdx = -1; 
});

segmentTypeSelect.addEventListener('change', () => {
    if (selectedSegmentIdx !== null && waypoints[selectedSegmentIdx]) {
        saveState();
        if (currentMode === 'field' && segmentTypeSelect.value === 'arc') {
            setMode('robot');
        }

        const wp = waypoints[selectedSegmentIdx];
        wp.segmentType = segmentTypeSelect.value;
        if (wp.segmentType === 'arc') {
            wp.arcControlPoint = null;
        } else {
            delete wp.arcControlPoint;
        }
        draw(); generateCode();
    }
});

arcSpeedInput.addEventListener('change', () => {
    if (selectedSegmentIdx !== null && waypoints[selectedSegmentIdx]) {
        saveState();
        waypoints[selectedSegmentIdx].arcSpeed = parseFloat(arcSpeedInput.value);
        generateCode();
    }
});

window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && addActionMode) {
        addActionMode = false; draw();
    }
    if (e.key === 'Delete' && selectedSegmentIdx !== null) {
        deleteSelectedPoint();
    }
});

// ========================================
// Code generation dispatcher
// ========================================

function generateCode() {
    if (currentMode === 'field') {
        generateFieldOrientedCode();
    } else {
        generateRobotOrientedCode();
    }
}

// Initial draw
draw();
