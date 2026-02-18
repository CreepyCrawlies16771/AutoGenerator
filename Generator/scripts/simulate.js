// simulate.js - Path Simulation Engine

function simulate() {
    if (isSimulating) return;
    const points = [start].concat(waypoints);
    if (points.length < 2) return;
    isSimulating = true;

    const driveSpeed = 0.8;
    const turnSpeedDeg = 120;

    let seg = 0;
    let phase = 'turn';
    let pos = { x: points[0].x, y: points[0].y };
    let heading = radToDeg(-points[0].h);
    let totalDistance = 0;
    let distanceTraveled = 0;

    function segInfo(i) {
        const a = points[i];
        const b = points[i+1];
        const dx = b.x - a.x; const dy = b.y - a.y;
        const len = Math.hypot(dx, dy);
        const ang = radToDeg(Math.atan2(dy, dx));
        const segmentType = b.segmentType || 'drive';
        return { len, ang, a, b, segmentType };
    }

    function bezierPoint(t, p0x, p0y, p1x, p1y, p2x, p2y) {
        const mt = 1 - t;
        const x = mt * mt * p0x + 2 * mt * t * p1x + t * t * p2x;
        const y = mt * mt * p0y + 2 * mt * t * p1y + t * t * p2y;
        return { x, y };
    }

    function getArcControlPoint(segIdx) {
        if (segIdx < 1 || segIdx >= waypoints.length) return null;
        const wp = waypoints[segIdx];
        if (wp.segmentType === 'arc') {
            if (!wp.arcControlPoint) {
                const prevWp = (segIdx === 0) ? start : waypoints[segIdx-1];
                wp.arcControlPoint = computeDefaultArcControl(prevWp, wp);
            }
            return wp.arcControlPoint;
        }
        return null;
    }

    // Calculate total distance
    for (let i = 0; i < points.length - 1; i++) {
        totalDistance += segInfo(i).len;
    }

    // Pre-calculate path points for visualization
    let pathPoints = [];
    if (currentMode === 'field') {
        pathPoints = computeFieldPath();
        pathPoints = pathPoints.filter((p, i, a) => i === 0 || Math.hypot(p.x - a[i-1].x, p.y - a[i-1].y) > 1e-6);
        const refined = [];
        for (let i = 0; i < pathPoints.length; i++) {
            refined.push(pathPoints[i]);
            if (i < pathPoints.length - 1) {
                const a = pathPoints[i];
                const b = pathPoints[i+1];
                const d = Math.hypot(b.x - a.x, b.y - a.y);
                if (d > 0.5) {
                    const steps = Math.ceil(d / 0.2);
                    for (let k = 1; k < steps; k++) {
                        refined.push({
                            x: a.x + (b.x - a.x) * (k/steps),
                            y: a.y + (b.y - a.y) * (k/steps)
                        });
                    }
                }
            }
        }
        pathPoints = refined;
    } else {
        for (let i = 0; i < points.length - 1; i++) {
            const info = segInfo(i);
            
            if (info.segmentType === 'arc' && currentMode === 'robot') {
                let controlPt = getArcControlPoint(i);
                if (!controlPt) {
                    controlPt = computeDefaultArcControl(info.a, info.b);
                }
                
                for (let t = 0; t <= 1; t += 0.05) {
                    const pt = bezierPoint(t, info.a.x, info.a.y, controlPt.x, controlPt.y, info.b.x, info.b.y);
                    pathPoints.push(pt);
                }
            } else {
                pathPoints.push(info.a);
                pathPoints.push(info.b);
            }
        }
    }

    // in field mode we simulate by walking along the smoothed pathPoints instead of segments
    if (currentMode === 'field') {
        let simIndex = 0;

        function stepField(dt) {
            if (!isSimulating) return;
            if (simIndex >= pathPoints.length - 1) {
                isSimulating = false;
                draw();
                return;
            }
            const target = pathPoints[simIndex + 1];
            const dx = target.x - pos.x;
            const dy = target.y - pos.y;
            const dist = Math.hypot(dx, dy);
            const travel = driveSpeed * (dt / 1000) * simSpeed;
            if (dist <= travel) {
                pos.x = target.x;
                pos.y = target.y;
                simIndex++;
            } else {
                pos.x += (dx / dist) * travel;
                pos.y += (dy / dist) * travel;
            }
            heading = radToDeg(Math.atan2(dy, dx));

            draw();
            const ppx = toPixels(pos.x, pos.y);
            ctx.fillStyle = '#ef4444';
            ctx.beginPath();
            ctx.arc(ppx.x, ppx.y, 8, 0, Math.PI * 2);
            ctx.fill();
            const hRad = degToRad(heading);
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(ppx.x, ppx.y);
            ctx.lineTo(ppx.x + Math.cos(hRad) * 18, ppx.y - Math.sin(hRad) * 18);
            ctx.stroke();
            ctx.lineWidth = 1;

            if (!isSimulating) return;
            requestAnimationFrame((now) => { stepField(16); });
        }

        requestAnimationFrame((now) => { stepField(16); });
        return;
    }

    let driveProgress = 0;

    function step(dt) {
        if (!isSimulating) return;
        const info = (seg < points.length-1) ? segInfo(seg) : null;

        if (!info) { isSimulating = false; draw(); return; }

        const segType = info.segmentType;

        if (phase === 'turn') {
            const target = wrapDeg(info.ang);
            const diff = wrapDeg(target - heading);
            const maxStep = turnSpeedDeg * (dt/1000) * simSpeed;
            if (Math.abs(diff) <= 0.5) {
                heading = target;
                if (segType === 'strafe') {
                    const roundedHeading = Math.round(heading / 90) * 90;
                    heading = wrapDeg(roundedHeading);
                    phase = 'strafe';
                } else if (segType === 'arc') {
                    phase = 'arc';
                } else {
                    phase = 'drive';
                }
                driveProgress = 0;
            } else {
                heading += Math.sign(diff) * Math.min(maxStep, Math.abs(diff));
            }
        } else if (phase === 'drive') {
            const travel = driveSpeed * (dt/1000) * simSpeed;
            driveProgress += travel;
            distanceTraveled += travel;
            const frac = Math.min(1, driveProgress / info.len || 1);
            pos.x = info.a.x + (info.b.x - info.a.x) * frac;
            pos.y = info.a.y + (info.b.y - info.a.y) * frac;
            heading = info.ang;

            if (frac >= 1) {
                const wpHeading = radToDeg(-points[seg+1].h);
                if (Math.abs(wpHeading) > 0.0001) {
                    phase = 'wpTurn';
                } else {
                    seg++;
                    phase = 'turn';
                }
            }
        } else if (phase === 'strafe') {
            const travel = driveSpeed * (dt/1000) * simSpeed;
            driveProgress += travel;
            distanceTraveled += travel;
            const frac = Math.min(1, driveProgress / info.len || 1);
            
            pos.x = info.a.x + (info.b.x - info.a.x) * frac;
            pos.y = info.a.y + (info.b.y - info.a.y) * frac;

            if (frac >= 1) {
                const wpHeading = radToDeg(-points[seg+1].h);
                if (Math.abs(wpHeading) > 0.0001) {
                    phase = 'wpTurn';
                } else {
                    seg++;
                    phase = 'turn';
                }
            }
        } else if (phase === 'arc') {
            const prevWp = (seg === 0) ? start : waypoints[seg-1];
            const curWp = waypoints[seg];
            const startHeading = (seg === 0) ? start.h : waypoints[seg-1].h;
            const endHeading = curWp.h;
            const headingDelta = wrapDeg(radToDeg(endHeading - startHeading));

            const travel = driveSpeed * (dt/1000) * simSpeed;
            driveProgress += travel;
            distanceTraveled += travel;
            const frac = Math.min(1, driveProgress / info.len || 1);

            let controlPt = getArcControlPoint(seg);
            if (!controlPt) {
                controlPt = computeDefaultArcControl(info.a, info.b);
            }
            const bezPt = bezierPoint(frac, info.a.x, info.a.y, controlPt.x, controlPt.y, info.b.x, info.b.y);
            pos.x = bezPt.x;
            pos.y = bezPt.y;

            const startH = (seg === 0) ? radToDeg(-start.h) : radToDeg(-waypoints[seg-1].h);
            const endH = radToDeg(-curWp.h);
            heading = startH + (endH - startH) * frac;
            heading = wrapDeg(heading);

            if (frac >= 1) {
                const wpHeading = radToDeg(-points[seg+1].h);
                if (Math.abs(wpHeading) > 0.0001) {
                    phase = 'wpTurn';
                } else {
                    seg++;
                    phase = 'turn';
                }
            }
        } else if (phase === 'wpTurn') {
            const target = wrapDeg(radToDeg(-points[seg+1].h));
            const diff = wrapDeg(target - heading);
            const maxStep = turnSpeedDeg * (dt/1000) * simSpeed;
            if (Math.abs(diff) <= 0.5) {
                heading = target;
                seg++;
                phase = 'turn';
            } else {
                heading += Math.sign(diff) * Math.min(maxStep, Math.abs(diff));
            }
        }

        // Draw everything
        draw();

        // Draw the planned path in green
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 3;
        ctx.globalAlpha = 0.6;
        
        for (let i = 0; i < pathPoints.length - 1; i++) {
            const p1 = toPixels(pathPoints[i].x, pathPoints[i].y);
            const p2 = toPixels(pathPoints[i + 1].x, pathPoints[i + 1].y);
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.stroke();
        }

        ctx.globalAlpha = 1.0;
        ctx.lineWidth = 1;

        // Draw robot position
        const ppx = toPixels(pos.x, pos.y);
        ctx.fillStyle = '#ef4444'; 
        ctx.beginPath(); 
        ctx.arc(ppx.x, ppx.y, 8, 0, Math.PI*2); 
        ctx.fill();
        
        // Draw robot heading
        const hRad = degToRad(heading);
        ctx.strokeStyle = '#ffffff'; 
        ctx.lineWidth = 2;
        ctx.beginPath(); 
        ctx.moveTo(ppx.x, ppx.y); 
        ctx.lineTo(ppx.x + Math.cos(hRad) * 18, ppx.y - Math.sin(hRad) * 18); 
        ctx.stroke();
        ctx.lineWidth = 1;

        if (seg >= points.length-1) { isSimulating = false; return; }

        requestAnimationFrame((now) => { step(16); });
    }

    requestAnimationFrame((now) => { step(16); });
}
