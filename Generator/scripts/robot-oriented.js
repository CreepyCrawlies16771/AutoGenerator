// robot-oriented.js - Robot-oriented path code generation

function generateRobotOrientedCode() {
    function radToDeg(r) { return r * 180 / Math.PI; }
    function wrapDeg(d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

    let code = "// Robot-Oriented Path (using OdoEngine)\n";
    code += "// Start: x=0.00 y=0.00 h=0deg (always relative to origin)\n\n";
    code += `turnPID(0); // face start heading\n`;

    const pts = [start].concat(waypoints);
    let curHeading = 0;

    for (let i = 1; i < pts.length; i++) {
        const prev = pts[i-1];
        const cur = pts[i];
        const dx = cur.x - prev.x;
        const dy = cur.y - prev.y;
        const dist = Math.sqrt(dx*dx + dy*dy);

        if (dist < 0.001) {
            if (Math.abs(cur.h) > 0.0001) {
                const wh = radToDeg(-cur.h);
                code += `turnPID(${Math.round(wh)}); // snap to waypoint heading\n`;
            }
            continue;
        }

        const segAngleRad = Math.atan2(dy, dx);
        const segAngleDeg = radToDeg(segAngleRad);

        const segmentType = cur.segmentType || 'drive';
        const arcSpeed = cur.arcSpeed || 0.8;

        if (segmentType === 'arc') {
            const startH = 0; // Always start from 0 in robot mode
            const endH = radToDeg(-cur.h);

            let travelDist = dist;
            if (cur.arcControlPoint) {
                const P0 = { x: prev.x, y: prev.y };
                const P1 = cur.arcControlPoint;
                const P2 = { x: cur.x, y: cur.y };
                let approxLen = 0;
                let lastPt = P0;
                for (let s = 1; s <= 10; s++) {
                    const t = s / 10;
                    const mt = 1 - t;
                    const px = mt*mt*P0.x + 2*mt*t*P1.x + t*t*P2.x;
                    const py = mt*mt*P0.y + 2*mt*t*P1.y + t*t*P2.y;
                    approxLen += Math.hypot(px - lastPt.x, py - lastPt.y);
                    lastPt = {x:px, y:py};
                }
                travelDist = approxLen;
            }

            let lines = [];
            lines.push(`arc(${travelDist.toFixed(2)}, ${arcSpeed.toFixed(2)}, t -> t`);
            lines.push(`                .at(0.0, ${Math.round(startH)})`);
            if (cur.arcControlPoint) {
                const P0 = { x: prev.x, y: prev.y };
                const P1 = cur.arcControlPoint;
                const P2 = { x: cur.x, y: cur.y };
                for (let s = 1; s < arcSamples; s++) {
                    const t = s / arcSamples;
                    const mt = 1 - t;
                    const dx = 2*mt*(P1.x - P0.x) + 2*t*(P2.x - P1.x);
                    const dy = 2*mt*(P1.y - P0.y) + 2*t*(P2.y - P1.y);
                    const h = radToDeg(Math.atan2(dy, dx));
                    lines.push(`                .at(${t.toFixed(2)}, ${Math.round(h)})`);
                }
            }
            lines.push(`                .at(1.0, ${Math.round(endH)})`);
            lines.push(`        );`);

            code += lines.join("\n") + "\n";
            curHeading = endH;
        } else if (segmentType === 'strafe') {
            code += `strafePID(${dist.toFixed(2)}, ${Math.round(segAngleDeg)});\n`;
            curHeading = segAngleDeg;
        } else {
            code += `turnPID(${Math.round(segAngleDeg)});\n`;
            code += `drivePID(${dist.toFixed(2)}, ${Math.round(segAngleDeg)});\n`;
            curHeading = segAngleDeg;
        }

        if (Math.abs(cur.h) > 0.0001 && segmentType !== 'arc') {
            const wh = radToDeg(-cur.h);
            const delta = wrapDeg(wh - segAngleDeg);
            if (Math.abs(delta) > 1.0) {
                code += `turnPID(${Math.round(wh)}); // waypoint heading\n`;
            }
        }

        if (cur.isAction && cur.actionId) {
            const a = markers.find(m => m.id === cur.actionId);
            if (a) {
                code += `\n// Action: ${a.name}\n`;
                if (a.args && a.args.length > 0) code += `${a.name}(${a.args});\n`; 
                else code += `${a.name}();\n`;
            }
        }
    }

    displayCodeOutput(code);
}
