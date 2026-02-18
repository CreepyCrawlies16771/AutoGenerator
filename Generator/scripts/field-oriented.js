// field-oriented.js - Field-oriented path code generation

function generateFieldOrientedCode() {
    function radToDeg(r) { return r * 180 / Math.PI; }

    let code = "private List<Pose2d> createPath() {\n";
    code += "    List<Pose2d> waypoints = new ArrayList<>();\n\n";

    // Add start pose
    const startCmX = (start.x * 100).toFixed(1);
    const startCmY = (start.y * 100).toFixed(1);
    const startDeg = Math.round(radToDeg(-start.h));
    code += `    // Start pose\n`;
    code += `    waypoints.add(new Pose2d(${startCmX}, ${startCmY}, Rotation2d.fromDegrees(${startDeg})));\n\n`;

    // Add waypoints (only non-action waypoints)
    waypoints.forEach((wp, i) => {
        if (!wp.isAction) {
            const cmX = (wp.x * 100).toFixed(1);
            const cmY = (wp.y * 100).toFixed(1);
            const deg = Math.round(radToDeg(-wp.h));
            code += `    waypoints.add(new Pose2d(${cmX}, ${cmY}, Rotation2d.fromDegrees(${deg})));\n`;
        }
    });

    code += "\n    return waypoints;\n";
    code += "}\n\n";

    // Generate createMarkers() method if there are actions
    const hasActions = waypoints.some(wp => wp.isAction);
    if (hasActions) {
        code += "private List<PathMarker> createMarkers() {\n";
        code += "    List<PathMarker> markers = new ArrayList<>();\n\n";

        // Add markers for actions
        let totalWaypoints = waypoints.filter(wp => !wp.isAction).length;
        let currentWaypointIndex = 0;
        
        waypoints.forEach((wp, i) => {
            if (!wp.isAction) {
                currentWaypointIndex++;
            } else {
                const marker = markers.find(m => m.id === wp.actionId);
                if (marker) {
                    const percent = ((currentWaypointIndex / totalWaypoints) * 100).toFixed(1);
                    code += `    markers.add(new PathMarker(${percent}, () -> {\n`;
                    code += `        telemetry.addData("Marker", "${percent}% - ${marker.name}");\n`;
                    code += `        telemetry.update();\n`;
                    code += `    }));\n\n`;
                }
            }
        });

        code += "    return markers;\n";
        code += "}\n";
    } else {
        code += "private List<PathMarker> createMarkers() {\n";
        code += "    return new ArrayList<>();\n";
        code += "}\n";
    }

    displayCodeOutput(code);
}
