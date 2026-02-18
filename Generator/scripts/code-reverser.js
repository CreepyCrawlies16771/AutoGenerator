// code-reverser.js - Decode and reverse-engineer auto code back to visual paths
// Supports both field-oriented (Pose2d ArrayList) and robot-oriented (turnPID/drivePID/etc) code

function toggleDecoder() {
    const section = document.getElementById('decoderSection');
    section.classList.toggle('show');
}

function decodeAndVisualize() {
    const code = document.getElementById('decoderInput').value;
    if (!code.trim()) return;

    try {
        // Preserve current start position
        const prevStart = { x: start.x, y: start.y, h: start.h };
        
        saveState();
        waypoints = [];
        markers = [];
        
        // Detect code type and parse accordingly
        const isFieldOriented = code.includes('Pose2d') || code.includes('ArrayList');
        const isRobotOriented = code.includes('drivePID') || code.includes('strafePID') || code.includes('turnPID') || code.includes('arc(');
        
        if (isFieldOriented) {
            parseFieldOrientedCode(code);
        } else if (isRobotOriented) {
            parseRobotOrientedCode(code);
        } else {
            alert('Unable to detect code format. Please paste either field-oriented (Pose2d ArrayList) or robot-oriented (PID commands) code.');
            return;
        }
        
        // If no start was found in code, keep the previous start position
        if (start.x === 0 && start.y === 0 && start.h === 0 && (prevStart.x !== 0 || prevStart.y !== 0 || prevStart.h !== 0)) {
            start = prevStart;
        }
        
        if (waypoints.length > 0 || (start.x !== 0 || start.y !== 0 || start.h !== 0)) {
            selectedSegmentIdx = 0;
            document.getElementById('decoderSection').classList.remove('show');
            draw();
            generateCode();
        } else {
            alert('No waypoints found in the code.');
        }
    } catch (e) {
        alert('Failed to decode: ' + e.message);
    }
}

function parseFieldOrientedCode(code) {
    // Parse Pose2d coordinates from the code
    const poseRegex = /new\s+Pose2d\((-?\d+\.?\d*),\s*(-?\d+\.?\d*),.*?(?:Rotation2d\.fromDegrees\((-?\d+\.?\d*)\)|new\s+Rotation2d\((-?\d+\.?\d*)\))\)/g;
    
    let match;
    let idx = 0;
    while ((match = poseRegex.exec(code)) !== null) {
        const x = parseFloat(match[1]) / 100; // Convert CM to meters
        const y = parseFloat(match[2]) / 100;
        const angle = parseFloat(match[3] || match[4] || 0);
        
        if (idx === 0) {
            // Only update start if we have a match; preserve existing start if not
            start = { x, y, h: degToRad(-angle) };
        } else {
            waypoints.push({
                x, y,
                h: degToRad(-angle),
                segmentType: 'drive',
                arcSpeed: 0.8
            });
        }
        idx++;
    }
}

function parseRobotOrientedCode(code) {
    // Robot-oriented code assumes start at (0, 0, 0) but we preserve the current start position
    // Only reset if there's no existing start position set
    if (start.x === 0 && start.y === 0 && start.h === 0) {
        start = { x: 0, y: 0, h: 0 };
    }
    // If user already set a start position, keep it and use those coordinates as reference
    
    let currentX = start.x, currentY = start.y, currentHeading = 0;
    let lines = code.split('\n').filter(line => line.trim().length > 0);
    
    // Track the current heading direction
    let curHeadingDeg = 0;
    
    for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();
        
        // Skip comments and empty lines
        if (line.startsWith('//') || line.length === 0) continue;
        
        // Parse turnPID(heading)
        const turnMatch = line.match(/turnPID\s*\(\s*(-?\d+\.?\d*)\s*\)/);
        if (turnMatch) {
            curHeadingDeg = parseFloat(turnMatch[1]);
            continue;
        }
        
        // Parse drivePID(distance, heading)
        const driveMatch = line.match(/drivePID\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/);
        if (driveMatch) {
            const distance = parseFloat(driveMatch[1]);
            const headingDeg = parseFloat(driveMatch[2]);
            const headingRad = degToRad(headingDeg);
            
            // Calculate next position
            currentX += distance * Math.cos(headingRad);
            currentY += distance * Math.sin(headingRad);
            
            waypoints.push({
                x: currentX,
                y: currentY,
                h: degToRad(-headingDeg),
                segmentType: 'drive',
                arcSpeed: 0.8
            });
            curHeadingDeg = headingDeg;
            continue;
        }
        
        // Parse strafePID(distance, heading)
        const strafeMatch = line.match(/strafePID\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/);
        if (strafeMatch) {
            const distance = parseFloat(strafeMatch[1]);
            const headingDeg = parseFloat(strafeMatch[2]);
            const headingRad = degToRad(headingDeg);
            
            currentX += distance * Math.cos(headingRad);
            currentY += distance * Math.sin(headingRad);
            
            waypoints.push({
                x: currentX,
                y: currentY,
                h: degToRad(-headingDeg),
                segmentType: 'strafe',
                arcSpeed: 0.8
            });
            curHeadingDeg = headingDeg;
            continue;
        }
        
        // Parse arc - more complex with heading samples
        const arcStartMatch = line.match(/arc\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,/);
        if (arcStartMatch) {
            const arcDistance = parseFloat(arcStartMatch[1]);
            const arcSpeed = parseFloat(arcStartMatch[2]);
            
            // Look for .at() calls to extract heading samples
            let arcLines = [line];
            let j = i + 1;
            let startHeading = 0, endHeading = 0;
            
            // Collect arc definition lines until closing );
            while (j < lines.length && !lines[j].includes(');')) {
                arcLines.push(lines[j]);
                j++;
            }
            if (j < lines.length) {
                arcLines.push(lines[j]);
            }
            
            const arcCode = arcLines.join('\n');
            
            // Extract heading at start and end
            const atMatches = arcCode.match(/\.at\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/g);
            if (atMatches && atMatches.length >= 2) {
                const firstAt = atMatches[0].match(/\.at\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/);
                const lastAt = atMatches[atMatches.length - 1].match(/\.at\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/);
                
                startHeading = parseFloat(firstAt[2]);
                endHeading = parseFloat(lastAt[2]);
            }
            
            // Approximate end position based on distance and start heading
            const headingRad = degToRad(startHeading);
            currentX += arcDistance * Math.cos(headingRad);
            currentY += arcDistance * Math.sin(headingRad);
            
            waypoints.push({
                x: currentX,
                y: currentY,
                h: degToRad(-endHeading),
                segmentType: 'arc',
                arcSpeed: arcSpeed
            });
            
            curHeadingDeg = endHeading;
            i = j; // Skip the arc lines we just processed
            continue;
        }
    }
}
